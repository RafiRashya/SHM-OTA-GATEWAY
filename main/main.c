#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

// ================= PENGATURAN JARINGAN =================
#define WIFI_SSID "Wifi-nya Rachelle"
#define WIFI_PASS "minimaltaudiri"
#define FIRMWARE_URL "http://192.168.100.184:5000/firmware/download/nimble-shm-ota.bin"

// ================= STRUKTUR DATA SHM ===================
typedef struct __attribute__((packed)) {
    float ax;
    float ay;
    float az;
} SHMData;

// === UUID BLE ===
static ble_uuid_any_t shm_svc_uuid;
static ble_uuid_any_t shm_chr_uuid;
static ble_uuid_any_t ota_svc_uuid;
static ble_uuid_any_t ota_chr_ctrl_uuid;
static ble_uuid_any_t ota_chr_data_uuid;

// === STATE & HANDLE ===
static uint8_t own_addr_type; 
static uint16_t target_conn_handle = BLE_HS_CONN_HANDLE_NONE;

// Handle SHM
static uint16_t notify_char_val_handle = 0;

// Handle OTA
static uint16_t ota_ctrl_handle = 0;
static uint16_t ota_data_handle = 0;

static bool wifi_connected = false;
static bool ota_in_progress = false;
static bool ota_sudah_dilakukan = false; // Mencegah OTA berulang-ulang (Loop mati)

static void ble_app_scan(void);
static void check_and_start_ota(void);

// ==================== TASK OTA STREAMING ====================
void ota_download_and_send_task(void *pvParameters) {
    ota_in_progress = true;
    printf("\n[GATEWAY OTA] Memulai proses Streaming dari Server ke Node...\n");

    // 1. Kirim Perintah START (0x01) ke Node
    uint8_t cmd_start = 0x01;
    ble_gattc_write_flat(target_conn_handle, ota_ctrl_handle, &cmd_start, 1, NULL, NULL);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Beri waktu Node menghapus flash

    // 2. Siapkan HTTP Client
    esp_http_client_config_t config = {
        .url = FIRMWARE_URL,
        .method = HTTP_METHOD_GET,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_open(client, 0);
    
    if (err == ESP_OK) {
        esp_http_client_fetch_headers(client);
        int total_len = esp_http_client_get_content_length(client);
        printf("[GATEWAY OTA] Ukuran Firmware: %d bytes. Mulai streaming...\n", total_len);

        uint8_t buffer[256]; 
        int data_read = 0;
        int total_sent = 0;

        // 3. Loop Membaca HTTP -> Menulis ke BLE
        while ((data_read = esp_http_client_read(client, (char *)buffer, sizeof(buffer))) > 0) {
            
            int rc;
            do {
                rc = ble_gattc_write_no_rsp_flat(target_conn_handle, ota_data_handle, buffer, data_read);
                if (rc == 6 || rc == 130) { // Jika Buffer Penuh (ENOMEM / ESTALLED)
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            } while (rc == 6 || rc == 130);

            if (rc != 0) {
                printf("[GATEWAY OTA] Error fatal kirim BLE: %d\n", rc);
                break;
            }

            total_sent += data_read;
            
            // =========================================================
            // SMART DELAY: Penyelamat Flash Memory!
            // Karena ESP32 menghapus flash per 4096 byte (4KB), 
            // kita beri jeda ekstra panjang setiap kali batas ini terlewati.
            // =========================================================
            if (total_sent % 4096 == 0) {
                // Jeda 150ms khusus untuk memberi waktu Node menghapus Sektor Flash
                vTaskDelay(pdMS_TO_TICKS(150)); 
            } else {
                // Jeda normal antar paket 256 byte
                vTaskDelay(pdMS_TO_TICKS(20));  
            }
            
            if (total_sent % 10240 < sizeof(buffer)) {
                printf("[GATEWAY OTA] Progres: %d / %d bytes...\n", total_sent, total_len);
            }
        }
        
        printf("[GATEWAY OTA] Streaming selesai! Mengirim perintah END...\n");

        // 4. Kirim Perintah END (0x02) ke Node
        uint8_t cmd_end = 0x02;
        ble_gattc_write_flat(target_conn_handle, ota_ctrl_handle, &cmd_end, 1, NULL, NULL);

        ota_sudah_dilakukan = true; // Tandai OTA sukses agar tidak diulang

    } else {
        printf("[GATEWAY OTA] Gagal terhubung ke Server Flask!\n");
    }

    esp_http_client_cleanup(client);
    ota_in_progress = false;
    vTaskDelete(NULL);
}

// ==================== CALLBACK BLE GATT ====================

// Callback Descriptor (Khusus untuk Subscribe Data SHM CCCD 0x2902)
static int on_disc_dsc(uint16_t conn_handle, const struct ble_gatt_error *error,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg) {
    if (error->status == 0) {
        if (ble_uuid_u16(&dsc->uuid.u) == 0x2902) {
            printf(">> Mengaktifkan Notifikasi SHM (Subscribe)...\n");
            uint8_t value[2] = {0x01, 0x00};
            ble_gattc_write_flat(conn_handle, dsc->handle, value, sizeof(value), NULL, NULL);
        }
    }
    return 0;
}

// Callback Karakteristik (Mencari Handle SHM dan OTA)
static int on_disc_chr(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg) {
    if (error->status == 0) {
        
        // 1. Cek Karakteristik SHM
        if (ble_uuid_cmp(&chr->uuid.u, &shm_chr_uuid.u) == 0) {
            printf(">> Karakteristik SHM Ditemukan! Mencari Descriptor...\n");
            notify_char_val_handle = chr->val_handle;
            ble_gattc_disc_all_dscs(conn_handle, chr->val_handle, chr->val_handle + 10, on_disc_dsc, NULL);
        }
        // 2. Cek Karakteristik OTA Control
        else if (ble_uuid_cmp(&chr->uuid.u, &ota_chr_ctrl_uuid.u) == 0) {
            printf(">> Karakteristik OTA Control Ditemukan!\n");
            ota_ctrl_handle = chr->val_handle;
        }
        // 3. Cek Karakteristik OTA Data
        else if (ble_uuid_cmp(&chr->uuid.u, &ota_chr_data_uuid.u) == 0) {
            printf(">> Karakteristik OTA Data Ditemukan!\n");
            ota_data_handle = chr->val_handle;
        }

       check_and_start_ota();
    }
    return 0;
}

// Callback Service
static int on_disc_svc(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *svc, void *arg) {
    if (error->status == 0) {
        // Jika nemu service SHM atau OTA, minta NimBLE membongkar Karakteristiknya
        if (ble_uuid_cmp(&svc->uuid.u, &shm_svc_uuid.u) == 0 || ble_uuid_cmp(&svc->uuid.u, &ota_svc_uuid.u) == 0) {
            ble_gattc_disc_all_chrs(conn_handle, svc->start_handle, svc->end_handle, on_disc_chr, NULL);
        }
    }
    return 0;
}

// Callback Event GAP
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    struct ble_hs_adv_fields fields;

    switch (event->type) {
        case BLE_GAP_EVENT_DISC:
            ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
            if (fields.name_len > 0 && strncmp((char*)fields.name, "SHM_Node_C3", fields.name_len) == 0) {
                printf("\n[GATEWAY] Target 'SHM_Node_C3' Ditemukan! Memulai koneksi...\n");
                ble_gap_disc_cancel();
                ble_gap_connect(own_addr_type, &event->disc.addr, 30000, NULL, ble_gap_event_cb, NULL);
            }
            break;

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                printf("\n=== KONEKSI BLE BERHASIL! ===\n");
                target_conn_handle = event->connect.conn_handle;
                ota_ctrl_handle = 0; ota_data_handle = 0; notify_char_val_handle = 0;
                
                ble_gattc_exchange_mtu(target_conn_handle, NULL, NULL);
                ble_gattc_disc_all_svcs(target_conn_handle, on_disc_svc, NULL); // Cari semua service
            } else {
                ble_app_scan();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            printf("Terputus dari Node! Kembali scanning...\n");
            target_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_app_scan();
            break;

        case BLE_GAP_EVENT_NOTIFY_RX:
            // TERIMA DATA SHM
            if (event->notify_rx.attr_handle == notify_char_val_handle) {
                SHMData shm;
                uint16_t data_len = OS_MBUF_PKTLEN(event->notify_rx.om);
                if (data_len == sizeof(SHMData)) {
                    os_mbuf_copydata(event->notify_rx.om, 0, sizeof(SHMData), &shm);
                    // Print data SHM (Akan terus mengalir walau OTA sudah selesai)
                    printf("[SHM] AX:%.2f AY:%.2f AZ:%.2f\n", shm.ax, shm.ay, shm.az);
                }
            }
            break;
    }
    return 0;
}

static void ble_app_scan(void) {
    struct ble_gap_disc_params disc_params;
    memset(&disc_params, 0, sizeof(disc_params));
    disc_params.filter_duplicates = 1; disc_params.passive = 0;
    ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, ble_gap_event_cb, NULL);
}

// Fungsi untuk mengecek apakah semua syarat OTA sudah terpenuhi
static void check_and_start_ota(void) {
    if (wifi_connected && ota_ctrl_handle != 0 && ota_data_handle != 0 && !ota_in_progress && !ota_sudah_dilakukan) {
        printf("\n[SYSTEM] Wi-Fi dan BLE OTA siap! Memulai Task OTA...\n");
        xTaskCreate(ota_download_and_send_task, "ota_task", 8192, NULL, 5, NULL);
    }
}

static void ble_app_on_sync(void) {
    // UUID SHM
    ble_uuid_from_str(&shm_svc_uuid, "12345678-1234-1234-1234-1234567890ab");
    ble_uuid_from_str(&shm_chr_uuid, "abcd1234-5678-90ab-cdef-1234567890ab");
    // UUID OTA
    ble_uuid_from_str(&ota_svc_uuid, "12345678-0000-0000-0000-1234567890ab");
    ble_uuid_from_str(&ota_chr_ctrl_uuid, "abcd1234-0001-0000-0000-1234567890ab");
    ble_uuid_from_str(&ota_chr_data_uuid, "abcd1234-0002-0000-0000-1234567890ab");
    
    ble_hs_id_infer_auto(0, &own_addr_type);
    ble_app_scan();
}

void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ==================== INISIALISASI WI-FI ====================
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        printf("[WIFI] Terhubung! IP Address: " IPSTR "\n", IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        check_and_start_ota();
    }
}

void wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

// ==================== APP MAIN ====================
void app_main(void) {
    printf("\n--- GATEWAY SHM ESP32-S3 (WIFI + SHM + OTA) ---\n");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }

    wifi_init_sta();

    nimble_port_init();
    ble_svc_gap_device_name_set("SHM_Gateway_S3");
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(ble_host_task);
}