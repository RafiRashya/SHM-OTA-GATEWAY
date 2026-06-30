#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "mqtt_client.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "cJSON.h"
#include "secrets.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "driver/gpio.h"


#define MAX_NODES 3 

char current_ota_url[1024] = "";
char target_ota_version[20] = "";
char rebooting_macs[MAX_NODES][18]; 

// ================= STRUKTUR DATA ===================
typedef struct __attribute__((packed)) {
    float ax;
    float ay;
    float az;
    float vbatt;
} SHMData;

typedef struct {
    uint16_t conn_handle;
    char mac_address[18];
    uint16_t notify_handle;
    uint16_t ota_ctrl_handle;
    uint16_t ota_data_handle;
    uint16_t ota_ver_handle; 
    bool is_active;
    bool ota_in_progress;
    bool is_verifying;
} ConnectedNode;

ConnectedNode nodes[MAX_NODES];

// ==================== LED CONFIGURATION ====================
// Pemetaan Pin LED (Sesuai Konfigurasi Solder Mahasiswa)
#define LED_WIFI_PIN    GPIO_NUM_7   // LED Kuning 5mm
#define LED_BLE_PIN     GPIO_NUM_10  // LED Biru 5mm
#define LED_RGB_R_PIN   GPIO_NUM_39  // LED RGB Merah
#define LED_RGB_G_PIN   GPIO_NUM_40  // LED RGB Hijau
#define LED_RGB_B_PIN   GPIO_NUM_42  // LED RGB Biru (Disolder ke GPIO 42)

// Inisialisasi GPIO LED
void init_leds(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_WIFI_PIN) | 
                        (1ULL << LED_BLE_PIN) | 
                        (1ULL << LED_RGB_R_PIN) | 
                        (1ULL << LED_RGB_G_PIN) | 
                        (1ULL << LED_RGB_B_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Setel kondisi awal semua LED mati (LOW)
    gpio_set_level(LED_WIFI_PIN, 0);
    gpio_set_level(LED_BLE_PIN, 0);
    gpio_set_level(LED_RGB_R_PIN, 0);
    gpio_set_level(LED_RGB_G_PIN, 0);
    gpio_set_level(LED_RGB_B_PIN, 0);
}

// Fungsi bantu kendali warna RGB
void set_fota_rgb_led(int r, int g, int b) {
    gpio_set_level(LED_RGB_R_PIN, r);
    gpio_set_level(LED_RGB_G_PIN, g);
    gpio_set_level(LED_RGB_B_PIN, b);
}

// Task delayed turn-off untuk RGB LED (agar tidak memblokir NimBLE thread)
static void rgb_off_timer_task(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(5000)); // Menyala selama 5 detik setelah sukses/gagal
    set_fota_rgb_led(0, 0, 0);
    vTaskDelete(NULL);
}

void trigger_rgb_off_delay(void) {
    xTaskCreate(rgb_off_timer_task, "rgb_off", 2048, NULL, 5, NULL);
}

// Fungsi bantu hitung & perbarui status koneksi BLE
void update_ble_led(void) {
    int active_count = 0;
    for (int i = 0; i < MAX_NODES; i++) {
        if (nodes[i].is_active) {
            active_count++;
        }
    }
    gpio_set_level(LED_BLE_PIN, active_count > 0 ? 1 : 0);
}
// ===========================================================

static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;
static bool wifi_connected = false;

// === UUID BLE ===
static ble_uuid_any_t shm_svc_uuid;
static ble_uuid_any_t shm_chr_uuid;
static ble_uuid_any_t ota_svc_uuid;
static ble_uuid_any_t ota_chr_ctrl_uuid;
static ble_uuid_any_t ota_chr_data_uuid;

// UUID Pembaca Versi
static const ble_uuid128_t ota_chr_ver_uuid = BLE_UUID128_INIT(
    0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 
    0x00, 0x00, 0x03, 0x00, 0x34, 0x12, 0xcd, 0xab
);

static uint8_t own_addr_type; 
static void ble_app_scan(void);

void ota_download_and_send_task(void *pvParameters);

static void get_gateway_mac(char *mac_str) {
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(mac_str, 18, "%02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ==================== FUNGSI BANTUAN ====================
static int find_node_by_handle(uint16_t conn_handle) {
    for (int i = 0; i < MAX_NODES; i++) {
        if (nodes[i].is_active && nodes[i].conn_handle == conn_handle) return i;
    }
    return -1;
}

static void publish_ota_status(const char* mac, const char* status) {
    if (mqtt_connected && mqtt_client != NULL) {
        char payload[128];
        snprintf(payload, sizeof(payload), "{\"cmd\":\"ota_status\", \"node_mac\":\"%s\", \"status\":\"%s\"}", mac, status);
        esp_mqtt_client_publish(mqtt_client, "shm/ota/status", payload, 0, 1, 0);
        printf("[MQTT] Melaporkan Status OTA Node %s: %s\n", mac, status);
    }

    // Kendali LED RGB berdasarkan status FOTA
    if (strcmp(status, "IN_PROGRESS") == 0) {
        set_fota_rgb_led(0, 0, 1); // Biru (sedang transfer biner)
    } else if (strcmp(status, "REBOOTING") == 0) {
        set_fota_rgb_led(0, 0, 1); // Tetap Biru (atau kombinasi ungu/cyan)
    } else if (strcmp(status, "SUCCESS") == 0) {
        set_fota_rgb_led(0, 1, 0); // Hijau (sukses)
        trigger_rgb_off_delay();   // Matikan setelah delay 5 detik
    } else if (strcmp(status, "FAILED") == 0 || strcmp(status, "FAILED_ROLLBACK") == 0) {
        set_fota_rgb_led(1, 0, 0); // Merah (gagal)
        trigger_rgb_off_delay();   // Matikan setelah delay 5 detik
    }
}

// CEK STATUS OTA GLOBAL UNTUK SISTEM DO NOT DISTURB
static bool is_ota_running(void) {
    for (int i = 0; i < MAX_NODES; i++) {
        if (nodes[i].is_active && nodes[i].ota_in_progress) return true;
    }
    return false;
}

// ==================== CALLBACK VERIFIKASI VERSI ====================
static int read_version_cb(uint16_t conn_handle, const struct ble_gatt_error *error, 
                           struct ble_gatt_attr *attr, void *arg) {
    int idx = find_node_by_handle(conn_handle);
    printf("[VERIFIKASI] Callback read_version_cb dipanggil. conn_handle: %d, idx: %d, status: %d\n", conn_handle, idx, error->status);
    if (idx == -1) return 0;
    
    printf("[VERIFIKASI] Status is_verifying pada slot %d: %d\n", idx, nodes[idx].is_verifying);

    if (error->status == 0) {
        char node_ver[20] = {0};
        int len = OS_MBUF_PKTLEN(attr->om);
        
        if (len > 19) len = 19; 
        os_mbuf_copydata(attr->om, 0, len, node_ver);
        node_ver[len] = '\0'; 

        if (nodes[idx].is_verifying) {
            printf("\n[VERIFIKASI] Node MAC %s melapor versi: '%s'\n", nodes[idx].mac_address, node_ver);
            char *n_ver = node_ver;
            char *t_ver = target_ota_version;
            
            if (n_ver[0] == 'v' || n_ver[0] == 'V') n_ver++;
            if (t_ver[0] == 'v' || t_ver[0] == 'V') t_ver++;

            if (strcmp(n_ver, t_ver) == 0) {
                printf("[VERIFIKASI] SUKSES! Versi cocok.\n");
                publish_ota_status(nodes[idx].mac_address, "SUCCESS");
            } else {
                printf("[VERIFIKASI] GAGAL! Versi tidak sama.\n");
                publish_ota_status(nodes[idx].mac_address, "FAILED_ROLLBACK");
            }
            nodes[idx].is_verifying = false;
        }
    } else {
        printf("[VERIFIKASI] Gagal membaca versi GATT! Error: %d\n", error->status);
        publish_ota_status(nodes[idx].mac_address, "FAILED_ROLLBACK");
        nodes[idx].is_verifying = false;
    }
    return 0;
}

// ==================== TASK VERIFIKASI (Tahan Banting) ====================
void verify_version_task(void *pvParameter) {
    int idx = (int)(intptr_t)pvParameter;
    printf("[VERIFIKASI] Task verifikasi dimulai untuk slot %d (MAC: %s)\n", idx, nodes[idx].mac_address);
    
    int wait_time = 0;
    // Gunakan volatile cast agar compiler memuat ulang data dari RAM di setiap iterasi
    while (((volatile ConnectedNode*)&nodes[idx])->ota_ver_handle == 0 && wait_time < 50) {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait_time++;
    }

    uint16_t ver_handle = ((volatile ConnectedNode*)&nodes[idx])->ota_ver_handle;
    printf("[VERIFIKASI] Selesai menunggu. Handle versi: %d (wait_time: %d/50)\n", ver_handle, wait_time);

    if (ver_handle != 0) {
        vTaskDelay(pdMS_TO_TICKS(500)); // Beri jeda agar discovery stabil
        int rc = ble_gattc_read(nodes[idx].conn_handle, ver_handle, read_version_cb, NULL);
        printf("[VERIFIKASI] Eksekusi ble_gattc_read returned: %d\n", rc);
    } else {
        printf("[VERIFIKASI] Timeout! Gagal menemukan handle versi.\n");
        publish_ota_status(nodes[idx].mac_address, "FAILED_ROLLBACK");
        nodes[idx].is_verifying = false;
    }
    vTaskDelete(NULL);
}

// ==================== MQTT HANDLER ====================
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            printf("\n[MQTT] Terhubung secara AMAN (TLS) ke EMQX!\n");
            mqtt_connected = true;

            char gw_mac[18];
            get_gateway_mac(gw_mac);
            char online_payload[64];
            snprintf(online_payload, sizeof(online_payload), "{\"status\":\"online\", \"gateway_mac\":\"%s\"}", gw_mac);
            esp_mqtt_client_publish(mqtt_client, "shm/gateway/status", online_payload, 0, 1, 1);
            printf("[MQTT] Melaporkan Status Gateway: ONLINE\n");

            esp_mqtt_client_subscribe(mqtt_client, "shm/ota/trigger", 1);
            break;

        case MQTT_EVENT_DATA:
            if (strncmp(event->topic, "shm/ota/trigger", event->topic_len) == 0) {
                char *json_str = malloc(event->data_len + 1);
                memcpy(json_str, event->data, event->data_len);
                json_str[event->data_len] = '\0';

                cJSON *root = cJSON_Parse(json_str);
                if (root != NULL) {
                    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
                    cJSON *target_mac = cJSON_GetObjectItem(root, "target_mac");
                    cJSON *url = cJSON_GetObjectItem(root, "url");
                    cJSON *target_ver = cJSON_GetObjectItem(root, "target_version");
                    
                    if (cmd && cJSON_IsString(cmd) && strcmp(cmd->valuestring, "start_ota") == 0) {
                        if (target_mac && url && target_ver) {
                            int target_idx = -1;
                            for (int i = 0; i < MAX_NODES; i++) {
                                if (nodes[i].is_active && strcmp(nodes[i].mac_address, target_mac->valuestring) == 0) {
                                    target_idx = i;
                                    break;
                                }
                            }

                            if (target_idx != -1) {
                                if (!nodes[target_idx].ota_in_progress) {
                                    strncpy(current_ota_url, url->valuestring, sizeof(current_ota_url)-1);
                                    strncpy(target_ota_version, target_ver->valuestring, sizeof(target_ota_version)-1);
                                    printf("\n[MQTT] TRIGGER OTA DITERIMA! NODE: %s\n", nodes[target_idx].mac_address);
                                    
                                    int *arg_idx = malloc(sizeof(int)); 
                                    *arg_idx = target_idx;
                                    xTaskCreate(ota_download_and_send_task, "ota_task", 8192, arg_idx, 5, NULL);
                                }
                            }
                        }
                    }
                    cJSON_Delete(root);
                }
                free(json_str);
            }
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            break;
        default: break;
    }
}

static void mqtt_app_start(void) {
    char gateway_mac[18];
    get_gateway_mac(gateway_mac);

    char lwt_payload[64];
    snprintf(lwt_payload, sizeof(lwt_payload), "{\"status\":\"offline\", \"gateway_mac\":\"%s\"}", gateway_mac);

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = MQTT_BROKER_URI,
            .verification.certificate = (const char *)emqx_ca_cert, 
        },
        .credentials = {
            .username = MQTT_USERNAME,
            .authentication.password = MQTT_PASSWORD,
        },
        .session = {
            .keepalive = 10,
            .last_will = {
                .topic = "shm/gateway/status",
                .msg = lwt_payload,
                .msg_len = strlen(lwt_payload),
                .qos = 1,
                .retain = 1,
            },
        },
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

void ota_download_and_send_task(void *pvParameters) {
    int idx = *((int*)pvParameters);
    free(pvParameters);

    nodes[idx].ota_in_progress = true;
    publish_ota_status(nodes[idx].mac_address, "IN_PROGRESS");
    printf("\n[GATEWAY OTA] Memulai proses Streaming dari Server ke Node %s...\n", nodes[idx].mac_address);

    printf("[GATEWAY OTA] Mematikan background scanner sementara...\n");
    ble_gap_disc_cancel(); 

    uint8_t cmd_start = 0x01;
    ble_gattc_write_flat(nodes[idx].conn_handle, nodes[idx].ota_ctrl_handle, &cmd_start, 1, NULL, NULL);
    vTaskDelay(pdMS_TO_TICKS(1000));

    esp_http_client_config_t config = {
        .url = current_ota_url,
        .method = HTTP_METHOD_GET,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size = 8192,
        .buffer_size_tx = 2048,
        .timeout_ms = 30000, 
        .keep_alive_enable = true, 
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_open(client, 0);
    
    if (err == ESP_OK) {
        esp_http_client_fetch_headers(client);
        int total_len = esp_http_client_get_content_length(client);
        printf("[GATEWAY OTA] Ukuran Firmware dari header: %d bytes. Mulai streaming...\n", total_len);

        uint8_t buffer[500]; 
        int data_read = 0;
        int total_sent = 0;
        bool is_failed = false;
        int bytes_since_last_pause = 0;

        // 2. PERBAIKAN: Logika looping yang bisa mendeteksi putusnya koneksi HTTP
        while (1) {
            data_read = esp_http_client_read(client, (char *)buffer, sizeof(buffer));
            
            // Jika data_read kurang dari 0, berarti Wi-Fi/HTTP terputus!
            if (data_read < 0) {
                printf("\n[GATEWAY OTA] ERROR: Koneksi HTTP terputus di tengah jalan! (Kode: %d)\n", data_read);
                is_failed = true;
                break;
            } else if (data_read == 0) {
                // Jika 0, berarti benar-benar sudah mencapai akhir file
                printf("[GATEWAY OTA] Selesai membaca file dari server.\n");
                break;
            }

            int rc;
            do {
                rc = ble_gattc_write_no_rsp_flat(nodes[idx].conn_handle, nodes[idx].ota_data_handle, buffer, data_read);
                if (rc == 6 || rc == 130) vTaskDelay(pdMS_TO_TICKS(10));
            } while (rc == 6 || rc == 130);

            if (rc != 0) { 
                printf("[GATEWAY OTA] ERROR: Gagal menulis BLE (Kode: %d)\n", rc);
                is_failed = true; 
                break; 
            }

            total_sent += data_read;
            bytes_since_last_pause += data_read; 
            
            if (bytes_since_last_pause >= 4096) {
                vTaskDelay(pdMS_TO_TICKS(150)); 
                bytes_since_last_pause -= 4096; 
            } else {
                vTaskDelay(pdMS_TO_TICKS(15));  
            }
            
            if (total_sent % 10000 < sizeof(buffer)) {
                printf("[GATEWAY OTA] Progres: %d / %d bytes...\n", total_sent, total_len);
            }
        }
        
        // 3. PERBAIKAN: Validasi absolut sebelum mengirim perintah END (Anti-Tertipu)
        bool is_complete = esp_http_client_is_complete_data_received(client);
        
        if (is_failed || !is_complete || (total_len > 0 && total_sent < total_len)) {
            printf("\n[GATEWAY OTA] GAGAL! File tidak utuh. (Terkirim: %d bytes)\n", total_sent);
            publish_ota_status(nodes[idx].mac_address, "FAILED");
        } else {
            printf("\n[GATEWAY OTA] Streaming UTUH selesai! Mengirim perintah END...\n");
            vTaskDelay(pdMS_TO_TICKS(500)); 
            
            uint8_t cmd_end = 0x02;
            int rc_end;
            do {
                rc_end = ble_gattc_write_flat(nodes[idx].conn_handle, nodes[idx].ota_ctrl_handle, &cmd_end, 1, NULL, NULL);
                if (rc_end == 6 || rc_end == 130) vTaskDelay(pdMS_TO_TICKS(50));
            } while (rc_end == 6 || rc_end == 130);

            // Tambahkan MAC ke daftar rebooting/verifikasi
            for (int i = 0; i < MAX_NODES; i++) {
                if (strlen(rebooting_macs[i]) == 0) {
                    strncpy(rebooting_macs[i], nodes[idx].mac_address, 18);
                    break;
                }
            }
            publish_ota_status(nodes[idx].mac_address, "REBOOTING");
        }
    } else {
        printf("[GATEWAY OTA] Gagal terhubung ke URL Server!\n");
        publish_ota_status(nodes[idx].mac_address, "FAILED");
    }

    esp_http_client_cleanup(client);
    nodes[idx].ota_in_progress = false;

    // Menyalakan ulang Scanner hanya jika tidak ada Node lain yang sedang OTA
    if (!is_ota_running()) {
        printf("[GATEWAY OTA] OTA Selesai. Menyalakan kembali BLE Scanner...\n");
        ble_app_scan();
    }
    vTaskDelete(NULL);
}

// ==================== CALLBACK BLE GATT ====================
typedef struct {
    uint16_t conn_handle;
    uint16_t dsc_handle;
} subscribe_args_t;

static void subscribe_task(void *pvParameters) {
    subscribe_args_t *args = (subscribe_args_t *)pvParameters;
    uint16_t conn_handle = args->conn_handle;
    uint16_t dsc_handle = args->dsc_handle;
    free(args);

    vTaskDelay(pdMS_TO_TICKS(100)); // Beri jeda 100ms agar discovery selesai sepenuhnya

    uint8_t value[2] = {0x01, 0x00};
    int rc;
    int retries = 0;
    
    do {
        rc = ble_gattc_write_flat(conn_handle, dsc_handle, value, sizeof(value), NULL, NULL);
        if (rc == 6 || rc == 130) {
            vTaskDelay(pdMS_TO_TICKS(50));
            retries++;
        }
    } while ((rc == 6 || rc == 130) && retries < 10);

    if (rc == 0) {
        printf(">> Subscribe Sukses!\n");
    } else {
        printf(">> Subscribe Gagal (Error: %d)\n", rc);
    }
    
    if (!is_ota_running()) {
        printf(">> Membuka gerbang pencarian untuk Node selanjutnya...\n");
        ble_app_scan();
    }
    vTaskDelete(NULL);
}

static int on_disc_dsc(uint16_t conn_handle, const struct ble_gatt_error *error,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg) {
    if (error->status == 0) {
        if (ble_uuid_u16(&dsc->uuid.u) == 0x2902) {
            printf(">> Mengaktifkan Notifikasi SHM (Subscribe)...\n"); 
            
            subscribe_args_t *args = malloc(sizeof(subscribe_args_t));
            if (args != NULL) {
                args->conn_handle = conn_handle;
                args->dsc_handle = dsc->handle;
                xTaskCreate(subscribe_task, "sub_task", 4096, args, 5, NULL);
            } else {
                printf(">> Gagal alokasi memori untuk task subscribe!\n");
            }
        }
    }
    return 0;
}

static int on_disc_chr(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg) {
    if (error->status == 0) {
        int idx = find_node_by_handle(conn_handle);
        if (idx == -1) return 0;

        if (ble_uuid_cmp(&chr->uuid.u, &shm_chr_uuid.u) == 0) {
            nodes[idx].notify_handle = chr->val_handle;
            ble_gattc_disc_all_dscs(conn_handle, chr->val_handle, chr->val_handle + 10, on_disc_dsc, NULL);
        }
        else if (ble_uuid_cmp(&chr->uuid.u, &ota_chr_ctrl_uuid.u) == 0) {
            nodes[idx].ota_ctrl_handle = chr->val_handle;
        }
        else if (ble_uuid_cmp(&chr->uuid.u, &ota_chr_data_uuid.u) == 0) {
            nodes[idx].ota_data_handle = chr->val_handle;
        }
        else if (ble_uuid_cmp(&chr->uuid.u, &ota_chr_ver_uuid.u) == 0) {
            nodes[idx].ota_ver_handle = chr->val_handle; 
        }
    }
    return 0;
}

static int on_disc_svc(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *svc, void *arg) {
    if (error->status == 0) {
        if (ble_uuid_cmp(&svc->uuid.u, &shm_svc_uuid.u) == 0 || ble_uuid_cmp(&svc->uuid.u, &ota_svc_uuid.u) == 0) {
            ble_gattc_disc_all_chrs(conn_handle, svc->start_handle, svc->end_handle, on_disc_chr, NULL);
        }
    }
    return 0;
}

static int on_mtu_exchanged(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t mtu, void *arg) {
    if (error->status == 0) {
        printf("[BLE] MTU Sukses Dinegosiasikan! Kapasitas: %d bytes\n", mtu);
    } else {
        printf("[BLE] Gagal MTU (Error: %d). Memakai Default 23 bytes.\n", error->status);
    }
    
    // KUNCI JAWABAN: Panggil Service Discovery HANYA SETELAH MTU Selesai!
    ble_gattc_disc_all_svcs(conn_handle, on_disc_svc, NULL);
    return 0;
}

// ==================== EVENT GAP ====================
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
                int empty_idx = -1;
                for (int i = 0; i < MAX_NODES; i++) {
                    if (!nodes[i].is_active) { empty_idx = i; break; }
                }

                if (empty_idx != -1) {
                    nodes[empty_idx].is_active = true;
                    nodes[empty_idx].conn_handle = event->connect.conn_handle;
                    
                    struct ble_gap_conn_desc desc;
                    ble_gap_conn_find(event->connect.conn_handle, &desc);
                    snprintf(nodes[empty_idx].mac_address, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
                             desc.peer_id_addr.val[5], desc.peer_id_addr.val[4], desc.peer_id_addr.val[3],
                             desc.peer_id_addr.val[2], desc.peer_id_addr.val[1], desc.peer_id_addr.val[0]);
                             
                    printf("\n=== KONEKSI BLE BERHASIL! Node MAC: %s di slot %d ===\n", nodes[empty_idx].mac_address, empty_idx);
                    update_ble_led();

                    ble_gattc_exchange_mtu(event->connect.conn_handle, on_mtu_exchanged, NULL);
                    
                    bool needs_verification = false;
                    for (int i = 0; i < MAX_NODES; i++) {
                        if (strlen(rebooting_macs[i]) > 0 && strcmp(nodes[empty_idx].mac_address, rebooting_macs[i]) == 0) {
                            needs_verification = true;
                            memset(rebooting_macs[i], 0, 18);
                            break;
                        }
                    }
                    
                    if (needs_verification) {
                        nodes[empty_idx].is_verifying = true;
                        xTaskCreate(verify_version_task, "verify_task", 4096, (void*)empty_idx, 5, NULL);
                    }
                } else {
                    printf(">> Kapasitas Gateway Penuh! Memutus koneksi...\n");
                    ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                }

            } else {
                // Jika koneksi gagal, baru boleh scan lagi
                if (!is_ota_running()) ble_app_scan();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            printf(">> Terputus dari Node!\n");
            int idx = find_node_by_handle(event->disconnect.conn.conn_handle);
            if (idx != -1) {
                memset(&nodes[idx], 0, sizeof(ConnectedNode)); 
            }
            update_ble_led();
            // Jangan nyalakan scanner jika OTA sedang berjalan! (Fitur Do Not Disturb)
            if (!is_ota_running()) ble_app_scan();
            break;

        case BLE_GAP_EVENT_NOTIFY_RX: { 
            int n_idx = find_node_by_handle(event->notify_rx.conn_handle);
            if (n_idx != -1 && event->notify_rx.attr_handle == nodes[n_idx].notify_handle) {
                SHMData shm;
                if (OS_MBUF_PKTLEN(event->notify_rx.om) == sizeof(SHMData)) { 
                    os_mbuf_copydata(event->notify_rx.om, 0, sizeof(SHMData), &shm);
                    
                    printf("[SHM] Node %s | AX:%.2f AY:%.2f AZ:%.2f | Vbatt: %.2fV\n", 
                           nodes[n_idx].mac_address, shm.ax, shm.ay, shm.az, shm.vbatt);
                    
                    if (mqtt_connected) {
                        char json_payload[128];
                        snprintf(json_payload, sizeof(json_payload), 
                                 "{\"node_mac\":\"%s\", \"ax\":%.2f, \"ay\":%.2f, \"az\":%.2f, \"vbatt\":%.2f}", 
                                 nodes[n_idx].mac_address, shm.ax, shm.ay, shm.az, shm.vbatt);
                        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, json_payload, 0, 1, 0);
                    }
                }
            }
            break;
        }
    }
    return 0;
}

static void ble_app_scan(void) {
    // 1. CEK KAPASITAS: Jangan scan jika slot sudah penuh!
    int active_nodes = 0;
    for (int i = 0; i < MAX_NODES; i++) {
        if (nodes[i].is_active) active_nodes++;
    }
    
    if (active_nodes >= MAX_NODES) {
        printf(">> Kapasitas penuh (%d/%d Node). Scanner dimatikan untuk menghemat antena.\n", active_nodes, MAX_NODES);
        return; 
    }

    struct ble_gap_disc_params disc_params;
    memset(&disc_params, 0, sizeof(disc_params));
    disc_params.filter_duplicates = 1; 
    disc_params.passive = 0; // Harus 0 (Active) agar bisa membaca nama "SHM_Node_C3"
    
    // ========================================================================
    // 2. ATUR NAFAS ANTENA (DUTY CYCLE 50% COEXISTENCE)
    // Satuan itvl dan window adalah 0.625 ms.
    // Interval 160 * 0.625 = 100 milidetik (Total siklus)
    // Window   80 * 0.625  = 50 milidetik (Waktu aktif scan)
    // Sisa 50ms akan otomatis diberikan ke Wi-Fi & Node yang sudah terkoneksi!
    // ========================================================================
    disc_params.itvl = 160;   
    disc_params.window = 80;  

    ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, ble_gap_event_cb, NULL);
    printf(">> BLE Scanner dinyalakan (Mode Hemat Antena)...\n");
}

static void ble_app_on_sync(void) {
    ble_uuid_from_str(&shm_svc_uuid, "12345678-1234-1234-1234-1234567890ab");
    ble_uuid_from_str(&shm_chr_uuid, "abcd1234-5678-90ab-cdef-1234567890ab");
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

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        gpio_set_level(LED_WIFI_PIN, 0); // Mati saat inisiasi koneksi
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        gpio_set_level(LED_WIFI_PIN, 0); // Mati saat terputus
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        printf("[WIFI] Terhubung! IP Address: " IPSTR "\n", IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        gpio_set_level(LED_WIFI_PIN, 1); // Menyala stabil saat mendapat IP
        if (mqtt_client == NULL) {
            mqtt_app_start();
        }
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
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, 
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

void app_main(void) {
    esp_log_level_set("NimBLE", ESP_LOG_WARN);
    printf("\n--- GATEWAY SHM ESP32-S3 (WIFI + SHM + OTA) ---\n");

    // Inisialisasi LED GPIO
    init_leds();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }

    wifi_init_sta();

    nimble_port_init();
    ble_att_set_preferred_mtu(512); 
    ble_svc_gap_device_name_set("SHM_Gateway_S3");
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(ble_host_task);
}