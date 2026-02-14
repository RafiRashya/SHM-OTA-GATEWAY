#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

#define SERVICE_UUID_STR "12345678-1234-1234-1234-1234567890ab"
#define CHAR_UUID_STR    "abcd1234-5678-90ab-cdef-1234567890ab"

typedef struct __attribute__((packed)) {
    float ax;
    float ay;
    float az;
} SHMData;

static ble_uuid_any_t svc_uuid;
static ble_uuid_any_t chr_uuid;
static uint16_t target_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t notify_char_val_handle = 0;
static uint8_t own_addr_type; 

static void ble_app_scan(void);

// 5. Callback Descriptor (Mencari CCCD 0x2902)
static int on_disc_dsc(uint16_t conn_handle, const struct ble_gatt_error *error,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg) {
    if (error->status == 0) {
        if (ble_uuid_u16(&dsc->uuid.u) == 0x2902) {
            printf(">> CCCD 0x2902 Ditemukan! Mengaktifkan Notifikasi (Subscribe)...\n");
            uint8_t value[2] = {0x01, 0x00};
            ble_gattc_write_flat(conn_handle, dsc->handle, value, sizeof(value), NULL, NULL);
        }
    }
    return 0;
}

// 4. Callback Karakteristik
static int on_disc_chr(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg) {
    if (error->status == 0) {
        if (ble_uuid_cmp(&chr->uuid.u, &chr_uuid.u) == 0) {
            printf(">> Karakteristik Data Ditemukan! Mencari Descriptor 0x2902...\n");
            notify_char_val_handle = chr->val_handle;
            ble_gattc_disc_all_dscs(conn_handle, chr->val_handle, chr->val_handle + 10, on_disc_dsc, NULL);
        }
    }
    return 0;
}

// 3. Callback Service
static int on_disc_svc(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *svc, void *arg) {
    if (error->status == 0) {
        printf(">> Service SHM Ditemukan! Mencari Karakteristik...\n");
        ble_gattc_disc_chrs_by_uuid(conn_handle, svc->start_handle, svc->end_handle, &chr_uuid.u, on_disc_chr, NULL);
    }
    return 0;
}

// 2. Fungsi Utama GAP (Koneksi & Terima Data)
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    struct ble_hs_adv_fields fields;

    switch (event->type) {
        case BLE_GAP_EVENT_DISC:
            ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
            
            // HANYA proses jika perangkat memancarkan Nama
            if (fields.name_len > 0) {
                if (strncmp((char*)fields.name, "SHM_Node_C3", fields.name_len) == 0) {
                    printf("\n[GATEWAY] Target 'SHM_Node_C3' Ditemukan! (MAC: %02x:%02x:%02x:%02x:%02x:%02x)\n",
                           event->disc.addr.val[5], event->disc.addr.val[4], event->disc.addr.val[3],
                           event->disc.addr.val[2], event->disc.addr.val[1], event->disc.addr.val[0]);
                    
                    printf("[GATEWAY] Menghentikan scan & memulai koneksi...\n");
                    ble_gap_disc_cancel();
                    ble_gap_connect(own_addr_type, &event->disc.addr, 30000, NULL, ble_gap_event_cb, NULL);
                    return 0;
                }
            }
            break;

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                printf("\n=== KONEKSI BERHASIL! ===\n");
                target_conn_handle = event->connect.conn_handle;
                // Mulai mencari Service di dalam Node
                ble_gattc_disc_svc_by_uuid(target_conn_handle, &svc_uuid.u, on_disc_svc, NULL);
            } else {
                printf("Koneksi Gagal, kembali scanning...\n");
                ble_app_scan();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            printf("Terputus dari Node! Kembali scanning...\n");
            target_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_app_scan();
            break;

        case BLE_GAP_EVENT_NOTIFY_RX:
            if (event->notify_rx.attr_handle == notify_char_val_handle) {
                SHMData shm;
                uint16_t data_len = OS_MBUF_PKTLEN(event->notify_rx.om);
                
                if (data_len == sizeof(SHMData)) {
                    os_mbuf_copydata(event->notify_rx.om, 0, sizeof(SHMData), &shm);
                    printf("[DATA MASUK] AX:%.2f AY:%.2f AZ:%.2f\n", shm.ax, shm.ay, shm.az);
                }
            }
            break;
    }
    return 0;
}

// 1. Fungsi Scanning
static void ble_app_scan(void) {
    struct ble_gap_disc_params disc_params;
    memset(&disc_params, 0, sizeof(disc_params));
    disc_params.filter_duplicates = 1;
    disc_params.passive = 0; // Harus Active Scan agar mendapat Nama Perangkat
    disc_params.itvl = 0;
    disc_params.window = 0;

    printf("Memulai Scanning mencari Node...\n");
    ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, ble_gap_event_cb, NULL);
}

// Sinkronisasi
static void ble_app_on_sync(void) {
    printf("NimBLE Host tersinkronisasi. Menyiapkan UUID...\n");
    ble_uuid_from_str(&svc_uuid, SERVICE_UUID_STR);
    ble_uuid_from_str(&chr_uuid, CHAR_UUID_STR);
    
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        printf("Error Address Type\n");
        return;
    }
    
    ble_app_scan();
}

// Task FreeRTOS
void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void) {
    // Tambahkan log awal agar kita tahu board menyala
    printf("\n\n--- GATEWAY SHM ESP32-S3 MENYALA ---\n");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    nimble_port_init();
    ble_svc_gap_device_name_set("SHM_Gateway_S3");
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(ble_host_task);
}