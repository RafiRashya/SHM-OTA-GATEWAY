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

#define MAX_NODES 3 

char current_ota_url[1024] = "";
char target_ota_version[20] = "";
char verifying_mac[18] = ""; 

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
    uint16_t ota_ver_handle; // Ditambahkan untuk menyimpan handle versi
    bool is_active;
    bool ota_in_progress;
} ConnectedNode;

ConnectedNode nodes[MAX_NODES];

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
}

// ==================== CALLBACK VERIFIKASI VERSI ====================
static int read_version_cb(uint16_t conn_handle, const struct ble_gatt_error *error, 
                           struct ble_gatt_attr *attr, void *arg) {
    if (error->status == 0) {
        char node_ver[20] = {0};
        int len = OS_MBUF_PKTLEN(attr->om);
        
        if (len > 19) len = 19; 
        os_mbuf_copydata(attr->om, 0, len, node_ver);
        node_ver[len] = '\0'; 

        int idx = find_node_by_handle(conn_handle);
        if (idx == -1) return 0; 

        if (strlen(verifying_mac) > 0 && strcmp(nodes[idx].mac_address, verifying_mac) == 0) {
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
            memset(verifying_mac, 0, sizeof(verifying_mac)); 
        }
    }
    return 0;
}

// ==================== TASK VERIFIKASI (Mencegah Tabrakan GATT) ====================
void verify_version_task(void *pvParameter) {
    int idx = (int)pvParameter;
    // Tunggu 2 detik agar proses Subscribe CCCD dari on_disc_dsc selesai sepenuhnya
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    ble_gattc_read(nodes[idx].conn_handle, nodes[idx].ota_ver_handle, read_version_cb, NULL);
    vTaskDelete(NULL);
}

// ==================== MQTT HANDLER ====================
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            printf("\n[MQTT] Terhubung secara AMAN (TLS) ke EMQX!\n");
            mqtt_connected = true;

            // === KIRIM STATUS ONLINE SAAT BERHASIL CONNECT ===
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
                                    void ota_download_and_send_task(void *pvParameters);
                                    xTaskCreate(ota_download_and_send_task, "ota_task", 8192, arg_idx, 5, NULL);
                                } else {
                                    printf("[MQTT] Node %s sedang sibuk OTA!\n", target_mac->valuestring);
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

// ==================== TASK OTA STREAMING (48 DETIK) ====================
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
        .timeout_ms = 10000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_open(client, 0);
    
    if (err == ESP_OK) {
        esp_http_client_fetch_headers(client);
        int total_len = esp_http_client_get_content_length(client);
        printf("[GATEWAY OTA] Ukuran Firmware: %d bytes. Mulai streaming...\n", total_len);

        uint8_t buffer[500]; 
        int data_read = 0;
        int total_sent = 0;
        bool is_failed = false;

        int bytes_since_last_pause = 0;

        while ((data_read = esp_http_client_read(client, (char *)buffer, sizeof(buffer))) > 0) {
            
            int rc;
            do {
                rc = ble_gattc_write_no_rsp_flat(nodes[idx].conn_handle, nodes[idx].ota_data_handle, buffer, data_read);
                if (rc == 6 || rc == 130) { 
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            } while (rc == 6 || rc == 130);

            if (rc != 0) {
                printf("[GATEWAY OTA] Error fatal kirim BLE: %d\n", rc);
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
        
        if (is_failed || total_sent < total_len) {
            printf("[GATEWAY OTA] GAGAL! File terputus/Error BLE.\n");
            publish_ota_status(nodes[idx].mac_address, "FAILED");
        } else {
            printf("[GATEWAY OTA] Streaming selesai! Mengirim perintah END...\n");
            vTaskDelay(pdMS_TO_TICKS(500)); 

            uint8_t cmd_end = 0x02;
            int rc_end;
            do {
                rc_end = ble_gattc_write_flat(nodes[idx].conn_handle, nodes[idx].ota_ctrl_handle, &cmd_end, 1, NULL, NULL);
                if (rc_end == 6 || rc_end == 130) vTaskDelay(pdMS_TO_TICKS(50));
            } while (rc_end == 6 || rc_end == 130);

            strncpy(verifying_mac, nodes[idx].mac_address, sizeof(verifying_mac) - 1);
            verifying_mac[sizeof(verifying_mac) - 1] = '\0';
            publish_ota_status(nodes[idx].mac_address, "REBOOTING");
        }

    } else {
        printf("[GATEWAY OTA] Gagal terhubung ke URL GCS!\n");
        publish_ota_status(nodes[idx].mac_address, "FAILED");
    }

    esp_http_client_cleanup(client);
    nodes[idx].ota_in_progress = false;

    printf("[GATEWAY OTA] OTA Selesai. Menyalakan kembali BLE Scanner...\n");
    ble_app_scan();

    vTaskDelete(NULL);
}

// ==================== CALLBACK BLE GATT ====================
static int on_disc_dsc(uint16_t conn_handle, const struct ble_gatt_error *error,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg) {
    if (error->status == 0) {
        if (ble_uuid_u16(&dsc->uuid.u) == 0x2902) {
            // KEMBALIKAN LOG PRINTF SUBSCRIBE!
            printf(">> Mengaktifkan Notifikasi SHM (Subscribe)...\n"); 
            uint8_t value[2] = {0x01, 0x00};
            ble_gattc_write_flat(conn_handle, dsc->handle, value, sizeof(value), NULL, NULL);
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
            // SIMPAN HANDLE SAJA DI SINI. JANGAN LANGSUNG DIBACA!
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

                    ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);
                    ble_gattc_disc_all_svcs(event->connect.conn_handle, on_disc_svc, NULL);
                    
                    // TRIGGER TASK PEMBACAAN VERSI JIKA SEDANG VALIDASI OTA
                    if (strlen(verifying_mac) > 0 && strcmp(nodes[empty_idx].mac_address, verifying_mac) == 0) {
                        xTaskCreate(verify_version_task, "verify_task", 2048, (void*)empty_idx, 5, NULL);
                    }
                } else {
                    printf(">> Kapasitas Gateway Penuh! Memutus koneksi...\n");
                    ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                }

                ble_app_scan();
            } else {
                ble_app_scan();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            printf(">> Terputus dari Node!\n");
            int idx = find_node_by_handle(event->disconnect.conn.conn_handle);
            if (idx != -1) {
                memset(&nodes[idx], 0, sizeof(ConnectedNode)); 
            }
            ble_app_scan();
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
                        
                        // === KEMBALIKAN FORMAT JSON DENGAN MAC ADDRESS ===
                        snprintf(json_payload, sizeof(json_payload), 
                                 "{\"node_mac\":\"%s\", \"ax\":%.2f, \"ay\":%.2f, \"az\":%.2f, \"vbatt\":%.2f}", 
                                 nodes[n_idx].mac_address, shm.ax, shm.ay, shm.az, shm.vbatt);
                                 
                        // Publish ke topik MQTT
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
    struct ble_gap_disc_params disc_params;
    memset(&disc_params, 0, sizeof(disc_params));
    disc_params.filter_duplicates = 1; disc_params.passive = 0;
    ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, ble_gap_event_cb, NULL);
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
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
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
    
    // PERTAHANKAN MTU 512 AGAR TRANSFER 500 BYTE MAKSIMAL
    ble_att_set_preferred_mtu(512); 
    
    ble_svc_gap_device_name_set("SHM_Gateway_S3");
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(ble_host_task);
}