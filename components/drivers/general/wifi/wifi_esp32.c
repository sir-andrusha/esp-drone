#include <string.h>
#include <stdlib.h>

#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "queuemonitor.h"
#include "wifi_esp32.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "WIFI_UDP"
#include "debug_cf.h"

#include "espnow.h"
#include "espnow_ctrl.h"
#include "espnow_utils.h"

#include <inttypes.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_bridge.h"
#include "esp_mesh_lite.h"

#define UDP_SERVER_PORT 2390
#define UDP_SERVER_BUFSIZE 64

static struct sockaddr_storage source_addr;

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

static int sock;
static xQueueHandle udpDataRx;
static xQueueHandle udpDataTx;

static bool isInit = false;
static bool isUDPInit = false;
static bool isUDPConnected = false;

static esp_err_t udp_server_create(void *arg);

static uint8_t calculate_cksum(void *data, size_t len)
{
    unsigned char *c = data;
    int i;
    unsigned char cksum = 0;

    for (i = 0; i < len; i++)
    {
        cksum += *(c++);
    }

    return cksum;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        DEBUG_PRINT_LOCAL("station" MACSTR "join, AID=%d", MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        DEBUG_PRINT_LOCAL("station" MACSTR "leave, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

bool wifiTest(void)
{
    return isInit;
};

bool wifiGetDataBlocking(UDPPacket *in)
{
    /* command step - receive  02  from udp rx queue */
    while (xQueueReceive(udpDataRx, in, portMAX_DELAY) != pdTRUE)
    {
        vTaskDelay(M2T(10));
    }; // Don't return until we get some data on the UDP

    return true;
};

bool wifiSendData(uint32_t size, uint8_t *data)
{
    UDPPacket outStage = {0};
    outStage.size = size;
    memcpy(outStage.data, data, size);
    // Dont' block when sending
    return (xQueueSend(udpDataTx, &outStage, M2T(100)) == pdTRUE);
};

static void udp_server_rx_task(void *pvParameters)
{
    socklen_t socklen = sizeof(source_addr);
    char rx_buffer[UDP_SERVER_BUFSIZE];
    UDPPacket inPacket = {0};

    while (true)
    {
        if (isUDPInit == false)
        {
            vTaskDelay(20);
            continue;
        }
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);
        /* command step - receive  01 from Wi-Fi UDP */
        if (len < 0)
        {
            DEBUG_PRINT_LOCAL("recvfrom failed: errno %d", errno);
            continue;
        }
        else if (len > WIFI_RX_TX_PACKET_SIZE)
        {
            DEBUG_PRINT_LOCAL("Received data length = %d > %d", len, WIFI_RX_TX_PACKET_SIZE);
            continue;
        }
        else
        {
            uint8_t cksum = rx_buffer[len - 1];
            // remove cksum, do not belong to CRTP
            // check packet
            if (cksum == calculate_cksum(rx_buffer, len - 1))
            {
                // copy part of the UDP packet, the size not include cksum
                inPacket.size = len - 1;
                memcpy(inPacket.data, rx_buffer, inPacket.size);
                xQueueSend(udpDataRx, &inPacket, M2T(10));
                if (!isUDPConnected)
                    isUDPConnected = true;
            }
            else
            {
                DEBUG_PRINT_LOCAL("udp packet cksum unmatched");
            }

#ifdef DEBUG_UDP
            printf("\nReceived size = %d cksum = %02X\n", inPacket.size, cksum);
            for (size_t i = 0; i < inPacket.size; i++)
            {
                printf("%02X ", inPacket.data[i]);
            }
            printf("\n");
#endif
        }
    }
}

static struct sockaddr_in dest_addr = {0};

static void udp_server_tx_task(void *pvParameters)
{
    UDPPacket outPacket = {0};
    while (TRUE)
    {
        if (isUDPInit == false)
        {
            vTaskDelay(20);
            continue;
        }
        if ((xQueueReceive(udpDataTx, &outPacket, portMAX_DELAY) == pdTRUE) && isUDPConnected)
        {
            // append cksum to the packet
            outPacket.data[outPacket.size] = calculate_cksum(outPacket.data, outPacket.size);
            outPacket.size += 1;

            int err = sendto(sock, outPacket.data, outPacket.size, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0)
            {
                DEBUG_PRINT_LOCAL("Error occurred during sending: errno %d", errno);
                continue;
            }
#ifdef DEBUG_UDP
            printf("\nSend size = %d checksum = %02X\n", outPacket.size, outPacket.data[outPacket.size - 1]);
            for (size_t i = 0; i < outPacket.size; i++)
            {
                printf("%02X ", outPacket.data[i]);
            }
            printf("\n");
#endif
        }
    }
}

static void espnow_ctrl_data_cb(espnow_attribute_t initiator_attribute,
                                espnow_attribute_t responder_attribute,
                                uint32_t status1,
                                int status2,
                                int lx_value,
                                int ly_value,
                                int rx_value,
                                int ry_value,
                                int channel_one_value,
                                int channel_two_value)
{
    UDPPacket inPacket;
    inPacket.size = 7;
    inPacket.data[0] = 'n';
    inPacket.data[1] = 'o';
    inPacket.data[2] = 'w';
    inPacket.data[3] = lx_value & 0xFF;
    inPacket.data[4] = ly_value & 0xFF;
    inPacket.data[5] = ry_value & 0xFF;
    inPacket.data[6] = rx_value & 0xFF;
    xQueueSend(udpDataRx, &inPacket, 0);
}

static void app_espnow_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    if (base != ESP_EVENT_ESPNOW)
    {
        return;
    }

    switch (id)
    {
    case ESP_EVENT_ESPNOW_CTRL_BIND:
    {
        espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
        DEBUG_PRINT_LOCAL("bind, uuid: " MACSTR ", initiator_type: %d", MAC2STR(info->mac), info->initiator_attribute);
        break;
    }

    case ESP_EVENT_ESPNOW_CTRL_UNBIND:
    {
        espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
        DEBUG_PRINT_LOCAL("unbind, uuid: " MACSTR ", initiator_type: %d", MAC2STR(info->mac), info->initiator_attribute);
        break;
    }

    default:
        break;
    }
}

static char softap_ssid[32];

void app_wifi_set_softap_info(void)
{
    
    char softap_psw[64];
    uint8_t softap_mac[6];
    size_t size = sizeof(softap_psw);
    esp_wifi_get_mac(WIFI_IF_AP, softap_mac);
    memset(softap_ssid, 0x0, sizeof(softap_ssid));

#ifdef CONFIG_BRIDGE_SOFTAP_SSID_END_WITH_THE_MAC
    snprintf(softap_ssid, sizeof(softap_ssid), "%.25s_%02x%02x%02x", CONFIG_BRIDGE_SOFTAP_SSID, softap_mac[3], softap_mac[4], softap_mac[5]);
#else
    snprintf(softap_ssid, sizeof(softap_ssid), "%.32s", CONFIG_BRIDGE_SOFTAP_SSID);
#endif
    if (esp_mesh_lite_get_softap_ssid_from_nvs(softap_ssid, &size) != ESP_OK)
    {
        esp_mesh_lite_set_softap_ssid_to_nvs(softap_ssid);
    }
    if (esp_mesh_lite_get_softap_psw_from_nvs(softap_psw, &size) != ESP_OK)
    {
        esp_mesh_lite_set_softap_psw_to_nvs(CONFIG_BRIDGE_SOFTAP_PASSWORD);
    }
    esp_mesh_lite_set_softap_info(softap_ssid, CONFIG_BRIDGE_SOFTAP_PASSWORD);
}

static const char *TAG_APP = "app";

static void print_system_info_timercb(TimerHandle_t timer)
{
    uint8_t primary = 0;
    uint8_t sta_mac[6] = {0};
    wifi_ap_record_t ap_info = {0};
    wifi_second_chan_t second = 0;
    wifi_sta_list_t wifi_sta_list = {0x0};

    esp_wifi_sta_get_ap_info(&ap_info);
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);

    ESP_LOGI(TAG_APP, "System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR ", parent rssi: %d, free heap: %" PRIu32 "", primary,
             esp_mesh_lite_get_level(), MAC2STR(sta_mac), MAC2STR(ap_info.bssid),
             (ap_info.rssi != 0 ? ap_info.rssi : -120), esp_get_free_heap_size());
#if CONFIG_MESH_LITE_MAXIMUM_NODE_NUMBER
    ESP_LOGI(TAG_APP, "child node number: %d", esp_mesh_lite_get_child_node_number());
#endif /* MESH_LITE_NODE_INFO_REPORT */
    for (int i = 0; i < wifi_sta_list.num; i++)
    {
        ESP_LOGI(TAG_APP, "Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
    }
}

static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = CONFIG_SERVER_IP;
    int addr_family = 0;
    int ip_protocol = 0;

    struct in_addr ip_struct;
    esp_netif_ip_info_t sta_ip;
    memset(&sta_ip, 0x0, sizeof(esp_netif_ip_info_t));
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &sta_ip);
    ip_struct.s_addr = sta_ip.ip.addr;
    char *ipStr = inet_ntoa(ip_struct);
    ESP_LOGI(TAG_APP, "my IP: %s", ipStr);
    char *ping_sencence = NULL;
    asprintf(&ping_sencence, "%s:%s:%s", softap_ssid, ipStr, "ping");

    struct sockaddr_in health_dest_addr = {0};
    health_dest_addr.sin_addr.s_addr = inet_addr(CONFIG_SERVER_IP);
    health_dest_addr.sin_family = AF_INET;
    health_dest_addr.sin_port = htons(CONFIG_SERVER_PORT);
    
    while (1) {
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG_APP, "Unable to create socket: errno %d", errno);
            break;
        }

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        ESP_LOGI(TAG_APP, "Socket created, starting pinging server %s:%d", CONFIG_SERVER_IP, CONFIG_SERVER_PORT);

        while (1) {
            int err = sendto(sock, ping_sencence, strlen(ping_sencence), 0, (struct sockaddr *)&health_dest_addr, sizeof(health_dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG_APP, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG_APP, "ping sent");

            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG_APP, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG_APP, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG_APP, "%s", rx_buffer);
                // if (strncmp(rx_buffer, "pong", 4) != 0) {
                //     ESP_LOGI(TAG_APP, "Received unexpected message, reconnecting");
                //     break;
                // }
                uint32_t new_port = atoi(rx_buffer);
                dest_addr.sin_addr.s_addr = inet_addr(CONFIG_SERVER_IP);
                dest_addr.sin_family = AF_INET;
                dest_addr.sin_port = htons(new_port);

                int err = sendto(sock, ping_sencence, strlen(ping_sencence), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                if (err < 0) {
                    ESP_LOGE(TAG_APP, "Error occurred during sending: errno %d", errno);
                    break;
                }
                isUDPInit = true;
            }

            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            isUDPInit = false;
            ESP_LOGE(TAG_APP, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

static void ip_event_sta_got_ip_handler(void *arg, esp_event_base_t event_base,
                                        int32_t event_id, void *event_data)
{
    static bool tcp_task = false;

    if (!tcp_task)
    {
        xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
        tcp_task = true;
    }
}

void wifiInit(void)
{
    if (isInit)
    {
        return;
    }
    // This should probably be reduced to a CRTP packet size
    udpDataRx = xQueueCreate(16, sizeof(UDPPacket));
    DEBUG_QUEUE_MONITOR_REGISTER(udpDataRx);
    udpDataTx = xQueueCreate(16, sizeof(UDPPacket));
    DEBUG_QUEUE_MONITOR_REGISTER(udpDataTx);

    {
        espnow_storage_init();
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
    }

    {
        esp_bridge_create_all_netif();

        // Station
        wifi_config_t wifi_config = {
            .sta = {
                .ssid = CONFIG_ROUTER_SSID,
                .password = CONFIG_ROUTER_PASSWORD,
            },
        };
        esp_bridge_wifi_set_config(WIFI_IF_STA, &wifi_config);

        // Softap
        snprintf((char *)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid), "%s", CONFIG_BRIDGE_SOFTAP_SSID);
        strlcpy((char *)wifi_config.ap.password, CONFIG_BRIDGE_SOFTAP_PASSWORD, sizeof(wifi_config.ap.password));
        esp_bridge_wifi_set_config(WIFI_IF_AP, &wifi_config);

        esp_mesh_lite_config_t mesh_lite_config = ESP_MESH_LITE_DEFAULT_INIT();
        esp_mesh_lite_init(&mesh_lite_config);
        app_wifi_set_softap_info();
        esp_mesh_lite_start();
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_sta_got_ip_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    // uint8_t mac[6];
    // ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac));

    espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();
    espnow_init(&espnow_config);
    esp_event_handler_register(ESP_EVENT_ESPNOW, ESP_EVENT_ANY_ID, app_espnow_event_handler, NULL);
    ESP_ERROR_CHECK(espnow_ctrl_responder_bind(30 * 1000, -55, NULL));
    espnow_ctrl_responder_data(espnow_ctrl_data_cb);

    ESP_LOGI(TAG_APP, "wifi_init_softap complete.SSID:%s password:%s", CONFIG_BRIDGE_SOFTAP_SSID, CONFIG_BRIDGE_SOFTAP_PASSWORD);

    xTaskCreate(udp_server_tx_task, UDP_TX_TASK_NAME, UDP_TX_TASK_STACKSIZE, NULL, UDP_TX_TASK_PRI, NULL);
    xTaskCreate(udp_server_rx_task, UDP_RX_TASK_NAME, UDP_RX_TASK_STACKSIZE, NULL, UDP_RX_TASK_PRI, NULL);

    TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_PERIOD_MS,
                                       true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);

    isInit = true;
}