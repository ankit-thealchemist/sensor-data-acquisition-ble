/**
 * @file wifi_sta.c
 * @author Ankit Bansal (iotdevelope@gmail.com)
 * @brief contains the wifi configuration and functions. 
 * @version 0.1
 * @date 2021-10-11
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <esp_log.h>

#include "lwip/err.h"
#include "lwip/sys.h"
#include "mytask.h"

#define EXAMPLE_ESP_WIFI_SSID CONFIG_EXAMPLE_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_EXAMPLE_ESP_WIFI_PASS
#define EXAMPLE_ESP_WIFI_CHANNEL 0
#define EXAMPLE_MAX_STA_CONN 5

static const char *TAG = "WiFi";

esp_netif_t *myStationIP;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,int32_t event_id, void *event_data);
static void IP_event_handler(void *arg, esp_event_base_t event_base,int32_t event_id, void *event_data);

void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());                // create the lwip task
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // create the event loop task

    // saving the handle of the network interface so that we can retrive the ip address later
    // myAccessPoint = esp_netif_create_default_wifi_ap();

    myStationIP = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)); // create the wifi task

    char *desc;
    
    

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,          // wifi event to be filtered out
                                                        ESP_EVENT_ANY_ID,    // any event id with the event base of WIFI_EVENT
                                                        &wifi_event_handler, //handler function for the wifi events
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,          // wifi event to be filtered out
                                                        ESP_EVENT_ANY_ID,    // any event id with the event base of WIFI_EVENT
                                                        &IP_event_handler, //handler function for the wifi events
                                                        NULL,
                                                        NULL));
    

    wifi_config_t wifi_config = {
      
        
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
             .scan_method = WIFI_ALL_CHANNEL_SCAN,
            // .bssid_set = 0,
            // .channel = 0,
            // .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        }
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}


void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_WIFI_READY:
        ESP_LOGI(TAG, "WIFI_EVENT_WIFI_READY");
        break;

    case WIFI_EVENT_SCAN_DONE:
    {
        ESP_LOGI(TAG, "WIFI_EVENT_SCAN_DONE");
        uint16_t number = 5;
        wifi_ap_record_t *ap_records;
        ap_records = (wifi_ap_record_t *)malloc(number * sizeof(ap_records));
        memset(ap_records, 0, number * sizeof(ap_records));

        uint16_t ap_count = 0;
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_records));
        ESP_LOGI(TAG, "No of ap count is %d", ap_count);
        for (uint16_t i = 0; (i < number) && (i < ap_count); i++)
        {
            ESP_LOGI(TAG, "SsIDs: %s\n", ap_records[i].ssid);
            ESP_LOGI(TAG, "primary channel: %d\n", ap_records[i].primary);
            ESP_LOGI(TAG, "rssi: %d\n", ap_records[i].rssi);
            ESP_LOGI(TAG, "MAC ADDRESS:%02x:%02x:%02x:%02x:%02x:%02x\n",
                     (unsigned char)ap_records[i].bssid[0],
                     (unsigned char)ap_records[i].bssid[1],
                     (unsigned char)ap_records[i].bssid[2],
                     (unsigned char)ap_records[i].bssid[3],
                     (unsigned char)ap_records[i].bssid[4],
                     (unsigned char)ap_records[i].bssid[5]);
            ESP_LOGI(TAG, "======================================================================================================");
        }
        free(ap_records);
        break;
    }

    case WIFI_EVENT_STA_START:
    {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_START,");
        ESP_ERROR_CHECK(esp_wifi_connect());
        
        break;
    }

    case WIFI_EVENT_STA_STOP:
    {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_STOP");
        break;
    }

    case WIFI_EVENT_STA_CONNECTED:
    {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_CONNECTED");

        
    }
    break;

    case WIFI_EVENT_STA_DISCONNECTED:
    {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED");

        ESP_ERROR_CHECK(esp_wifi_connect());  // if some how wifi disconnect it will reconnect again
    }
    break;

    case WIFI_EVENT_STA_AUTHMODE_CHANGE:
    {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_AUTHMODE_CHANGE");
    }
    break;

    case WIFI_EVENT_STA_WPS_ER_SUCCESS:
    {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_WPS_ER_SUCCESS");
    }
    break;

    case WIFI_EVENT_STA_WPS_ER_FAILED:
    {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_WPS_ER_FAILED");
    }
    break;

    case WIFI_EVENT_STA_WPS_ER_TIMEOUT:
    {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_WPS_ER_TIMEOUT");
    }
    break;

    case WIFI_EVENT_STA_WPS_ER_PIN:
    {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_WPS_ER_PIN");
    }
    break;

    case WIFI_EVENT_STA_WPS_ER_PBC_OVERLAP:
    {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_WPS_ER_PBC_OVERLAP");
    }
    break;


    default:
        ESP_LOGI(TAG, "Value of the event is %d", event_id);
        break;
    }
}

void IP_event_handler(void *arg, esp_event_base_t event_base,int32_t event_id, void *event_data){

    switch (event_id)
    {
    case IP_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG,"IP_EVENT_STA_GOT_IP");
        esp_netif_ip_info_t ip_info;
        esp_netif_get_ip_info(myStationIP, &ip_info);
        printf("My IP: " IPSTR "\n", IP2STR(&ip_info.ip));
        printf("My GW: " IPSTR "\n", IP2STR(&ip_info.gw));
        printf("My NETMASK: " IPSTR "\n", IP2STR(&ip_info.netmask));
        // after getting the ip address we can safely start the mqtt task
         xTaskCreate(connect_to_mqtt,"connect to mqtt",CONNECT_MQTT_TASK_SIZE,NULL,5,NULL);
    break;

    case IP_EVENT_AP_STAIPASSIGNED:
        ESP_LOGI(TAG,"IP_EVENT_AP_STAIPASSIGNED");
    break;         
    case IP_EVENT_STA_LOST_IP:
        ESP_LOGI(TAG,"IP_EVENT_STA_LOST_IP");

    break;         
    case IP_EVENT_GOT_IP6:
        ESP_LOGI(TAG,"IP_EVENT_GOT_IP6");

    break;         
    case IP_EVENT_ETH_GOT_IP: 
        ESP_LOGI(TAG,"IP_EVENT_ETH_GOT_IP");

    break;         
    case IP_EVENT_PPP_GOT_IP:
        ESP_LOGI(TAG,"IP_EVENT_PPP_GOT_IP");

    break;        
    case IP_EVENT_PPP_LOST_IP:
        ESP_LOGI(TAG,"IP_EVENT_PPP_LOST_IP");

    break;
    
    default:
        break;
    }
}