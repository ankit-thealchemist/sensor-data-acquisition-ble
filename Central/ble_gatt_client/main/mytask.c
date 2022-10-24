/**
 * @file mytask.c       
 * @author Ankit Bansal (iotdevelope@gmail.com)
 * @brief This files contain all the task related funtion related to the task created by the user. All the system related task are in their relative files.
 * also contains the custom function also.
 * @version 1.1
 * @date 2021-10-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "wifi_sta.h"
#include  "projMqtt.h"
#include <esp_log.h>
#include <esp_sleep.h>
#include <cJSON.h>
#include <string.h>

#define TAG "task"
#define DEEP_SLEEP_TIME_IN_SECONDS  CONFIG_DEEP_SLEEP_TIME_IN_SECONDS

extern char* receivedValue;

/**
 * @brief function for the task to print out the json value. It will fire only one time and 
 * 
 * @param pvParameters 
 */
void print_task(void *pvParameters){

    printf("\nReceived value is %s \n",receivedValue);
    // now as per the requirement value has been printed. Now I need to initiate the  wifi. 
    wifi_init_sta();
    vTaskDelete(NULL);
}

void connect_to_mqtt(void* pvParameters) 
{   
    ESP_LOGI(TAG,"stating mqtt");
    vTaskDelay(pdMS_TO_TICKS(1000));
    mqtt_app_start();
    vTaskDelete(NULL);
}

void deep_sleep_task(void* pvParameters) 
{
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG,"too tired.. going to sleep\n");
    esp_deep_sleep(DEEP_SLEEP_TIME_IN_SECONDS * 1000000);
    vTaskDelete(NULL);
}

const char* getDeviceIdFromReceivedJson(const char* value) 
{
    cJSON *value_parse = cJSON_Parse(value);
    cJSON *deviceId = NULL;
    if(value_parse==NULL){
    const char *error_ptr = cJSON_GetErrorPtr();
    ESP_LOGE(TAG,"Error occured while parsing the object %s\n",error_ptr);

    return NULL;
    }
    
    deviceId =  cJSON_GetObjectItemCaseSensitive(value_parse, "deviceId");
    if (cJSON_IsString(deviceId) && (deviceId->valuestring != NULL))
    {
        ESP_LOGI(TAG,"Checking deviceId \"%s\"\n", deviceId->valuestring);
        return deviceId->valuestring;
    }

    return NULL;
}


