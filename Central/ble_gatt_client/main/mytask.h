/**
 * @file mytask.h
 * @author Ankit Bansal (iotdevelope@gmail.com)
 * @brief This is the header file for the task 
 * @version 1.1
 * @date 2021-10-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#define PRINT_TASK_SIZE 4096

#define CONNECT_MQTT_TASK_SIZE 4096

#define DEEP_SLEEP_TASK_SIZE 4096

/**
 * @brief function for the task to print out the json value. It will fire only one time and 
 * 
 * @param pvParameters 
 */
void print_task(void *pvParameters);

/**
 * @brief This will create the task for the mqtt server to be used 
 * 
 * @param pvParameters 
 */
void connect_to_mqtt(void* pvParameters);


/**
 * @brief this task function will be called to deep sleep the module
 * 
 * @param pvParameters 
 */
void deep_sleep_task(void* pvParameters);


/**
 * @brief Get the Device Id From Received Json object
 * 
 * @param value received json object
 * @return const char* device Id
 */
const char* getDeviceIdFromReceivedJson(const char* value);