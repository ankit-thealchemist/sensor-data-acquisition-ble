# Sensor Data acquisition over Ble

This example shows how to aquire the sensor data over the ble and send it to the server using the wifi. The soc we used in this project is esp32

The example composed of the 2 devices, one is peripheral and other is central.

### Peripheral

Peripheral is the device with have the sensor on board, Data has been collected periodically and sended over the ble to the central device. This is our server, which exposes the temperature sensor. Also it is the constarined device, which is on the battery . To conserve the battery, we are using the deepsleep. Data is sended over the json format.

### Central

Central device is our client device. Central device wakes up at the definite interval and search for the server to get the data. It searched the device and connect to it. Accquire the data the parse it  according to the json format.

After acquisition of data, it create the wifi connection ot send the data over the mqtt. After that it goes again for sleep.
