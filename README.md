# Renogy-ble-mqtt-bridge
A bridge for renogy charge controllers BT-1 (possibly also BT-2) Bluetooth modules, to bridge the data to mqtt via WiFi 

# Prerequisites
Hardware: 
- ESP32 with dual cores
- Renogy Rover Charge controller with BT-1 (or possibly BT-2) bluetooth module connected

Libraries: 
- PubSubClient (for MQTT)
- ElegantOTA (for over the air/WiFi updates)

Software/server:
- Mosquitto or other MQTT broker (with authentication, unless you remove that from the code)

You'll need to configure your WiFi credentials and MQTT server address and credentials in the top of the sketch, and use a ble scanner app on your phone to find out the MAC address of the bluetooth module when in range, and add that to the top of the sketch too in the relevant fields.
