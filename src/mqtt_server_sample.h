# after edit, rename mqtt_server_sample.h to mqtt_server.h

#ifndef _MQTT_SERVER_h
#define _MQTT_SERVER_h

// MQTT Broker
const char *mqtt_broker = ""; // broker address
const char *topic = ""; // define topic
const char *mqtt_username = ""; // username for authentication
const char *mqtt_password = ""; // password for authentication
const int mqtt_port = 8883; // port of MQTT over TLS/SSL

#endif