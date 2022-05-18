#ifndef __DEFINES_H__
#define __DEFINES_H__

#include "config.h"

//-------------------------------------
// PINOUT (Default, can be changed in setup)
//-------------------------------------
#define RF24_CS_PIN         15
#define RF24_CE_PIN         2
#define RF24_IRQ_PIN        0


//-------------------------------------
// VERSION
//-------------------------------------
#define VERSION_MAJOR       0
#define VERSION_MINOR       4
#define VERSION_PATCH       3


//-------------------------------------
typedef struct {
    uint8_t rxCh;
    uint8_t packet[MAX_RF_PAYLOAD_SIZE];
} packet_t;


//-------------------------------------
// EEPROM
//-------------------------------------
#define SSID_LEN            32
#define PWD_LEN             63
#define DEVNAME_LEN         16
#define CRC_LEN             2 // uint16_t

#define INV_ADDR_LEN        MAX_NUM_INVERTERS * 8                // uint64_t
#define INV_NAME_LEN        MAX_NUM_INVERTERS * MAX_NAME_LENGTH  // char[]
#define INV_TYPE_LEN        MAX_NUM_INVERTERS * 1                // uint8_t
#define INV_INTERVAL_LEN    2                                    // uint16_t

#define PINOUT_LEN          3 // 3 pins: CS, CE, IRQ

#define RF24_AMP_PWR_LEN    1

#define MQTT_ADDR_LEN       4 // IP
#define MQTT_USER_LEN       16
#define MQTT_PWD_LEN        32
#define MQTT_TOPIC_LEN      32
#define MQTT_INTERVAL_LEN   2 // uint16_t
#define MQTT_PORT_LEN       2 // uint16_t

#define SER_ENABLE_LEN      1 // uint8_t
#define SER_DEBUG_LEN       1 // uint8_t
#define SER_INTERVAL_LEN    2 // uint16_t


#define ADDR_START          0
#define ADDR_SSID           ADDR_START
#define ADDR_PWD            ADDR_SSID          + SSID_LEN
#define ADDR_DEVNAME        ADDR_PWD           + PWD_LEN
#define ADDR_WIFI_CRC       ADDR_DEVNAME       + DEVNAME_LEN
#define ADDR_START_SETTINGS ADDR_WIFI_CRC      + CRC_LEN

#define ADDR_PINOUT         ADDR_START_SETTINGS

#define ADDR_RF24_AMP_PWR   ADDR_PINOUT        + PINOUT_LEN

#define ADDR_INV_ADDR       ADDR_RF24_AMP_PWR  + RF24_AMP_PWR_LEN
#define ADDR_INV_NAME       ADDR_INV_ADDR      + INV_ADDR_LEN
#define ADDR_INV_TYPE       ADDR_INV_NAME      + INV_NAME_LEN // obsolete
#define ADDR_INV_INTERVAL   ADDR_INV_TYPE      + INV_TYPE_LEN

#define ADDR_MQTT_ADDR      ADDR_INV_INTERVAL  + INV_INTERVAL_LEN
#define ADDR_MQTT_USER      ADDR_MQTT_ADDR     + MQTT_ADDR_LEN
#define ADDR_MQTT_PWD       ADDR_MQTT_USER     + MQTT_USER_LEN
#define ADDR_MQTT_TOPIC     ADDR_MQTT_PWD      + MQTT_PWD_LEN
#define ADDR_MQTT_INTERVAL  ADDR_MQTT_TOPIC    + MQTT_TOPIC_LEN
#define ADDR_MQTT_PORT      ADDR_MQTT_INTERVAL + MQTT_INTERVAL_LEN

#define ADDR_SER_ENABLE     ADDR_MQTT_PORT     + MQTT_PORT_LEN
#define ADDR_SER_DEBUG      ADDR_SER_ENABLE    + SER_ENABLE_LEN
#define ADDR_SER_INTERVAL   ADDR_SER_DEBUG     + SER_DEBUG_LEN
#define ADDR_NEXT           ADDR_SER_INTERVAL  + SER_INTERVAL_LEN

#define ADDR_SETTINGS_CRC   400

#if(ADDR_SETTINGS_CRC <= ADDR_NEXT)
#error address overlap!
#endif

#endif /*__DEFINES_H__*/
