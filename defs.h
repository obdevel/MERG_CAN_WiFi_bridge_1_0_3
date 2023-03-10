//
/// ESP32 CAN WiFi Bridge
/// (c) Duncan Greenwood, 2019, 2020
//

/*

  Copyright (C) Duncan Greenwood, 2019

  This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                   and indicate if changes were made. You may do so in any reasonable manner,
                   but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                  your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                  legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

#pragma once
#include "driver/twai.h"

//
/// constants
//

#define VER_MAJ 1               // code version number
#define VER_MIN 0
#define VER_PATCH 3

#define EEPROM_SIZE 256
#define MAX_NET_PEERS 8
#define MAX_GC_CLIENTS 4
#define SERIAL_CLIENT MAX_GC_CLIENTS
#define MAX_WITHROTTLE_CLIENTS 4
#define MAX_DCCPPSER_CLIENTS 4
#define NUM_LEDS 6
#define HBFREQ 1000
#define WIFI_SCAN_MS 350
#define GC_INP_SIZE 32
#define PROXY_BUF_LEN 32
#define NUM_PROXY_CMDS 8
#define NUM_CBUS_NVS 16
#define CAN_QUEUE_DEPTH 128

#define ERR_IND_PIN GPIO_NUM_4            // error / low batt
#define NET_ACT_PIN GPIO_NUM_12           // network activity
#define ARDUINO_PIN GPIO_NUM_13           // standard Arduino LED
#define CAN_ACT_PIN GPIO_NUM_15           // CAN bus activity *** changed for board rev O ***
#define SLIM_LED_PIN GPIO_NUM_25          // SLiM LED
#define FLIM_LED_PIN GPIO_NUM_26          // FLiM LED

#define SCL_PIN GPIO_NUM_22
#define SDA_PIN GPIO_NUM_23

#define CAN_TX_PIN GPIO_NUM_16
#define CAN_RX_PIN GPIO_NUM_17

#define HW_TX_PIN GPIO_NUM_18
#define HW_RX_PIN GPIO_NUM_19

#define WAKEUP_PIN GPIO_NUM_33            // pin connected to wakeup switch, may be touch sensor
#define CBUS_SWITCH_PIN GPIO_NUM_33       // CBUS switch uses same input pin
#define BOOST_ENABLE_PIN GPIO_NUM_14      // 12V boost regulator enable (active high)
#define BAT_MON_PIN GPIO_NUM_35           // battery voltage sense ADC

#define I2C_DISPLAY_ADDR 0x30
#define I2C_GPIO_ADDR 0x20

#define DEBUG_FILE "/wbdebug.txt"
#define DEBUG_FILE_PREV "/wbdebug.prev.txt"
#define DEBUG_MSG_LEN 160

#define QUEUE_OP_TIMEOUT_NONE (TickType_t)0          // non-blocking
#define QUEUE_OP_TIMEOUT_SHORT (TickType_t)2         // max 2 ms block
#define QUEUE_OP_TIMEOUT (TickType_t)5               // max 5 ms block
#define QUEUE_OP_TIMEOUT_LONG (TickType_t)10         // max 10 ms block
#define QUEUE_OP_TIMEOUT_INFINITE (TickType_t)(-1)   // block forever

#define QUEUE_LOGGER_IN (1 << 0)
#define QUEUE_LED_IN (1 << 1)
#define QUEUE_CAN_OUT_FROM_NET (1 << 2)
#define QUEUE_CAN_OUT_FROM_GC (1 << 3)
#define QUEUE_CAN_OUT_FROM_WI (1 << 4)
#define QUEUE_NET_OUT (1 << 5)
#define QUEUE_NET_TO_NET (1 << 6)
#define QUEUE_GC_OUT (1 << 7)
#define QUEUE_GC_TO_GC (1 << 8)
#define QUEUE_WITHROTTLE_IN (1 << 9)
#define QUEUE_BATTERY_MONITOR_IN (1 << 10)
#define QUEUE_WEBSOCKETS_IN (1 << 11)
#define QUEUE_CMDPROXY_IN (1 << 12)
#define QUEUE_CBUS_EXTERNAL (1 << 13)
#define QUEUE_CBUS_INTERNAL (1 << 14)

//
/// forward function declarations for webserver page handlers
//

void start_webserver(void);
void handle_root(void);
void handle_config(void);
void handle_notfound(void);
void handle_store(void);
void handle_info(void);
void save_config(void);
void handle_stats(void);
void handle_stop(void);
void do_deepsleep(void);
void handle_restart(void);
void handle_default(void);
void handle_file_upload(void);
void handle_rqnn(void);
void handle_enum(void);
void handle_success(void);
void handle_delete_file(void);

//
/// utility function declarations
//

void LOG(const char s[]);
void VLOG(const char fmt[], ...);
void PULSE_LED(byte led);
bool CANtoGC(twai_message_t *frame, char buffer[]);
bool GCtoCAN(char buffer[], twai_message_t *frame);
char *format_CAN_frame(twai_message_t *frame);
void device_sleep(void);
void peer_record_op(const uint8_t *mac_addr, byte op, unsigned int val = 0);    // default val for arg 3
char *mac_to_char(const uint8_t mac_addr[6]);
bool send_message_to_queues(uint16_t queues, void *msg, const char *source_task, TickType_t time_to_wait);

//
/// enumerations
//

enum {
  TRANSPARENT_MODE = 0,     // all traffic is forwarded
  SPLIT_BUS = 1             // some traffic is filtered, not implemented yet
};

enum {
  CONFIG_USES_SW = 0,       // config set from web interface & stored in EEPROM
  CONFIG_USES_HW = 1        // config set by onboard switches
};

enum {
  ROLE_SLAVE = 0,
  ROLE_MASTER = 1
};

enum {
  ERR_IND_LED = 0,
  ARDUINO_LED = 1,
  CAN_ACT_LED = 2,
  NET_ACT_LED = 3,
  SLIM_LED = 4,
  FLIM_LED = 5
};

enum {
  LED_OFF = 0,
  LED_ON = 1,
  LED_BLINK = 2,
  LED_FAST_BLINK = 3,
  LED_PULSE = 4,
  LED_LONG_BLINK = 5,
  LED_SHORT_BLINK,
  LED_NONE
};

enum {
  DCC_UNK = 0,
  DCC_MERG = 1,
  DCC_DCCPP = 2
};

enum {
  DCC_DIR_REV = 0,
  DCC_DIR_FWD = 1
};

enum {
  W_FREE = 0,
  W_CONNECTED = 1,
  W_AWAITING_SESSION_ID = 2,
  W_ACTIVE = 3,
  W_CLOSING = 99
};

enum {
  BATT_RAW = 0,
  BATT_MV = 1,
  BATT_SOC = 2
};

enum {
  PEER_INCR_ERR = 0,
  PEER_DECR_ERR = 1,
  PEER_RESET_ERR = 2,
  PEER_INIT_ALL = 3,
  PEER_INCR_TX = 4,
  PEER_INCR_RX = 5,
  PEER_INCR_TX_ALL = 6,
  PEER_SET_BATT_MV = 7,
  PEER_SET_CANID = 8,
  PEER_SET_BATT_SOC = 9
};

enum {
  CBUS_MODE_SLIM = 0,
  CBUS_MODE_FLIM = 1,
  CBUS_MODE_CHANGING = 2,
  CBUS_MODE_NONE = 3
};

enum {
  MAIN_NODE = 0,
  CMD_NODE = 1
};

enum {
  WAKE_TOUCH = 0,
  WAKE_SWITCH = 1,
  WAKE_NEITHER = 2
};

//
/// type definitions
//

typedef struct  {
  byte role;
  byte network_number;
  byte config_mode;
  byte bridge_mode;
  byte slave_number;
  bool gc_server_on;
  unsigned int gc_server_port;
  bool debug;
  bool wifi_connect;
  char ssid[32];
  char pwd[32];
  bool withrottle_on;
  unsigned int withrottle_port;
  byte guard_val;
  byte dcc_type;
  bool slave_send_battery;
  byte peer_err_limit;
  bool use_network_password;
  char network_password[14];
  char softap_password[14];
  int sleep_delay_mins;
  byte default_wifi_channel;
  unsigned int low_battery_threshold;
  bool send_estop_on_sleep;
  bool forward_battery_msgs_to_cbus;
  bool ser_on;
  unsigned int ser_port;
  bool gc_serial_on;
  bool cmdproxy_on;
  byte CANID;
  bool cbus_mode;
  uint16_t node_number;
  byte node_variables[NUM_CBUS_NVS];
  byte wakeup_source;
  byte touch_threshold;
} config_t;

typedef struct {
  WiFiClient *client;
  char buffer[GC_INP_SIZE];
  char input[GC_INP_SIZE];
  byte idx;
  char addr[16];
  int port;
} gcclient_t;

typedef struct {
  char msg[GC_INP_SIZE];
  char addr[16];
  int port;
} wrapped_gc_t;

typedef struct {
  byte led;
  byte cmd;
  byte val;
} led_command_t;

typedef struct {
  byte pin;
  byte type;
  byte val;
  byte last_cmd;
  byte curr_state;
  byte next_state;
  unsigned long next_time;
} led_state_t;

typedef struct {
  uint8_t mac_addr[6];
  uint8_t CANID;
  int tx, rx;
  int num_errs;
  int battery_mv;
  int battery_soc;
} peer_state_t;

typedef struct {
  uint8_t mac_addr[6];
  twai_message_t frame;
} wrapped_frame_t;

typedef struct {
  unsigned long m;
  char s[DEBUG_MSG_LEN];
} log_message_t;

typedef struct {
  uint8_t mac_addr[6];
  int error_count;
} net_peer_t;

typedef struct {
  unsigned long can_rx, can_tx;
  unsigned long net_rx, net_tx;
  unsigned long gc_rx, gc_tx;
  unsigned long wi_rx, wi_tx;
  unsigned long dc_rx, dc_tx;
} stats_t;

typedef struct {
  int state;
  char desc[32];
} can_status_desc_t;

typedef struct {
  char buffer[NUM_PROXY_CMDS][PROXY_BUF_LEN];
  byte head, tail;
} message_buffer_t;

typedef struct {
  TaskFunction_t func;
  const char *name;
  unsigned int stack_size, prev_hwm, hwm;
  UBaseType_t priority;
  TaskHandle_t handle;
  bool do_create;
  BaseType_t core;
} task_info_t;

typedef struct {
  char name[32];
  QueueHandle_t handle;
  int num_items;
  size_t item_size;
} queue_t;
