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

#include <WiFi.h>
#include <SPIFFS.h>
#include <HardwareSerial.h>
#include "defs.h"

extern QueueHandle_t logger_in_queue;
extern config_t config_data;

HardwareSerial AltSerial(1);      // UART2

//
/// task to log error and debug messages to a file and a serial port
//

void logger_task(void *params) {

  log_message_t lm;
  char tmp[16];
  File file;
  bool file_is_open = false;
  bool output_port_changed = false, debug_changed = false;

  // initialise main serial/USB port
  Serial.begin(115200);
  Serial.setTimeout(1);

  LOG("logger_task: logger task starting");
  VLOG("logger_task: debug = %d", config_data.debug);

  // infinite loop
  for (;;) {

    // configure debug output file if required at any point
    if (config_data.debug && !debug_changed) {
      debug_changed = true;

      // mount SPIFFS filesystem
      if (!SPIFFS.begin(true)) {
        LOG("logger_task: SPIFFS mount failed");
      } else {
        LOG("logger_task: SPIFFS mounted");
      }

      // copy debug file to backup
      if (SPIFFS.exists(DEBUG_FILE)) {
        SPIFFS.rename(DEBUG_FILE, DEBUG_FILE_PREV);
      }

      VLOG("logger_task: SPIFFS: bytes total = %d, used = %d", SPIFFS.totalBytes(), SPIFFS.usedBytes());

      file = SPIFFS.open(DEBUG_FILE, FILE_WRITE);

      if (!file) {
        LOG("logger_task: error opening debug file");
      } else {
        LOG("logger_task: opened debug file");
        file_is_open = true;
      }
    }

    // use alternate serial port if GC client is using the primary serial port
    if (config_data.gc_serial_on && !output_port_changed) {
      Serial.println("*** logger redirecting output to alternate serial port ***");
      AltSerial.begin(115200, SERIAL_8N1, HW_TX_PIN, HW_RX_PIN);
      AltSerial.setTimeout(0);
      output_port_changed = true;
    }

    // process the incoming message queue
    if (xQueueReceive(logger_in_queue, &lm, QUEUE_OP_TIMEOUT_SHORT) == pdTRUE) {
      sprintf(tmp, "%11.6f: ", (float)(lm.m / 1000000.0));

      // write to file
      if (config_data.debug && file_is_open) {
        file.print(tmp);
        file.println(lm.s);
        file.flush();
      }

      // write to console
      if (output_port_changed) {
        AltSerial.print(tmp);
        AltSerial.println(lm.s);
      } else {
        Serial.print(tmp);
        Serial.println(lm.s);
      }
    }
  }
}

//
/// convenience function to log a message with a timestamp
//

void LOG(const char s[]) {

  log_message_t lm;

  lm.m = micros();
  strncpy(lm.s, s, DEBUG_MSG_LEN - 1);

  xQueueSend(logger_in_queue, &lm, QUEUE_OP_TIMEOUT_SHORT);
  return;
}

//
/// log a message with a variable number of arguments
//

void VLOG(const char fmt[], ...) {

  va_list vptr;
  char tmpbuff[DEBUG_MSG_LEN];

  va_start(vptr, fmt);
  vsnprintf(tmpbuff, DEBUG_MSG_LEN - 1, fmt, vptr);
  va_end(vptr);

  LOG(tmpbuff);
  return;
}
