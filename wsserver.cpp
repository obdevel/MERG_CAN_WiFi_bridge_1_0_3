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

//
/// experimental websocket server
/// to publish logger data and CAN messages
//

#include <WiFi.h>
#include <WebSocketsServer.h>

#include "defs.h"

typedef struct {
  bool connected;
  uint8_t num;
} ws_client_t;

ws_client_t ws_clients[WEBSOCKETS_SERVER_CLIENT_MAX];

extern QueueHandle_t logger_in_queue, led_cmd_queue, wsserver_out_queue;
extern config_t config_data;

void on_websocket_event(uint8_t num, WStype_t type, uint8_t *payload, size_t length);

WebSocketsServer websocket = WebSocketsServer(81);
bool wsserver_running = false;

void wsserver_task(void *params) {

  byte i;
  twai_message_t cf;
  unsigned long stats_timer = 0UL;
  char gcbuff[32];

  VLOG("wsserver_task: websocket server starting, max clients = %d", WEBSOCKETS_SERVER_CLIENT_MAX);
  wsserver_running = true;

  // initialise clients
  for (i = 0; i < WEBSOCKETS_SERVER_CLIENT_MAX; i++) {
    ws_clients[i].connected = false;
    ws_clients[i].num = 0;
  }

  // start websocket server
  websocket.begin();
  websocket.onEvent(on_websocket_event);

  for (;;) {

    // allow server to run
    websocket.loop();

    // log stats
    if (millis() - stats_timer > 10000) {
      stats_timer = millis();

      for (i = 0; i < WEBSOCKETS_SERVER_CLIENT_MAX; i++) {
        if (ws_clients[i].connected) {
          VLOG("wsserver_task: client %u, channel %u", i, ws_clients[i].num);
        }
      }
    }

    // get next CAN frame from incoming queue
    if (xQueueReceive(wsserver_out_queue, &cf, QUEUE_OP_TIMEOUT_LONG) == pdTRUE) {
      for (i = 0; i < WEBSOCKETS_SERVER_CLIENT_MAX; i++) {
        if (ws_clients[i].connected) {
          CANtoGC(&cf, gcbuff);
          websocket.sendTXT(ws_clients[i].num, gcbuff);
        }
      }

    }  // got queue message
  }  // for (;;)
}

void on_websocket_event(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {

  byte i;

  switch (type) {

    case WStype_DISCONNECTED:
      VLOG("wsserver_task: [%u] disconnected", num);

      for (i = 0; i < WEBSOCKETS_SERVER_CLIENT_MAX; i++) {
        if (ws_clients[i].num == num) {
          ws_clients[i].connected = false;
          ws_clients[i].num = 0;
          break;
        }
      }

      break;

    case WStype_CONNECTED:
      VLOG("wsserver_task: [%u] connection from %s, url = %s", num, websocket.remoteIP(num).toString().c_str(), payload);

      for (i = 0; i < WEBSOCKETS_SERVER_CLIENT_MAX; i++) {
        if (!ws_clients[i].connected) {
          ws_clients[i].connected = true;
          ws_clients[i].num = num;
          break;
        }
      }

      break;

    case WStype_TEXT:
      VLOG("wsserver_task: [%u] text = %s, len = %d", num, payload, length);
      break;
    case WStype_BIN:
      VLOG("wsserver_task: [%u] unhandled event type BIN", num);
      break;
    case WStype_ERROR:
      VLOG("wsserver_task: [%u] unhandled event type ERROR", num);
      break;
    case WStype_FRAGMENT_TEXT_START:
      VLOG("wsserver_task: [%u] unhandled event type FRAGMENT_TEXT_START", num);
      break;
    case WStype_FRAGMENT_BIN_START:
      VLOG("wsserver_task: [%u] unhandled event type FRAGMENT_BIN_START", num);
      break;
    case WStype_FRAGMENT:
      VLOG("wsserver_task: [%u] unhandled event type FRAGMENT", num);
      break;
    case WStype_FRAGMENT_FIN:
      VLOG("wsserver_task: [%u] unhandled event type FRAGMENT_FIN", num);
      break;
    case WStype_PING:
      VLOG("wsserver_task: [%u] unhandled event type PING", num);
      break;
    case WStype_PONG:
      VLOG("wsserver_task: [%u] unhandled event type PONG", num);
      break;
    default:
      VLOG("wsserver_task: [%u] unknown event type = %d", num, type);
      break;
  }
}
