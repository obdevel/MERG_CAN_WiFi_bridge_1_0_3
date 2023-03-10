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
/// a task to present the DCC++ serial interface via messages buffers and a single-instance network server
/// clients are withrottle task, command station proxy task, or an external JMRI instance
//

#include <WiFi.h>
#include "defs.h"

extern QueueHandle_t logger_in_queue, led_cmd_queue;
extern config_t config_data;
extern stats_t stats, errors;

message_buffer_t msgbuf_wi_in, msgbuf_wi_out, msgbuf_proxy_in, msgbuf_proxy_out;
SemaphoreHandle_t mtx_wi, mtx_proxy;

void dccppser_task(void *params) {

  char buffer[64];
  byte idx = 0;
  unsigned long stimer = millis();
  WiFiClient client;
  WiFiServer server;
  gcclient_t net_client;
  unsigned long nettx = 0, netrx = 0, msgtx = 0, msgrx = 0, errs = 0;

  VLOG("dccppser_task: task starting");

  // don't continue if server is not configured
  if (config_data.role == ROLE_SLAVE || !config_data.ser_on) {
    LOG("dccppser_task: DCC++ serial server task not configured to run, suspending task");
    vTaskSuspend(NULL);
  }

  // create mutexes to protect buffers
  mtx_wi = xSemaphoreCreateMutex();
  mtx_proxy = xSemaphoreCreateMutex();

  // start server on configured port number
  server.begin(config_data.ser_port);
  VLOG("dccppser_task: started DCC++ server on port = %d", config_data.ser_port);

  // config and open serial port to DCC++ basestation
  Serial2.begin(115200, SERIAL_8N1, HW_TX_PIN, HW_RX_PIN);

  for (;;) {

    // yield here, as this doesn't happen implicitly elsewhere
    vTaskDelay(5);

    //
    /// check for new network client connection
    //

    WiFiClient client = server.available();

    if (client) {
      if (net_client.client == NULL) {
        net_client.client = new WiFiClient(client);
        net_client.idx = 0;
        strcpy(net_client.addr, net_client.client->remoteIP().toString().c_str());
        net_client.port = net_client.client->remotePort();
        VLOG("dccppser_task: accepted net client connection from %s/%d", net_client.addr, net_client.port);
        PULSE_LED(NET_ACT_LED);
      } else {
        LOG("dccppser_task: net client connection is already taken, new connection rejected");
        client.stop();
        PULSE_LED(ERR_IND_LED);
      }
    }

    //
    /// check for active clients sending us data
    //

    /// from net client, e.g. JMRI

    if (net_client.client != NULL) {
      if (net_client.client->connected()) {
        if (net_client.client->available()) {

          ssize_t num_read = net_client.client->read((uint8_t *)net_client.input, GC_INP_SIZE);

          switch (num_read) {
            case -1:
              VLOG("dccppser_task: error reading from net client, errno = %d", errno);
              PULSE_LED(ERR_IND_LED);
              ++errs;
              break;

            case 0:
              LOG("dccppser_task: read 0 bytes from net client");
              break;

            default:
              net_client.input[num_read] = 0;
              VLOG("dccppser_task: read %d bytes from net client, input = |%s|", num_read, net_client.input);
              Serial2.write(net_client.input);
              PULSE_LED(NET_ACT_LED);
              netrx += num_read;
              break;
          }
        }
      } else {
        LOG("dccppser_task: net client has disconnected, reaping connection");
        net_client.client->stop();
        delete net_client.client;
        net_client.client = NULL;
        net_client.input[0] = 0;
        net_client.addr[0] = 0;
        net_client.port = 0;
        PULSE_LED(NET_ACT_LED);
      }   // is connected
    }   // is null

    /// from withrottle task

    if (xSemaphoreTake(mtx_wi, QUEUE_OP_TIMEOUT) == pdTRUE) {
      if (msgbuf_wi_out.head != msgbuf_wi_out.tail) {
        VLOG("dccppser_task: got new message from withrottle task at buffer = %d, msg = %s", msgbuf_wi_out.head, msgbuf_wi_out.buffer[msgbuf_wi_out.tail]);
        Serial2.write(msgbuf_wi_out.buffer[msgbuf_wi_out.tail]);
        msgbuf_wi_out.tail = (msgbuf_wi_out.tail + 1 ) % NUM_PROXY_CMDS;
        ++msgrx;
      }

      xSemaphoreGive(mtx_wi);
    }

    /// from proxy task

    if (xSemaphoreTake(mtx_proxy, QUEUE_OP_TIMEOUT) == pdTRUE) {
      if (msgbuf_proxy_out.head != msgbuf_proxy_out.tail) {
        VLOG("dccppser_task: got new message from proxy task at buffer = %d, msg = %s", msgbuf_proxy_out.head, msgbuf_proxy_out.buffer[msgbuf_proxy_out.tail]);
        Serial2.write(msgbuf_proxy_out.buffer[msgbuf_proxy_out.tail]);
        msgbuf_proxy_out.tail = (msgbuf_proxy_out.tail + 1 ) % NUM_PROXY_CMDS;
        ++msgrx;
      }

      xSemaphoreGive(mtx_proxy);
    }

    //
    /// read input fron DCC++ and send to clients
    //

    while (Serial2.available()) {

      char c = Serial2.read();

      // write to net client, char at a time
      if (net_client.client != NULL && net_client.client->connected()) {
        net_client.client->write(c);
        ++nettx;
        PULSE_LED(NET_ACT_LED);
      }

      // for local clients, buffer whole commands and write to task buffers
      // thus multiple responses are chunked, e.g. the status response has multiple parts

      switch (c) {
        case '<':
          idx = 0;
          buffer[idx++] = c;
          break;

        case '>':
          buffer[idx++] = c;
          buffer[idx] = 0;
          VLOG("dccppser_task: received response from DCC++ = %s", buffer);

          while (xSemaphoreTake(mtx_wi, QUEUE_OP_TIMEOUT) != pdTRUE);
          strncpy(msgbuf_wi_in.buffer[msgbuf_wi_in.head], buffer, PROXY_BUF_LEN);
          msgbuf_wi_in.head = (msgbuf_wi_in.head + 1) % NUM_PROXY_CMDS;
          xSemaphoreGive(mtx_wi);
          ++msgtx;
          LOG("dccppser_task: wrote line to withrottle input buffer");

          while (xSemaphoreTake(mtx_proxy, QUEUE_OP_TIMEOUT) != pdTRUE);
          strncpy(msgbuf_proxy_in.buffer[msgbuf_proxy_in.head], buffer, PROXY_BUF_LEN);
          msgbuf_proxy_in.head = (msgbuf_proxy_in.head + 1) % NUM_PROXY_CMDS;
          xSemaphoreGive(mtx_proxy);
          ++msgtx;
          LOG("dccppser_task: wrote line to proxy input buffer");

          break;

        case '\n':      // discard these
        case '\r':
          break;

        default:
          buffer[idx] = c;
          idx = (idx + 1) % sizeof(buffer);
          break;
      }
    }   // serial available

    //
    /// log stats
    //

    if (millis() - stimer >= 10000UL) {
      VLOG("dccppser_task: nettx = %d, netrx = %d, msgtx = %d, msgrx = %d, net client = %d", nettx, netrx, msgtx, msgrx, net_client.client != NULL);
      stimer = millis();
    }

  }   // for (;;)
}   // task func
