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
/// task to implement a WiThrottle protocol server
/// supports MERG and DCC++ command station back-ends
//
/// for a MERG backend, we pretend to be a CANCAB
/// so the node number is fixed (0xffff) and we use a common CANID
//

#include <WiFi.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>

#include "defs.h"
#include "cbusdefs.h"

typedef struct {
  WiFiClient *client;
  char buffer[64];
  byte idx;
  char ip[16];
  int port;
  int state;
  char device_name[64];
  char device_id[64];
  int loco_addr;
  char loco_addr_type;
  byte speed;
  bool direction;
  byte session_id;
  bool throttle_sends_heartbeat;
  unsigned long last_heartbeat_received;
} withrottle_client_t;

withrottle_client_t w_clients[MAX_WITHROTTLE_CLIENTS];

// external definitions
extern QueueHandle_t withrottle_queue, logger_in_queue, led_cmd_queue, CAN_out_from_withrottle_queue, \
net_out_queue, gc_out_queue, cmdproxy_queue;
extern message_buffer_t msgbuf_wi_in, msgbuf_wi_out;
extern SemaphoreHandle_t mtx_wi;
extern config_t config_data;
extern char mdnsname[];
extern byte num_gc_clients, num_peers;

// global variables
char wi_config_text[512];
byte num_wi_clients = 0;

// forward function delarations
bool send_wt_message_to_throttle(int i, char msg[]);
bool process_wt_message(int i, char cmd[]);
bool get_wt_session(int i);
bool release_wt_session(int i);
bool do_wt_action(int i, char tok0[], char tok1[]);;
void send_merg_keepalive(int i);
uint32_t make_can_header(void);
void send_merg_dspd(int i);
void send_merg_func_dfn(int i, byte func, byte state);
void send_merg_func_dfun(int i, byte fb1, byte fb2);
void send_dccpp_command(const char cmd[]);
void send_to_queues(twai_message_t *cf);
byte get_client_from_loco_addr(uint16_t loco_addr);

// default config data
const char config_filename[] = "/withrottle.txt";
const char default_config_file_data[] = "VN2.0\n"
                                        "RL1]\\[SP 2101}|{2102}|{L\n"
                                        "PPA2\n"
                                        "PTT]\\[Turnouts}|{Turnout]\\[Closed}|{2]\\[Thrown}|{4\n"
                                        "PRT]\\[Routes}|{Route]\\[Active}|{2]\\[Inactive}|{4\n"
                                        "RCC0\n"
                                        "PW12080\n"
                                        "*10\n";

void withrottle_task(void *params) {

  WiFiServer server;
  twai_message_t cf;
  char tbuff[64], buffer[PROXY_BUF_LEN];
  byte i, j;
  unsigned long hb_timer = millis(), ka_timer = millis(), stimer = millis();
  bool mdns_registered = false;
  File fp;
  bool got_message = false;

  LOG("withrottle_task: task starting");

  if (config_data.role == ROLE_SLAVE || !config_data.withrottle_on) {
    VLOG("withrottle_task: withrottle server not configured to run, suspending task");
    vTaskSuspend(NULL);
  }

  VLOG("withrottle_task: DCC backend = %s", (config_data.dcc_type == DCC_MERG) ? "MERG" : "DCC++");

  if (config_data.dcc_type == DCC_DCCPP && !config_data.ser_on) {
    VLOG("withrottle_task: using DCC++ backend but serial server task is not configured to run, suspending task");
    vTaskSuspend(NULL);
  }

  // initialise client records
  for (i = 0; i < MAX_WITHROTTLE_CLIENTS; i++) {
    bzero((void *)&w_clients[i], sizeof(w_clients[i]));
    w_clients[i].state = W_FREE;               // slot is free
    w_clients[i].client = NULL;                // client object is null

    if (config_data.dcc_type == DCC_DCCPP) {
      w_clients[i].session_id = i + 1;         // session id is fixed
    }
  }

  num_wi_clients = 0;

  // mount SPIFFS filesystem
  if (!SPIFFS.begin(true)) {
    LOG("withrottle_task: error mounting SPIFFS filesystem, will format");
    SPIFFS.format();
  } else {
    LOG("withrottle_task: SPIFFS filesystem mounted ok");
  }

  // create default config file if it doesn't already exist
  if (!SPIFFS.exists(config_filename)) {

    LOG("withrottle_task: config file does not exist, will create");
    fp = SPIFFS.open(config_filename, FILE_WRITE);

    if (!fp) {
      LOG("withrottle_task: unable to open config file for write");
    } else {
      fp.write((uint8_t *)default_config_file_data, sizeof(default_config_file_data));
      fp.close();
      LOG("withrottle_task: wrote default config file");
    }
  }

  // read config file into memory
  fp = SPIFFS.open(config_filename, FILE_READ);
  ssize_t bytes = 0;

  if (!fp) {
    LOG("withrottle_task: unable to open config file for read");
  } else {
    LOG("withrottle_task: reading config file");
    while (fp.available()) {
      bytes += fp.read((uint8_t *)&wi_config_text[bytes], sizeof(wi_config_text) - bytes);
    }

    wi_config_text[bytes] = 0;
    VLOG("withrottle_task: read %d bytes from config file", bytes);
    fp.close();
  }

  // start TCP/IP socket server on configured port number
  server.begin(config_data.withrottle_port);
  VLOG("withrottle_task: started server on port = %d", config_data.withrottle_port);

  // allow DCC++ task time to create shared objects
  if (config_data.dcc_type == DCC_DCCPP) {
    vTaskDelay(500);
  }

  /// main loop

  for (;;) {

    //
    /// yield here if using the DCC++ backend, because we don't block anywhere else
    //

    if (config_data.dcc_type == DCC_DCCPP) {
      vTaskDelay(5);
    }

    //
    /// add withrottle service to mDNS-SD; applies to both soft AP and wifi station interfaces
    //

    if (!mdns_registered) {
      sprintf(tbuff, "%s.local", mdnsname);
      MDNS.addService("_withrottle", "_tcp", config_data.withrottle_port);
      MDNS.addServiceTxt("_withrottle", "_tcp", "node", (const char *)tbuff);
      MDNS.addServiceTxt("_withrottle", "_tcp", "version", "1.0.0");
      LOG("withrottle_task: service registered with mDNS-SD");
      mdns_registered = true;
    }

    //
    /// check for new incoming connections
    //

    WiFiClient client = server.available();

    if (client) {
      VLOG("withrottle_task: new incoming connection, current total = %d", num_wi_clients);

      // find a free table slot
      for (i = 0; i < MAX_WITHROTTLE_CLIENTS; i++) {
        if (w_clients[i].state == W_FREE) {
          w_clients[i].client = new WiFiClient(client);
          w_clients[i].idx = 0;
          strcpy(w_clients[i].ip, w_clients[i].client->remoteIP().toString().c_str());
          w_clients[i].port = w_clients[i].client->remotePort();
          w_clients[i].state = W_CONNECTED;
          w_clients[i].throttle_sends_heartbeat = false;
          w_clients[i].last_heartbeat_received = millis();
          w_clients[i].speed = 0;
          w_clients[i].direction = DCC_DIR_FWD;
          VLOG("withrottle_task: new client at index = %d, IP = %s, remote port = %d", i, w_clients[i].ip, w_clients[i].port);
          ++num_wi_clients;
          break;
        }
      }

      if (i == MAX_WITHROTTLE_CLIENTS) {
        LOG("withrottle_task: too many clients, connection rejected");
        client.stop();
      } else {

        // send config data to new client, a line at a time
        byte b = 0, j = 0;
        size_t s = strlen(wi_config_text);

        LOG("withrottle_task: sending config data to client");

        for (b = 0; b < s; b++) {
          if (wi_config_text[b] == '\n' || wi_config_text[b] == '\r' || b == s || j >= (sizeof(tbuff) - 1)) {
            tbuff[j] = 0;
            send_wt_message_to_throttle(i, tbuff);
            j = 0;
          } else {
            tbuff[j++] = wi_config_text[b];
          }
        }

        if (b == s && j > 0) {
          tbuff[j] = 0;
          send_wt_message_to_throttle(i, tbuff);
        }
      }
    }  // if new client connected

    //
    /// loop through each client record
    //

    for (i = 0; i < MAX_WITHROTTLE_CLIENTS && num_wi_clients > 0; i++) {

      //
      /// reap terminated connections
      //

      if (w_clients[i].state == W_CLOSING) {
        VLOG("withrottle_task: reaping client = %d", i);
        release_wt_session(i);
        w_clients[i].client->stop();
        delete w_clients[i].client;
        bzero((void *)&w_clients[i], sizeof(w_clients[i]));
        --num_wi_clients;
        continue;
      }

      //
      /// for active client connections
      //

      if (w_clients[i].client != NULL) {

        // check heartbeats once a second, timeout after 10 seconds
        if (millis() - hb_timer > 1000) {
          hb_timer = millis();

          if (w_clients[i].throttle_sends_heartbeat && millis() - w_clients[i].last_heartbeat_received > 10000) {
            VLOG("withrottle_task: client = %d, hb has expired", i);
            w_clients[i].state = W_CLOSING;
            --num_wi_clients;

            // !! release MERG session
            // !! JMRI protocol sends e-stop

            continue;
          }
        }

        if (w_clients[i].client->connected()) {

          //
          /// read incoming data
          //

          while (w_clients[i].client->available()) {

            // VLOG("withrottle_task: client %d is available for reading", i);

            // read a character at a time
            ssize_t c = w_clients[i].client->read();

            if (c >= 0) {

              /// assemble a command string until the end-of-line is reached

              // VLOG("withrottle_task: got char = |%c|", c);
              switch (c) {
                case '\r':
                case '\n':
                  // we now have a complete withrottle message string from the client

                  // VLOG("end of string buffer due to CR/LF at idx = %d", w_clients[i].idx);
                  w_clients[i].buffer[w_clients[i].idx] = 0;
                  w_clients[i].idx = 0;

                  // VLOG("withrottle_task: input processing complete for client %d, string = |%s|", i, w_clients[i].buffer);
                  PULSE_LED(NET_ACT_LED);

                  if (!process_wt_message(i, w_clients[i].buffer)) {
                    // VLOG("withrottle_task: client = %d sends quit message, reaping connection", i);
                    w_clients[i].state = W_CLOSING;
                    continue;
                  }

                  break;

                default:
                  // any other character is added to the end of the buffer
                  w_clients[i].buffer[w_clients[i].idx] = c;
                  w_clients[i].idx = (w_clients[i].idx + 1) % sizeof(w_clients[i].buffer);
                  break;

              }  // switch c
            }  // char 0
          }  // is available

        } else {

          // client has disconnected, connection will be reaped next time around the loop
          VLOG("withrottle_task: client has disconnected, index = %d", i);
          w_clients[i].state = W_CLOSING;
        }  // is connected

        // send keepalive message to MERG command station
        if (config_data.dcc_type == DCC_MERG && millis() - ka_timer >= 4000 && w_clients[i].session_id > 0) {
          send_merg_keepalive(i);
          ka_timer = millis();
        }
      }  // not NULL
    }  // for each client

    //
    /// read messages from command station, either MERG CANCMD or DCC++
    //

    if (config_data.dcc_type == DCC_MERG) {

      //
      /// process CAN frames from local bus
      /// we are only interested in CANCMD opcodes
      //

      if (xQueueReceive(withrottle_queue, &cf, QUEUE_OP_TIMEOUT) == pdTRUE) {
        // VLOG("withrottle_task: got new frame from output queue: %s", format_CAN_frame(&cf));

        // check whether the opcode is something relevant from the CANCMD command station
        // ...

        switch (cf.data[0]) {
          case OPC_PLOC:
            // PLOC <0xE1><Session><AddrH><AddrL><Speed/Dir><Fn1><Fn2><Fn3>
            VLOG("withrottle_task: PLOC from command station, session = %d, loco = %d", cf.data[1], (cf.data[2] << 8) + cf.data[3]);
            j = get_client_from_loco_addr((cf.data[2] << 8) + cf.data[3]);

            if (j == MAX_WITHROTTLE_CLIENTS) {
              LOG("withrottle_task: no matching connected client");
            } else {
              w_clients[j].session_id = cf.data[1];
              w_clients[j].state = W_ACTIVE;
            }
            break;

          case OPC_ERR:
            // ERR <63><Dat 1><Dat 2><Dat 3>, First two bytes are loco address, third is error number.
            VLOG("withrottle_task: error from command station, %d %d %d", cf.data[1], cf.data[2], cf.data[3]);
            j = get_client_from_loco_addr((cf.data[1] << 8) + cf.data[2]);

            if (j == MAX_WITHROTTLE_CLIENTS) {
              LOG("withrottle_task: no matching connected client");
            } else {
              w_clients[j].session_id = cf.data[1];
              w_clients[j].state = W_ACTIVE;
            }
            break;

          default:
            break;
        }
      }
    }   // is MERG

    if (config_data.dcc_type == DCC_DCCPP) {

      //
      /// read and process messages from DCC++ serial port
      //

      while (xSemaphoreTake(mtx_wi, QUEUE_OP_TIMEOUT) != pdTRUE);
      if (msgbuf_wi_in.head != msgbuf_wi_in.tail) {
        VLOG("withrottle_task: got new message from DCC++, buffer = %d, %s", msgbuf_wi_in.tail, msgbuf_wi_in.buffer[msgbuf_wi_in.tail]);
        strncpy(buffer, msgbuf_wi_in.buffer[msgbuf_wi_in.tail], sizeof(buffer));
        msgbuf_wi_in.tail = (msgbuf_wi_in.tail + 1) % NUM_PROXY_CMDS;
        got_message = true;
      }

      xSemaphoreGive(mtx_wi);

      if (got_message) {
        VLOG("withrottle_task: processing incoming DCC++ message = %s", buffer);
        got_message = false;

        // only one message type of interest
        // <T REGISTER SPEED DIRECTION>

        if (buffer[1] == 'T') {
          unsigned int  treg, taddr, tspeed, tdir;
          byte ntokens = sscanf(buffer + 2, "%d %d %d %d", &treg, &taddr, &tspeed, &tdir);
          VLOG("withrottle_task: parsed %d tokens from DCC++ message = |%s| to %h, %d, %h, %h", ntokens, buffer, treg, taddr, tspeed, tdir);

          if (treg <= MAX_WITHROTTLE_CLIENTS) {
            w_clients[treg - 1].session_id = treg;
            w_clients[treg - 1].loco_addr = taddr;
            w_clients[treg - 1].speed = tspeed;
            w_clients[treg - 1].direction = tdir;
          } else {
            LOG("withrottle_task: no matching session");
          }
        }
      }   // got message
    }   // is DCC++

    //
    /// display connected clients
    //

    if (millis() - stimer >= 10000) {
      stimer = millis();

      VLOG("withrottle_task: [%d] clients = %d", config_data.CANID, num_wi_clients);

      for (i = 0; i < MAX_WITHROTTLE_CLIENTS && num_wi_clients > 0; i++) {

        if (w_clients[i].client != NULL) {
          VLOG("withrottle_task: [%d] %s/%d, %d: %d %c, %d %d", i, w_clients[i].ip, w_clients[i].port, w_clients[i].session_id, \
               w_clients[i].loco_addr, w_clients[i].loco_addr_type, w_clients[i].speed, w_clients[i].direction);
        }
      }
    }

  }   // for (;;)
}

//
/// send a message to a connected throttle
//

bool send_wt_message_to_throttle(int i, char msg[]) {

  size_t s = strlen(msg);
  ssize_t b;
  bool ret = true;

  VLOG("withrottle_task: send_wt_message_to_throttle: client = %d, message = |%s|", i,  msg);

  if ((b = w_clients[i].client->write(msg, s)) != s) {
    VLOG("withrottle_task: send_wt_message_to_throttle: = %d, expected = %d, sent = %d", i, s, b);
    PULSE_LED(ERR_IND_LED);
    ret = false;
  } else {

    /// !! ugh, fixme !!
    w_clients[i].client->write('\n');
    PULSE_LED(NET_ACT_LED);
  }

  return ret;
}

//
/// process a command string received from a connected throttle
/// return false if client quits, otherwise true
//

bool process_wt_message(int i, char cmd[]) {

  VLOG("withrottle_task: process_wt_message: client = %d, command = |%s|", i, cmd);

  char addrbuff[8], tokens[2][16], *ptr;
  byte addridx = 0;
  byte j = 0;

  // update heartbeat on every message
  w_clients[i].last_heartbeat_received = millis();

  // parse into tokens
  ptr = strtok(cmd, "<;>");

  while (ptr) {
    strcpy(tokens[j++], ptr);
    ptr = strtok(NULL, "<;>");
  }

  VLOG("withrottle_task: process_wt_message: parsed %d tokens", j);

  switch (tokens[0][0]) {
    case 'Q':
      VLOG("withrottle_task: process_wt_message: client = %d, quitting", i);
      return false;
      break;

    case 'N':
      VLOG("withrottle_task: process_wt_message: client = %d, device name = %s", i, cmd + 1);
      strcpy(w_clients[i].device_name, cmd + 1);
      return true;
      break;

    case 'H':
      VLOG("withrottle_task: process_wt_message: client = %d, device hw id = %s", i, cmd + 1);
      strcpy(w_clients[i].device_id, cmd + 1);
      return true;
      break;

    case '*':
      VLOG("withrottle_task: process_wt_message: client = %d, heartbeat", i);

      if (tokens[0][1] == '+' || tokens[0][1] == '-') {
        w_clients[i].throttle_sends_heartbeat = (tokens[0][1] == '+');
        VLOG("withrottle_task: process_wt_message: client = %d, will send heartbeats = %d", i, w_clients[i].throttle_sends_heartbeat);
      }

      return true;
      break;

    case 'R':
      LOG("withrottle_task: process_wt_message: consist commands not supported");
      break;

    case 'P':
      LOG("withrottle_task: process_wt_message: turnout and route commands not supported");
      break;

    case 'M':
      VLOG("withrottle_task: process_wt_message: client = %d, throttle request = %s", i, cmd);

      // parse the loco address

      // find the beginning of the address number
      for (j = 2; j < strlen(tokens[0]); j++) {
        if (isdigit(cmd[j])) {
          break;
        }
      }

      // copy out the numbers
      while (j < strlen(tokens[0]) && isdigit(tokens[0][j])) {
        addrbuff[addridx++] = cmd[j++];
      }

      // convert to an integer & save
      addrbuff[addridx] = 0;
      w_clients[i].loco_addr = atoi(addrbuff);
      w_clients[i].loco_addr_type = tokens[0][3];
      VLOG("withrottle_task: process_wt_message: address = %d, type = %c", w_clients[i].loco_addr, w_clients[i].loco_addr_type);

      // parse and dispatch the command
      switch (tokens[0][2]) {
        case '+':
          LOG("withrottle_task: process_wt_message: request to add loco to throttle");
          get_wt_session(i);
          break;

        case '-':
          LOG("withrottle_task: process_wt_message: request to release loco from throttle");
          release_wt_session(i);
          w_clients[i].loco_addr = 0;
          break;

        case 'A':
          LOG("withrottle_task: process_wt_message: action request");
          do_wt_action(i, tokens[0], tokens[1]);
          break;

        case 'r':
        case 'd':
          LOG("withrottle_task: process_wt_message: dispatch/release not supported");
          break;

        default:
          VLOG("withrottle_task: process_wt_message: unknown action = %c", tokens[0][2]);
          break;
      }

      // send confirmation back to throttle
      send_wt_message_to_throttle(i, tokens[0]);
      break;

    default:
      VLOG("withrottle_task: process_wt_message: client = %d, unhandled cmd = %s", i, cmd);
  }

  return true;
}

//
/// get a new session for this throttle & loco address
//

bool get_wt_session(int i) {

  char buffer[PROXY_BUF_LEN];
  VLOG("withrottle_task: get_wt_session, i = %d, addr = %d", i, w_clients[i].loco_addr);

  w_clients[i].state = W_AWAITING_SESSION_ID;

  if (config_data.dcc_type == DCC_MERG) {
    // MERG
    twai_message_t cf;
    cf.identifier = make_can_header();
    cf.data_length_code = 3;
    cf.data[0] = OPC_RLOC;
    cf.data[1] = highByte(w_clients[i].loco_addr);
    cf.data[2] = lowByte(w_clients[i].loco_addr);
    send_to_queues(&cf);
  } else {
    // DCC++
    // <t REGISTER CAB SPEED DIRECTION>
    w_clients[i].session_id = i + 1;
    snprintf(buffer, PROXY_BUF_LEN, "<t %d %d %d %d>", w_clients[i].session_id, w_clients[i].loco_addr, w_clients[i].speed, w_clients[i].direction);
    send_dccpp_command(buffer);
  }

  // state and session id will be updated once command station responds to session request
  w_clients[i].state = W_AWAITING_SESSION_ID;
  return true;
}

//
/// release a session for this throttle and loco address
//

bool release_wt_session(int i) {

  VLOG("withrottle_task: release_wt_session, i = %d, addr = %d, session id = %d", i, w_clients[i].loco_addr, w_clients[i].session_id);

  if (w_clients[i].loco_addr == 0 || w_clients[i].session_id == 0) {
    LOG("withrottle_task: client has no current session");
    return true;
  }

  if (config_data.dcc_type == DCC_MERG) {
    // MERG
    twai_message_t cf;
    cf.identifier = make_can_header();
    cf.data_length_code = 2;
    cf.data[0] = OPC_KLOC;
    cf.data[1] = w_clients[i].session_id;
    send_to_queues(&cf);
  } else {
    // DCC++
    // no action required
  }

  // reset client record
  w_clients[i].session_id = 0;
  w_clients[i].state = W_CONNECTED;
  w_clients[i].loco_addr = 0;
  w_clients[i].loco_addr_type = ' ';
  w_clients[i].speed = 0;
  w_clients[i].direction = DCC_DIR_FWD;

  return true;
}

//
/// perform an action depending on the command string from the throttle
//

bool do_wt_action(int i, char tok0[], char tok1[]) {

  byte func_num, func_state, fb1, fb2;
  char buffer[PROXY_BUF_LEN], tbuff[16];

  VLOG("withrottle_task: do_wt_action, i = %d, tok0 = %s, tok1 = %s", i, tok0, tok1);

  switch (tok1[0]) {
    case 'V':
      w_clients[i].speed = atoi(&tok1[1]);
      VLOG("withrottle_task: do_wt_action, setting speed to %d", w_clients[i].speed);

      if (config_data.dcc_type == DCC_MERG) {
        send_merg_dspd(i);
      } else {
        // <t REGISTER CAB SPEED DIRECTION>
        snprintf(buffer, PROXY_BUF_LEN, "<t %d %d %d %d>", w_clients[i].session_id, w_clients[i].loco_addr, w_clients[i].speed, w_clients[i].direction);
        send_dccpp_command(buffer);
      }

      break;

    case 'R':
      w_clients[i].direction = atoi(&tok1[1]);
      VLOG("withrottle_task: do_wt_action, changing direction to %d", w_clients[i].direction);

      if (config_data.dcc_type == DCC_MERG) {
        send_merg_dspd(i);
      } else {
        // <t REGISTER CAB SPEED DIRECTION>
        snprintf(buffer, PROXY_BUF_LEN, "<t %d %d %d %d>", w_clients[i].session_id, w_clients[i].loco_addr, w_clients[i].speed, w_clients[i].direction);
        send_dccpp_command(buffer);
      }

      break;

    case 'X':
      w_clients[i].speed = 1;
      VLOG("withrottle_task: do_wt_action, emergency stop, setting speed to %d", w_clients[i].speed);

      if (config_data.dcc_type == DCC_MERG) {
        send_merg_dspd(i);
      } else {
        // <t REGISTER CAB SPEED DIRECTION>
        snprintf(buffer, PROXY_BUF_LEN, "<t %d %d %d %d>", w_clients[i].session_id, w_clients[i].loco_addr, w_clients[i].speed, w_clients[i].direction);
        send_dccpp_command(buffer);
      }

      break;

    case 'I':
      w_clients[i].speed = 0;
      VLOG("withrottle_task: do_wt_action, idle command, setting speed to %d", w_clients[i].speed);

      if (config_data.dcc_type == DCC_MERG) {
        send_merg_dspd(i);
      } else {
        // <t REGISTER CAB SPEED DIRECTION>
        snprintf(buffer, PROXY_BUF_LEN, "<t %d %d %d %d>", w_clients[i].session_id, w_clients[i].loco_addr, w_clients[i].speed, w_clients[i].direction);
        send_dccpp_command(buffer);
      }

      break;

    case 'F':
      func_state = (tok1[1] == '1');
      func_num = atoi(&tok1[2]);
      VLOG("withrottle_task: do_wt_action, function command, num = %d, state = %d", func_num, func_state);

      switch (func_num) {
        case 0:
          fb1 = 128 + (func_state * 16);
          fb2 = 0;
          break;
        case 1:
          fb1 = 128 + (func_state * 1);
          fb2 = 0;
          break;
        case 2:
          fb1 = 128 + (func_state * 2);
          fb2 = 0;
          break;
        case 3:
          fb1 = 128 + (func_state * 4);
          fb2 = 0;
          break;
        case 4:
          fb1 = 128 + (func_state * 8);
          fb2 = 0;
          break;
        case 5:
          fb1 = 176 + (func_state * 1);
          fb2 = 0;
          break;
        case 6:
          fb1 = 128 + (func_state * 2);
          fb2 = 0;
          break;
        case 7:
          fb1 = 128 + (func_state * 4);
          fb2 = 0;
          break;
        case 8:
          fb1 = 128 + (func_state * 8);
          fb2 = 0;
          break;
        case 9:
          fb1 = 160 + (func_state * 1);
          fb2 = 0;
          break;
        case 10:
          fb1 = 160 + (func_state * 2);
          fb2 = 0;
          break;
        case 11:
          fb1 = 160 + (func_state * 4);
          fb2 = 0;
          break;
        case 12:
          fb1 = 160 + (func_state * 8);
          fb2 = 0;
          break;
        default:
          fb1 = 0;
          fb2 = 0;
          break;
      }

      if (config_data.dcc_type == DCC_MERG) {
        // send_merg_func_dfn(i, func_num, func_state);
        send_merg_func_dfun(i, fb1, fb2);
      } else {
        // <f CAB BYTE1 [BYTE2]>
        snprintf(buffer, sizeof(buffer), "<f %d %d %d>", w_clients[i].loco_addr, fb1, fb2);
        send_dccpp_command(buffer);
      }

      // ack to throttle e.g. M0AL341<;>F10
      sprintf(buffer, "M0A%c%d<;>F%d%d", w_clients[i].loco_addr_type, w_clients[i].loco_addr, func_state, func_num);
      send_wt_message_to_throttle(i, buffer);
      break;

    case 'q':
      VLOG("withrottle_task: do_wt_action, query command = %c", tok1[1]);

      if (tok1[1] == 'V') {
        snprintf(tbuff, sizeof(tbuff), "M0A%c%d<;>V%d", w_clients[i].loco_addr_type, w_clients[i].loco_addr, w_clients[i].speed);
        send_wt_message_to_throttle(i, tbuff);
      } else if (tok1[1] == 'R') {
        snprintf(tbuff, sizeof(tbuff), "M0A%c%d<;>R%d", w_clients[i].loco_addr_type, w_clients[i].loco_addr, w_clients[i].direction);
        send_wt_message_to_throttle(i, tbuff);
      }

      break;

    default:
      VLOG("withrottle_task: do_wt_action, unhandled command = %c", tok1[0]);
      break;
  }

  return true;
}

//
/// send a CBUS keepalive message to a command station
//

void send_merg_keepalive(int i) {

  twai_message_t cf;

  if (w_clients[i].state != W_ACTIVE || w_clients[i].session_id == 0) {
    VLOG("withrottle_task: send_merg_keepalive: client %d has no current session", i);
    return;
  }

  VLOG("withrottle_task: sending MERG keepalive");

  cf.identifier = make_can_header();
  cf.data_length_code = 2;
  cf.data[0] = OPC_DKEEP;
  cf.data[1] = w_clients[i].session_id;
  send_to_queues(&cf);
  return;
}

//
/// send a DSPD CBUS message to a command station
//

void send_merg_dspd(int i) {

  twai_message_t cf;

  if (w_clients[i].state != W_ACTIVE || w_clients[i].session_id == 0) {
    VLOG("withrottle_task: send_merg_dspd: client %d has no current session", i);
    return;
  }

  VLOG("withrottle_task: send_merg_dspd: sending speed/dir message to command station, client = %d, speed = %d, dir = %d", i, w_clients[i].speed, w_clients[i].direction);

  cf.identifier = make_can_header();
  cf.data_length_code = 3;
  cf.data[0] = OPC_DSPD;
  cf.data[1] = w_clients[i].session_id;
  cf.data[2] = w_clients[i].speed;
  bitWrite(cf.data[2], 7, w_clients[i].direction);
  send_to_queues(&cf);
  return;
}

//
/// send a MERG DCC function command using the DFNON/DFNOF opcodes
//

void send_merg_func_dfn(int i, byte func, byte state) {

  twai_message_t cf;

  if (w_clients[i].state != W_ACTIVE || w_clients[i].session_id == 0) {
    VLOG("withrottle_task: send_merg_func_dfn: client %d has no current session", i);
    return;
  }

  VLOG("withrottle_task: send_merg_func_dfn: sending function command, client = %d, func = %d, state = %d", i, func, state);

  cf.identifier = make_can_header();
  cf.data_length_code = 3;
  cf.data[0] = (state) ? OPC_DFNON : OPC_DFNOF;
  cf.data[1] = w_clients[i].session_id;
  cf.data[2] = func;
  send_to_queues(&cf);
  return;
}

//
/// send a MERG DCC function command using the DFUN opcode
//

void send_merg_func_dfun(int i, byte fb1, byte fb2) {

  twai_message_t cf;

  if (w_clients[i].state != W_ACTIVE || w_clients[i].session_id == 0) {
    VLOG("withrottle_task: send_merg_func_dfun: client %d has no current session", i);
    return;
  }

  VLOG("withrottle_task: send_merg_func_dfun: sending function command, client = %d, fb1 = %d, fb2 = %d", i, fb1, fb2);

  cf.identifier = make_can_header();
  cf.data_length_code = 3;
  cf.data[0] = OPC_DFUN;
  cf.data[1] = fb1;
  cf.data[2] = fb2;
  send_to_queues(&cf);
  return;
}

//
/// send a command to the DCC++ command station via the serial port server task
//

void send_dccpp_command(const char cmd[]) {

  while (xSemaphoreTake(mtx_wi, QUEUE_OP_TIMEOUT) != pdTRUE);
  strncpy(msgbuf_wi_out.buffer[msgbuf_wi_out.head], cmd, PROXY_BUF_LEN);
  msgbuf_wi_out.head = (msgbuf_wi_out.head + 1) % NUM_PROXY_CMDS;
  xSemaphoreGive(mtx_wi);
  VLOG("withrottle_task: send_dccpp_command, sent cmd to DCC++ command station = %s, h = %d, t = %d", cmd, \
       msgbuf_wi_out.head, msgbuf_wi_out.tail);

  return;
}

//
/// send outgoing CAN message to output queues
//

void send_to_queues(twai_message_t *cf) {

  uint16_t queues = QUEUE_CAN_OUT_FROM_WI | QUEUE_NET_OUT | QUEUE_GC_OUT | QUEUE_CMDPROXY_IN | QUEUE_CBUS_INTERNAL;

  if (!send_message_to_queues(queues, cf, "withrottle_task", QUEUE_OP_TIMEOUT_NONE)) {
    LOG("withrottle_task: error queuing message");
    PULSE_LED(ERR_IND_LED);
  }

  return;
}

byte get_client_from_loco_addr(uint16_t loco_addr) {

  byte i;

  for (i = 0; i < MAX_WITHROTTLE_CLIENTS; i++) {
    if (w_clients[i].loco_addr == loco_addr) {
      break;
    }
  }

  return i;
}
