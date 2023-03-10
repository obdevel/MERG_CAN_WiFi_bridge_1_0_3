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
/// a task to implement a CBUS FLiM module
/// handles CBUS messages and bus enumeration
/// only required for master node implementing withrottle with CANCMD backend, or for CANCMD proxy
/// identifies as a MERG CANCMD
//


#include <WiFi.h>
#include "defs.h"
#include "cbusdefs.h"
#include "switch.h"

// variables defined in other source files
extern QueueHandle_t cbus_in_queue, net_out_queue, gc_out_queue, withrottle_queue, cmdproxy_queue, cmdproxy_queue, \
CAN_out_from_net_queue, led_cmd_queue, cbus_internal;
extern config_t config_data;
extern byte num_gc_clients, num_peers, num_wi_clients;
extern peer_state_t peers[MAX_NET_PEERS];

// global variables
byte node_params[21];
unsigned char node_mname[7] = { 'C', 'M', 'D' };
bool enum_required = true, enum_in_progress = false, respond_to_enum = false;
byte enum_responses[16];
byte proxy_canids[MAX_NET_PEERS];
bool learn_mode = false, in_transition = false;
unsigned long enum_start_time;
Switch sw;
unsigned long ttimer = 0UL;
QueueSetHandle_t queue_set;

// forward function declarations
void enumerate_can_bus(void);
uint32_t make_can_header(void);
void send_message_to_queues(twai_message_t *cf);
void send_WRACK(void);
void send_CMDERR(byte num);
void transition_to_flim(void);
void indicate_cbus_mode(byte mode);

//
/// main task function
//

void cbus_task(void *params) {

  twai_message_t cf, of;
  unsigned long ptimer = millis();
  uint16_t nn;
  byte rcanid;

  if (config_data.role != ROLE_MASTER) {
    LOG("cbus_task: node is a slave, task suspended");
    vTaskSuspend(NULL);
  }

  VLOG("cbus_task: task starting, CANID = %d, node number = %d, mode = %s", config_data.CANID, config_data.node_number, (config_data.cbus_mode) ? "FLiM" : "SLiM");

  // config CBUS switch & LEDs
  sw.setPin(CBUS_SWITCH_PIN, LOW);

  pinMode(SLIM_LED_PIN, OUTPUT);
  pinMode(FLIM_LED_PIN, OUTPUT);
  indicate_cbus_mode(config_data.cbus_mode);

  // set module parameters
  node_params[0] = 20;                     //  0 num params = 20
  node_params[1] = 0xa5;                   //  1 manf = MERG, 165
  node_params[2] = VER_MIN;                //  2 code minor version
  node_params[3] = MTYP_CANCMD;            //  3 module id = CANCMD
  node_params[4] = 0;                      //  4 num events = 0
  node_params[5] = 0;                      //  5 num evs per event = 0
  node_params[6] = NUM_CBUS_NVS;           //  6 num NVs
  node_params[7] = VER_MAJ;                //  7 code major version
  node_params[8] = 0x07;                   //  8 flags = 7, FLiM, consumer/producer
  node_params[9] = 0x00;                   //  9 processor id = 50
  node_params[10] = PB_CAN;                // 10 interface protocol = CAN, 1
  node_params[11] = 0x01;                  // load address
  node_params[12] = 0x02;
  node_params[13] = 0x03;
  node_params[14] = 0x04;
  node_params[15] = 0x04;                  // some other chip id
  node_params[16] = 0x01;
  node_params[17] = 0x02;
  node_params[18] = 0x03;
  node_params[19] = 0x04;                  // some other manufacturer
  node_params[20] = VER_PATCH;

  // check for unconfigured CANID
  if (config_data.CANID == 0 || config_data.CANID > 128) {
    VLOG("cbus_task: invalid CANID = %d, CAN bus enumeration is required", config_data.CANID);
    enum_required = true;
  }

  /// we have two queues
  /// the external queue holds messages from the external CBUS
  /// the internal queue holds messages sent by internal tasks
  /// internal tasks will use a common module CANID
  /// external messages should not conflict with our CANID and will trigger a re-enumeration

  /// create a queue set so we can block on one object rather than two separate queues
  queue_set = xQueueCreateSet(250);
  xQueueAddToSet(cbus_in_queue, queue_set);
  xQueueAddToSet(cbus_internal, queue_set);

  /// main loop

  for (;;) {

    //
    /// process CBUS switch input
    //

    sw.run();

    // long press and hold
    if (sw.isPressed() && sw.getCurrentStateDuration() > 6000 && !in_transition) {
      LOG("cbus_task: long switch hold, initiating FLiM transition");
      transition_to_flim();
      sw.resetCurrentDuration();
    }

    if (sw.stateChanged() && !sw.isPressed()) {
      if (sw.getLastStateDuration() < 1000) {
        LOG("cbus_task: short switch press, starting bus enumeration");
        enum_required = true;
      } else if (sw.getLastStateDuration() >= 1000 && sw.getLastStateDuration() < 2000) {
        if (in_transition) {                             // cancel transition
          LOG("cbus_task: medium switch press, cancelling transition");
          indicate_cbus_mode(config_data.cbus_mode);
          in_transition = false;
        } else {
          if (config_data.cbus_mode == CBUS_MODE_FLIM) {  // reconfirm node number
            LOG("cbus_task: medium switch press, confirming node number");
            transition_to_flim();
          }
        }
      }
    }

    //
    /// mode transition timeout
    //

    if (in_transition && (millis() - ttimer > 30000)) {
      LOG("cbus_task: FLiM transition has timed out");
      indicate_cbus_mode(config_data.cbus_mode);
      in_transition = false;
    }

    //
    /// enumerate CAN bus to get a unique CANID
    //

    if (enum_required) {
      LOG("cbus_task: enumeration required flag has been set");
      enum_required = false;
      enumerate_can_bus();
    }

    //
    /// respond to enumeration request from another node
    //

    if (respond_to_enum) {
      of.data_length_code = 0;

      for (int8_t i = -1; i < MAX_NET_PEERS; i++) {
        if (i == -1) {
          of.identifier = config_data.CANID;
        } else {
          of.identifier = proxy_canids[i];
        }

        send_message_to_queues(&of);
        vTaskDelay((TickType_t)5);
      }

      VLOG("cbus_task: enumeration response sent, my CANID = %d", config_data.CANID);
      respond_to_enum = false;
    }

    //
    /// check for end of enumeration cycle and process data
    //

    if (enum_in_progress && (millis() - enum_start_time >= 100)) {
      byte free_id, selected_id = 0;
      int8_t proxy_idx = -1;

      LOG("cbus_task: end of enumeration cycle");
      enum_in_progress = false;

      for (byte i = 0; i < 16; i++) {
        for (byte b = 0; b < 8; b++) {

          if (i == 0 && b == 0) {
            continue;
          }

          if (enum_responses[i] == 0xff) {
            continue;
          }

          if (bitRead(enum_responses[i], b) == 0) {
            // free_id = ((i * 16) + b);
            free_id = ((i * 8) + b);

            if (proxy_idx == -1) {
              selected_id = free_id;
              ++proxy_idx;
            } else if (proxy_idx < MAX_NET_PEERS) {
              proxy_canids[proxy_idx] = selected_id;
              ++proxy_idx;
            } else if (proxy_idx == MAX_NET_PEERS) {
              goto enum_done;
            }
          }
        }
      }

enum_done:

      VLOG("cbus_task: selected free CANID = %d", selected_id);
      config_data.CANID = selected_id;
      save_config();

      char ptmp[8], pbuff[32];

      for (byte i = 0; i < MAX_NET_PEERS; i++) {
        sprintf(ptmp, "%d ", proxy_canids[i]);
        strcat(pbuff, ptmp);
      }

      VLOG("cbus_task: proxy CANIDs = %s", proxy_canids);

      // send NNACK
      twai_message_t ack;
      ack.identifier = make_can_header();
      ack.data_length_code = 3;
      ack.data[0] = OPC_NNACK;
      ack.data[1] = highByte(config_data.node_number);
      ack.data[2] = lowByte(config_data.node_number);
      send_message_to_queues(&ack);
    }

    //
    /// process incoming CAN messages from either input queue
    //

    QueueSetMemberHandle_t active_queue = xQueueSelectFromSet(queue_set, QUEUE_OP_TIMEOUT);

    if (active_queue != NULL) {

      // get the next message from the active queue
      xQueueReceive(active_queue, &cf, QUEUE_OP_TIMEOUT);

      if (active_queue == cbus_in_queue) {
        // VLOG("cbus_task: message from external CAN bus, %s", format_CAN_frame(&cf));
      } else {
        // VLOG("cbus_task: message from internal task, %s", format_CAN_frame(&cf));
      }

      // display the incoming message
      // VLOG("cbus_task: got CBUS message, %s", format_CAN_frame(&cf));

      //
      /// check for CANID clash from external CAN bus
      //

      if (!enum_in_progress) {
        if (((cf.identifier & 0x7f) == config_data.CANID) && active_queue == cbus_in_queue) {
          VLOG("cbus_task: my CANID %d clashes with another node, will re-enumerate", (cf.identifier & 0x7f));
          VLOG("cbus_task: %s", format_CAN_frame(&cf));
          enum_required = true;
        }
      }

      //
      /// ignore extended frames from FCU bootloader
      //

      if (cf.flags & TWAI_MSG_FLAG_EXTD) {
        // LOG("cbus_task: ignoring extended frame");
        continue;
      }

      //
      /// set flag due to CAN enumeration request from another CAN node
      //

      if ((cf.flags & TWAI_MSG_FLAG_RTR) && cf.data_length_code == 0) {
        // VLOG("cbus_task: bus enumeration request received from node CANID = %d", (cf.identifier & 0x7f));
        respond_to_enum = true;
        continue;
      }

      //
      /// capture enumeration responses
      //

      if (enum_in_progress && (millis() - enum_start_time < 100) && cf.data_length_code == 0) {
        rcanid = cf.identifier & 0x7f;
        bitSet(enum_responses[(rcanid / 8)], rcanid % 8);
        // VLOG("cbus_task: got enum response from CANID = %d", rcanid);
        continue;
      }

      //
      /// process CBUS opcodes
      //

      if (cf.data_length_code > 0) {

        nn = (cf.data[1] << 8) + cf.data[2];
        byte rcanid = cf.identifier & 0x7f;

        switch (cf.data[0]) {

          case OPC_SNN:
            if (in_transition) {
              in_transition = false;
              ttimer = millis();
              nn = (cf.data[1] << 8) + cf.data[2];
              // VLOG("cbus_task: responding to SNN for nn = %d with NNACK", nn);

              config_data.node_number = nn;
              of.identifier = make_can_header();
              of.data_length_code = 3;
              of.data[0] = OPC_NNACK;
              of.data[1] = highByte(config_data.node_number);
              of.data[2] = lowByte(config_data.node_number);
              send_message_to_queues(&of);

              config_data.cbus_mode = CBUS_MODE_FLIM;
              save_config();
              indicate_cbus_mode(config_data.cbus_mode);
              // LOG("cbus_task: triggering bus enumeration");
              enum_required = true;
            }
            break;

          case OPC_QNN:
            if (config_data.node_number > 0) {
              // LOG("cbus_task: responding to QNN with PNN");
              of.identifier = make_can_header();
              of.data_length_code = 6;
              of.data[0] = OPC_PNN;
              of.data[1] = highByte(config_data.node_number);
              of.data[2] = lowByte(config_data.node_number);
              of.data[3] = node_params[1];
              of.data[4] = node_params[3];
              of.data[5] = node_params[8];
              send_message_to_queues(&of);
            }
            break;

          case OPC_CANID:
            if (nn == config_data.node_number) {
              if (cf.data[3] < 1 || cf.data[3] > 99) {
                send_CMDERR(7);
              } else {
                // VLOG("cbus_task: setting my CANID to %d", cf.data[3]);
                config_data.CANID = cf.data[3];
                save_config();
              }
            }
            break;

          case OPC_ENUM:
            if (nn == config_data.node_number && rcanid != config_data.CANID && !enum_in_progress) {
              // LOG("cbus_task: ENUM request");
              enum_required = true;
            }
            break;

          case OPC_NNLRN:
            if (nn == config_data.node_number) {
              // LOG("entering learn mode");
              bitSet(node_params[8], 5);
              learn_mode = true;
            }
            break;

          case OPC_NNULN:
            if (nn == config_data.node_number) {
              // LOG("leaving learn mode");
              bitClear(node_params[8], 5);
              learn_mode = false;;
            }
            break;

          case OPC_RQNP:
            if (in_transition) {
              // LOG("cbus_task: request for parameters whilst in transition");
              of.identifier = make_can_header();
              of.data_length_code = 8;
              of.data[0] = OPC_PARAMS;         // opcode
              of.data[1] = node_params[1];     // manf code -- MERG
              of.data[2] = node_params[2];     // minor code ver
              of.data[3] = node_params[3];     // module ident
              of.data[4] = node_params[4];     // number of events
              of.data[5] = node_params[5];     // events vars per event
              of.data[6] = node_params[6];     // number of NVs
              of.data[7] = node_params[7];     // major code ver
              send_message_to_queues(&of);
            }
            break;

          case OPC_RQNPN:
            if (nn == config_data.node_number) {
              // VLOG("cbus_task: request for parameter %d", cf.data[3]);
              if (cf.data[3] <= node_params[0]) {
                of.identifier = make_can_header();
                of.data_length_code = 5;
                of.data[0] = OPC_PARAN;
                of.data[1] = highByte(config_data.node_number);
                of.data[2] = lowByte(config_data.node_number);
                of.data[3] = cf.data[3];
                of.data[4] = node_params[cf.data[3]];
                send_message_to_queues(&of);
              } else {
                send_CMDERR(CMDERR_INV_PARAM_IDX);
              }
            }
            break;

          case OPC_RQMN:    // <0x11>
            if (in_transition) {
              // LOG("cbus_task: request for module name");
              // <0xE2><><char1><char2><char3><char4><char5><char6><char7>
              of.identifier = make_can_header();
              of.data_length_code = 8;
              of.data[0] = OPC_NAME;
              memcpy(of.data + 1, node_mname, 7);
              send_message_to_queues(&of);
            }
            break;

          case OPC_NVRD:
            if (nn == config_data.node_number) {
              // LOG("cbus_task: NV read request");
              if (cf.data[3] <= 16) {
                of.identifier = make_can_header();
                of.data_length_code = 5;
                of.data[0] = OPC_NVANS;
                of.data[1] = highByte(config_data.node_number);
                of.data[2] = lowByte(config_data.node_number);
                of.data[3] = cf.data[3];
                of.data[4] = config_data.node_variables[cf.data[3] - 1];  // NVs count from 1 not 0
                send_message_to_queues(&of);
              } else {
                send_CMDERR(CMDERR_INV_NV_IDX);
              }
            }
            break;

          case OPC_NVSET:
            if (nn == config_data.node_number) {
              // LOG("cbus_task: NV write request");
              if (cf.data[3] <= 16) {
                config_data.node_variables[cf.data[3] - 1] = cf.data[4];
                save_config();
                send_WRACK();
              } else {
                send_CMDERR(CMDERR_INV_NV_IDX);
              }
            }
            break;

          case OPC_RQEVN:
            if (nn == config_data.node_number) {
              // LOG("cbus_task: RQENV");
              of.identifier = make_can_header();
              of.data_length_code = 4;
              of.data[0] = OPC_NUMEV;
              of.data[1] = highByte(config_data.node_number);
              of.data[2] = lowByte(config_data.node_number);
              of.data[3] = 0;     // we don't store events
              send_message_to_queues(&of);
            }
            break;

          case OPC_NERD:
            // do nothing - we have no events
            break;

          case OPC_NNEVN:
            if (nn == config_data.node_number) {
              // LOG("cbus_task: NNEVN");
              of.identifier = make_can_header();
              of.data_length_code = 4;
              of.data[0] = OPC_EVNLF;
              of.data[1] = highByte(config_data.node_number);
              of.data[2] = lowByte(config_data.node_number);
              of.data[3] = 0;     // we don't store events
              send_message_to_queues(&of);
            }
            break;

          case OPC_RSTAT:
            // command station proxy replies to this, if it is running
            LOG("cbus_task: command station proxy will respond to this RSTAT request");
            break;

          case OPC_ARST:
            LOG("cbus_task: request to reset");
            break;

          default:
            // opcode not handled
            break;

        }  // switch
      }  // len > 0
    }   // queue set is available

    // periodic tasks
    if (millis() - ptimer > 10000) {
      ptimer = millis();
    }

  }  // for (;;)
}

//
/// initiate enumeration of the CAN bus
//

void enumerate_can_bus(void) {

  twai_message_t cf;

  LOG("cbus_task: enumerate_can_bus: initiating bus enumeration");

  for (byte i = 0; i < 16; i++) {
    enum_responses[i] = 0;
  }

  cf.identifier = make_can_header();
  cf.data_length_code = 0;
  cf.flags |= TWAI_MSG_FLAG_RTR;
  send_message_to_queues(&cf);
  enum_start_time = millis();
  enum_in_progress = true;

  return;
}

//
/// create a CAN message header with CANID and priority bits
//

uint32_t make_can_header() {

  uint32_t t = 0;

  // set the CANID
  t = config_data.CANID;

  // set the CBUS message priority - zeroes equate to higher priority
  // bits 7 and 8 are the minor priority, so 11 = 'low'
  bitSet(t, 7);
  bitSet(t, 8);

  // bits 9 and 10 are the major priority, so 01 = 'medium'
  bitClear(t, 9);
  bitSet(t, 10);

  return t;
}

//
/// send a CAN message to all queues
//

void send_message_to_queues(twai_message_t *cf) {

  // VLOG("cbus_task: send_message_to_queues: %s", format_CAN_frame(cf));

  uint16_t queues = QUEUE_CAN_OUT_FROM_NET | QUEUE_NET_OUT | QUEUE_GC_OUT | QUEUE_WITHROTTLE_IN | QUEUE_CMDPROXY_IN;

  if (!send_message_to_queues(queues, cf, "cbus_task", QUEUE_OP_TIMEOUT_LONG)) {
    LOG("cbus_task: error queuing message");
    PULSE_LED(ERR_IND_LED);
  }

  return;
}

void send_WRACK(void) {

  twai_message_t of;

  LOG("cbus_task: sending WRACK");
  of.identifier = make_can_header();
  of.data_length_code = 3;
  of.data[0] = OPC_WRACK;
  of.data[1] = highByte(config_data.node_number);
  of.data[2] = lowByte(config_data.node_number);
  send_message_to_queues(&of);
  return;
}

void send_CMDERR(byte num) {

  twai_message_t of;

  LOG("cbus_task: sending CMDERR");
  of.identifier = make_can_header();
  of.data_length_code = 4;
  of.data[0] = OPC_CMDERR;
  of.data[1] = highByte(config_data.node_number);
  of.data[2] = lowByte(config_data.node_number);
  of.data[3] = num;
  send_message_to_queues(&of);
  return;
}

void transition_to_flim(void) {

  twai_message_t of;

  if (in_transition) {
    LOG("cbus_task: FLiM transition already in progress");
    return;
  }

  LOG("cbus_task: transition to FLiM initiated");
  of.identifier = make_can_header();
  of.data_length_code = 3;
  of.data[0] = OPC_RQNN;
  of.data[1] = highByte(config_data.node_number);
  of.data[2] = lowByte(config_data.node_number);
  send_message_to_queues(&of);

  indicate_cbus_mode(CBUS_MODE_CHANGING);
  in_transition = true;
  ttimer = millis();
  return;
}

//
/// indicate the current CBUS mode on the SLiM and FLiM LEDs
//

void indicate_cbus_mode(byte mode) {

  led_command_t lc;

  switch (mode) {

    case CBUS_MODE_NONE:
      lc = { SLIM_LED, LED_OFF, 0 };
      xQueueSend(led_cmd_queue, &lc, QUEUE_OP_TIMEOUT_NONE);
      lc = { FLIM_LED, LED_OFF, 0 };
      xQueueSend(led_cmd_queue, &lc, QUEUE_OP_TIMEOUT_NONE);
      break;

    case CBUS_MODE_SLIM:
      lc = { SLIM_LED, LED_ON, 0 };
      xQueueSend(led_cmd_queue, &lc, QUEUE_OP_TIMEOUT_NONE);
      lc = { FLIM_LED, LED_OFF, 0 };
      xQueueSend(led_cmd_queue, &lc, QUEUE_OP_TIMEOUT_NONE);
      break;

    case CBUS_MODE_FLIM:
      lc = { SLIM_LED, LED_OFF, 0 };
      xQueueSend(led_cmd_queue, &lc, QUEUE_OP_TIMEOUT_NONE);
      lc = { FLIM_LED, LED_ON, 0 };
      xQueueSend(led_cmd_queue, &lc, QUEUE_OP_TIMEOUT_NONE);
      break;

    case CBUS_MODE_CHANGING:
      lc = { SLIM_LED, LED_OFF, 0 };
      xQueueSend(led_cmd_queue, &lc, QUEUE_OP_TIMEOUT_NONE);
      lc = { FLIM_LED, LED_BLINK, 0 };
      xQueueSend(led_cmd_queue, &lc, QUEUE_OP_TIMEOUT_NONE);
      break;

    default:
      VLOG("indicate_cbus_mode: unknown mode = %d", mode);
      break;
  }

  return;
}

//
/// send a CBUS message with the battery data for a peer
/// runs in the context of the calling battery task, not the CBUS task
//

void send_cbus_battery_message(int i) {

  twai_message_t cf;
  uint16_t queues;

  VLOG("cbus_task: sending battery messages for peer = %d", i);
  cf.identifier = make_can_header();
  cf.data_length_code = 8;
  cf.data[0] = OPC_ACDAT;

  if (peers[i].battery_mv > 0) {
    cf.data[1] = highByte(config_data.node_number);
    cf.data[2] = lowByte(config_data.node_number);
    cf.data[3] = 0;
    cf.data[4] = highByte(peers[i].battery_mv);
    cf.data[5] = lowByte(peers[i].battery_mv);

    queues = QUEUE_CAN_OUT_FROM_NET | QUEUE_NET_OUT | QUEUE_GC_OUT | QUEUE_WITHROTTLE_IN | QUEUE_CMDPROXY_IN;

    if (!send_message_to_queues(queues, &cf, "cbus_task", QUEUE_OP_TIMEOUT)) {
      LOG("cbus_task: error queuing battery mv message");
      PULSE_LED(ERR_IND_LED);
    }
  }

  if (peers[i].battery_soc > 0) {
    cf.data[1] = highByte(config_data.node_number);
    cf.data[2] = lowByte(config_data.node_number);
    cf.data[3] = 1;
    cf.data[4] = highByte(peers[i].battery_soc);
    cf.data[5] = lowByte(peers[i].battery_soc);

    queues = QUEUE_CAN_OUT_FROM_NET | QUEUE_NET_OUT | QUEUE_GC_OUT | QUEUE_WITHROTTLE_IN | QUEUE_CMDPROXY_IN;

    if (!send_message_to_queues(queues, &cf, "cbus_task", QUEUE_OP_TIMEOUT)) {
      LOG("cbus_task: error queuing battery soc message");
      PULSE_LED(ERR_IND_LED);
    }
  }
  return;
}
