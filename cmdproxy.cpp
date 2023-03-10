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
/// a task to present a DCC++ command station as a CANCMD to CANCABs attached by CBUS
/// it uses the dccppser serial server task to communicate with the DCC++ command station
//

#include <WiFi.h>
#include "defs.h"
#include "cbusdefs.h"

#define MAX_PROXY_SESSIONS 8

// externally defined variables
extern QueueHandle_t logger_in_queue, led_cmd_queue, cmdproxy_queue, CAN_out_from_net_queue, \
net_out_queue, gc_out_queue, wsserver_out_queue, withrottle_queue;
extern config_t config_data;
extern stats_t stats, errors;
extern message_buffer_t msgbuf_proxy_in, msgbuf_proxy_out;
extern SemaphoreHandle_t mtx_proxy;
extern byte num_gc_clients, num_peers, num_wi_clients;

// externally defined functions
uint32_t make_can_header();

// forward function declarations
void send_dccpp_command(char cmd[]);
void send_CAN_message(twai_message_t *cf);
void send_cbus_session_error(byte errnum, twai_message_t *cf);

// session table definition
struct _session {
  bool active;
  byte cbus_session_num;
  unsigned int loco_addr;
  byte speed;
  bool direction;
  bool session_ack;
  byte CANID;
  unsigned long last_activity;
} session_tab[MAX_PROXY_SESSIONS];

// session number maps directly from DCC++ register to MERG DCC session
// we choose the next free number in the table, from 1 to max
// zero/0 means unused and available

// DCC++ only replies to:
// - loco request and speed/direction changes
// - prog track CV programming
// - track power commands
// - track current command

// we also use the following internally to this task
// - status requests


//
/// task main function
//

void cmdproxy_task(void *params) {

  twai_message_t cf, of;
  unsigned long stimer = millis();
  byte num_active, num_dispatched, i, j, ntokens, cs_flags = 0;
  char buffer[32];
  bool got_message = false, track_power_on = false, reusing_dispatched_session = false, session_error = false;
  int treg, taddr, tspeed, tdir;
  unsigned long track_current;

  LOG("cmdproxy_task: task starting");

  if (config_data.role == ROLE_SLAVE || !config_data.cmdproxy_on || !config_data.ser_on) {
    VLOG("cmdproxy_task: this task or DCC++ serial server task not configured to run, suspending task");
    vTaskSuspend(NULL);
  }

  // init session table
  bzero(session_tab, sizeof(session_tab));

  for (i = 0; i < MAX_PROXY_SESSIONS; i++) {
    session_tab[i].cbus_session_num = i + 1;
  }

  // send a status request to the DCC++ command station
  send_dccpp_command((char *)"<s>");

  // main loop

  for (;;) {

    //
    /// check CAB actvity - timed out sessions are dispatched
    //

    for (i = 0; i < MAX_PROXY_SESSIONS; i++) {
      if (session_tab[i].active && (millis() - session_tab[i].last_activity > 60000)) {
        VLOG("cmdproxy_task: session = %d has timed out");
        session_tab[i].active = false;
      }
    }

    //
    /// receive and interpret CBUS messages from CANCABs and translate to DCC++ messages
    //

    if (xQueueReceive(cmdproxy_queue, &cf, QUEUE_OP_TIMEOUT) == pdTRUE) {
      // VLOG("cmdproxy_task: got CAN message %s", format_CAN_frame(&cf));

      switch (cf.data[0]) {

        case OPC_RLOC:        // <0x40><AAAAAAAA><AAAAAAAA>
          taddr = (cf.data[1] << 8) + cf.data[2];
          reusing_dispatched_session = false;
          session_error = false;
          VLOG("cmdproxy_task: got RLOC session request, for loco addr = %d", taddr);

          // attempt to find existing session for this loco address
          for (i = 0; i < MAX_PROXY_SESSIONS; i++) {
            if (session_tab[i].loco_addr == taddr) {
              if (!session_tab[i].active) {
                VLOG("cmdproxy_task: reusing previous session, num = %d, loco addr = %d", i, taddr);
                reusing_dispatched_session = false;
              } else {
                VLOG("cmdproxy_task: loco address = %d is already in use at slot = %d by CANID = %d", taddr, i, session_tab[i].CANID);
                send_cbus_session_error(2, &cf);      // loco taken error
                session_error = true;
              }
              break;
            }
          }

          // don't continue if loco taken error
          if (session_error) {
            LOG("cmdproxy_task: bailing out due to loco taken error");
            continue;
          }

          // otherwise, find a free, unused slot
          if (i == MAX_PROXY_SESSIONS) {
            LOG("cmdproxy_task: looking for free session");
            for (i = 0; i < MAX_PROXY_SESSIONS; i++) {
              if (!session_tab[i].active && session_tab[i].loco_addr == 0) {
                VLOG("cmdproxy_task: found free, unused session slot = %d", i);
                break;
              }
            }
          }

          // finally, find a free slot with a dispatched loco
          if (i == MAX_PROXY_SESSIONS) {
            LOG("cmdproxy_task: looking for free session with dispatched loco");
            for (i = 0; i < MAX_PROXY_SESSIONS; i++) {
              if (!session_tab[i].active && session_tab[i].loco_addr > 0) {
                VLOG("cmdproxy_task: found free slot = %d, with dispatched loco add = %d", i, session_tab[i].loco_addr);
                VLOG("cmdproxy_task: stopping loco and clearing this session first");
                reusing_dispatched_session = true;
                // construct DCC++ command, <t REGISTER CAB SPEED DIRECTION>
                snprintf(buffer, sizeof(buffer), "<t %d %d %d %d>", i + 1, session_tab[i].loco_addr, 0, session_tab[i].direction);
                send_dccpp_command(buffer);
              }
            }
          }

          if (i == MAX_PROXY_SESSIONS) {
            // no free sessions
            LOG("cmdproxy_task: error, no free session slots, will send OPC_ERR");
            send_cbus_session_error(1, &cf);       // no slots/session error
            session_error = true;
          }

          // don't continue if no free sessions error
          if (session_error) {
            LOG("cmdproxy_task: bailing out due to no free sessions error");
            continue;
          }

          /// we have a valid session for this command

          if (i < MAX_PROXY_SESSIONS) {

            // update/refresh session slot data
            session_tab[i].active = true;
            session_tab[i].loco_addr = taddr;
            session_tab[i].session_ack = false;
            session_tab[i].CANID = (cf.identifier & 0x7f);
            session_tab[i].last_activity = millis();

            // keep speed and dir from previously dispatched loco
            if (!reusing_dispatched_session) {
              session_tab[i].speed = 0;
              session_tab[i].direction = DCC_DIR_FWD;
            }

            VLOG("cmdproxy_task: allocated session num = %d for loco addr = %d", session_tab[i].cbus_session_num, session_tab[i].loco_addr);

            // construct DCC++ command, <t REGISTER CAB SPEED DIRECTION>
            snprintf(buffer, sizeof(buffer), "<t %d %d %d %d>", session_tab[i].cbus_session_num, session_tab[i].loco_addr, session_tab[i].speed, session_tab[i].direction);
            send_dccpp_command(buffer);

            /// a PLOC message will be sent once the DCC++ command station replies to the <t ...> message with a <T ...>
          }

          break;

        case OPC_GLOC:        // <0x61><AddrH><AddrL><Flags>
          VLOG("cmdproxy_task: got GLOC for session = %d", cf.data[1]);

          // not currently supported so send an error
          send_cbus_session_error(1, &cf);       // no slots/session error
          break;

        case OPC_KLOC:        // <0x21><Session>
          VLOG("cmdproxy_task: got KLOC to release session = %d", cf.data[1]);
          i = cf.data[1] - 1;

          if (i < MAX_PROXY_SESSIONS && session_tab[i].active) {
            session_tab[i].active = false;
            VLOG("cmdproxy_task: cleared session slot = %d", i);

            if (session_tab[i].loco_addr > 0) {
              VLOG("cmdproxy_task: loco = %d now dispatched at speed = %d", session_tab[i].loco_addr, session_tab[i].speed);
            }
          }

          if (i == MAX_PROXY_SESSIONS) {
            VLOG("cmdproxy_task: session = %d out of range", cf.data[1]);
            // send error ??
          }

          break;

        case OPC_ALOC:          // <0x43><Session><AllocCode>
          VLOG("cmdproxy_task: got ALOC to for session = %d, activity code = %d", cf.data[1], cf.data[2]);
          LOG("cmdproxy_task: ALOC not currently supported, command ignored");
          break;

        case OPC_DKEEP:
          VLOG("cmd_proxy: got DKEEP keepalive for session = %d", cf.data[1]);
          if (session_tab[cf.data[1] - 1].active) {
            session_tab[cf.data[1] - 1].last_activity = millis();
          }
          break;

        case OPC_DSPD:          // <0x47><Session><Speed/Dir>
          VLOG("cmdproxy_task: got DSPD for session = %d, speed/dir = %d", cf.data[1], cf.data[2]);
          i = cf.data[1] - 1;

          if (i > MAX_PROXY_SESSIONS) {
            VLOG("cmdproxy_task: error: DSPD opcode, session = %d out of range", cf.data[1]);
            send_cbus_session_error(1, &cf);     // no slots/session error
          } else {

            if (session_tab[i].CANID != (cf.identifier & 0x7f)) {
              VLOG("cmdproxy_task: DSPD: throttle changed CANID from %d to %d", session_tab[i].CANID, (cf.identifier & 0x7f));
            }

            session_tab[i].active = true;
            session_tab[i].speed = cf.data[2] & 0x7f;
            session_tab[i].direction = bitRead(cf.data[2], 7);
            session_tab[i].last_activity = millis();
            session_tab[i].CANID = (cf.identifier & 0x7f);

            VLOG("cmdproxy_task: sending DCC+ command, slot = %d, sess = %d, loco addr = %d, speed = %d, dir = %d", i, session_tab[i].cbus_session_num, \
                 session_tab[i].loco_addr, session_tab[i].speed, session_tab[i].direction);
            snprintf(buffer, sizeof(buffer), "<t %d %d %d %d>", session_tab[i].cbus_session_num, session_tab[i].loco_addr, session_tab[i].speed, \
                     session_tab[i].direction);
            send_dccpp_command(buffer);
          }

          break;

        case OPC_DFUN:        // <0x60><Session><FR><Fn byte>
          VLOG("cmdproxy_task: got DFUN for session = %d, fr = %d, fn = %d", cf.data[1], cf.data[2], cf.data[3]);
          i = cf.data[1] - 1;

          if (i > MAX_PROXY_SESSIONS) {
            VLOG("cmdproxy_task: error: DFUN opcode, session = %d out of rangee", cf.data[1]);
            send_cbus_session_error(1, &cf);     // no slots/session error
          } else {

            if (session_tab[i].CANID != (cf.identifier & 0x7f)) {
              VLOG("cmdproxy_task: DFUN: throttle changed CANID from %d to %d", session_tab[i].CANID, (cf.identifier & 0x7f));
            }

            session_tab[i].active = true;
            session_tab[i].last_activity = millis();
            session_tab[i].CANID = (cf.identifier & 0x7f);

            // <f CAB BYTE1 [BYTE2]>
            VLOG("cmdproxy_task: sending DCC+ command, slot = %d, loco addr = %d", i, session_tab[i].loco_addr);
            snprintf(buffer, sizeof(buffer), "<f %d %d %d>", session_tab[i].loco_addr, cf.data[2], cf.data[3]);
            send_dccpp_command(buffer);
          }

          break;

        case OPC_RSTAT:
          LOG("cmdproxy_task: responding to RSTAT command station opcode");

          bitWrite(cs_flags, 2, track_power_on);           // track power state
          bitSet(cs_flags, 3);                             // bus on
          bitSet(cs_flags, 5);                             // reset done
          bitSet(cs_flags, 6);                             // prog mode is available

          of.identifier = make_can_header();
          of.flags = 0;
          of.data_length_code = 8;
          of.data[0] = OPC_STAT;                                // opcode
          of.data[1] = (config_data.node_number) >> 8;          // nn high
          of.data[2] = (config_data.node_number) & 0xff;        // nn low
          of.data[3] = 0;                                       // cs num
          of.data[4] = cs_flags;                                // flags
          of.data[5] = VER_MAJ;                                 // major rev
          of.data[6] = VER_MIN;                                 // minor rev
          of.data[7] = VER_PATCH;                               // build num
          send_CAN_message(&of);
          break;

        case OPC_RTON:
          LOG("cmdproxy_task: responding to RTON command station opcode");
          track_power_on = true;
          bitWrite(cs_flags, 2, track_power_on);           // track power state
          strcpy(buffer, "<1>");                           // returns <p1>
          send_dccpp_command(buffer);
          break;

        case OPC_RTOF:
          LOG("cmdproxy_task: responding to RTOF command station opcode");
          track_power_on = true;
          bitWrite(cs_flags, 2, track_power_on);           // track power state
          strcpy(buffer, "<0>");                           // returns <p0>
          send_dccpp_command(buffer);
          break;

        default:
          // VLOG("cmdproxy_task: unhandled opcode = %d", cf.data[0]);
          break;

      }   // switch opcode
    }   // got CAN messages

    //
    /// check and receive messages from DCC++ command station message buffer
    //

    if (xSemaphoreTake(mtx_proxy, QUEUE_OP_TIMEOUT) == pdTRUE) {
      if (msgbuf_proxy_in.head != msgbuf_proxy_in.tail) {
        strncpy(buffer, msgbuf_proxy_in.buffer[msgbuf_proxy_in.tail], PROXY_BUF_LEN);
        msgbuf_proxy_in.tail = (msgbuf_proxy_in.tail + 1) % NUM_PROXY_CMDS;
        VLOG("cmdproxy_task: new data from dccppser task = %s", buffer);
        got_message = true;
      }
      xSemaphoreGive(mtx_proxy);
    }

    //
    /// translate and dispatch the message from DCC++
    //

    if (got_message) {
      got_message = false;
      VLOG("cmdproxy_task: translating and dispatching message = %s", buffer);

      switch (buffer[1]) {          // reponse to register/speed/dir request
        case 'T':                   // <T REGISTER SPEED DIRECTION>
          VLOG("cmdproxy_task: got loco register response = |%s|", buffer);
          ntokens = sscanf(buffer + 2, "%d %d %d %d", &treg, &taddr, &tspeed, &tdir);
          VLOG("cmdproxy_task: parsed %d tokens from DCC++ message = |%s| to %h, %d, %h, %h", ntokens, buffer, treg, taddr, tspeed, tdir);
          i = treg - 1;

          if (i < MAX_PROXY_SESSIONS) {

            // update the session data
            session_tab[i].session_ack = true;
            session_tab[i].last_activity = millis();

            // send a PLOC message to the CAB
            of.identifier = make_can_header();
            of.data_length_code = 8;
            of.flags = 0;
            of.data[0] = OPC_PLOC;
            of.data[2] = taddr >> 8;          // loco addr hi
            of.data[3] = taddr & 0xff;        // loco addr lo
            of.data[4] = tspeed;              // speed
            bitWrite(of.data[4], 7, tdir);    // dir
            of.data[5] = 0;                   // Fn1
            of.data[6] = 0;                   // Fn2
            of.data[7] = 0;                   // Fn3
            send_CAN_message(&of);
          } else {
            VLOG("cmdproxy_task: error: session %d is out of range", treg);
          }
          break;

        case 'p':
          if (buffer[2] == '0') {
            LOG("cmdproxy_task: track power is off");
            track_power_on = false;
            of.identifier = make_can_header();
            of.flags = 0;
            of.data_length_code = 1;
            of.data[0] = OPC_TOF;
            send_CAN_message(&of);
          } else if (buffer[2] == '1') {
            LOG("cmdproxy_task: track power is on");
            track_power_on = true;
            of.identifier = make_can_header();
            of.flags = 0;
            of.data_length_code = 1;
            of.data[0] = OPC_TON;
            send_CAN_message(&of);
          }
          break;

        case 'a':
          track_current = atol(&buffer[3]);
          VLOG("cmdproxy_task: DCC++ track current = %d, %d%%", track_current, (track_current * 100) / 1024);
          break;

        case 'i':
          LOG("cmdproxy_task: got device info from DCC++");
          VLOG("cmdproxy_task: %s", buffer);
          break;

        case 'N':
          LOG("cmdproxy_task: got connection info from DCC++");
          VLOG("cmdproxy_task: connection = %s", buffer);
          break;

        case 'f':
          VLOG("cmdproxy_task: DCC++ free memory = %d", atoi(&buffer[3]));
          break;

        default:
          VLOG("cmdproxy_task: unhandled DCC++ command letter = %c, buffer = %s", buffer[1], buffer);
          break;

      }   // switch command char
    }   // got DCC++ message

    //
    /// display stats
    //

    if (millis() - stimer >= 10000) {
      stimer = millis();
      num_active = 0;
      num_dispatched = 0;

      for (j = 0; j < MAX_PROXY_SESSIONS; j++) {
        if (session_tab[j].active || session_tab[j].loco_addr > 0) {
          VLOG("cmdproxy_task: [%d] in use = %d, addr = %d, speed = %d, dir = %d, ack = %d, CANID = %d, since activity = %d", session_tab[j].cbus_session_num, session_tab[j].active, \
               session_tab[j].loco_addr, session_tab[j].speed, session_tab[j].direction, session_tab[j].session_ack, session_tab[j].CANID, millis() - session_tab[j].last_activity);
        }

        if (session_tab[j].active) {
          ++num_active;
        }

        if (!session_tab[j].active && session_tab[j].loco_addr > 0) {
          ++num_dispatched;
        }
      }

      VLOG("cmdproxy_task: num sessions, active = %d, dispatched = %d", num_active, num_dispatched);
    }
  }   // for (;;)
}

//
/// send a command to the DCC++ command station via the serial port server task
//

void send_dccpp_command(char cmd[]) {

  VLOG("cmdproxy_task: send_dccpp_command: sending DCC++ command = %s to proxy task", cmd);

  while (xSemaphoreTake(mtx_proxy, QUEUE_OP_TIMEOUT) != pdTRUE);
  strncpy(msgbuf_proxy_out.buffer[msgbuf_proxy_out.head], cmd, PROXY_BUF_LEN);
  msgbuf_proxy_out.head = (msgbuf_proxy_out.head + 1) % NUM_PROXY_CMDS;
  xSemaphoreGive(mtx_proxy);

  return;
}

//
/// send a CAN message to various queues
//

void send_CAN_message(twai_message_t *frame) {

  VLOG("cmdproxy_task: send_CAN_message: sending CAN message = %s", format_CAN_frame(frame));

  uint16_t queues = QUEUE_CAN_OUT_FROM_NET | QUEUE_NET_OUT | QUEUE_GC_OUT | QUEUE_WITHROTTLE_IN | QUEUE_CBUS_INTERNAL;

  if (!send_message_to_queues(queues, frame, "cmdproxy_task", QUEUE_OP_TIMEOUT_NONE)) {
    LOG("cmdproxy_task: error queuing message");
    PULSE_LED(ERR_IND_LED);
  }

  return;
}

void send_cbus_session_error(byte errnum, twai_message_t *cf) {

  twai_message_t of;

  VLOG("cmdproxy_task: send_cbus_session_error: sending OPC_ERR with errnum = %d", errnum);
  of.identifier = make_can_header();
  of.data_length_code = 4;
  of.flags = 0;
  of.data[0] = OPC_ERR;
  of.data[1] = cf->data[1];
  of.data[2] = cf->data[2];
  of.data[3] = 1;              // no slots/session error
  send_CAN_message(&of);

  return;
}
