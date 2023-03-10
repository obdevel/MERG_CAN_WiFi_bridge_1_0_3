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
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.r

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,e
                   and indicate if changes were made. You may do so in any reasonable manner,
                   but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                  your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                  legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSEs

*/

#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_now.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <EEPROM.h>
#include <Wire.h>
#include <SPIFFS.h>

#include "driver/rtc_io.h"

#include "defs.h"
#include "cbusdefs.h"
#include "gpio.h"

// object definitions
WebServer webserver(80);
MCP23008 mcp;

// global variables
int num_peers = 0;
int channel;
signed int CANCMD_session_num = -1;
bool do_restart = false;
bool switches_present = false, display_present = false;
bool master_found_and_paired = false;
bool slave_received_first_message = false;
bool boost_enable_on = false;
byte slave_canid = 0;
char mdnsname[32];
uint8_t mac_master[6] = {0, 0, 0, 0, 0, 0};
unsigned long slave_last_net_msg_received_time = 0UL, slave_last_can_frame_received_time = 0UL;
config_t config_data;
peer_state_t peers[MAX_NET_PEERS];
stats_t stats, errors;

// variables declared in other source files
extern byte num_gc_clients, num_wi_clients, node;
extern bool wsserver_running;
extern byte proxy_canids[MAX_NET_PEERS];

// forward function declarations
void IRAM_ATTR touch_callback(void);

// task functions
void CAN_task(void *params);
void net_send_task(void *params);
void gc_task(void *params);
void led_task(void *params);
void logger_task(void *params);
void battery_monitor_task(void *params);
void display_task(void *params);
void withrottle_task(void *params);
void wsserver_task(void *params);
void dccppser_task(void *params);
void cmdproxy_task(void *params);
void cbus_task(void *params);

// list of tasks to create and monitor
// tskNO_AFFINITY = can run on either core

task_info_t task_list[] = {
  { led_task, "LED task", 1500, 0, 0, 13, NULL, true, tskNO_AFFINITY },
  { logger_task, "Logger task", 2500, 0, 0, 13, NULL, true, 1 },
  { CAN_task, "CAN task", 2500, 0, 0, 15, NULL, true, 0 },
  { net_send_task, "Net send task", 2500, 0, 0, 15, NULL, true, 1 },
  { gc_task, "GC task", 3000, 0, 0, 15, NULL, true, 1 },
  { battery_monitor_task, "Battery monitor task", 1500, 0, 0, 12, NULL, true, 1 },
  { display_task, "Display task", 1500, 0, 0, 10, NULL, true, tskNO_AFFINITY },
  { withrottle_task, "Withrottle task", 2500, 0, 0, 13, NULL, true, 1 },
  { wsserver_task, "Websockets task", 3000, 0, 0, 10, NULL, true, tskNO_AFFINITY },
  { dccppser_task, "DCC++ serial server task", 2500, 0, 0, 13, NULL, true, 1 },
  { cmdproxy_task, "CANCMD proxy task", 2500, 0, 0, 12, NULL, true, 1 },
  { cbus_task, "CBUS task", 2500, 0, 0, 13, NULL, true, 1 }
};

// queue handles
QueueHandle_t CAN_out_from_net_queue, CAN_out_from_GC_queue, net_out_queue, gc_out_queue, \
led_cmd_queue, logger_in_queue, battery_monitor_queue, net_to_net_queue, gc_to_gc_queue, \
withrottle_queue, CAN_out_from_withrottle_queue, wsserver_out_queue, cmdproxy_queue, cbus_in_queue, cbus_internal;
QueueHandle_t queues[14];

queue_t queue_tab[] = {
  { "Logger", logger_in_queue, 50, sizeof(log_message_t) },
  { "LED", led_cmd_queue, 10, sizeof(led_command_t) },
  { "CAN from net", CAN_out_from_net_queue, 200, sizeof(twai_message_t) },
  { "CAN from GC", CAN_out_from_GC_queue, 200, sizeof(twai_message_t) },
  { "CAN from Withrottle", CAN_out_from_withrottle_queue, 50, sizeof(twai_message_t) },
  { "ESP-NOW out", net_out_queue, 200, sizeof(twai_message_t) },
  { "Net to net", net_to_net_queue, 100, sizeof(wrapped_frame_t) },
  { "GC out", gc_out_queue, 200, sizeof(twai_message_t) },
  { "GC to GC", gc_to_gc_queue, 200, sizeof(wrapped_gc_t) },
  { "Withrottle", withrottle_queue, 50, sizeof(twai_message_t) },
  { "Battery monitor", battery_monitor_queue, 20, 16 },
  { "Websocket", wsserver_out_queue, 20, sizeof(twai_message_t) },
  { "CMD proxy", cmdproxy_queue, 50, sizeof(twai_message_t) },
  { "CBUS ext", cbus_in_queue, 200, sizeof(twai_message_t) },
  { "CBUS int", cbus_internal, 50, sizeof(twai_message_t) }
};

can_status_desc_t can_status_desc[4] = {
  { TWAI_STATE_STOPPED, "TWAI_STATE_STOPPED" },
  { TWAI_STATE_RUNNING, "TWAI_STATE_RUNNING" },
  { TWAI_STATE_BUS_OFF, "TWAI_STATE_BUS_OFF" },
  { TWAI_STATE_RECOVERING, "TWAI_STATE_RECOVERING" }
};

//
/// this function is called by ESP-NOW with the delivery status of each message sent to a peer
/// note that it runs in the wifi task context, so we need to be careful not to block
/// we just record stats, so that the master can eject unreponsive slaves
//

void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {

  // VLOG("on_data_sent, MAC = %s, status = %d", mac_to_char(mac_addr), status);

  if (status == ESP_NOW_SEND_SUCCESS) {

    if (config_data.role == ROLE_MASTER) {
      peer_record_op(mac_addr, PEER_DECR_ERR);
      peer_record_op(mac_addr, PEER_INCR_TX);
    }

    stats.net_tx++;
    PULSE_LED(NET_ACT_LED);

  } else {

    // VLOG("on_data_sent: send failure to peer = %s", mac_to_char(mac_addr));

    if (config_data.role == ROLE_MASTER) {
      peer_record_op(mac_addr, PEER_INCR_ERR);
    }

    ++errors.net_tx;
    PULSE_LED(ERR_IND_LED);
  }

  return;
}

//
/// this function is called by ESP-NOW when data has been received from an ESP-NOW peer
/// note that it runs in the wifi task context, so we need to be careful not to block for long, if at all
/// ensuring that queues are never full should ensure that writers don't block
//

void on_data_rcvd(const uint8_t *mac_addr, const uint8_t *data, int data_len) {

  twai_message_t *frame;
  wrapped_frame_t wframe;
  byte tbuff[8];

  // check length of received data is that of a CAN frame
  if (data_len != sizeof(twai_message_t)) {
    VLOG("on_data_received: bytes expected = %d, got = %d", sizeof(twai_message_t), data_len);
    ++errors.net_rx;
    PULSE_LED(ERR_IND_LED);
    return;
  }

  // update stats
  ++stats.net_rx;

  // pulse the network activity LED
  PULSE_LED(NET_ACT_LED);

  // update the slave network activity timer
  if (config_data.role == ROLE_SLAVE) {
    slave_last_net_msg_received_time = millis();
  }

  //
  /// master registers this peer if it's the first time we've heard from it
  /// a password may be required
  /// slaves have only one peer - the master node - so have no need to manage multiple peers
  //

  if (config_data.role == ROLE_MASTER) {
    if (!esp_now_is_peer_exist(mac_addr)) {

      // the first message from an unknown connecting slave should be the network password
      // if we are using this functionality

      VLOG("on_data_rcvd: received message from unknown peer %s", mac_to_char(mac_addr));

      if (config_data.use_network_password) {
        if (memcmp(data, "PW", 2) == 0) {
          VLOG("on_data_rcvd: received message is a network password");

          char tmppwd[16];
          memcpy(tmppwd, (void *)&data[2], 14);
          tmppwd[15] = 0;

          VLOG("on_data_rcvd: checking password, |%s| against |%s|", tmppwd, config_data.network_password);

          // compare passwords
          if (strncmp(tmppwd, config_data.network_password, strlen(config_data.network_password)) != 0) {
            // passwords do not match - don't add the slave as a peer and don't respond
            VLOG("on_data_rcvd: incorrect password, peer will not be paired");
            return;
          } else {
            VLOG("on_data_rcvd: password matched ok, peer will be paired");
          }
        } else {
          VLOG("on_data_rcvd: sender is unknown and message is not a password");
          return;
        }
      }

      // at this point, either the new slave has passed the password check
      // or we are not using the layout password facility

      // add slave node as a peer
      VLOG("on_data_rcvd: registering new peer");

      // new peer info record
      esp_now_peer_info_t peer;
      bzero(&peer, sizeof(esp_now_peer_info_t));
      memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
      // peer.ifidx = ESP_IF_WIFI_AP;
      peer.ifidx = WIFI_IF_AP;
      peer.channel = channel;
      peer.encrypt = false;

      // register the slave
      esp_err_t ret = esp_now_add_peer(&peer);

      if (ret == ESP_OK) {
        VLOG("on_data_rcvd: new satellite has been paired ok");
        // create a peer state record for collecting stats, etc
        add_peer(mac_addr);
      } else {
        VLOG("on_data_rcvd: error pairing satellite, err = %d", ret);
        log_esp_now_err(ret);
      }
    }   // is already peered

    peer_record_op(mac_addr, PEER_INCR_RX);
    peer_record_op(mac_addr, PEER_DECR_ERR);

  }  // is master

  // slave has received the first message from the master, which confirms it has joined the network
  // so, no longer necessary to send the network password

  if (config_data.role == ROLE_SLAVE && !slave_received_first_message) {
    slave_received_first_message = true;
    VLOG("on_data_rcvd: satellite has received first message from master; network joined ok");
  }

  // is this a heartbeat message or a superfluous password message ?
  // we do nothing more than increment the stats counters above

  if (memcmp(data, "HB", 2) == 0 || memcmp(data, "PW", 2) == 0) {
    VLOG("on_data_rcvd: message: %c%c", data[0], data[1]);
    return;
  }

  // capture messages from the slave's battery monitor task

  if (config_data.role == ROLE_MASTER) {
    // battery voltage measurement -> 0-9999mV
    if (memcmp("MV", data, 2) == 0) {
      memcpy(tbuff, &data[2], 4);
      tbuff[4] = 0;
      peer_record_op(mac_addr, PEER_SET_BATT_MV, atoi((char *)&tbuff[2]));
      VLOG("on_data_received: satellite battery voltage = %d", atoi((char *)&tbuff[2]));
      return;
    }

    // battery state_of_charge measurement -> 0-100%
    if (memcmp("SO", data, 2) == 0) {
      memcpy(tbuff, &data[2], 3);
      tbuff[3] = 0;
      peer_record_op(mac_addr, PEER_SET_BATT_SOC, atoi((char *)&tbuff[2]));
      VLOG("on_data_received: satellite battery soc = %d", atoi((char *)&tbuff[2]));
      return;
    }
  }

  if (config_data.role == ROLE_SLAVE) {
    // proxy CANID
    if (memcmp("CA", data, 2) == 0) {
      slave_canid = atoi((char *)&tbuff[2]);
      VLOG("on_data_received: satellite CANID = %d", slave_canid);
    }
  }

  //
  /// otherwise, this data is a CAN message
  //

  // get a pointer to the frame data
  frame = (twai_message_t *)data;

  // VLOG("on_data_rcvd: data received from = %s, len = %d", mac_to_char(mac_addr), data_len);
  // LOG(format_CAN_frame(&frame));

  //
  /// capture info about attached CAB and CANCMD session
  //

  if (config_data.role == ROLE_SLAVE) {
    // capture CANCMD session number
    if (frame->data[0] == OPC_PLOC ) {
      CANCMD_session_num = frame->data[1];
      // VLOG("on_data_received: slave, CANCMD session = %d", CANCMD_session_num);
    }

    if (frame->data[0] == OPC_ERR) {
      CANCMD_session_num = -1;
      // LOG("on_data_received: slave, CANCMD session error");
    }
  } else {
    peer_record_op(mac_addr, PEER_SET_CANID, frame->identifier & 0x7f);
  }

  //
  /// all nodes forward the frame to CAN output queue for transmission to local CAN bus
  /// this is either the attached CAB or the layout
  //

  if (!send_message_to_queues(QUEUE_CAN_OUT_FROM_NET, frame, "on_data_rcvd", QUEUE_OP_TIMEOUT_SHORT)) {
    LOG("on_data_rcvd: error queuing message");
    PULSE_LED(ERR_IND_LED);
  }

  //
  /// master node forwards data on to other task queues
  //

  if (config_data.role == ROLE_MASTER) {

    //
    /// master forwards wrapped frame to net-to-net queue for onward transmission to other net slaves
    /// the wrapper includes the source MAC address
    /// and ensures the frame is not reflected back to the sending slave, as would otherwise happen
    /// does not apply to slaves, which only have one peer, the master itself
    //

    if (num_peers > 1) {
      memcpy(&wframe.mac_addr, mac_addr, sizeof(uint8_t[6]));
      memcpy(&wframe.frame, frame, sizeof(twai_message_t));

      if (!send_message_to_queues(QUEUE_NET_TO_NET, &wframe, "on_data_rcvd", QUEUE_OP_TIMEOUT_SHORT)) {
        LOG("on_data_rcvd: error queuing message");
        PULSE_LED(ERR_IND_LED);
      }
    }

    //
    /// master forwards frame to other task queues if tasks are configured to run
    //

    uint16_t queues = QUEUE_GC_OUT | QUEUE_WITHROTTLE_IN | QUEUE_CMDPROXY_IN | QUEUE_CBUS_EXTERNAL;

    if (!send_message_to_queues(queues, frame, "on_data_rcvd", QUEUE_OP_TIMEOUT_SHORT)) {
      LOG("on_data_rcvd: error queuing message");
      PULSE_LED(ERR_IND_LED);
    }
  }

  return;
}

//
/// create a soft WiFi access point (AP) to advertise our MAC address, and to allow initial config connections
/// the channel number must have previously been set to that of the external WiFi AP, the master's soft AP, or
/// the default value
//

void create_soft_AP(int channel) {

  /// e.g. Master0:xx:xx:xx:xx:xx:xx or Satellite0-0:xx:xx:xx:xx:xx:xx

  String Prefix;

  if (config_data.role == ROLE_MASTER) {
    Prefix = "Master" + String(config_data.network_number) + ":";
  } else {
    Prefix = "Satellite" + String(config_data.network_number) + "-" + String(config_data.slave_number) + ":";
  }

  String Mac = WiFi.macAddress();
  String SSID = Prefix + Mac;

  VLOG("create_soft_AP: creating soft AP on channel = %d", channel);

  WiFi.softAPdisconnect(true);
  bool result = WiFi.softAP(SSID.c_str(), config_data.softap_password, channel, 0);

  if (!result) {
    VLOG("create_soft_AP: error creating AP, err = %d", result);
  } else {
    VLOG("create_soft_AP: AP created ok, SSID = %s, on channel = %d", WiFi.softAPmacAddress().c_str(), channel);
  }

  return;
}

//
/// slave attempts to locate and pair with the master node for this network
/// it looks for the master's soft AP, the name of which contains the master's MAC address
/// returns true if the master has been successfully found and paired
//

bool find_and_pair_master(void) {

  byte mac[6];
  signed int apchannel = -1;
  bool peer_ok = false;
  esp_now_peer_info_t master = {};

  if (master_found_and_paired) {
    LOG("find_and_pair_master: master already found and paired");
    return true;
  }

  // construct the SSID prefix we are looking for
  String prefix = "Master" + String(config_data.network_number);

  VLOG("find_and_pair_master: satellite looking for master node with prefix = %s", prefix.c_str());

  // scan for the visible WiFi APs
  // int16_t WiFiScanClass::scanNetworks(bool async, bool show_hidden, bool passive, uint32_t max_ms_per_chan, uint8_t channel)
  int16_t num_aps = WiFi.scanNetworks(false, true, false, (uint32_t)WIFI_SCAN_MS);   // 350mS/channel by default

  // return if scan error
  switch (num_aps) {
    case WIFI_SCAN_RUNNING:
      LOG("find_and_pair_master: WiFi scan is already running");
      WiFi.scanDelete();
      return false;
      break;
    case WIFI_SCAN_FAILED:
      LOG("find_and_pair_master: WiFi scan failed");
      WiFi.scanDelete();
      return false;
      break;
    default:
      break;
  }

  VLOG("find_and_pair_master: number of APs found = %d", num_aps);

  // iterate through discovered APs
  for (int i = 0 ; i < num_aps; i++) {

    String SSID = WiFi.SSID(i);

    // if the AP name begins with the word Master and the configured network number, e.g. Master0
    if (SSID.indexOf(prefix) == 0) {
      String BSSIDstr = WiFi.BSSIDstr(i);
      char *SSID_mac = strchr(WiFi.SSID(i).c_str(), ':') + 1;
      apchannel = WiFi.channel(i);

      VLOG("find_and_pair_master: found master AP for network = %d, channel = %d", config_data.network_number, channel);
      VLOG("SSID = %s", SSID.c_str());
      VLOG("SSID MAC = %s", SSID_mac);
      VLOG("BSSID = %s", BSSIDstr.c_str());
      VLOG("channel = %d", apchannel);

      bzero(&master, sizeof(esp_now_peer_info_t));

      // VLOG("find_and_pair_master: scanning BSSID for MAC address");
      sscanf(BSSIDstr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);

      for (int j = 0; j < 6; j++) {
        master.peer_addr[j] = mac[j];
      }

      // check whether master is already peered, otherwise add it
      if (esp_now_is_peer_exist(master.peer_addr)) {
        LOG("find_and_pair_master: master node already peered");
        peer_ok = true;
      } else {
        // master.ifidx = ESP_IF_WIFI_AP;      // use the AP interface as the STA interface may not be up
        master.ifidx = WIFI_IF_AP;      // use the AP interface as the STA interface may not be up
        master.channel = apchannel;
        master.encrypt = 0;

        esp_err_t ret = esp_now_add_peer(&master);

        if (ret == ESP_OK) {
          VLOG("find_and_pair_master: peered with master node = %s", mac_to_char(master.peer_addr));

          // save master MAC address
          memcpy(&mac_master, &master.peer_addr, sizeof(uint8_t[6]));

          // set return value
          peer_ok = true;

        } else {
          VLOG("find_and_pair_master: error peering master node, ret = %d", ret);
          log_esp_now_err(ret);
        }
      }     // peer already exists

      break;
    }     // does prefix match
  }     // for each AP

  WiFi.scanDelete();
  return peer_ok;
}

//
/// return the channel number of a WiFi access point, either a WiFi AP or the master's soft AP
/// return -1 if the AP is not found
//

signed int get_wifi_ap_channel(char *ssid_to_match) {

  String SSID, BSSIDstr;
  signed int apchannel = -1;

  // initiate a scan of visible wifi access points
  VLOG("get_wifi_ap_channel: scanning for AP SSID = %s", ssid_to_match);

  // int16_t WiFiScanClass::scanNetworks(bool async, bool show_hidden, bool passive, uint32_t max_ms_per_chan, uint8_t channel)
  int16_t num_aps = WiFi.scanNetworks(false, true, false, (uint32_t)WIFI_SCAN_MS);   // 350mS/channel by default

  switch (num_aps) {
    case WIFI_SCAN_RUNNING:
      LOG("get_wifi_ap_channel: WiFi scan is already running");
      WiFi.scanDelete();
      return -1;
      break;
    case WIFI_SCAN_FAILED:
      LOG("get_wifi_ap_channel: WiFi scan failed");
      WiFi.scanDelete();
      return -1;
      break;
    default:
      break;
  }

  VLOG("get_wifi_ap_channel: number of APs found = %d", num_aps);

  // iterate through the list of discovered APs
  for (int i = 0 ; i < num_aps; i++) {

    SSID = WiFi.SSID(i);
    BSSIDstr = WiFi.BSSIDstr(i);

    // VLOG("get_wifi_ap_channel: SSID = %s", SSID.c_str());
    // VLOG("get_wifi_ap_channel: BSSID = %s", BSSIDstr.c_str());
    // VLOG("get_wifi_ap_channel: channel = %d", channel);

    // if SSID matches what we're looking for
    if (strncmp(SSID.c_str(), ssid_to_match, strlen(ssid_to_match)) == 0) {
      apchannel = WiFi.channel(i);
      VLOG("get_wifi_ap_channel: channel for %s = %d", SSID.c_str(), apchannel);
      break;
    }
  }

  if (apchannel == -1) {
    LOG("get_wifi_ap_channel: AP not found");
  }

  WiFi.scanDelete();
  return apchannel;
}

//
/// task to manage communications to and from the local CAN bus
/// either a CAB for a slave node, or the layout for the master node
/// also distributes CAN messages to other optional tasks
//
/// this task does not interpret the CBUS protocol at all
/// this is done by the CBUS task which receives CAN messaages from this task
//

void CAN_task(void *params) {

  bool got_slave_canid = false;
  unsigned long stimer = millis();
  twai_message_t rx_frame, tx_frame;
  esp_err_t cret;

  LOG("CAN_task: task starting");

  for (;;) {

    // display slave CANID once it has been learnt
    if (config_data.role == ROLE_SLAVE && !got_slave_canid && slave_canid > 0) {
      got_slave_canid = true;
      VLOG("CAN_task: satellite: got CAB identity, CANID = %d", slave_canid);
    }

    //
    /// slave local CAN bus inactivity timeout
    //

    if (config_data.role == ROLE_SLAVE && config_data.sleep_delay_mins > 0 && (millis() - slave_last_can_frame_received_time > (config_data.sleep_delay_mins * 60000))) {
      LOG("CAN_task: slave: local CAN bus inactivity timeout - sleep in 5 seconds");
      device_sleep();
    }

    //
    /// get next incoming frame from the local CAN bus
    //

    cret = twai_receive(&rx_frame, QUEUE_OP_TIMEOUT);

    switch (cret) {

      case ESP_ERR_TIMEOUT:
        // no frame waiting
        break;

      case ESP_OK:
        // VLOG("CAN_task: local CAN frame received, %s", format_CAN_frame(&rx_frame));

        // capture CANID and DCC session data from outgoing messages from cab
        if (config_data.role == ROLE_SLAVE) {

          if (slave_canid != (rx_frame.identifier & 0x7f)) {
            VLOG("CAN_task: satellite CANID changed");
            slave_canid = rx_frame.identifier & 0x7f;
            got_slave_canid = false;
          }

          // update slave inactivity timer - DKEEP keepalives do not count for this purpose, otherwise we would never time out
          if (rx_frame.data[0] != OPC_DKEEP) {
            slave_last_can_frame_received_time = millis();
          }

          // capture CANCMD session number
          switch (rx_frame.data[0]) {
            case OPC_DSPD:
            case OPC_DFUN:
            case OPC_DFNON:
            case OPC_DFNOF:
            case OPC_PCON:
              // etc ??
              CANCMD_session_num = rx_frame.data[1];
              VLOG("CAN_task: satellite, CANCMD session = %d", CANCMD_session_num);
              break;

            case OPC_KLOC:
              CANCMD_session_num = -1;
              LOG("CAN_task: satellite, CANCMD session num cleared");
              break;
          }
        }

        // all nodes forward the frame to net output queue for onward transmission to peer(s)
        if (!send_message_to_queues(QUEUE_NET_OUT, &rx_frame, "CAN_task", QUEUE_OP_TIMEOUT_SHORT)) {
          LOG("CAN_task: error queuing message");
          PULSE_LED(ERR_IND_LED);
        }

        // master forwards incoming CAN message to other task queues
        if (config_data.role == ROLE_MASTER) {
          uint16_t queues = QUEUE_GC_OUT | QUEUE_WITHROTTLE_IN | QUEUE_CMDPROXY_IN | QUEUE_CBUS_EXTERNAL;

          if (!send_message_to_queues(queues, &rx_frame, "CAN_task", QUEUE_OP_TIMEOUT_SHORT)) {
            LOG("CAN_task: error queuing message");
            PULSE_LED(ERR_IND_LED);
          }
        }

        // pulse the activity LED
        PULSE_LED(CAN_ACT_LED);

        // increment stats
        ++stats.can_rx;
        break;

      default:
        LOG("CAN_task: error receiving frame from local bus");
        log_esp_now_err(cret);
        PULSE_LED(ERR_IND_LED);
        break;

    }   // switch can receive result

    //
    /// process the output queues, for messages to be sent to the local CAN bus
    ///

    //
    /// get the next outgoing frame from the ESP-NOW receive callback and other tasks
    //

    if (xQueueReceive(CAN_out_from_net_queue, &tx_frame, QUEUE_OP_TIMEOUT_SHORT) == pdTRUE) {
      // VLOG("CAN_task: received frame from net output queue: %s", format_CAN_frame(&tx_frame));

      // forward frame to local CAN bus
      cret = twai_transmit(&tx_frame, QUEUE_OP_TIMEOUT);

      if (cret == ESP_OK) {
        // VLOG("CAN_task: forwarded frame to local CAN bus, ret = %d, %s", cret, format_CAN_frame(&tx_frame));
        PULSE_LED(CAN_ACT_LED);
        ++stats.can_tx;
      } else {
        LOG("CAN_task: error writing CAN frame fron net task to driver queue");
        log_esp_now_err(cret);
        PULSE_LED(ERR_IND_LED);
        ++errors.can_tx;
      }
    }

    //
    /// get next outgoing CAN frame from GC task
    //

    if (config_data.role == ROLE_MASTER && config_data.gc_server_on) {
      if (xQueueReceive(CAN_out_from_GC_queue, &tx_frame, QUEUE_OP_TIMEOUT_SHORT) == pdTRUE) {
        // LOG("CAN_task: output frame received from GC task");

        // forward frame to local CAN bus
        cret = twai_transmit(&tx_frame, QUEUE_OP_TIMEOUT);

        if (cret == ESP_OK) {
          // VLOG("CAN_task: forwarded frame to local CAN bus, ret = %d, %s", cret, format_CAN_frame(&tx_frame));
          // VLOG("CAN_task: source CANID = %d", tx_frame.identifier & 0x7);
          PULSE_LED(CAN_ACT_LED);
          ++stats.can_tx;
        } else {
          LOG("CAN_task: error writing CAN frame from GC task to driver queue");
          log_esp_now_err(cret);
          PULSE_LED(ERR_IND_LED);
          ++errors.can_tx;
        }
      }
    }

    //
    /// get next outgoing CAN frame from withrottle task
    //

    if (config_data.role == ROLE_MASTER && config_data.withrottle_on) {
      if (xQueueReceive(CAN_out_from_withrottle_queue, &tx_frame, QUEUE_OP_TIMEOUT_NONE) == pdTRUE) {
        // LOG("CAN_task: output frame received from withrottle task");

        // forward frame to local CAN bus
        cret = twai_transmit(&tx_frame, QUEUE_OP_TIMEOUT);

        if (cret == ESP_OK) {
          // VLOG("CAN_task: forwarded frame to CAN driver queue, ret = %d, %s", cret, format_CAN_frame(&tx_frame));
          PULSE_LED(CAN_ACT_LED);
          ++stats.can_tx;
        } else {
          LOG("CAN_task: error writing CAN frame from withrottle task to driver queue");
          log_esp_now_err(cret);
          PULSE_LED(ERR_IND_LED);
          ++errors.can_tx;
        }
      }
    }

    //
    /// read CAN driver alerts and display any error
    //

    uint32_t can_alerts;
    static uint32_t prev_can_alerts = 0;
    esp_err_t ret;

    if (millis() - stimer % 250) {

      ret  = twai_read_alerts(&can_alerts, QUEUE_OP_TIMEOUT_SHORT);

      if (ret == ESP_OK) {

        if (can_alerts != prev_can_alerts) {
          if ((can_alerts == 3 && prev_can_alerts == 2) || (can_alerts == 2 && prev_can_alerts == 3)) {
            // reduce the noise
          } else {
            VLOG("CAN_task: CAN alert status has changed, from %d to %d", prev_can_alerts, can_alerts);
            prev_can_alerts = can_alerts;

            if (can_alerts & TWAI_ALERT_TX_IDLE) {
              LOG("CAN_task: TWAI_ALERT_TX_IDLE");
            }

            if (can_alerts && TWAI_ALERT_BELOW_ERR_WARN) {
              LOG("CAN_task: TWAI_ALERT_BELOW_ERR_WARN");
            }

            if (can_alerts & TWAI_ALERT_TX_SUCCESS) {
              LOG("CAN_task: TWAI_ALERT_TX_SUCCESS");
            }

            if (can_alerts & TWAI_ALERT_ERR_ACTIVE) {
              LOG("CAN_task: TWAI_ALERT_ERR_ACTIVE");
            }

            if (can_alerts & TWAI_ALERT_RECOVERY_IN_PROGRESS) {
              LOG("CAN_task: TWAI_ALERT_RECOVERY_IN_PROGRESS");
            }

            if (can_alerts & TWAI_ALERT_BUS_RECOVERED) {
              LOG("CAN_task: TWAI_ALERT_BUS_RECOVERED");
            }

            if (can_alerts & TWAI_ALERT_ARB_LOST) {
              LOG("CAN_task: TWAI_ALERT_ARB_LOST");
            }

            if (can_alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
              LOG("CAN_task: TWAI_ALERT_ABOVE_ERR_WARN");
            }

            if (can_alerts & TWAI_ALERT_BUS_ERROR) {
              LOG("CAN_task: TWAI_ALERT_BUS_ERROR");
            }

            if (can_alerts & TWAI_ALERT_TX_FAILED) {
              LOG("CAN_task: TWAI_ALERT_TX_FAILED");
            }

            if (can_alerts & TWAI_ALERT_RX_QUEUE_FULL) {
              LOG("CAN_task: TWAI_ALERT_RX_QUEUE_FULL");
            }

            if (can_alerts & TWAI_ALERT_ERR_PASS) {
              LOG("CAN_task: TWAI_ALERT_ERR_PASS");
            }

            if (can_alerts & TWAI_ALERT_BUS_OFF) {
              LOG("CAN_task: TWAI_ALERT_BUS_OFF, initiating recovery");
              ret = twai_initiate_recovery();
              VLOG("CAN_task: bus recovery returns %d", ret);
              log_esp_now_err(ret);
            }
          }
        }

        /// blink error LED on CAN error states

        /*
          led_command_t cmd;

          if ((can_alerts & TWAI_ALERT_BUS_ERROR) || (can_alerts & TWAI_ALERT_TX_FAILED) || (can_alerts & TWAI_ALERT_ARB_LOST)) {
          cmd = { ERR_IND_LED, LED_BLINK, 0 };
          } else {
          cmd = { ERR_IND_LED, LED_OFF, 0 };
          }

          xQueueSend(led_cmd_queue, &cmd, QUEUE_OP_TIMEOUT);
        */

      }
    }

    //
    /// periodically display CAN state and stats
    //

    if (millis() - stimer >= 1000) {

      twai_status_info_t can_status;
      twai_get_status_info(&can_status);
      led_command_t cmd;
      static int scount = 1;

      if (scount == 10) {      // every 10 secs
        VLOG("CAN_task: [%d] %s, txq = %d, rxq = %d, tec = %d, rec = %d, tx fail = %d, rx drop = %d, lost arb = %d, bus error = %d, alert = %d", \
             config_data.CANID, can_status_desc[can_status.state].desc, can_status.msgs_to_tx, can_status.msgs_to_rx, can_status.tx_error_counter, \
             can_status.rx_error_counter, can_status.tx_failed_count, can_status.rx_missed_count, can_status.arb_lost_count, can_status.bus_error_count, can_alerts);
        scount = 1;
      } else {
        ++scount;
      }

      //
      /// attempt driver restart
      //

      if (can_status.state == TWAI_STATE_STOPPED) {
        LOG("CAN_task: state is STOPPED, restarting CAN driver");
        ret = twai_start();
        VLOG("CAN_task: start returns %d", ret);
        log_esp_now_err(ret);

        if (ret == ESP_OK) {
          cmd = { ERR_IND_LED, LED_OFF, 0 };
        } else {
          cmd = { ERR_IND_LED, LED_FAST_BLINK, 0 };
        }
      }

      xQueueSend(led_cmd_queue, &cmd, QUEUE_OP_TIMEOUT);
      stimer = millis();
    }

  }  // for (;;)
}

//
/// task to send messages to connected ESP-NOW peers
/// also sends heartbeat and password messages
//

void net_send_task(void *params) {

  unsigned long ptimer = 0UL, heartbeat_timer = 0UL;
  char hbdata[sizeof(twai_message_t)] = {'H', 'B'};
  char pwdata[sizeof(twai_message_t)] = {'P', 'W'};
  twai_message_t frame;
  wrapped_frame_t wframe;
  esp_now_peer_num_t peer_num;

  LOG("net_send_task: task starting");

  // init peers table
  peer_record_op(NULL, PEER_INIT_ALL);

  // network password message
  memcpy((void *)&pwdata[2], config_data.network_password, strlen(config_data.network_password));

  //
  /// main loop
  //

  for (;;) {

    //
    /// master sends a regular heartbeat message to all slaves
    /// slaves send just initial message to introduce themselves to the master, repeated until the first message is received
    //

    if (millis() - heartbeat_timer >= HBFREQ) {
      heartbeat_timer = millis();

      if (config_data.role == ROLE_SLAVE) {

        // need to restablish contact with master if not heard from recently
        // we may have moved out of range and been unpaired by the master
        // or the master may have restarted

        if (slave_received_first_message && (millis() - slave_last_net_msg_received_time > 10000)) {
          LOG("net_send_task: satellite: no contact from master for 10 secs");
          slave_received_first_message = false;
        }

        // slaves need to find and pair the MAC address of the master for the configured network
        if (!master_found_and_paired) {
          LOG("net_send_task: satellite trying to pair the master node");
          master_found_and_paired = find_and_pair_master();
          VLOG("net_send_task: find_and_pair_master returns %d", master_found_and_paired);
        }

        // slaves need to introduce themselves to the master, until they have received the first message
        // either with the password or a heartbeat message

        if (!slave_received_first_message) {

          // if the network password is being used, slaves must send the network password until
          // they receive the first message from the master, indicating they have been paired successfully

          if (config_data.use_network_password) {

            VLOG("net_send_task: satellite: sending password to join layout network %d", config_data.network_number);
            esp_err_t result = esp_now_send(mac_master, (const uint8_t *)pwdata, sizeof(twai_message_t));

            if (result != ESP_OK) {
              VLOG("net_send_task: satellite: error sending password to master %s", mac_to_char(mac_master));
              log_esp_now_err(result);
              PULSE_LED(ERR_IND_LED);
            } else {
              // LOG("net_send_task: slave: sent password to master");
            }
          } else {

            // if network password is not being used, slaves must send a hearbeat to introduce themselves to the master
            // until they have received the first message

            VLOG("net_send_task: satellite: introducing self to master on network %d", config_data.network_number);
            esp_err_t result = esp_now_send(mac_master, (const uint8_t *)hbdata, sizeof(twai_message_t));

            if (result != ESP_OK) {
              VLOG("net_send_task: satellite: error sending hello to master %s", mac_to_char(mac_master));
              log_esp_now_err(result);
              PULSE_LED(ERR_IND_LED);
            } else {
              LOG("net_send_task: satellite: sent heartbeat to master");
            }
          }
        }

        // slaves do nothing more here once paired and first message has been received

      } else if (config_data.role == ROLE_MASTER) {

        // master sends heartbeat to all peers, if any
        esp_now_get_peer_num(&peer_num);
        num_peers = peer_num.total_num;

        if (num_peers > 0) {

          // MAC address NULL means broadcast to all known peers
          esp_err_t result = esp_now_send(NULL, (const uint8_t *)hbdata, sizeof(hbdata));

          if (result != ESP_OK) {
            LOG("net_send_task: master: error sending hearbeat to peers");
            log_esp_now_err(result);
            PULSE_LED(ERR_IND_LED);
          } else {
            // LOG("net_send_task: master: sent heartbeat to slaves");
            peer_record_op(NULL, PEER_INCR_TX_ALL);
            PULSE_LED(NET_ACT_LED);
          }
        }   // num peers > 0
      }   // is master
    }   // hb timer

    //
    /// get next outgoing net message from output queue
    /// this contains messages destined for all ESP-NOW peers
    //

    if (xQueueReceive(net_out_queue, &frame, QUEUE_OP_TIMEOUT_SHORT) == pdTRUE) {
      // VLOG("net_send_task: forwarding received frame to network");

      // slaves need to find and pair the MAC address of the master for the configured network
      if (config_data.role == ROLE_SLAVE && !master_found_and_paired) {
        LOG("net_send_task: satellite trying to pair the master node");
        master_found_and_paired = find_and_pair_master();
        VLOG("net_send_task: find_and_pair_master returns %d", master_found_and_paired);
      }

      // if slave, there is only one possible peer - the master - so we send to its MAC address explicitly

      if (config_data.role == ROLE_SLAVE) {
        // VLOG("net_send_task: sending frame to network master");

        if (!master_found_and_paired) {
          LOG("net_send_task: the master cannot be paired yet, placing frame back on input queue");

          // place the frame back on the queue, so we can try again shortly
          // it is placed at the front of the queue to preserve message ordering
          if (xQueueSendToFront(net_out_queue, &frame, QUEUE_OP_TIMEOUT) != pdTRUE) {
            LOG("net_send_task: error placing frame back on input queue");
            PULSE_LED(ERR_IND_LED);
          }
        } else {

          esp_err_t result = esp_now_send(mac_master, (const uint8_t *)&frame, sizeof(twai_message_t));

          if (result == ESP_OK) {
            // LOG("net_send_task: sent frame to master");
            PULSE_LED(NET_ACT_LED);
          } else {
            VLOG("net_send_task: error sending frame to master, err = %d", result);
            log_esp_now_err(result);
            PULSE_LED(ERR_IND_LED);
          }
        }   // master paired
      }   // is slave

      // if master, forward to all slaves; sending to NULL MAC address means all currently paired peers
      // we don't retry as there may be multiple slaves and we don't want to send duplicate messages

      if (config_data.role == ROLE_MASTER) {

        // get number of currently paired peers
        esp_now_get_peer_num(&peer_num);
        num_peers = peer_num.total_num;
        // VLOG("net_send_task: master: peers currently paired = %d", peer_num.total_num);

        if (num_peers > 0) {
          esp_err_t result = esp_now_send(NULL, (const uint8_t *)&frame, sizeof(twai_message_t));

          if (result == ESP_OK) {
            // VLOG("net_send_task: master: sent frame to network peers");
            peer_record_op(NULL, PEER_INCR_TX_ALL);
            PULSE_LED(NET_ACT_LED);
          } else {
            VLOG("net_send_task: master: error sending frame to network peers, err = %d", result);
            log_esp_now_err(result);
            PULSE_LED(ERR_IND_LED);
          }
        }   // peers > 0
      }   // is master
    }   // get next message from output queue

    //
    /// master gets next message from net-to-net queue
    /// incoming frames are from network slaves
    /// CAN frames on the queue are wrapped in a struct along with the MAC address of the sending node
    /// this ensures that incoming network messages are not reflected back to the sender
    /// applies only on the master node when there is more than one active slave
    //

    if (config_data.role == ROLE_MASTER) {
      if (xQueueReceive(net_to_net_queue, &wframe, QUEUE_OP_TIMEOUT_NONE) == pdTRUE) {
        // VLOG("net_send_task: master: received wrapped frame from net-to-net queue, source MAC = %s", mac_to_char(wframe.mac));

        // enumerate currently paired peers, sending frame if MAC addresses do not match
        esp_now_peer_info_t peer;
        esp_err_t e;

        // fetch first known peer
        e = esp_now_fetch_peer(true, &peer);

        while (e == ESP_OK) {

          // VLOG("net_send_task: master: comparing MAC, peer = %s, source = %s", mac_to_char(peer.peer_addr), mac_to_char(wframe.mac));

          if (mac_is_equal(peer.peer_addr, wframe.mac_addr)) {
            // VLOG("net_send_task: master: MAC addresses are equal, frame not sent");
          } else {
            // VLOG("net_send_task: master: MAC addresses are not equal, will send frame to peer");

            esp_err_t result = esp_now_send(peer.peer_addr, (const uint8_t *)&wframe.frame, sizeof(twai_message_t));

            if (result == ESP_OK) {
              // VLOG("net_send_task: master: sent frame from net-to-net queue to network peer, MAC = %s", mac_to_char(peer.peer_addr));
              PULSE_LED(NET_ACT_LED);
            } else {
              VLOG("net_send_task: master: error sending frame from net-to-net queue to network peer %s, err = %d", mac_to_char(peer.peer_addr), result);
              log_esp_now_err(result);
              PULSE_LED(ERR_IND_LED);
            }   // sent ok ?
          }   // MAC is equal ?

          e = esp_now_fetch_peer(false, &peer);
        }   // for each peer
      }   // got net-to-net queue message
    }   // if master

    // periodically, show stats and the currently paired ESP-NOW peers
    if ((millis() - ptimer >= 10000)) {
      ptimer = millis();

      VLOG("net_send_task: [%c] num peers = %d", (config_data.role == ROLE_MASTER) ? 'M' : 'S', num_peers);

      for (byte i = 0; i < MAX_NET_PEERS; i++) {
        if (peers[i].mac_addr[0] != 0) {
          VLOG("net_send_task: [%d], MAC = %s, tx = %d, rx = %d, errs = %d, batt = %ld", i, mac_to_char(peers[i].mac_addr), peers[i].tx, peers[i].rx, peers[i].num_errs, peers[i].battery_mv);
        }
      }
    }
  }   // for (;;)
}

//
/// setup
//

void setup() {

  led_command_t lc;

  // ensure external 12V is off
  pinMode(BOOST_ENABLE_PIN, OUTPUT);
  digitalWrite(BOOST_ENABLE_PIN, LOW);

  // enable EEPROM for config data
  EEPROM.begin(EEPROM_SIZE);

  // create logger and LED queues first
  queue_tab[0].handle = xQueueCreate(queue_tab[0].num_items, queue_tab[0].item_size);
  logger_in_queue = queue_tab[0].handle;
  queue_tab[1].handle = xQueueCreate(queue_tab[1].num_items, queue_tab[1].item_size);
  led_cmd_queue = queue_tab[1].handle;

  LOG("setup: module restart");
  VLOG("MERG ESP32 Wireless CAN bridge %d.%d.%d", VER_MAJ, VER_MIN, VER_PATCH);
  LOG(__FILE__);
  LOG("(C) Duncan Greenwood, M5767, 2019-2023");

  // start logger and LED tasks as early as possible
  for (byte i = 0; i < 2; i++) {
    if (xTaskCreatePinnedToCore(task_list[i].func, task_list[i].name, task_list[i].stack_size, NULL, task_list[i].priority, &task_list[i].handle, task_list[i].core) != pdPASS) {
      VLOG("setup: error creating task for %s", task_list[i].name);
    }
  }

  // query device wakeup cause
  esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();

  switch (wakeup_cause) {
    case ESP_SLEEP_WAKEUP_EXT0 : LOG("setup: wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : LOG("setup: wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : LOG("setup: wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : LOG("setup: wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : LOG("setup: wakeup caused by ULP program"); break;
    default : LOG("setup: device was not in deep sleep"); break;
  }

  if (wakeup_cause == ESP_SLEEP_WAKEUP_TOUCHPAD) {
    touch_pad_t touch_pin = esp_sleep_get_touchpad_wakeup_status();

    switch (touch_pin) {
      case 8 : LOG("setup: touch detected on GPIO 33"); break;
      case 9 : LOG("setup: touch detected on GPIO 32"); break;
      default: LOG("setup: wakeup not caused by touch"); break;
    }
  }

  //
  /// emergency reset - set configuration to defaults
  /// connect SCL pin to 0V for at least 5 seconds
  //

#if 0
  pinMode(SCL_PIN, INPUT);    // pulled high by external bus resistor; connect to 0V to reset

  if (!digitalRead(SCL_PIN)) {

    LOG("setup: detected emergency reset state");

    // blink all LEDs
    for (byte l = 0; l < 4; l++) {
      lc.led = l;
      lc.cmd = LED_FAST_BLINK;
      xQueueSend(led_cmd_queue, &lc, QUEUE_OP_TIMEOUT);
    }

    // ensure user really means this
    LOG("setup: pausing to recheck wakeup pin");

    // wait 5 seonds
    vTaskDelay(5000);

    // if pin is still grounded
    if (!digitalRead(SCL_PIN)) {

      LOG("setup: emergency reset");

      // set default config
      default_config();
      save_config();

      // delay then restart
      vTaskDelay(1000);
      ESP.restart();
    }
  } else {
    LOG("setup: emergency reset not triggered");
  }
#endif

  // test LEDs - all on
  for (byte l = 0; l < 4; l++) {
    lc.led = l;
    lc.cmd = LED_ON;
    xQueueSend(led_cmd_queue, &lc, QUEUE_OP_TIMEOUT);
  }

  // mount the local filesystem - format it first if required
  if (SPIFFS.begin(true)) {
    VLOG("setup: mounted SPIFFS filesystem, free space = %d", (SPIFFS.totalBytes() - SPIFFS.usedBytes()));
  } else {
    LOG("setup: error mounting SPIFFS filesystem");
  }

  // start I2C peripheral as bus master
  Wire.begin(SDA_PIN, SCL_PIN);

  LOG("setup: scanning the I2C bus");
  byte device_count = 0;

  // I2C bus scan
  for (byte i = 0; i < 128; i++) {
    Wire.beginTransmission(i);
    byte ret = Wire.endTransmission();

    if (ret == 0) {
      ++ device_count;
      VLOG("setup: detected device at 0x%hx", i);
    }
  }

  VLOG("setup: detected %d devices", device_count);

  // check for display
  Wire.beginTransmission(I2C_DISPLAY_ADDR);

  if (Wire.endTransmission() == 0) {
    display_present = true;
    LOG("setup: display is present");
  } else {
    display_present = false;
    LOG("setup: display is not present");
  }

  // check for GPIO expander
  Wire.beginTransmission(I2C_GPIO_ADDR);

  if (Wire.endTransmission() == 0) {
    switches_present = true;
    LOG("setup: GPIO expander is present");
  } else {
    switches_present = false;
    LOG("setup: GPIO expander is not present");
  }

  if (switches_present) {
    // IO expander initialisation
    mcp.begin(I2C_GPIO_ADDR);

    // set all pins to input pullup mode
    for (byte i = 0; i < 8; i++) {
      mcp.pinMode(i, INPUT);
      mcp.pullUp(i, HIGH);
    }

    // display current switch setting
    VLOG("GPIO port = %d", mcp.readGPIO());
    String t = "GPIO switches = " + String(mcp.get_port_state_as_char());
    VLOG((const char *)t.c_str());
  }

  //
  /// load local configuration, from switches or EEPROM
  //

  load_config();

  // display this node's configured role
  VLOG("setup: role = %s", (config_data.role == ROLE_MASTER ? "MASTER" : "SATELLITE"));

  // find the channel number for the configured WiFi AP
  // all nodes must use the same channel for ESP-NOW to work
  // the master node looks for external WiFi AP channel and uses this number for its soft AP
  // if not configured to use an external AP, it selects the default channel
  // slave nodes look for the master's soft AP and create a peer relationship using its channel number

  channel = -1;
  int retries = 0;

  LOG("setup: looking for wifi channel");

  // try a few times
  while (channel == -1 && retries < 5) {

    if (config_data.role == ROLE_MASTER)  {
      if (config_data.wifi_connect && strlen(config_data.ssid) > 0) {
        channel = get_wifi_ap_channel(config_data.ssid);
      } else {
        LOG("setup: external wifi not configured");
        break;
      }
    } else {
      String Prefix = "Master" + String(config_data.network_number);
      channel = get_wifi_ap_channel((char *)Prefix.c_str());
    }

    // wait and try again
    if (channel == -1) {
      ++retries;
      vTaskDelay((TickType_t)500);
    }
  }

  // if failed after several attempts, use the default channel
  if (channel == -1) {
    channel = config_data.default_wifi_channel;
    LOG( "setup: using default wifi channel");
  }

  VLOG("setup: wifi channel = %d", channel);

  // connect to wifi if so configured
  WiFi.disconnect();
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);

  if (config_data.wifi_connect && strlen(config_data.ssid) > 0) {
    WiFi.mode(WIFI_AP_STA);
    VLOG("setup: connecting to external WiFi AP = %s", config_data.ssid);
    WiFi.begin(config_data.ssid, config_data.pwd);
  } else {
    LOG("setup: not configured to connect to external wifi or AP not provided");
    WiFi.mode(WIFI_AP);
  }

  // create local access point
  // this advertises our ESP-NOW MAC address as well as providing access for web-based configuration
  // using the channel number selected above
  create_soft_AP(channel);

  // display MAC address
  VLOG("setup: station MAC address = %s", WiFi.macAddress().c_str());
  VLOG("setup: soft AP MAC address = %s", WiFi.softAPmacAddress().c_str());

  // disable WiFi power saving
  WiFi.setSleep(false);

  // we don't use bluetooth so switch it off
  btStop();

  // initialise peer state table
  for (byte i = 0; i < MAX_NET_PEERS; i++) {
    bzero((void *)&peers[i], sizeof(peer_state_t));
  }

  // initialise traffic stats data
  bzero(&stats, sizeof(stats_t));
  bzero(&errors, sizeof(stats_t));

  // initialise ESP-NOW
  if (esp_now_init() == ESP_OK) {
    LOG("setup: ESP-NOW initialised ok");
  } else {
    LOG("setup: ESP-NOW initialisation failed");
  }

  // slave attempts to locate master and add it as an ESP-NOW peer
  // this does not imply the master is reachable, only that its soft AP is visible
  // if this fails here it will be repeated on demand later

  if (config_data.role == ROLE_SLAVE && !master_found_and_paired) {
    LOG("setup: satellite trying to pair the master node");
    master_found_and_paired = find_and_pair_master();
    num_peers = master_found_and_paired ? 1 : 0;
    VLOG("setup: find_and_pair_master returns %d, num peers = %d", master_found_and_paired, num_peers);
  }

  // construct mDNS name (accessed as xyz.local)
  if (config_data.role == ROLE_MASTER) {
    sprintf(mdnsname, "mergwifi-m-%d", config_data.network_number);
  } else {
    sprintf(mdnsname, "mergwifi-s-%d-%d", config_data.network_number, config_data.slave_number);
  }

  // start mDNS responder
  if (!MDNS.begin(mdnsname)) {
    LOG("setup: error starting mDNS server");
  } else {
    VLOG("setup: mDNS server started, name = %s", mdnsname);
  }

  // start webserver
  start_webserver();

  // add webserver service to mDNS-SD
  MDNS.addService("_http", "_tcp", 80);

  // set hostname, same as mDNS name
  WiFi.setHostname(mdnsname);

  // create remaining queues
  for (byte j = 2; j < (sizeof(queue_tab) / sizeof(queue_t)); j++) {
    queue_tab[j].handle = xQueueCreate(queue_tab[j].num_items, queue_tab[j].item_size);
  }

  CAN_out_from_net_queue = queue_tab[2].handle;
  CAN_out_from_GC_queue = queue_tab[3].handle;
  CAN_out_from_withrottle_queue = queue_tab[4].handle;
  net_out_queue = queue_tab[5].handle;
  net_to_net_queue = queue_tab[6].handle;
  gc_out_queue = queue_tab[7].handle;
  gc_to_gc_queue = queue_tab[8].handle;
  withrottle_queue = queue_tab[9].handle;
  battery_monitor_queue = queue_tab[10].handle;
  wsserver_out_queue = queue_tab[11].handle;
  cmdproxy_queue = queue_tab[12].handle;
  cbus_in_queue = queue_tab[13].handle;
  cbus_internal = queue_tab[14].handle;

  for (byte i = 0; i < (sizeof(queue_tab) / sizeof(queue_t)); i++) {
    if (queue_tab[i].handle == NULL) {
      VLOG("setup: ERROR: NULL queue at pos = %d, %s", i, queue_tab[i].name);
    }
  }

  // configure CAN bus driver
  twai_general_config_t g_config;

  g_config.mode = TWAI_MODE_NORMAL;
  g_config.tx_io = (gpio_num_t)CAN_TX_PIN;
  g_config.rx_io = (gpio_num_t)CAN_RX_PIN;
  g_config.clkout_io = (gpio_num_t)TWAI_IO_UNUSED;
  g_config.bus_off_io = (gpio_num_t)TWAI_IO_UNUSED;
  g_config.tx_queue_len = CAN_QUEUE_DEPTH;
  g_config.rx_queue_len = CAN_QUEUE_DEPTH;
  g_config.alerts_enabled = TWAI_ALERT_ALL;
  g_config.clkout_divider = 0;

  VLOG("setup: TWAI queue depth = %d\n", CAN_QUEUE_DEPTH);

  const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
  const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // install TWAI driver
  esp_err_t iret = twai_driver_install(&g_config, &t_config, &f_config);

  if (iret == ESP_OK) {
    LOG("setup: TWAI driver installed ok");
  } else {
    LOG("setup: error installing TWAI driver");
    log_esp_now_err(iret);
  }

  // start CAN driver
  iret = twai_start();

  if (iret == ESP_OK) {
    LOG("setup: TWAI driver started ok");
  } else {
    LOG("setup: error starting TWAI driver");
    log_esp_now_err(iret);
  }

  // register ESP-NOW send and receive callbacks
  // after queues have been created but before tasks are started
  esp_now_register_send_cb(on_data_sent);
  esp_now_register_recv_cb(on_data_rcvd);

  //
  /// create and start the remaining tasks, using the task list
  //

  for (byte i = 2; i < (sizeof(task_list) / sizeof(task_info_t)); i++) {
    if (task_list[i].do_create) {
      if (xTaskCreatePinnedToCore(task_list[i].func, task_list[i].name, task_list[i].stack_size, NULL, task_list[i].priority, &task_list[i].handle, task_list[i].core) != pdPASS) {
        VLOG("setup: error creating task for %s", task_list[i].name);
      }
    }
  }

  // misc system/task info
  VLOG("setup: ESP32 IDF version = %s", esp_get_idf_version());
  VLOG("setup: CPU frequency = %d MHz", getCpuFrequencyMhz());
  VLOG("setup: task priority = %d, task name = %s, CPU core = %d", uxTaskPriorityGet(NULL), pcTaskGetTaskName(NULL), xPortGetCoreID());
  VLOG("setup: portTICK_PERIOD_MS = %d", portTICK_PERIOD_MS);
  VLOG("setup: configTICK_RATE_HZ = %d", configTICK_RATE_HZ);
  VLOG("setup: number of tasks = %d", uxTaskGetNumberOfTasks());

  // register shutdown handler
  esp_register_shutdown_handler([]() {
    Serial.println("shutdown handler: running ...");
  });

  // test LEDs - all off
  for (byte l = 0; l < 4; l++) {
    lc.led = l;
    lc.cmd = LED_OFF;
    xQueueSend(led_cmd_queue, &lc, QUEUE_OP_TIMEOUT);
  }

  // blink the blue LED until wifi is connected
  lc.led = ARDUINO_LED;
  lc.cmd = LED_FAST_BLINK;
  xQueueSend(led_cmd_queue, &lc, QUEUE_OP_TIMEOUT);

  // add handler for touch testing
  // !! interfers with CBUS switch handling
  // !! only run on slaves without physical switch

  if (config_data.role == ROLE_SLAVE) {
    LOG("setup: satellite installing touch interrupt handler");
    touchAttachInterrupt(T8, touch_callback, 40);
    touchAttachInterrupt(T9, touch_callback, 40);
  }

  // for slaves, switch on the 12V boost circuit as late as possible
  // as the attached CANCAB will try to enumerate the CAN bus as soon as it boots,
  // so layout connectivity must be up

  if (config_data.role == ROLE_SLAVE && !boost_enable_on) {
    VLOG("setup: enabling 12V boost circuit, pin = %d", BOOST_ENABLE_PIN);
    digitalWrite(BOOST_ENABLE_PIN, HIGH);
    boost_enable_on = true;
  }

  // end of setup
  VLOG("setup: free heap size = %d bytes", xPortGetFreeHeapSize());
  LOG("setup complete");

  return;
}

//
/// loop
//

void loop() {

  static bool wifi_connected = false;
  static signed long restart_timer = 0L;
  static unsigned long ptimer = millis();
  led_command_t lc;

  // allow webserver some processor time
  webserver.handleClient();

  // restart flag has been set
  if (do_restart) {

    // restart once the timer expires
    if (restart_timer - millis() <= 0L) {
      ESP.restart();        // calls shutdown handler
    }

    // set the restart timer for 5 seconds in the future
    if (restart_timer == 0L) {
      restart_timer = millis() + 5000L;
      LOG("loop: restart countdown timer set");

      // flash LED
      lc.led = ARDUINO_LED;
      lc.cmd = LED_FAST_BLINK;
      xQueueSend(led_cmd_queue, &lc, QUEUE_OP_TIMEOUT);
    }
  }

  // log message once wifi connected
  if (!wifi_connected && WiFi.status() == WL_CONNECTED) {
    LOG("loop: WiFi is now connected");
    VLOG("loop: local IP = %s, channel = %d, hostname = %s", WiFi.localIP().toString().c_str(), WiFi.channel(), WiFi.getHostname());
    wifi_connected = true;
    lc.led = ARDUINO_LED;
    lc.cmd = LED_SHORT_BLINK;
    xQueueSend(led_cmd_queue, &lc, QUEUE_OP_TIMEOUT);
  }

  // periodic tasks every 10 secs
  if (millis() - ptimer >= 10000) {
    ptimer = millis();

    /*
      wifi_sta_list_t wifi_sta_list;
      tcpip_adapter_sta_list_t adapter_sta_list;
      esp_err_t err;

      bzero(&wifi_sta_list, sizeof(wifi_sta_list));
      bzero(&adapter_sta_list, sizeof(adapter_sta_list));

      if ((err = esp_wifi_ap_get_sta_list(&wifi_sta_list)) == ESP_OK) {
        VLOG("AP: connected stations = %d",  wifi_sta_list.num);

        for (byte k = 0; k < wifi_sta_list.num; k++) {
          tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);
          VLOG("AP: %d - %s, %s", k, mac_to_char(adapter_sta_list.sta[k].mac), ip4addr_ntoa(&(adapter_sta_list.sta[k].ip)));
        }
      } else {
        VLOG("AP: error getting connected stations, err = %d", err);
      }
    */

    char role = (config_data.role == ROLE_MASTER) ? 'M' : 'S';
    VLOG("%c: messages - net: tx = %lu rx = %lu, CAN: tx = %lu rx = %lu, GC: tx = %lu, rx = %lu", role, stats.net_tx, stats.net_rx, stats.can_tx, stats.can_rx, stats.gc_tx, stats.gc_rx);
    VLOG("%c: errors   - net: tx = %lu rx = %lu, CAN: tx = %lu rx = %lu, GC: tx = %lu, rx = %lu", role, errors.net_tx, errors.net_rx, errors.can_tx, errors.can_rx, errors.gc_tx, errors.gc_rx);

    VLOG("loop: free heap size = %u bytes", xPortGetFreeHeapSize());

    // task stack hwm
    for (byte i = 0; i < (sizeof(task_list) / sizeof(task_info_t)); i++) {
      unsigned int hwm = uxTaskGetStackHighWaterMark(task_list[i].handle);

      if (task_list[i].hwm < hwm) {
        task_list[i].prev_hwm = task_list[i].hwm;
        task_list[i].hwm = hwm;
      }
    }
  }
}  // loop()

//
/// format a CAN frame as a char string
//

char *format_CAN_frame(twai_message_t *frame) {

  static char fbuff[48];      // static as we return a pointer to it
  char tbuff[8];

  bzero((void *)fbuff, sizeof(fbuff));
  snprintf(fbuff, sizeof(fbuff), "[%3d] [%1d] [ ", frame->identifier & 0x7f, frame->data_length_code);

  for (byte i = 0; i < frame->data_length_code; i++) {
    snprintf(tbuff, sizeof(tbuff), "0x%02x ", frame->data[i]);
    strcat(fbuff, tbuff);
  }

  strcat(fbuff, "]");

  if (frame->flags & TWAI_MSG_FLAG_RTR) {
    strcat(fbuff, " R");
  }

  return fbuff;
}

//
/// load configuration data from EEPROM and onboard switches
//

void load_config(void) {

  LOG("load_config: loading and setting configuration");
  VLOG("load_config: config data size = %d", sizeof(config_data));
  VLOG("load_config: switches present = %d", switches_present);

  // load soft config from EEPROM
  VLOG("load_config: loading saved config from EEPROM");
  EEPROM.readBytes(0, (void *)&config_data, sizeof(config_data));

  // set and save defaults if module is brand new or reset
  if (config_data.guard_val != 99) {
    VLOG("load_config: guard value is not set, setting configuration defaults");
    default_config();
  }

  // overlay soft/default config with settings from physical switches
  if (config_data.config_mode == CONFIG_USES_HW && switches_present) {

    // note inverse logic - switch on = 0V = logic low = false
    LOG("load_config: updating stored config from physical switches");
    config_data.role = mcp.digitalRead(0);    // switch on = 0V = slave, off = 5V = master

    config_data.network_number = !mcp.digitalRead(1);
    config_data.network_number += (!mcp.digitalRead(2) << 1);
    config_data.network_number += (!mcp.digitalRead(3) << 2);

    config_data.slave_number = !mcp.digitalRead(4);
    config_data.slave_number += (!mcp.digitalRead(5) << 1);
    config_data.slave_number += (!mcp.digitalRead(6) << 2);

    config_data.config_mode = CONFIG_USES_HW;
    save_config();
    VLOG("load_config: from hardware: role = %d, network = %d, satellite = %d", config_data.role, config_data.network_number, config_data.slave_number);
  }

  VLOG("  - role = %d", config_data.role);
  VLOG("  - network = %d", config_data.network_number);
  VLOG("  - satellite = %d", config_data.slave_number);
  VLOG("  - config mode = %d", config_data.config_mode);
  VLOG("  - gc server = %d", config_data.gc_server_on);
  VLOG("  - gc server port = %d", config_data.gc_server_port);
  VLOG("  - gc serial on = %d", config_data.gc_serial_on);
  VLOG("  - bridge mode = %d", config_data.bridge_mode);
  VLOG("  - debug = %d", config_data.debug);
  VLOG("  - guard val = %d", config_data.guard_val);
  VLOG("  - WiFi connect = %d", config_data.wifi_connect);
  VLOG("  - SSID = %s", config_data.ssid);
  VLOG("  - password = %s", config_data.pwd);
  VLOG("  - withrottle server = %d", config_data.withrottle_on);
  VLOG("  - withrottle port = %d", config_data.withrottle_port);
  VLOG("  - DCC backend = %d", config_data.dcc_type);
  VLOG("  - satellite sends battery status = %d", config_data.slave_send_battery);
  VLOG("  - low battery sleep = %d", config_data.low_battery_threshold);
  VLOG("  - send estop on sleep = %d", config_data.send_estop_on_sleep);
  VLOG("  - peer error limit = %d", config_data.peer_err_limit);
  VLOG("  - use network password = %d", config_data.use_network_password);
  VLOG("  - network password = %s", config_data.network_password);
  VLOG("  - WiFi AP password = %s", config_data.softap_password);
  VLOG("  - sleep delay = %d", config_data.sleep_delay_mins);
  VLOG("  - default WiFi channel = %d", config_data.default_wifi_channel);
  VLOG("  - forward battery msgs to bus = %d", config_data.forward_battery_msgs_to_cbus);
  VLOG("  - serial server task = %d", config_data.ser_on);
  VLOG("  - serial server port = %d", config_data.ser_port);
  VLOG("  - CANCMD proxy = %d", config_data.cmdproxy_on);
  VLOG("  - CANID = %d", config_data.CANID);
  VLOG("  - CBUS node number = %d", config_data.node_number);
  VLOG("  - CBUS mode = %d", config_data.cbus_mode);
  VLOG("  - Wakeup source = %d", config_data.wakeup_source);
  VLOG("  - Touch threshold = %d", config_data.touch_threshold);

  String nvs = "";

  for (byte i = 0; i < NUM_CBUS_NVS; i++) {
    nvs += String(config_data.node_variables[i]) + " ";
  }

  VLOG("  - CBUS node variables = %s", nvs.c_str());
  return;
}

//
/// save configuration data to EEPROM
//

void save_config(void) {

  EEPROM.writeBytes(0, (void *)&config_data, sizeof(config_data));
  EEPROM.commit();
  VLOG("save_config: config data saved %d bytes", sizeof(config_data));

  return;
}

//
/// set default configuration values
//

void default_config(void) {

  LOG("default_config: setting default configuration values");

  bzero(&config_data, sizeof(config_data));
  EEPROM.writeBytes(0, (void *)&config_data, sizeof(config_data));
  EEPROM.commit();
  delay(5);

  config_data.role = ROLE_MASTER;
  config_data.network_number = 0;
  config_data.config_mode = (switches_present) ? CONFIG_USES_HW : CONFIG_USES_SW;
  config_data.bridge_mode = TRANSPARENT_MODE;
  config_data.slave_number = 0;
  config_data.gc_server_on = false;
  config_data.gc_server_port = 5550;
  config_data.debug = 0;
  config_data.guard_val = 99;
  config_data.wifi_connect = false;
  config_data.ssid[0] = 0;
  config_data.pwd[0] = 0;
  config_data.withrottle_on = false;
  config_data.withrottle_port = 49154;
  config_data.dcc_type = DCC_UNK;
  config_data.slave_send_battery = false;
  config_data.peer_err_limit = 5;
  config_data.use_network_password = true;
  strcpy(config_data.network_password, "secret_layout");
  strcpy(config_data.softap_password, "0123456789");
  config_data.sleep_delay_mins = 0;
  config_data.default_wifi_channel = 11;
  config_data.low_battery_threshold = 3300;
  config_data.send_estop_on_sleep = false;
  config_data.forward_battery_msgs_to_cbus = false;
  config_data.ser_on = false;
  config_data.ser_port = 5552;
  config_data.gc_serial_on = false;
  config_data.cmdproxy_on = false;
  config_data.CANID = 0;
  config_data.node_number = 0;
  config_data.cbus_mode = CBUS_MODE_SLIM;
  config_data.wakeup_source = WAKE_SWITCH;
  config_data.touch_threshold = 40;

  for (byte i = 0; i < NUM_CBUS_NVS; i++) {
    config_data.node_variables[i] = 0;
  }

  save_config();
  delay(5);

  return;
}

//
/// format a MAC address as a colon-delimited hex string xx:xx:xx:xx:xx:xx
//

char *mac_to_char(const uint8_t mac_addr[6]) {

  static char mbuff[20];          // must be static as we return a pointer to it

  snprintf(mbuff, sizeof(mbuff), "%02hhX:%02hhX:%02hhX:%02hhX:%02hhX:%02hhX",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  // VLOG("mac_to_char: string = %s", mbuff);

  return mbuff;
}

//
/// compare two MAC addresses, return true if equal
/// case insensitive comparison
/// count backwards as all devices will have same first two bytes, so it will fail quicker
//

bool mac_is_equal(const uint8_t *mac1, const uint8_t *mac2) {

  bool is_equal = true;

  for (int8_t i = 5; i >= 0; i--) {
    if (toupper(mac1[i]) != toupper(mac2[i])) {
      is_equal = false;
      break;              // break on first mismatch
    }
  }

  // VLOG("MAC_is_equal: %s / %s, is_equal = %d", mac_to_char(mac1), mac_to_char(mac2), is_equal);

  return is_equal;
}

//
/// add a new peer to the table
//

void add_peer(const uint8_t *mac_addr) {

  byte i;

  for (i = 0; i < MAX_NET_PEERS; i++) {
    if (peers[i].mac_addr[0] == 0) {
      memcpy(peers[i].mac_addr, mac_addr, sizeof(uint8_t[6]));
      peers[i].num_errs = 0;
      VLOG("add_peer: added new peer at table index = %d", i);
      break;
    }
  }

  if (i == MAX_NET_PEERS) {
    LOG("add_peer: table is full!!");
  }

  return;
}

//
/// master only
/// peer state table and record operations
/// unpair if tx error limit has been exceeded - this ensures that we don't keep trying to send messages to unresponsive clients
//

void peer_record_op(const uint8_t *mac_addr, byte op, unsigned int val) {

  byte i;

  // VLOG("peer_record_op: peer = %s, op = %d", mac_to_char(mac_addr), op);

  // slaves don't do this
  if (config_data.role == ROLE_SLAVE) {
    return;
  }

  // init all
  if (op == PEER_INIT_ALL) {
    for (i = 0; i < MAX_NET_PEERS; i++) {
      bzero((void *)&peers[i], sizeof(peer_state_t));
    }
    return;
  }

  // increment tx counter for all active slaves
  if (op == PEER_INCR_TX_ALL) {
    for (i = 0; i < MAX_NET_PEERS; i++) {
      if (peers[i].mac_addr[0] != 0) {
        ++peers[i].tx;
      }
    }
    return;
  }

  // for the following per-peer operations, find the peer record in the table by MAC address
  for (i = 0; i < MAX_NET_PEERS; i++) {
    if (mac_is_equal(mac_addr, peers[i].mac_addr)) {
      break;
    }
  }

  if (i == MAX_NET_PEERS) {
    VLOG("peer_record_op: peer record not found for MAC addr = %s", mac_to_char(mac_addr));
    return;
  }

  switch (op) {

    case PEER_INCR_ERR:
      ++peers[i].num_errs;

      if (peers[i].num_errs >= config_data.peer_err_limit) {
        VLOG("peer_record_op: error limit exceeded, unpairing peer = %s", mac_to_char(mac_addr));

        esp_err_t e = esp_now_del_peer(mac_addr);

        if (e == ESP_OK) {
          LOG("peer_record_op: peer unpaired and record reset");
        } else {
          VLOG("peer_record_op: error unpairing, e = %d", e);
          log_esp_now_err(e);
        }

        bzero((void *)&peers[i], sizeof(peer_state_t));
      } else {
        // VLOG("peer_record_op: peer new error count = %d", peers[i].num_errs);
      }
      break;

    case PEER_DECR_ERR:
      if (peers[i].num_errs > 0) {
        --peers[i].num_errs;
      }
      break;

    case PEER_RESET_ERR:
      peers[i].num_errs = 0;
      break;

    case PEER_INCR_TX:
      peers[i].tx += 1;
      break;

    case PEER_INCR_RX:
      peers[i].rx += 1;
      break;

    case PEER_SET_BATT_MV:
      peers[i].battery_mv = val;
      break;

    case PEER_SET_BATT_SOC:
      peers[i].battery_soc = val;
      break;

    case PEER_SET_CANID:
      peers[i].CANID = val & 0xff;
      break;

    default:
      VLOG("peer_record_op: unknown operation = %d", op);
      break;
  }

  return;
}

//
/// log an ESP-NOW error number as a string
//

void log_esp_now_err(int err_num) {

  VLOG("log_esp_now_err: [%d} %s", err_num, esp_err_to_name(err_num));
  return;
}

//
/// put the slave device into deep sleep mode
/// optionally send an e-stop command to the command station
/// also disable the 12V boost circuit
//

void device_sleep(void) {

  twai_message_t cf;

  VLOG("device_sleep: device sleep initiated");

  // fast blink blue LED
  led_command_t cmd = { ARDUINO_LED, LED_FAST_BLINK, 0 };
  xQueueSend(led_cmd_queue, &cmd, QUEUE_OP_TIMEOUT_NONE);

  if (config_data.send_estop_on_sleep) {

    if (CANCMD_session_num >= 0) {
      VLOG("device_sleep: sending stop message, session num = %d", CANCMD_session_num);
      cf.identifier = slave_canid;
      cf.data_length_code = 3;
      cf.data[0] = OPC_DSPD;
      cf.data[1] = CANCMD_session_num;
      cf.data[2] = 129;                       // dir = fwd, speed = 1

      if (!send_message_to_queues(QUEUE_NET_OUT, &cf, "device_sleep", QUEUE_OP_TIMEOUT_SHORT)) {
        LOG("device_sleep: error queuing message");
        PULSE_LED(ERR_IND_LED);
      }

    } else {
      VLOG("device_sleep: session num = %d, not sending emergency stop message", CANCMD_session_num);
    }
  }

  VLOG("device_sleep: deep sleep in 5 seconds");
  vTaskDelay(5000);
  digitalWrite(BOOST_ENABLE_PIN, LOW);      // disable 12V boost circuit to save power
  boost_enable_on = false;

  // configure wakeup source
  switch (config_data.wakeup_source) {
    case  WAKE_TOUCH:
      LOG("device_sleep: enabling touchpad wakeup source");
      esp_sleep_enable_touchpad_wakeup();
      break;

    case WAKE_SWITCH:
      VLOG("device_sleep: enabling switch wakeup source, pin %d, state = %d", GPIO_NUM_33, digitalRead(GPIO_NUM_33));
      rtc_gpio_pulldown_en(GPIO_NUM_33);                      // pulldown to 0V
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 1);           // wake on high
      break;

    default:
      LOG("device_sleep: no wakeup source defined");
      break;
  }

  LOG("device_sleep: sleeping now");
  vTaskDelay(100);
  esp_deep_sleep_start();

  /// does not return
}

//
/// test callback function for testing touch wakeup source
//

void IRAM_ATTR touch_callback(void) {

  led_command_t lc = { ERR_IND_LED, LED_PULSE, 0 };
  LOG("touch callback runs");
  xQueueSend(led_cmd_queue, &lc, 0);
  return;
}

//
/// covenience function to send a message to multiple queues
/// pass target queue list as an or'd bit field in a 16 bit integer
/// e.g. uint16_t queues = QUEUE_CAN_OUT_FROM_GC | QUEUE_NET_OUT;
//

bool send_message_to_queues(uint16_t target_queues, void *msg, const char *source_task, TickType_t time_to_wait) {

  bool ret = true;

  // VLOG("send_message_to_queues: targets = %d, source = %s", target_queues, source_task);

  for (byte i = 0; i < sizeof(queue_tab) / sizeof(queue_t); i++) {

    // iterate through target bits

    if (target_queues & (1 << i)) {

      // apply per-queue constraints
      if ((i < 5) || \
          (i == 5 && num_peers > 0) || \
          (i == 6 && num_peers > 1) || \
          (i == 7 && num_gc_clients > 0) || \
          (i == 8 && num_gc_clients > 1) || \
          (i == 9 && num_wi_clients > 0 && config_data.dcc_type == DCC_MERG) || \
          (i == 10) || \
          (i == 11 && false) || \
          (i == 12 && config_data.cmdproxy_on) || \
          (i == 13 && config_data.role == ROLE_MASTER) || \
          (i == 14 && config_data.role == ROLE_MASTER)
         ) {

        // VLOG("send_message_to_queues: sending to queue %d/%s, from source = %s", i, queue_tab[i].name, source_task);

        /*
          if (i == 13) {
            LOG("send_message_to_queues: sending to QUEUE_CBUS_EXTERNAL");
          }

          if (i == 14) {
            LOG("send_message_to_queues: sending to QUEUE_CBUS_INTERNAL");
          }
        */

        if (xQueueSend(queue_tab[i].handle, msg, time_to_wait) != pdTRUE) {
          VLOG("send_message_to_queues: error sending message to queue = %d/%s, from source = %s", i, queue_tab[i].name, source_task);
          ret = false;
        }
      } else {
        // VLOG("send_message_to_queues: not sending to queue = %d/%s, from source = %s", i, queue_tab[i].name, source_task);
      }   // per-queue test
    }   // each target queue
  }   // loop

  return ret;
}
