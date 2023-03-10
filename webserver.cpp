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
#include <WebServer.h>
#include <Update.h>
#include <SPIFFS.h>

#include "defs.h"
#include "cbusdefs.h"
#include "gpio.h"


File fsUploadFile;
void save_uploaded_file(void);
void handle_file_upload(void);
bool handle_file_read(String path);
String get_content_type(String filename);
void log_esp_now_err(int err_num);

// external variables defined elsewhere
extern QueueHandle_t gc_out_queue, logger_in_queue, CAN_out_from_net_queue, led_cmd_queue, cbus_in_queue;
extern config_t config_data;
extern WebServer webserver;
extern bool do_restart;
extern bool switches_present;
extern char mdnsname[];
extern stats_t stats, errors;
extern peer_state_t peers[MAX_NET_PEERS];
extern gcclient_t gc_clients[MAX_GC_CLIENTS];
extern byte num_peers, num_gc_clients, num_wi_clients;
extern bool in_transition, enum_required;
extern task_info_t task_list[12];
extern MCP23008 mcp;

// externally defined functions
void transition_to_flim(void);

// HTML sources
// template variables {{xxx}} are updated/replaced at runtime when the page is served

const char htmlHeader[] = "<!DOCTYPE html>"
                          "<html>"
                          "<head>"
                          "<title>CAN WiFi Bridge</title>"
                          "<meta http-equiv='Cache-Control' content='no-cache, no-store, must-revalidate' />"
                          "<meta http-equiv='Pragma' content='no-cache'/>"
                          "<meta http-equiv='Expires' content='0' />"
                          "<meta content='width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=0' name='viewport'>"
                          "<meta name = 'format-detection' content = 'telephone=no'>"
                          "</head>"
                          "<body>"
                          "<div>";

const char htmlTitle[]  = "<header><h2>CAN WiFi Bridge Configuration</h2></header><div>";
const char htmlFooter[] = "</div></body></html>";

const char htmlMenu[]   = "<h3> Node: {{mdnsname}}</h3>"
                          "<div>Device configuration <button onclick=\"window.location.href = '/config';\">Click</button></div>"
                          "<div>Info <button onclick=\"window.location.href = '/info';\">Click</button></div>"
                          "<div>Stats <button onclick=\"window.location.href = '/stats';\">Click</button></div>"
                          "<div>File upload <button onclick=\"window.location.href = '/file_upload';\">Click</button></div>"
                          "<div>Software update <button onclick=\"window.location.href = '/softwareupdate';\">Click</button></div>"
                          "<div>Restart <button onclick=\"window.location.href = '/restart';\">Click</button></div>"
                          "<div>Reset to default configuration <button onclick=\"window.location.href = '/default_config';\">Click</button></div>"
                          "<div>Deep sleep <button onclick=\"window.location.href = '/stop';\">Click</button></div>";

const char htmlInfo[]   = "Version: {{version}} <br>"
                          "Uptime: {{uptime}} <hr>"
                          "WiFi SSID: {{ssid}} <br>"
                          "IP address: {{ip}} <br>"
                          "Gateway: {{gateway}} <br>"
                          "Subnet: {{subnet}} <br>"
                          "Status: {{stat}} <br>"
                          "<hr>"
                          "STA MAC: {{sta_mac}} <br>"
                          "AP MAC: {{ap_mac}} <br>"
                          "<hr>"
                          "CAN ID: {{cbus_canid}} <br>"
                          "CBUS node number: {{cbus_node_number}} <br>";

const char htmlStats[]  = "<h3>CAN bus:</h3>"
                          "sent = {{can_tx}}, received = {{can_rx}}, errors = {{can_tx_err}}/{{can_rx_err}}"
                          "<h3>ESP-NOW:</h3>"
                          "sent = {{net_tx}}, received = {{net_rx}}, errors = {{net_tx_err}}/{{net_rx_err}}"
                          "<h3>Gridconnect:</h3>"
                          "sent = {{gc_tx}}, received = {{gc_rx}}, errors = {{gc_tx_err}}/{{gc_rx_err}}";

const char htmlSleep[]   = "<form action = '/do_deepsleep' method = 'post'>"
                           "Deep sleep: <input type = 'submit' value = 'Deep sleep'>"
                           "</form>";

const char htmlConfig[] = "<h3>Node: {{mdnsname}}</h3>"
                          "<form action = '/store' method = 'post'>"
                          "Role: <br>"
                          "<input type = 'radio' name = 'role' value = 'master' {{master_selected}} > Master<br>"
                          "<input type = 'radio' name = 'role' value = 'slave' {{slave_selected}} > Satellite<br>"
                          "<br>"

                          "Network number (0-255): <input type = 'number' name = 'network_number' min = '0' max = '255' step = '1' value = '{{network_number}}'> <br>"
                          "Satellite number (0-255): <input type = 'number' name = 'slave_number' min = '0' max = '255' step = '1' value = '{{slave_number}}'> <br>"
                          "Satellite error limit: <input type = 'number' name = 'peer_err_limit' min = '1' max = '100' step = '1' value = '{{peer_err_limit}}'> <br>"
                          "<hr>"

                          "Configuration preference: <br>"
                          "<input type = 'radio' name = 'config_mode' value = 'browser' {{browser_selected}}> Browser<br>"
                          "<input type = 'radio' name = 'config_mode' value = 'switches' {{switches_selected}}> Hardware ({{switches_present}}present {{switch_val}})<br>"
                          "<hr>"

                          "<input type = 'checkbox' name = 'wifi_connect' {{wifi_connect}}> Connect to WiFi<br>"
                          "WiFi SSID: <input name = 'ssid' value = '{{ssid}}'>"
                          "<br>"
                          "WiFi password: <input type = 'password' name = 'pwd' value = '{{pwd}}'> <br>"
                          "Default WiFi channel: <input type = 'number' name = 'default_wifi_channel' min = '1' max = '12' step = '1' value = '{{default_wifi_channel}}'> <br>"
                          "<hr>"

                          "Device AP password: <input name = 'softap_password' type = 'password' maxlength = '14' value = '{{softap_password}}'> <br>"
                          "<hr>"
                          "<input type = 'checkbox' name = 'use_network_password' {{use_network_password}}> Use layout network password <br>"
                          "Layout network password: <input name = 'network_password' type = 'password' maxlength = '14' value = '{{network_password}}'> <br>"
                          "<hr>"

                          "CANID: <input type = 'number' name = 'canid' min = '1' max = '99' step = '1' value = '{{canid}}'>"
                          "<button type = 'button' id = 'btn_enum' onclick = \"(function() {"
                          "  var x1 = new XMLHttpRequest();"
                          "  x1.open('GET', '/enum', true);"
                          "  x1.send();"
                          "}"
                          ")"
                          "();\">"
                          "Enumerate"
                          "</button>"
                          "<br>"

                          // "<span><svg xmlns='http://www.w3.org/2000/svg'><circle cx='5' cy='5' r='5' fill='{{cbus_led}}' /></svg></span>"
                          "Node number: <input type = 'number' name = 'node_number' min = '1' max = '65535' step = '1' value = '{{node_number}}'>"
                          "<button type = 'button' id = 'btn_rqnn' onclick = \"(function() {"
                          "  var x2 = new XMLHttpRequest();"
                          "  x2.open('GET', '/rqnn', true);"
                          "  x2.send();"
                          "}"
                          ")"
                          "();\">"
                          "CBUS"
                          "</button>"
                          "<hr>"

                          "<input type = 'checkbox' name = 'slave_send_battery' {{slave_send_battery}}> Send battery status (satellite) <br>"
                          "<input type = 'checkbox' name = 'forward_battery_msgs_to_cbus' {{forward_battery_msgs_to_cbus}}> Send battery status to layout (master) <br>"
                          "Inactivity sleep: <input type = 'number' name = 'sleep_delay_mins' min = '0' max = '60' step = '1' value = '{{sleep_delay_mins}}'> mins<br>"
                          "Low battery sleep: <input type = 'number' name = 'low_battery_threshold' min = '2800' max = '4200' value = '{{low_battery_threshold}}'> mV<br>"
                          "<input type='checkbox' name = 'send_estop_on_sleep' {{send_estop_on_sleep}}> Send loco e-stop on sleep <br>"
                          "Wake up source: <br>"
                          "<input type = 'radio' name = 'wakeup_source' value = 'touch' {{wakeup_touch}}> Touch<br>"
                          "<input type = 'radio' name = 'wakeup_source' value = 'switch' {{wakeup_switch}}> Switch<br>"
                          "Touch threshold: <input type = 'number' name = 'touch_threshold' min = '0' max = '100' value = '{{touch_threshold}}'>"
                          "<hr>"

                          "<input type = 'checkbox' name = 'debug' {{debug}}> Debug to file (beware performance impact)<br>"
                          "<hr>"

                          "<input type = 'checkbox' name = 'gc_server_on' {{gc_server_on}}> GridConnect server (master only)<br>"
                          "GC server port: <input type = 'number' name = 'gc_server_port' min = '1024' max = '65535' step = '1' value = '{{gc_server_port}}'> <br>"
                          "<input type = 'checkbox' name = 'gc_serial_on' {{gc_serial_on}}> Enable USB serial port<br>"
                          "<hr>"

                          "<input type = 'checkbox' name = 'withrottle_on' {{withrottle_on}}> WiThrottle server (master only)<br>"
                          "WiThrottle server port: <input type = 'number' name = 'withrottle_port' min = '1024' max = '65535' step = '1' value = '{{withrottle_port}}'> <br>"
                          "DCC command station: <br>"
                          "<input type = 'radio' name = 'dcc_backend' value = 'merg' {{merg_dcc}}> MERG<br>"
                          "<input type = 'radio' name = 'dcc_backend' value = 'dccpp' {{dccpp_dcc}}> DCC++<br>"
                          "<hr>"

                          "<input type = 'checkbox' name = 'ser_on' {{ser_on}}> DCC++ serial server (master only)<br>"
                          "Server port: <input type = 'number' name = 'ser_port' min = '1024' max = '65535' step = '1' value = '{{ser_port}}'> <br>"
                          "<hr>"

                          "<input type = 'checkbox' name = 'cmdproxy_on' {{cmdproxy_on}}> DCC++ CANCMD proxy (master only)<br>"
                          "<hr>"

                          /*
                            Bridge mode: <br> \
                            <input type = 'radio' name = 'bridge_mode' value = 'transparent' {{transparent_selected}} > Transparent<br> \
                            <input type = 'radio' name = 'bridge_mode' value = 'filtering' {{filtering_selected}} > Filtering (not implemented yet)<br> \
                            <br> \
                          */

                          "<input type = 'submit' value = 'Save & restart'>"
                          "</form>";

const char softwareupdate[] =
  "<!DOCTYPE html>"
  "<html>"
  "<head>"
  "<title>CAN WiFi Bridge</title>"
  "<meta http-equiv='Cache-Control' content='no-cache, no-store, must-revalidate' />"
  "<meta http-equiv='Pragma' content='no-cache'/>"
  "<meta http-equiv='Expires' content='0' />"
  "<meta content='width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=0' name='viewport'>"
  "<meta name = 'format-detection' content = 'telephone=no'>"
  "</head>"
  "<body>"
  "<header><h2>CAN WiFi Bridge Configuration</h2></header><div>"
  "Select a bin file to load and click Update"
  "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
  "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
  "<input type='file' name='update'>"
  "<input type='submit' value='Update'>"
  "</form>"
  "<div id='prg'>progress: 0%</div>"
  "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')"
  "},"
  "error: function (a, b, c) {"
  "}"
  "});"
  "});"
  "</script></body>";

const char htmlFileupload[] = "<form method='post' enctype='multipart/form-data'>"
                              "<input type='file' name='filename'>"
                              "<input class='button' type='submit' value='Upload'>"
                              "</form>";
const char htmlSuccess[] =    "File uploaded ok";

void start_webserver(void) {

  // default if no other handler has been registered
  webserver.onNotFound(handle_notfound);

  // webserver page handlers
  webserver.begin();
  webserver.on("/", handle_root);
  webserver.on("/config", handle_config);
  webserver.on("/store", handle_store);
  webserver.on("/info", handle_info);
  webserver.on("/stats", handle_stats);
  webserver.on("/stop", handle_stop);
  webserver.on("/do_deepsleep", do_deepsleep);
  webserver.on("/restart", handle_restart);
  webserver.on("/default_config", handle_default);
  webserver.on("/rqnn", handle_rqnn);
  webserver.on("/enum", handle_enum);
  webserver.on("/success", handle_success);
  webserver.on("/delete_file", handle_delete_file);

  webserver.on("/upload", HTTP_POST, []() {
    webserver.send(200);
  }, save_uploaded_file);

  webserver.on("/softwareupdate", HTTP_GET, []() {
    LOG("webserver: handling /softwareupdate");
    webserver.sendHeader("Connection", "close");
    webserver.send(200, "text/html", softwareupdate);
  });

  webserver.on("/update", HTTP_POST, []() {
    webserver.sendHeader("Connection", "close");
    webserver.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = webserver.upload();

    if (upload.status == UPLOAD_FILE_START) {
      VLOG("update: %s", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        VLOG(Update.errorString());
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        VLOG(Update.errorString());
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        VLOG("update success: %u bytes, rebooting...", upload.totalSize);
      } else {
        VLOG(Update.errorString());
      }
    }
  });

  webserver.on("/file_upload", HTTP_GET, []() {
    LOG("webserver: handling /file_upload - GET");

    webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webserver.send(200, "text/html", "");
    webserver.sendContent(htmlHeader);
    webserver.sendContent(htmlTitle);
    webserver.sendContent(htmlFileupload);
    webserver.sendContent(htmlFooter);
    webserver.sendContent("");
    webserver.client().stop();
  });

  webserver.on("/file_upload", HTTP_POST, []() {
    LOG("webserver: handling /file_upload - POST");
    webserver.send(200);
  },
  handle_file_upload);

  VLOG("setup: webserver started");
}

//
/// upload a new file to SPIFFS
//

void handle_file_upload(void) {

  HTTPUpload& upload = webserver.upload();

  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    VLOG("webserver: handle_file_upload, filename = %s", filename.c_str());
    fsUploadFile = SPIFFS.open(filename, FILE_WRITE);
    filename = String();

  } else if (upload.status == UPLOAD_FILE_WRITE) {

    if (fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);

  } else if (upload.status == UPLOAD_FILE_END) {

    if (fsUploadFile) {
      fsUploadFile.close();
      VLOG("webserver: handle_file_upload, complete, size = %d", upload.totalSize);
      webserver.sendHeader("Location", "/success");
      webserver.send(303);
    } else {
      webserver.send(500, "text/plain", "500: couldn't create file");
    }
  }
}

//
/// stream a file
//

bool handle_file_read(String path) {

  VLOG("webserver: handle_file_read, path = %s", path.c_str());

  if (path.endsWith("/")) path += "index.html";

  if (SPIFFS.exists(path)) {
    String contentType = get_content_type(path);
    File file = SPIFFS.open(path, FILE_READ);
    size_t sent = webserver.streamFile(file, contentType);
    file.close();
    VLOG("webserver: handle_file_read, sent file = %s, bytes = %d", path.c_str(), sent);
    return true;
  }

  VLOG("webserver: handle_file_read, file not found = %s", path.c_str());
  return false;
}

String get_content_type(String filename) {

  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

//
/// webserver page handlers
//

void handle_root(void) {

  LOG("webserver: handling /");
  PULSE_LED(NET_ACT_LED);

  String tmp = String(htmlMenu);

  tmp.replace("{{mdnsname}}", mdnsname);

  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send(200, "text/html", "");
  webserver.sendContent(htmlHeader);
  webserver.sendContent(htmlTitle);
  webserver.sendContent(tmp);
  webserver.sendContent(htmlFooter);
  webserver.sendContent("");
  webserver.client().stop();

  return;
}

void handle_config(void) {

  LOG("webserver: handling /config");
  PULSE_LED(NET_ACT_LED);

  // update template variables
  String tmp = String(htmlConfig);

  tmp.replace("{{mdnsname}}", mdnsname);

  tmp.replace("{{network_number}}", String(config_data.network_number));
  tmp.replace("{{slave_number}}", String(config_data.slave_number));
  tmp.replace("{{gc_server_port}}", String(config_data.gc_server_port));
  tmp.replace("{{ser_port}}", String(config_data.ser_port));
  tmp.replace("{{ssid}}", String(config_data.ssid));
  tmp.replace("{{pwd}}", String(config_data.pwd));
  tmp.replace("{{withrottle_port}}", String(config_data.withrottle_port));
  tmp.replace("{{peer_err_limit}}", String(config_data.peer_err_limit));
  tmp.replace("{{network_password}}", String(config_data.network_password));
  tmp.replace("{{softap_password}}", String(config_data.softap_password));
  tmp.replace("{{sleep_delay_mins}}", String(config_data.sleep_delay_mins));
  tmp.replace("{{default_wifi_channel}}", String(config_data.default_wifi_channel));
  tmp.replace("{{low_battery_threshold}}", String(config_data.low_battery_threshold));
  tmp.replace("{{canid}}", String(config_data.CANID));
  tmp.replace("{{node_number}}", String(config_data.node_number));
  tmp.replace("{{touch_threshold}}", String(config_data.touch_threshold));

  if (config_data.role == ROLE_SLAVE) {
    tmp.replace("{{master_selected}}", "");
    tmp.replace("{{slave_selected}}", "checked");
  } else {
    tmp.replace("{{master_selected}}", "checked");
    tmp.replace("{{slave_selected}}", "");
  }

  if (config_data.bridge_mode) {
    tmp.replace("{{transparent_selected}}", "");
    tmp.replace("{{filtering_selected}}", "checked");
  } else {
    tmp.replace("{{transparent_selected}}", "checked");
    tmp.replace("{{filtering_selected}}", "");
  }

  if (config_data.config_mode) {
    tmp.replace("{{browser_selected}}", "");
    tmp.replace("{{switches_selected}}", "checked");
  } else {
    tmp.replace("{{browser_selected}}", "checked");
    tmp.replace("{{switches_selected}}", "");
  }

  if (switches_present) {
    tmp.replace("{{switches_present}}", "");
    tmp.replace("{{switch_val}}", String(mcp.get_port_state_as_char()));
  } else {
    tmp.replace("{{switches_present}}", "not ");
    tmp.replace("{{switch_val}}", "");
  }

  if (config_data.gc_server_on) {
    tmp.replace("{{gc_server_on}}", "checked");
  } else {
    tmp.replace("{{gc_server_on}}", "");
  }

  if (config_data.gc_serial_on) {
    tmp.replace("{{gc_serial_on}}", "checked");
  } else {
    tmp.replace("{{gc_serial_on}}", "");
  }

  if (config_data.ser_on) {
    tmp.replace("{{ser_on}}", "checked");
  } else {
    tmp.replace("{{ser_on}}", "");
  }

  if (config_data.debug) {
    tmp.replace("{{debug}}", "checked");
  } else {
    tmp.replace("{{debug}}", "");
  }

  if (config_data.wifi_connect) {
    tmp.replace("{{wifi_connect}}", "checked");
  } else {
    tmp.replace("{{wifi_connect}}", "");
  }

  if (config_data.withrottle_on) {
    tmp.replace("{{withrottle_on}}", "checked");
  }

  if (config_data.cmdproxy_on) {
    tmp.replace("{{cmdproxy_on}}", "checked");
  }

  switch (config_data.dcc_type) {
    case DCC_MERG:
      tmp.replace("{{merg_dcc}}", "checked");
      tmp.replace("{{dccpp_dcc}}", "");
      break;
    case DCC_DCCPP:
      tmp.replace("{{merg_dcc}}", "");
      tmp.replace("{{dccpp_dcc}}", "checked");
      break;
    default:
      tmp.replace("{{merg_dcc}}", "");
      tmp.replace("{{dccpp_dcc}}", "");
      break;
  }

  if (config_data.slave_send_battery) {
    tmp.replace("{{slave_send_battery}}", "checked");
  } else {
    tmp.replace("{{slave_send_battery}}", "");
  }

  if (config_data.send_estop_on_sleep) {
    tmp.replace("{{send_estop_on_sleep}}", "checked");
  } else {
    tmp.replace("{{send_estop_on_sleep}}", "");
  }

  if (config_data.use_network_password) {
    tmp.replace("{{use_network_password}}", "checked");
  } else {
    tmp.replace("{{use_network_password}}", "");
  }

  if (config_data.forward_battery_msgs_to_cbus) {
    tmp.replace("{{forward_battery_msgs_to_cbus}}", "checked");
  } else {
    tmp.replace("{{forward_battery_msgs_to_cbus}}", "");
  }

  switch (config_data.wakeup_source) {
    case WAKE_TOUCH:
      tmp.replace("{{wakeup_touch}}", "checked");
      tmp.replace("{{wakeup_switch}}", "");
      break;
    case WAKE_SWITCH:
      tmp.replace("{{wakeup_touch}}", "");
      tmp.replace("{{wakeup_switch}}", "checked");
      break;
    default:
      tmp.replace("{{wakeup_touch}}", "");
      tmp.replace("{{wakeup_switch}}", "");
      break;
  }

  if (config_data.cbus_mode == CBUS_MODE_FLIM) {
    tmp.replace("{{cbus_led}}", "yellow");
  } else {
    tmp.replace("{{cbus_led}}", "green");
  }

  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send(200, "text/html", "");
  webserver.sendContent(htmlHeader);
  webserver.sendContent(htmlTitle);
  webserver.sendContent(tmp);
  webserver.sendContent(htmlFooter);
  webserver.sendContent("");
  webserver.client().stop();

  return;
}

void handle_store(void) {

  LOG("webserver: handling /store");
  PULSE_LED(NET_ACT_LED);

  // update config
  config_data.role = (webserver.arg("role") == "master") ? ROLE_MASTER : ROLE_SLAVE;
  config_data.network_number = webserver.arg("network_number").toInt();
  config_data.config_mode = (webserver.arg("config_mode") == "browser") ? CONFIG_USES_SW : CONFIG_USES_HW;
  config_data.bridge_mode = (webserver.arg("bridge_mode") == "transparent") ? TRANSPARENT_MODE : SPLIT_BUS;
  config_data.slave_number = webserver.arg("slave_number").toInt();
  config_data.gc_server_on = (webserver.arg("gc_server_on") == "on") ? true : false;
  config_data.gc_server_port = webserver.arg("gc_server_port").toInt();
  config_data.gc_serial_on = (webserver.arg("gc_serial_on") == "on") ? true : false;
  config_data.ser_on = (webserver.arg("ser_on") == "on") ? true : false;
  config_data.ser_port = webserver.arg("ser_port").toInt();
  config_data.debug = (webserver.arg("debug") == "on") ? true : false;
  config_data.wifi_connect = (webserver.arg("wifi_connect") == "on") ? true : false;
  strcpy(config_data.ssid, webserver.arg("ssid").c_str());
  strcpy(config_data.pwd, webserver.arg("pwd").c_str());
  config_data.withrottle_on = (webserver.arg("withrottle_on") == "on") ? true : false;
  config_data.withrottle_port = webserver.arg("withrottle_port").toInt();
  config_data.slave_send_battery = (webserver.arg("slave_send_battery") == "on") ? true : false;
  config_data.peer_err_limit = webserver.arg("peer_err_limit").toInt();
  config_data.use_network_password = (webserver.arg("use_network_password") == "on") ? true : false;
  strcpy(config_data.network_password, webserver.arg("network_password").c_str());
  strcpy(config_data.softap_password, webserver.arg("softap_password").c_str());
  config_data.sleep_delay_mins = webserver.arg("sleep_delay_mins").toInt();
  config_data.default_wifi_channel = webserver.arg("default_wifi_channel").toInt();
  config_data.low_battery_threshold = webserver.arg("low_battery_threshold").toInt();
  // config_data.battery_monitor_msg_en = webserver.arg("battery_monitor_msg_en").toInt();
  config_data.send_estop_on_sleep = (webserver.arg("send_estop_on_sleep") == "on") ? true : false;
  config_data.forward_battery_msgs_to_cbus = (webserver.arg("forward_battery_msgs_to_cbus") == "on") ? true : false;
  config_data.cmdproxy_on = (webserver.arg("cmdproxy_on") == "on") ? true : false;
  config_data.CANID = webserver.arg("canid").toInt();
  config_data.node_number = webserver.arg("node_number").toInt();
  config_data.touch_threshold = webserver.arg("touch_threshold").toInt();

  if (webserver.arg("node_number").toInt() > 0) {
    config_data.cbus_mode = CBUS_MODE_FLIM;
  } else {
    config_data.cbus_mode = CBUS_MODE_SLIM;
  }

  if (strcmp(webserver.arg("wakeup_source").c_str(), "touch") == 0) {
    config_data.wakeup_source = WAKE_TOUCH;
  } else if (strcmp(webserver.arg("wakeup_source").c_str(), "switch") == 0) {
    config_data.wakeup_source = WAKE_SWITCH;
  } else {
    config_data.wakeup_source = WAKE_NEITHER;
  }

  if (strcmp(webserver.arg("dcc_backend").c_str(), "merg") == 0) {
    config_data.dcc_type = DCC_MERG;
  } else if (strcmp(webserver.arg("dcc_backend").c_str(), "dccpp") == 0) {
    config_data.dcc_type = DCC_DCCPP;
  } else {
    config_data.dcc_type = DCC_UNK;
  }

  // indicates a valid config
  config_data.guard_val = 99;

  // save config to EEPROM
  save_config();
  LOG("webserver: config data saved");

  webserver.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  webserver.sendHeader("Pragma", "no-cache");
  webserver.sendHeader("Expires", " - 1");

  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send(200, "text/plain", "");
  webserver.sendContent("\n\nConfiguration saved\n\n");
  webserver.sendContent("\n\nRestarting ...");
  webserver.sendContent("");

  // close connection to client browser
  webserver.client().stop();

  // set restart flag; restart happens a little later in main loop
  do_restart = true;
  return;
}

//
/// system info page
//

void handle_info(void) {

  String tmp, st;
  char tbuff[64];
  int num_files = 0;

  VLOG("webserver: handling %s", webserver.uri().c_str());
  PULSE_LED(NET_ACT_LED);

  byte mins = (millis() / 1000 / 60) % 60;
  byte hrs = millis() / 1000 / 60 / 60;
  tmp = htmlInfo;

  snprintf(tbuff, sizeof(tbuff), "%d.%d.%d", VER_MAJ, VER_MIN, VER_PATCH);
  tmp.replace("{{version}}", tbuff);

  snprintf(tbuff, sizeof(tbuff), "%02d:%02d", hrs, mins);
  tmp.replace("{{uptime}}", tbuff);

  /// network info

  switch (WiFi.status()) {
    case WL_IDLE_STATUS:
      st = "Idle";
      break;
    case WL_NO_SSID_AVAIL:
      st = "SSID not available";
      break;
    case WL_SCAN_COMPLETED:
      st = "Scan completed";
      break;
    case WL_CONNECTED:
      st = "Connected";
      break;
    case WL_CONNECT_FAILED:
      st = "Connect failed";
      break;
    case WL_CONNECTION_LOST:
      st = "Connection lost";
      break;
    case WL_DISCONNECTED:
      st = "Disconnected";
      break;
    default:
      st = "Unknown";
      break;
  }

  tmp.replace("{{ssid}}", WiFi.SSID());
  tmp.replace("{{ip}}", WiFi.localIP().toString());
  tmp.replace("{{gateway}}", WiFi.gatewayIP().toString());
  tmp.replace("{{subnet}}", WiFi.subnetMask().toString());
  tmp.replace("{{stat}}", st);

  tmp.replace("{{sta_mac}}", WiFi.macAddress());
  tmp.replace("{{ap_mac}}", WiFi.softAPmacAddress());

  tmp.replace("{{cbus_mode}}", (config_data.cbus_mode) ? "FLiM" : "SLiM");
  tmp.replace("{{cbus_canid}}", String(config_data.CANID));
  tmp.replace("{{cbus_node_number}}", String(config_data.node_number));

  /// filesystem info

  tmp += "<hr>File directory:<br>";

  File dir = SPIFFS.open("/");
  File f = dir.openNextFile();

  while (f) {
    snprintf(tbuff, sizeof(tbuff), "%s, %d</br>", f.name(), f.size());
    tmp += String(tbuff);
    ++num_files;
    f.close();
    f = dir.openNextFile();
  }

  dir.close();

  size_t free_bytes = SPIFFS.totalBytes() - SPIFFS.usedBytes();
  snprintf(tbuff, sizeof(tbuff), "%d files, bytes total = %d, used = %d, free = %d", num_files, SPIFFS.totalBytes(), SPIFFS.usedBytes(), free_bytes);
  tmp += String(tbuff) + "<hr>";

  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send(200, "text/html", "");
  webserver.sendContent(htmlHeader);
  webserver.sendContent(htmlTitle);
  webserver.sendContent(tmp);
  webserver.sendContent(htmlFooter);
  webserver.sendContent("");
  webserver.client().stop();

  return;
}

//
/// system stats
//

void handle_stats(void) {

  String tmp;
  char tmpbuff[100];

  LOG("webserver: handling /stats");
  PULSE_LED(NET_ACT_LED);

  tmp = String(htmlStats);

  tmp.replace("{{can_tx}}", String(stats.can_tx));
  tmp.replace("{{can_rx}}", String(stats.can_rx));
  tmp.replace("{{net_tx}}", String(stats.net_tx));
  tmp.replace("{{net_rx}}", String(stats.net_rx));
  tmp.replace("{{gc_tx}}", String(stats.gc_tx));
  tmp.replace("{{gc_rx}}", String(stats.gc_rx));

  tmp.replace("{{can_tx_err}}", String(errors.can_tx));
  tmp.replace("{{can_rx_err}}", String(errors.can_rx));
  tmp.replace("{{net_tx_err}}", String(errors.net_tx));
  tmp.replace("{{net_rx_err}}", String(errors.net_rx));
  tmp.replace("{{gc_tx_err}}", String(errors.gc_tx));
  tmp.replace("{{gc_rx_err}}", String(errors.gc_rx));

  tmp += "<h3>ESP-NOW satellites:</h3>";

  for (byte i = 0; i < MAX_NET_PEERS; i++) {
    if (peers[i].mac_addr[0] != 0) {
      sprintf(tmpbuff, "[%2d] %s, CANID = %d, tx = %d, rx = %d, errs = %d, battery = %d", i, mac_to_char(peers[i].mac_addr), peers[i].CANID, peers[i].tx, peers[i].rx, peers[i].num_errs, peers[i].battery_mv);
      tmp += String(tmpbuff);
      tmp += "<br/>";
    }
  }

  tmp += "<h3>Gridconnect clients:</h3>";

  for (byte i = 0; i < MAX_GC_CLIENTS; i++) {
    if (gc_clients[i].client != NULL) {
      sprintf(tmpbuff, "[%2d] %s, %d", i, gc_clients[i].addr, gc_clients[i].port);
      tmp += String(tmpbuff);
      tmp += "<br/>";
    }
  }

  tmp += "<hr>";
  tmp += "<h3>Task stack sizes:</h3>";

  for (byte i = 0; i < (sizeof(task_list) / sizeof(task_info_t)); i++) {
    snprintf(tmpbuff, sizeof(tmpbuff), "%s: %d, %d, %d", task_list[i].name, task_list[i].stack_size, task_list[i].hwm, task_list[i].prev_hwm);
    tmp += String(tmpbuff);
    tmp += "<br/>";
  }

  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send(200, "text/html", "");
  webserver.sendContent(htmlHeader);
  webserver.sendContent(htmlTitle);
  webserver.sendContent(tmp);
  webserver.sendContent(htmlFooter);
  webserver.sendContent("");
  webserver.client().stop();

  return;
}

void handle_stop(void) {

  LOG("webserver: handling /stop");
  PULSE_LED(NET_ACT_LED);

  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send(200, "text/html", "");
  webserver.sendContent(htmlHeader);
  webserver.sendContent(htmlTitle);
  webserver.sendContent(htmlSleep);
  webserver.sendContent(htmlFooter);
  webserver.sendContent("");
  webserver.client().stop();

  return;
}

void do_deepsleep(void) {

  LOG("webserver: handling /do_deepsleep");
  PULSE_LED(NET_ACT_LED);

  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send(200, "text/html", "");
  webserver.sendContent(htmlHeader);
  webserver.sendContent(htmlTitle);
  webserver.sendContent("Device will deep sleep in 5 seconds");
  webserver.sendContent(htmlFooter);
  webserver.sendContent("");
  webserver.client().stop();

  LOG("webserver: deep sleep in 5 seconds");
  device_sleep();
}

void handle_restart(void) {

  LOG("webserver: handling /restart");
  PULSE_LED(NET_ACT_LED);

  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send(200, "text/html", "");
  webserver.sendContent(htmlHeader);
  webserver.sendContent(htmlTitle);
  webserver.sendContent("Device will restart in 5 seconds");
  webserver.sendContent(htmlFooter);
  webserver.sendContent("");
  webserver.client().stop();

  VLOG("webserver: restart in 5 seconds");
  do_restart = true;
  return;
}

void handle_default(void) {

  LOG("webserver: handling /default_config");
  PULSE_LED(NET_ACT_LED);

  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send(200, "text/html", "");
  webserver.sendContent(htmlHeader);
  webserver.sendContent(htmlTitle);
  webserver.sendContent("Not implemented yet");
  webserver.sendContent(htmlFooter);
  webserver.sendContent("");
  webserver.client().stop();

  return;
}

void save_uploaded_file(void) {

  LOG("save_uploaded_file: saving file");

  HTTPUpload& upload = webserver.upload();

  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;

    if (!filename.startsWith("/"))
      filename = "/" + filename;

    VLOG("save_uploaded_file, fname = %s", filename.c_str());

    fsUploadFile = SPIFFS.open(filename, FILE_WRITE);
    filename = String();

  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);

  } else if (upload.status == UPLOAD_FILE_END) {
    if (fsUploadFile) {
      fsUploadFile.close();
      VLOG("save_uploaded_file: complete, size = %d", upload.totalSize);
      webserver.sendHeader("Location", "/success.html");
      webserver.send(303);
    } else {
      webserver.send(500, "text/plain", "500: couldn't create file");
    }
  }

  webserver.client().stop();
  return;
}

//
/// default handler for any URI not otherwise handled
//

void handle_notfound(void) {

  String mimetype = get_content_type(webserver.uri());

  VLOG("webserver: handle_notfound: handling %s, mimetype = %s", webserver.uri().c_str(), mimetype.c_str());
  PULSE_LED(NET_ACT_LED);

  if (SPIFFS.exists(webserver.uri())) {
    File f = SPIFFS.open(webserver.uri(), FILE_READ);

    if (!f) {
      VLOG("webserver: handle_notfound: error opening file = %s", webserver.uri().c_str());
    } else {
      webserver.streamFile(f, mimetype);
      f.close();
    }
  } else {
    VLOG("webserver: handle_notfound: file not found = %s", webserver.uri().c_str());
    webserver.send(404);
  }

  webserver.client().stop();
  return;
}

//
/// CBUS button triggers RQNN request for a new node number - master only
//

void handle_rqnn(void) {

  LOG("webserver: handling /rqnn");

  if (config_data.role == ROLE_MASTER) {
    transition_to_flim();
  }

  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send(200, "text/html", "");
  webserver.sendContent("Ok");
  webserver.sendContent("");
  webserver.client().stop();

  return;
}

//
/// enumerate CAN bus - master only
//

void handle_enum(void) {

  LOG("webserver: handling /enum");

  if (config_data.role == ROLE_MASTER) {
    enum_required = true;
  }

  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send(200, "text/html", "");
  webserver.sendContent("Ok");
  webserver.sendContent("");
  webserver.client().stop();

  return;
}

void handle_success() {

  LOG("webserver: handling /success");
  PULSE_LED(NET_ACT_LED);

  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send(200, "text/html", "");
  webserver.sendContent(htmlHeader);
  webserver.sendContent(htmlTitle);
  webserver.sendContent(htmlSuccess);
  webserver.sendContent(htmlFooter);
  webserver.sendContent("");
  webserver.client().stop();

  return;
}

void handle_delete_file(void) {

  String tmp = "";

  VLOG("webserver: handling /delete_file, filename = %s", webserver.arg("filename").c_str());
  PULSE_LED(NET_ACT_LED);

  if (SPIFFS.exists(webserver.arg("filename"))) {
    SPIFFS.remove(webserver.arg("filename"));
    tmp = "File deleted";
  } else {
    VLOG("webserver: file does not exist");
    tmp = "File does not exist";
  }

  webserver.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webserver.send(200, "text/html", "");
  webserver.sendContent(htmlHeader);
  webserver.sendContent(htmlTitle);
  webserver.sendContent(tmp);
  webserver.sendContent(htmlFooter);
  webserver.sendContent("");
  webserver.client().stop();

  return;
}
