
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
#include <Wire.h>
#include "defs.h"
#include "cbusdefs.h"

#define BATT_READ_FREQ 10000
#define NUM_BATT_STATS 6

#define FUEL_GAUGE_ADDR 0x36
#define VCELL_REGISTER 0x02
#define SOC_REGISTER 0x04
#define MODE_REGISTER 0x06
#define VERSION_REGISTER 0x08
#define CONFIG_REGISTER 0x0C
#define COMMAND_REGISTER 0xFE

// variables declared in other source files
extern QueueHandle_t logger_in_queue, battery_monitor_queue, net_out_queue, led_cmd_queue, CAN_out_from_net_queue, gc_out_queue;
extern config_t config_data;
extern byte num_gc_clients, num_wi_clients, slave_canid;
extern peer_state_t peers[MAX_NET_PEERS];

// forward function declarations
double read_adc_voltage(byte pin);
unsigned int read_fg_soc(void);
unsigned int read_fg_voltage(void);
void send_cbus_battery_message(int i);
void do_low_battery_check(unsigned int battery_mv);

//
/// battery monitor task
//

void battery_monitor_task(void *params) {

  byte array_index = 0, data_points;
  bool fuel_gauge_present = false;
  char tbuff[sizeof(twai_message_t)];
  unsigned int average_voltage = 0, soc = 0;
  unsigned int battery_reading_mv, stats_array[NUM_BATT_STATS];
  double voltage_reading;

  LOG("battery_monitor_task: task starting");

  // if we're a slave, we monitor the battery and send updates to the master
  // if we're the master, we receive, forward, and possibly display, battery status updates from slaves

  // initialise stats array
  for (byte i = 0; i < NUM_BATT_STATS; i++) {
    stats_array[i] = 0;
  }

  // determine whether MAX17048 battery fuel gauge chip is present
  Wire.begin();
  LOG("battery_monitor_task: looking for MAX17048 fuel gauge");

  Wire.beginTransmission(FUEL_GAUGE_ADDR);
  if (Wire.endTransmission() == 0) {
    LOG("battery_monitor_task: fuel gauge is present");
    fuel_gauge_present = true;

    // reset
    Wire.beginTransmission(FUEL_GAUGE_ADDR);
    Wire.write(COMMAND_REGISTER);
    Wire.write(0x00);
    Wire.write(0x54);
    Wire.endTransmission();

    // quickstart
    Wire.beginTransmission(FUEL_GAUGE_ADDR);
    Wire.write(MODE_REGISTER);
    Wire.write(0x40);
    Wire.write(0x00);
    Wire.endTransmission();

  } else {
    LOG("battery_monitor_task: fuel gauge is not present");
    fuel_gauge_present = false;
  }

  //
  /// main loop
  //

  for (;;) {

    // sleep until next reading due
    vTaskDelay(BATT_READ_FREQ);

    //
    /// master node
    //

    if (config_data.role == ROLE_MASTER) {
      if (config_data.forward_battery_msgs_to_cbus) {
        LOG("battery_monitor_task: master: sending slave battery data messages");

        for (byte i = 0; i < MAX_NET_PEERS; i++) {
          if (peers[i].mac_addr[0] != 0) {
            send_cbus_battery_message(i);
            vTaskDelay(5);
          }
        }
      }
    }

    //
    /// slave node
    //

    if (config_data.role == ROLE_SLAVE) {

      if (!fuel_gauge_present) {

        // we use the simple resistor divider to measure the battery voltage
        // value must be doubled because it is read from a 1:1 resistive divider, as the voltage would otherwise exceed 3.3V

        LOG("battery_monitor_task: slave, reading ADC");
        voltage_reading = read_adc_voltage(BAT_MON_PIN);
        battery_reading_mv = (voltage_reading * 2000.0);
        VLOG("battery_monitor_task: slave: battery reading = %f, mV = %d", voltage_reading, battery_reading_mv);

        // add reading to stats array at current index & increment index
        stats_array[array_index] = battery_reading_mv;
        array_index = (array_index + 1) % NUM_BATT_STATS;

        // calculate moving average - exclude zero values
        average_voltage = 0;
        data_points = 0;

        for (byte i = 0; i < NUM_BATT_STATS; i++) {
          if (stats_array[i] > 0) {
            average_voltage += stats_array[i];
            ++data_points;
          }
        }

        average_voltage = average_voltage / data_points;
        VLOG("battery_monitor_task: slave: moving average mV = %d", average_voltage);

        // send message for onward transmission to master
        sprintf(tbuff, "MV%4d", average_voltage);
        send_message_to_queues(QUEUE_NET_OUT, tbuff, "battery_monitor_task", QUEUE_OP_TIMEOUT_LONG);

        do_low_battery_check(average_voltage);

      } else {

        // we use the MAX17048 fuel gauge to read the battery voltage and state of charge
        LOG("battery_monitor_task: slave, reading fuel gauge");

        average_voltage = read_fg_voltage();
        VLOG("battery_monitor_task: fg voltage = %d", average_voltage);
        snprintf(tbuff, sizeof(tbuff), "MV%4d", average_voltage);
        send_message_to_queues(QUEUE_NET_OUT, tbuff, "battery_monitor_task", QUEUE_OP_TIMEOUT_LONG);

        soc = read_fg_soc();
        VLOG("battery_monitor_task: fg soc = %d", soc);
        snprintf(tbuff, sizeof(tbuff), "SO%3d", soc);
        send_message_to_queues(QUEUE_NET_OUT, tbuff, "battery_monitor_task", QUEUE_OP_TIMEOUT_LONG);

        do_low_battery_check(average_voltage);
      }   // fuel gauge present
    }   // is slave
  }   // for (;;)
}

// read battery voltage from fuel gauge, register VCELL 0x02

unsigned int read_fg_voltage(void) {

  Wire.beginTransmission(FUEL_GAUGE_ADDR);
  Wire.write(VCELL_REGISTER);
  Wire.endTransmission();

  Wire.requestFrom(FUEL_GAUGE_ADDR, 2);
  byte msb = Wire.read();
  byte lsb = Wire.read();

  unsigned int value = (msb << 4) | (lsb >> 4);
  return map(value, 0x000, 0xFFF, 0, 50000) / 10;
}

// read battery state-of-charge from fuel gauge, register SOC 0x04

unsigned int read_fg_soc(void) {

  Wire.beginTransmission(FUEL_GAUGE_ADDR);
  Wire.write(SOC_REGISTER);
  Wire.endTransmission();

  Wire.requestFrom(FUEL_GAUGE_ADDR, 2);
  byte msb = Wire.read();
  byte lsb = Wire.read();

  return (unsigned int)(msb + (lsb / 256));
}

//
/// warn or sleep if battery level is below minimum
//

void do_low_battery_check(unsigned int battery_mv) {

  led_command_t lc;

  // blink error LED if battery voltage is close (+500mV) to sleep threshold
  if (battery_mv <= (config_data.low_battery_threshold + 500)) {
    VLOG("battery_monitor_task: slave: battery = %d is below warning level", battery_mv);
    lc.cmd = LED_FAST_BLINK;
  } else {
    lc.cmd = LED_OFF;
  }

  lc.led = ERR_IND_LED;
  xQueueSend(led_cmd_queue, &lc, QUEUE_OP_TIMEOUT);

  // sleep if battery below minimum
  if (battery_mv <= config_data.low_battery_threshold) {
    VLOG("battery_monitor_task: slave: battery = %d at minimum, will sleep", battery_mv, config_data.low_battery_threshold);
    device_sleep();
  }

  return;
}

//
/// read the battery voltage through an ADC pin
/// from: https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function/blob/master/ESP32_ADC_Read_Voltage_Accurate.ino
//

double read_adc_voltage(byte pin) {

  double reading = analogRead(pin);
  return -0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089;
}
