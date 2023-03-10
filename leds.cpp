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
#include "defs.h"

extern QueueHandle_t logger_in_queue, led_cmd_queue;

//
/// implements a task to offload LED timing and control from other tasks
//

void led_task(void *params) {

  /// cmd.val member is reserved for future commands that may need a user-defined value

  led_command_t cmd;
  led_state_t led_states[NUM_LEDS] = {};

  LOG("led_task: task starting");

  // initialise LED states & switch off
  led_states[ERR_IND_LED].pin = ERR_IND_PIN;
  led_states[ARDUINO_LED].pin = ARDUINO_PIN;
  led_states[CAN_ACT_LED].pin = CAN_ACT_PIN;
  led_states[NET_ACT_LED].pin = NET_ACT_PIN;
  led_states[SLIM_LED].pin = SLIM_LED_PIN;
  led_states[FLIM_LED].pin = FLIM_LED_PIN;

  for (byte i = 0; i < NUM_LEDS; i++) {
    led_states[i].type = LED_OFF;
    led_states[i].curr_state = LED_OFF;
    led_states[i].next_state = LED_OFF;
    led_states[i].next_time = 0UL;
    pinMode(led_states[i].pin, OUTPUT);
    digitalWrite(led_states[i].pin, LOW);
  }

  for (;;) {

    // get next command from the input queue
    if (xQueueReceive(led_cmd_queue, &cmd, QUEUE_OP_TIMEOUT_SHORT) == pdTRUE) {

      if (cmd.cmd == LED_PULSE || (led_states[cmd.led].last_cmd != cmd.cmd)) {      // process only if one-shot pulse or different command

        // set LED state
        led_states[cmd.led].type = cmd.cmd;
        led_states[cmd.led].val = cmd.val;
        led_states[cmd.led].last_cmd = cmd.cmd;
        led_states[cmd.led].next_time = 0;                // schedule initial state change immediately

        switch (cmd.cmd) {
          case LED_OFF:                                   // switch off initially
            led_states[cmd.led].curr_state = LED_ON;
            led_states[cmd.led].next_state = LED_OFF;
            break;

          case LED_ON:                                    // switch on initially
          case LED_BLINK:
          case LED_FAST_BLINK:
          case LED_PULSE:
          case LED_LONG_BLINK:
          case LED_SHORT_BLINK:
            led_states[cmd.led].curr_state = LED_OFF;
            led_states[cmd.led].next_state = LED_ON;
            break;

          default:
            LOG("led_task: unknown command");
            break;
        }
      }
    }

    // do state processing; update LEDs as necessary
    for (byte i = 0; i < NUM_LEDS; i++) {

      // if current and next states are different
      if ((led_states[i].curr_state != led_states[i].next_state) && (millis() >= led_states[i].next_time)) {

        // bring curr_state to match next_state & switch the LED to the new curr_state
        led_states[i].curr_state = led_states[i].next_state;
        digitalWrite(led_states[i].pin, led_states[i].curr_state);

        // set next action time
        switch (led_states[i].type) {
          case LED_PULSE:
            led_states[i].next_state = LED_OFF;               // single shot
            led_states[i].next_time = millis() + 5;
            break;

          case LED_BLINK:
            led_states[i].next_state = !led_states[i].curr_state;
            led_states[i].next_time = millis() + 500;
            break;

          case LED_FAST_BLINK:
            led_states[i].next_state = !led_states[i].curr_state;
            led_states[i].next_time = millis() + 150;
            break;

          case LED_LONG_BLINK:
            led_states[i].next_state = !led_states[i].curr_state;
            led_states[i].next_time = millis() + (led_states[i].next_state ? 50 : 950);
            break;

          case LED_SHORT_BLINK:
            led_states[i].next_state = !led_states[i].curr_state;
            led_states[i].next_time = millis() + (led_states[i].next_state ? 990 : 10);
            break;

          default:
            break;

        }   // switch type
      }   // if state change due
    }   // for each led
  }   // for (;;)
}

//
/// convenience function to pulse an LED

void PULSE_LED(byte led) {

  led_command_t lc;

  lc.led = led;
  lc.cmd = LED_PULSE;
  xQueueSend(led_cmd_queue, &lc, QUEUE_OP_TIMEOUT_NONE);
  return;
}
