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

#include "switch.h"

void Switch::setPin(byte pin, byte pressedState = LOW) {

  _pin = pin;
  _pressedState = pressedState;

  if (_pressedState == LOW)
    pinMode(_pin, INPUT_PULLUP);

  reset();
  _currentState = readPin(_pin);
}

void Switch::reset(void) {

  _lastState = !_pressedState;
  _stateChanged = false;
  _lastStateChangeTime = 0UL;
  _lastStateDuration = 0UL;
  _prevReleaseTime = 0UL;
  _prevStateDuration = 0UL;
}

byte Switch::readPin(byte pin) {

  return digitalRead(pin);
}

void Switch::run(void) {

  // check for state change

  // read the pin
  _currentState = readPin(_pin);

  // has state changed ?
  if (_currentState != _lastState) {

    // yes - state has changed since last call to this method
    // Serial << endl << F("  -- switch state has changed state from ") << _lastState << " to " << _currentState << endl;
    // Serial << F("  -- last state change at ") << _lastStateChangeTime << ", diff = " << millis() - _lastStateChangeTime << endl;

    _lastState = _currentState;
    _prevStateDuration = _lastStateDuration;
    _lastStateDuration = millis() - _lastStateChangeTime;
    _lastStateChangeTime = millis();
    _stateChanged = true;

    if (_currentState == _pressedState) {
      // Serial << F("  -- switch has been pressed") << endl;
    } else {
      // Serial << F("  -- switch has been released") << endl;

      // double-click detection
      // two clicks of less than 250ms, less than 500ms apart

      // Serial << F("  -- last state duration = ") << _lastStateDuration << endl;
      // Serial << F("  -- this release = ") << _lastStateChangeTime << F(", last release = ") << _prevReleaseTime << endl;

      // save release time
      _prevReleaseTime = _lastStateChangeTime;
    }

  } else {

    // no -- state has not changed
    _stateChanged = false;
  }

  return;
}

bool Switch::stateChanged(void) {

  // has switch state changed ?
  // Serial << F("  -- switch state changed = ") << _stateChanged << endl;
  return _stateChanged;
}

bool Switch::getState(void) {

  // return the current switch state read
  // Serial << F("  -- switch state = ") << _currentState << endl;
  return _currentState;
}

bool Switch::isPressed(void) {

  // is the switch pressed ?
  // Serial << F("  -- switch pressed ? = ") << (_currentState == _pressedState) << endl;
  return (_currentState == _pressedState);
}

unsigned long Switch::getCurrentStateDuration(void) {

  // how long has the switch been in its current state ?
  // Serial << F("  -- current state duration = ") << (millis() - _lastStateChangeTime) << endl;
  return (millis() - _lastStateChangeTime);
}

unsigned long Switch::getLastStateDuration(void) {

  // how long was the last state active for ?
  // Serial << F("  -- last state duration = ") << _lastStateDuration << endl;
  return _lastStateDuration;
}

unsigned long Switch::getLastStateChangeTime(void) {

  // when was the last state change ?
  // Serial << F("  -- last state change at ") << _lastStateChangeTime << endl;
  return _lastStateChangeTime;
}

void Switch::resetCurrentDuration(void) {

  // reset the state duration counter
  _lastStateChangeTime = millis();
  return;
}
