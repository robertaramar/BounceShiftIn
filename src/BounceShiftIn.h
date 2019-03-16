/*
  The MIT License (MIT)

  Copyright (c) 2013 thomasfredericks

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  * * * Enhanced to read pins from shift-in by Robert Schneider
  (Robert.Schneider@aramar.de) Main code by Thomas O Fredericks (tof@t-o-f.info)
  Previous contributions by Eric Lowry, Jim Schimpf and Tom Harkaway
  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  * * * */

#ifndef BounceShiftIn_h
#define BounceShiftIn_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// Uncomment the following line for "LOCK-OUT" debounce method
//#define BOUNCE_LOCK_OUT

// Uncomment the following line for "BOUNCE_WITH_PROMPT_DETECTION" debounce
// method
//#define BOUNCE_WITH_PROMPT_DETECTION

#include <inttypes.h>

/**
    @example bounce.ino
    Simple example of the BounceShiftInlibrary that switches the debug LED when
   a button is pressed.
*/

/**
    @example change.ino
    This example toggles the debug LED (pin 13) on or off when a button on pin 2
   is pressed.
*/

/**
    @example bounce_multiple.ino
    Detect the falling edge of multiple buttons. Eight buttons with internal
   pullups. Toggles a LED when any button is pressed. Buttons on pins
   2,3,4,5,6,7,8,9
*/

/**
    @example bounce2buttons.ino
     Example of two instances of the BounceShiftInclass that switches the debug
   LED when either one of the two buttons is pressed.
 */

/**
     The BounceShiftInclass.
     */
class BounceShiftIn {
 public:
  /*!
      @brief  Create an instance of the BounceShiftIn class.

      @code

      // Create an instance of the BounceShiftIn class.
      BounceShiftIn() button;

      @endcode
  */
  BounceShiftIn();

  /**
   * Obtain the current state of the shift-in registers and store it in
   * currentState.
   */
  static uint16_t loop();

  /**
   * Set the pin numbers for talking to the shift-in register.
   */
  static void setup(uint8_t latch, uint8_t clock, uint8_t data);

  /*!
      @brief  Attach to a pin and sets that pin's mode (INPUT, INPUT_PULLUP or
     OUTPUT).

      @param    pin
                The pin that is to be debounced.
      @param    mode
                A valid Arduino pin mode (INPUT, INPUT_PULLUP or OUTPUT).
      @return True if the event read was successful, otherwise false.
  */
  void attach(int pin, int mode);

  /**
  Attach to a pin for advanced users. Only attach the pin this way once you have
  previously set it up. Otherwise use attach(int pin, int mode).
  */
  void attach(int pin);

  /**
  @brief  Sets the debounce interval in milliseconds.

  @param    interval_millis
              The interval time in milliseconds.

   */
  void interval(uint16_t interval_millis);

  /*!
      @brief   Updates the pin's state.

      Because BounceShiftIn does not use interrupts, you have to "update" the
     object before reading its value and it has to be done as often as possible
     (that means to include it in your loop()). Only call update() once per
     loop().

      @return True if the pin changed state.
  */

  bool update();

  /**
   @brief Returns the pin's state (HIGH or LOW).

   @return HIGH or LOW.
   */
  bool read();

  /**
  @brief Returns true if pin signal transitions from high to low.
  */
  bool fell();

  /**
  @brief Returns true if pin signal transitions from low to high.
  */
  bool rose();

  /**
   @brief Returns the duration in milliseconds of the current state.

   Is reset to 0 once the pin rises ( rose() ) or falls ( fell() ).

    @return The duration in milliseconds (unsigned long) of the current state.
   */

  unsigned long duration();

 protected:
  // Variables that are used throughout all BounceShiftIn instances
  static uint8_t dataPin;   // data pin of shift-in register chip
  static uint8_t clockPin;  // clock pin of shift-in register chip
  static uint8_t latchPin;  // latch pin of shift-in register chip
  static uint16_t
      currentState;  // last read state of the two shift-in register chips

  unsigned long previous_millis;
  uint16_t interval_millis;
  uint8_t state;
  uint8_t pin;
  unsigned long stateChangeLastTime;
  virtual bool readCurrentState() { return currentState & (1 << pin); }

 private:
  static uint8_t shiftIn();
  inline void changeState();
  inline void setStateFlag(const uint8_t flag) { state |= flag; }
  inline void unsetStateFlag(const uint8_t flag) { state &= ~flag; }
  inline void toggleStateFlag(const uint8_t flag) { state ^= flag; }
  inline bool getStateFlag(const uint8_t flag) { return ((state & flag) != 0); }
};

#endif
