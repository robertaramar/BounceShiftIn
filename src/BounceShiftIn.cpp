// Please read BounceShiftIn.h for information about the licence and authors

#include "BounceShiftIn.h"

static const uint8_t DEBOUNCED_STATE = 0b00000001;
static const uint8_t UNSTABLE_STATE = 0b00000010;
static const uint8_t CHANGED_STATE = 0b00000100;

uint8_t BounceShiftIn::dataPin = 15;
uint8_t BounceShiftIn::clockPin = 14;
uint8_t BounceShiftIn::latchPin = 13;
uint16_t BounceShiftIn::currentState = 0;

BounceShiftIn::BounceShiftIn()
    : previous_millis(0), interval_millis(10), state(0), pin(0) {}

static void BounceShiftIn::setup(uint8_t latch, uint8_t clock, uint8_t data) {
  latchPin = latch;
  clockPin = clock;
  dataPin = data;
  // define pin modes
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);
}

static uint16_t BounceShiftIn::loop() {
  // Pulse the latch pin:
  // set it to 1 to collect parallel data
  digitalWrite(latchPin, 1);
  // ADDED LINES
  for (int i = 7; i >= 0; i--) {
    digitalWrite(clockPin, LOW);
    delayMicroseconds(1);
    digitalWrite(clockPin, HIGH);
  }
  // ADDED LINES
  // set it to 0 to transmit data serially
  digitalWrite(latchPin, 0);

  // while the shift register is in serial mode
  // collect each shift register into a byte
  // the register attached to the chip comes in first
  uint8_t switchVar1 = BounceShiftIn::shiftIn();
  uint8_t switchVar2 = BounceShiftIn::shiftIn();
  currentState = ((uint16_t)switchVar2 << 8) | switchVar1;

  return currentState;
}

static uint8_t BounceShiftIn::shiftIn() {
  ////// ----------------------------------------shiftIn function
  ///// just needs the location of the data pin and the clock pin
  ///// it returns a byte with each bit in the byte corresponding
  ///// to a pin on the shift register. leftBit 7 = Pin 7 / Bit 0= Pin 0

  int i;
  int pinState = 0;
  uint8_t myDataIn = 0;

  // we will be holding the clock pin high 8 times (0,..,7) at the
  // end of each time through the for loop

  // at the begining of each loop when we set the clock low, it will
  // be doing the necessary low to high drop to cause the shift
  // register's DataPin to change state based on the value
  // of the next bit in its serial information flow.
  // The register transmits the information about the pins from pin 7 to pin 0
  // so that is why our function counts down
  for (i = 7; i >= 0; i--) {
    digitalWrite(clockPin, LOW);
    delayMicroseconds(1);
    pinState = digitalRead(dataPin);
    if (pinState) {
      // set the bit to 0 no matter what
      myDataIn = myDataIn | (1 << i);
    }

    digitalWrite(clockPin, HIGH);
  }

  return myDataIn;
}

void BounceShiftIn::attach(int pin) {
  this->pin = pin;
  state = 0;
  if (readCurrentState()) {
    setStateFlag(DEBOUNCED_STATE | UNSTABLE_STATE);
  }
#ifdef BOUNCE_LOCK_OUT
  previous_millis = 0;
#else
  previous_millis = millis();
#endif
}

void BounceShiftIn::attach(int pin, int mode) { this->attach(pin); }

void BounceShiftIn::interval(uint16_t interval_millis) {
  this->interval_millis = interval_millis;
}

bool BounceShiftIn::update() {
  unsetStateFlag(CHANGED_STATE);
#ifdef BOUNCE_LOCK_OUT

  // Ignore everything if we are locked out
  if (millis() - previous_millis >= interval_millis) {
    bool currentState = readCurrentState();
    if (currentState != getStateFlag(DEBOUNCED_STATE)) {
      previous_millis = millis();
      changeState();
    }
  }

#elif defined BOUNCE_WITH_PROMPT_DETECTION
  // Read the state of the switch port into a temporary variable.
  bool readState = readCurrentState();

  if (readState != getStateFlag(DEBOUNCED_STATE)) {
    // We have seen a change from the current button state.

    if (millis() - previous_millis >= interval_millis) {
      // We have passed the time threshold, so a new change of state is allowed.
      // set the STATE_CHANGED flag and the new DEBOUNCED_STATE.
      // This will be prompt as long as there has been greater than
      // interval_misllis ms since last change of input. Otherwise debounced
      // state will not change again until bouncing is stable for the timeout
      // period.
      changeState();
    }
  }

  // If the readState is different from previous readState, reset the debounce
  // timer - as input is still unstable and we want to prevent new button state
  // changes until the previous one has remained stable for the timeout.
  if (readState != getStateFlag(UNSTABLE_STATE)) {
    // Update Unstable Bit to macth readState
    toggleStateFlag(UNSTABLE_STATE);
    previous_millis = millis();
  }

#else
  // Read the state of the switch in a temporary variable.
  bool currentState = readCurrentState();

  // If the reading is different from last reading, reset the debounce counter
  if (currentState != getStateFlag(UNSTABLE_STATE)) {
    previous_millis = millis();
    toggleStateFlag(UNSTABLE_STATE);
  } else if (millis() - previous_millis >= interval_millis) {
    // We have passed the threshold time, so the input is now stable
    // If it is different from last state, set the STATE_CHANGED flag
    if (currentState != getStateFlag(DEBOUNCED_STATE)) {
      previous_millis = millis();

      changeState();
    }
  }

#endif

  return getStateFlag(CHANGED_STATE);
}
/*
// WIP HELD
unsigned long BounceShiftIn::held() {
        return durationOfPreviousState;
}
*/
unsigned long BounceShiftIn::duration() {
  return (millis() - stateChangeLastTime);
}

inline void BounceShiftIn::changeState() {
  toggleStateFlag(DEBOUNCED_STATE);
  setStateFlag(CHANGED_STATE);
  // WIP HELD : durationOfPreviousState = millis() - stateChangeLastTime;
  stateChangeLastTime = millis();
}

bool BounceShiftIn::read() { return getStateFlag(DEBOUNCED_STATE); }

bool BounceShiftIn::rose() {
  return getStateFlag(DEBOUNCED_STATE) && getStateFlag(CHANGED_STATE);
}

bool BounceShiftIn::fell() {
  return !getStateFlag(DEBOUNCED_STATE) && getStateFlag(CHANGED_STATE);
}
