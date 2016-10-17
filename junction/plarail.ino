#include <PrintEx.h>
#include <Servo.h>
#include <ThreadController.h>

#include "util.h"

static PrintEx serial = Serial;
static ThreadController thread_controller{};

/**
 * 
 */
class StopControl {
public:
  StopControl(unsigned pin)
    : pin_{pin}
  {}

  void begin() {
    servo_.attach(pin_);
    servo_.write(GO_ANGLE);
    servo_.write(GO_ANGLE - 5);
  }

  void stop() {
    Serial.println("Stop!");
    servo_.write(STOP_ANGLE);
    servo_.write(STOP_ANGLE + 5);
    stopped_ = true;
  }

  void go() {
    Serial.println("Go!");
    servo_.write(GO_ANGLE);
    servo_.write(GO_ANGLE - 5);
    stopped_ = false;
  }

  bool is_stopped() const {
    return stopped_;
  }

private:
  static constexpr unsigned GO_ANGLE = 78;
  static constexpr unsigned STOP_ANGLE = 0;

  unsigned pin_;
  Servo servo_;
  bool stopped_ { false };
};

/**
 * 
 */
class IR_Wall {
public:
  IR_Wall(unsigned pin)
    : pin_{pin}
  {}

  bool is_open() const {
    auto value = analogRead(pin_);
    serial.printf("value: %d\n", value);
    return value > THRESHOLD;
  }

private:
  static constexpr unsigned THRESHOLD = 300;
  
  unsigned pin_;
};

/**
 * 
 */
class JunctionInput : public Thread {
public:
  JunctionInput(unsigned ir_wall_pin, unsigned stop_pin)
    : Thread()
    , ir_wall_(ir_wall_pin)
    , stop_(stop_pin)
//    , empty_{}
  {
    for (unsigned i = 0; i < HISTORY_LENGTH; i++)
      empty_[i] = true;
      
    setInterval(100);
  }

  void begin() {
    stop_.begin();
    thread_controller.add(this);
  }
  
  bool is_empty() const {
    for (unsigned i = 0; i < HISTORY_LENGTH; i++)
      if (!empty_[i])
        return false;
    return true;
  }

  void stop(unsigned long length) {
    stop_.stop();
    stop_time_ = millis();
    stop_length_ = length;
  }

private:
  static constexpr unsigned HISTORY_LENGTH = 4;
  
  IR_Wall ir_wall_;
  StopControl stop_;
  
  bool empty_[HISTORY_LENGTH];
  unsigned long stop_time_;
  unsigned long stop_length_;

  void run() {
    for (unsigned i = HISTORY_LENGTH - 1; i > 0; i--)
      empty_[i] = empty_[i-1];
    empty_[0] = !ir_wall_.is_open();

    if (stop_.is_stopped()
        && (millis() - stop_time_) >= stop_length_)
      stop_.go();
    
    runned();
  }
};

/**
 * 
 */
class Junction : public Thread {
public:
  Junction(
    unsigned ir_wall0_pin,
    unsigned stop0_pin,
    unsigned ir_wall1_pin,
    unsigned stop1_pin)
    : inputs_{
      JunctionInput(ir_wall0_pin, stop0_pin),
      JunctionInput(ir_wall1_pin, stop1_pin)
    }
  {
    setInterval(100);
  }

  void begin() {
    inputs_[0].begin();
    inputs_[1].begin();
    thread_controller.add(this);
  }

private:
  static constexpr unsigned long BLOCKING_TIME { 5000 };
  
  JunctionInput inputs_[2];
  int blocking_input_ { -1 };
  unsigned long blocking_time_;
  
  void run() {
    if (millis() < 1000)
      return;
      
    try_to_unblock();
    
    if (!is_blocked()) {
      if (!inputs_[0].is_empty())
        block(0);
      else if (!inputs_[1].is_empty())
        block(1);
    }

//    if (is_blocked()) {
//      Serial.println("Stopping");
//      inputs_[other_input(blocking_input_)].stop(BLOCKING_TIME);
//    }

    runned();
  }

  bool is_blocked() const {
    return blocking_input_ != -1;
  }

  void block(unsigned input) {
    serial.printf("Blocking %d\n", input);
    blocking_input_ = static_cast<int>(input);
    blocking_time_ = millis();
    inputs_[other_input(input)].stop(BLOCKING_TIME);
  }

  void try_to_unblock() {
    if (!is_blocked())
      return;

    serial.printf("Trying to unblock: %d\n",
      (millis() - blocking_time_));
    
    if ((millis() - blocking_time_) >= BLOCKING_TIME) {
      Serial.println("Unblocking!");
      blocking_input_ = -1;
    }
  }

  unsigned other_input(unsigned input) {
    return !input;
  }
};

static Junction junction(A0, 11, A1, 13);

void setup() {
  Serial.begin(115200);
  delay(100);

  junction.begin();
}

void loop() {
  thread_controller.run();
//  delay(100);
}

