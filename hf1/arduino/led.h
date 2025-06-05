#ifndef LED_INCLUDED_
#define LED_INCLUDED_

#include <stdint.h>
#include "logger_interface.h"

// --- Legacy functions for the LED on the Teensy 3.2 ---
void LedToggle();
void LedShowAssert();
// ------------------------------------------------------

void InitLeds();

class Led {
public:
  friend class LedModulator;

  Led() : level_(0) {}

  void intensity(float i);
  float intensity() const;

protected:
  void level(uint8_t level) { level_ = level; }
  uint8_t level() const { return level_; }

  // Subclasses specialize this functions to their LED.
  virtual void Enable() const = 0;
  virtual void Disable() const = 0;

private:
  uint8_t level_;
};

class LedGreen : public Led {
protected:
  void Enable() const override;
  void Disable() const override;
};

template<int kMaxNumLeds>
class LedGroup {
public:
  LedGroup() : num_leds_(0) {}
  // Does not take ownership of the pointees, which must outlive this object.  
  template<typename... Leds>
  LedGroup(Led *first_led, Leds... other_leds) : num_leds_(0) {
    RegisterLed(first_led, other_leds...);
  }
  Led *operator[](int index) const { return leds_[index]; }
  int size() { return num_leds_; }

private:
  template<typename... Leds>
  void RegisterLed(Led *led, Leds... other_leds) {
    ASSERT_NOT_NULL(led);
    leds_[num_leds_++] = led;
    RegisterLed(other_leds...);
  }
  void RegisterLed() { }

  Led *leds_[kMaxNumLeds];
  uint8_t num_leds_;
};

constexpr int kLedModulatorMaxCapacity = 3;

class LedModulator : public LedGroup<kLedModulatorMaxCapacity> {
public:
  template<typename... Leds>
  LedModulator(Led *first_led, Leds... other_leds) : LedGroup(first_led, other_leds...) {}

  // Call with a constant period of kLedPWMFrequency.
  void Run();

private:
  uint8_t tick_count_;
};

class LedRGB {
public:
  // Does not take ownership of the pointees, which must outlive this object.
  LedRGB(LedGreen *green_led) : green_led_(*ASSERT_NOT_NULL(green_led)) {}

  void SetRGB(float red, float green, float blue);

private:
  LedGreen &green_led_;
};

#endif  // LED_INCLUDED_

