#include "ring_buffer.h"
#include "timer.h"
#include "encoders.h"
#include "body_imu.h"
#include "base_state_filter.h"
#include "logger_interface.h"

#define kEventRingBufferCapacity 16
#define kMinIMUPollingPeriodNs 20'000'000

typedef enum {
  kLeftWheelTick,
  kRightWheelTick,
  kLeftWheelDirectionCommand,
  kRightWheelDirectionCommand,
  kIMUReading,
} EventType;

typedef struct {
  EventType type;
  TimerTicksType timer_ticks;
  union {
    struct {
      bool is_forward;
    } wheel_direction;
    struct {  
      float position_acceleration[3];
      float attitude[3];
    } imu;
  } payload;
} Event;

static RingBuffer<Event, kEventRingBufferCapacity> event_buffer;
static BaseStateFilter base_state_filter;

static void LeftEncoderIsr(TimerTicksType timer_ticks) {
  // We have exclusive access to the event buffer while in the ISR.
  event_buffer.Write(Event{ .type = kLeftWheelTick, .timer_ticks = timer_ticks });
}

static void RightEncoderIsr(TimerTicksType timer_ticks) {
  // We have exclusive access to the event buffer while in the ISR.
  event_buffer.Write(Event{ .type = kRightWheelTick, .timer_ticks = timer_ticks });
}

TimerNanosType last_imu_poll_time_ns;

void InitRobotStateEstimator() {
  AddEncoderIsrs(&LeftEncoderIsr, &RightEncoderIsr);

  last_imu_poll_time_ns = 0;
  LOG_INFO("Initializing body IMU.");
  body_imu.Init();
}

static void RegisterIMUEvent() {  
  const auto attitude = body_imu.GetYawPitchRoll();
  const auto accels = body_imu.GetLinearAccelerations();
  NO_TIMER_IRQ {
    event_buffer.Write(Event{ 
      .type = kIMUReading, 
      .timer_ticks = GetTimerTicks(), 
      .payload = {
        .imu = { 
          .position_acceleration = { static_cast<float>(accels.x()), static_cast<float>(accels.y()), static_cast<float>(accels.z()) },
          .attitude = { static_cast<float>(attitude.x()), static_cast<float>(attitude.y()), static_cast<float>(attitude.z()) },
        }
      }
    });
  }
}

void NotifyLeftMotorDirection(TimerTicksType timer_ticks, bool forward) {
  NO_TIMER_IRQ {
    event_buffer.Write(Event{ 
      .type = kLeftWheelDirectionCommand, 
      .timer_ticks = timer_ticks,
      .payload = {
        .wheel_direction = {
          .is_forward = forward
        }
      }
    });
  }
}

void NotifyRightMotorDirection(TimerTicksType timer_ticks, bool forward) {
  NO_TIMER_IRQ {
    event_buffer.Write(Event{ 
      .type = kRightWheelDirectionCommand, 
      .timer_ticks = timer_ticks,
      .payload = {
        .wheel_direction = {
          .is_forward = forward
        }
      }
    });
  }
}

static int CompareEventPointers(const void *p1, const void *p2) {
  const Event * const e1 = *reinterpret_cast<const Event * const *>(p1);
  const Event * const e2 = *reinterpret_cast<const Event * const *>(p2);
  // Do not just subtract, as the timestamp may be of unsigned type.
  return e1->timer_ticks < e2->timer_ticks ? -1 : (e1->timer_ticks > e2->timer_ticks ? 1 : 0);
}

void RunRobotStateEstimator() {
  body_imu.Run();
  
  const TimerNanosType now_ns = GetTimerNanoseconds();
  if (now_ns - last_imu_poll_time_ns >= kMinIMUPollingPeriodNs) {
    last_imu_poll_time_ns = now_ns;
    // Poll the IMU and register event.
    RegisterIMUEvent();
  }

  // Copy all queue events to a separate buffer as processing them might take time  
  // and we don't want to block the queue to new events for too long.
  Event events[kEventRingBufferCapacity];
  int num_events;
  NO_TIMER_IRQ {
    num_events = event_buffer.Size();
    int i = 0;
    while(event_buffer.Size() > 0) {
      events[i++] = event_buffer.Read();
    }
  }
  if (num_events == 0) {
    base_state_filter.EstimateState(GetTimerTicks());
    return;
  }
  // Sort event order to process chronologically.
  Event *event_pointers[num_events];
  for (int i = 0; i < num_events; ++i) { event_pointers[i] = &events[i]; }
  qsort(event_pointers, num_events, sizeof(event_pointers[0]), CompareEventPointers);

  // Process events in chronological order.
  // Serial.println("--- Processing events ---");
  for (int i = 0; i < num_events; ++i) {
    const Event &event = *event_pointers[i];
    // char str[32];
    // Uint64ToString(event.timer_ticks, str);
    // Serial.printf("ts:%s\n", str);
    switch (event.type) {
      case kLeftWheelTick:
        // Serial.printf("left wheel tick\n");
        if (i == num_events - 1 || event_pointers[i + 1]->type != kRightWheelTick || event_pointers[i + 1]->timer_ticks != event.timer_ticks) {
          base_state_filter.NotifyWheelTicks(event.timer_ticks, 1, 0);
        } else {
          // There is a tick from the other wheel at the exact same time.
          base_state_filter.NotifyWheelTicks(event.timer_ticks, 1, 1);
          ++i;  // The next event has been processed.
        }
        break;
      case kRightWheelTick:
        // Serial.printf("right wheel tick\n");
        if (i == num_events - 1 || event_pointers[i + 1]->type != kLeftWheelTick || event_pointers[i + 1]->timer_ticks != event.timer_ticks) {
          base_state_filter.NotifyWheelTicks(event.timer_ticks, 0, 1);
        } else {
          // There is a tick from the other wheel at the exact same time.
          base_state_filter.NotifyWheelTicks(event.timer_ticks, 1, 1);
          ++i;  // The next event has been processed.
        }
        break;
      case kLeftWheelDirectionCommand:
        // Serial.printf("left wheel direction command\n");
        base_state_filter.NotifyLeftWheelDirection(event.payload.wheel_direction.is_forward);
        break;
      case kRightWheelDirectionCommand:
        // Serial.printf("left wheel direction command\n");
        base_state_filter.NotifyRightWheelDirection(event.payload.wheel_direction.is_forward);
        break;
      case kIMUReading:
        // Serial.printf("IMU reading\n");
        // Serial.printf("imu_ax:%f imu_ay:%f imu_a:%f\n", event.payload.imu.position_acceleration[0], event.payload.imu.position_acceleration[1], event.payload.imu.attitude[2]);
        base_state_filter.NotifyIMUReading(event.timer_ticks, event.payload.imu.position_acceleration[0], event.payload.imu.position_acceleration[1], event.payload.imu.attitude[2]);
        break;
    }
  }
}

BaseState GetBaseState() {
  return base_state_filter.state();
}

TimerNanosType GetBaseStateUpdateNanos() {
  return base_state_filter.state_update_nanos();
}