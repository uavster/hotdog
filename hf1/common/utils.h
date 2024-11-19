#ifndef UTILS_INCLUDED__
#define UTILS_INCLUDED__

#include <stddef.h>
#include <stdint.h>
#include <functional>
#include <math.h>

#ifdef ARDUINO
#include <DebugLog.h>

template<typename ParamType> class EndOfScopeExecutor {
public:
  EndOfScopeExecutor(void (*fn)(ParamType), ParamType param) : fn_(fn), param_(param) {}
  ~EndOfScopeExecutor() { if (fn_ != NULL) { fn_(param_); } }
private:
  void (*fn_)(ParamType);
  ParamType param_;
};

// Stores the value returned by push_fn, runs the following scope, and passes the stored value
// to pop_fn. The latter is called even when the scope returns or breaks.
//
// When used as:
// PUSH_POP_WRAPPER(state_type, push_fn, pop_fn) { 
//   scoped_code
// }
// 
// It is equivalent to:
// state_type state = push_fn();
// try {
//   scoped_code
// } finally {
//   pop_fn(state);
// }
//
// Please note that any break inside the scope will just leave the scope, but not break any
// containing loop or case, e.g.:
//
// for (;;) {
//   PUSH_POP_WRAPPER(state_type, push_fn, pop_fn) {
//     break;  // Leaves this scope, but does NOT end the for loop!
//   }
// }
#define PUSH_POP_WRAPPER(state_type, push_fn, pop_fn) for(struct { bool done; EndOfScopeExecutor<state_type> pop_executor; } s = { .done = false, .pop_executor = EndOfScopeExecutor<state_type>(&pop_fn, push_fn()) }; !s.done; s.done = true)

#endif

void Uint64ToString(uint64_t number, char *str);

// Maps `radians` to [-pi, pi).
float NormalizeRadians(float radians);

// Returns the array index for a monotonic index.
// A monotonics index may ever increase or decrease, while the corresponding array index
// will restart every array_size units in both positive and negative directions.
// The result has the same sign as array_size.
int IndexMod(int index, int array_size);
float IndexModf(float index, float array_size);

template<typename T> T DegreesFromRadians(const T radians) {
  return (static_cast<T>(180) * radians) / static_cast<T>(M_PI);
}
template<typename T> T RadiansFromDegrees(const T degrees) { 
  return (static_cast<T>(M_PI) * degrees) / static_cast<T>(180);
} 

#endif  // UTILS_INCLUDED__