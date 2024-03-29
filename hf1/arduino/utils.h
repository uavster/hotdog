#ifndef UTILS_INCLUDED__
#define UTILS_INCLUDED__

#include <stddef.h>
#include <stdint.h>
#include <functional>

void Uint64ToString(uint64_t number, char *str);

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