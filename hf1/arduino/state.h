#ifndef STATE_INCLUDED_
#define STATE_INCLUDED_

#include <type_traits>
#include "utils.h"

template<typename TStateVars, int Order>
class State {
  static_assert(Order >= 0, "Order of state variables must be >= 0.");
public:
  State() {}

  template<typename TInput, size_t NumStates>
  explicit State(const TInput (&states)[NumStates]) {
    static_assert(NumStates == Order + 1, "Incorrect number of input elements.");
    for (int i = 0; i < Order + 1; ++i) { states_[i] = states[i]; }
  }

  State operator+(const State &other) const {
    State result;
    for (int i = 0; i < Order + 1; ++i) { result.states_[i] = states_[i] + other.states_[i]; }
    return result;
  }

  State operator-(const State &other) const {
    State result;
    for (int i = 0; i < Order + 1; ++i) { result.states_[i] = states_[i] + (other.states_[i] * -1.0f); }
    return result;
  }

  State operator*(float factor) const {
    State result;
    for (int i = 0; i < Order + 1; ++i) { result.states_[i] = states_[i] * factor; }
    return result;
  }

  State operator/(float divisor) const {
    State result;
    for (int i = 0; i < Order + 1; ++i) { result.states_[i] = states_[i] * (1 / divisor); }
    return result;
  }

  int order() const { return Order; }

  const TStateVars &location() const { return states_[0]; }
  TStateVars &location() { return states_[0]; }

  // velocity() only exists if there is more than 1 state. 
  // The condition in the enable_if needs to be made dependent on a template parameter of the 
  // function template so that it is not substituted early.
  template<int Order_ = Order, typename = typename std::enable_if<(Order_ > 0)>::type>
  const TStateVars &velocity() const { return states_[1]; }
  template<int Order_ = Order, typename = typename std::enable_if<(Order_ > 0)>::type>
  TStateVars &velocity() { return states_[1]; }

  // acceleration() only exists if there is more than 2 states.
  // The condition in the enable_if needs to be made dependent on a template parameter of the 
  // function template so that it is not substituted early.
  template<int Order_ = Order, typename = typename std::enable_if<(Order_ > 1)>::type>
  const TStateVars &acceleration() const { return states_[2]; }
  template<int Order_ = Order, typename = typename std::enable_if<(Order_ > 1)>::type>
  TStateVars &acceleration() { return states_[2]; }
  
private:
  TStateVars states_[Order + 1];
};

template<typename TState> TState operator*(float factor, const TState &state) {
  return state * factor;
}

template<typename TState> TState operator*(double factor, const TState &state) {
  return state * factor;
}

#endif  // STATE_INCLUDED_