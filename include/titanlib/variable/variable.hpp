#pragma once

#include "pros/rtos.hpp"

namespace titanlib {
template <typename VariableType> class Variable {

private:
  VariableType value;
  pros::Mutex varLock;

public:
  Variable(VariableType var) {
    varLock.take();
    value = var;
    varLock.give();
  }

  void set(VariableType var) {
    varLock.take();
    value = var;
    varLock.give();
  }

  VariableType get() {
    varLock.take();
    VariableType temp = value;
    varLock.give();
    return temp;
  }
};
} // namespace titanlib