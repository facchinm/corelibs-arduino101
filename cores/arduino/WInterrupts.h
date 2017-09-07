/*
  Copyright (c) 2011-2012 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _WIRING_INTERRUPTS_
#define _WIRING_INTERRUPTS_

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

void attachInterrupt(uint32_t pin, void (*callback)(void), uint32_t mode);
void attachInterruptParam(uint32_t pin, void(*callback)(void*), uint32_t mode, void* param);

void detachInterrupt(uint32_t pin);

void interrupts(void);

void noInterrupts(void);

#ifdef __cplusplus
}
#endif

typedef void (*voidFuncPtr)(void);
typedef void (*voidFuncPtrParam)(void*);

#ifndef W_INTERRUPTS_CPP
#define W_INTERRUPTS_CPP
#ifdef __cplusplus

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "wiring_private.h"

template <typename T>
using voidTemplateFuncPtrParam =  void (*)(T param);

template<typename T> struct __container__ {
  void* param;
  voidTemplateFuncPtrParam<T> function;
};

// C++ only overloaded version of attachInterrupt function
template<typename T> void attachInterrupt(uint8_t interruptNum, voidTemplateFuncPtrParam<T> userFunc, int mode, T& param) {

  struct __container__<T> *cont = new __container__<T>();
  cont->param = &param;
  cont->function = userFunc;

  // TODO: check lambda scope
  // TODO: add structure to delete(__container__) when detachInterrupt() is called
  auto f = [](void* a) -> void
  {
    T param = *(T*)((struct __container__<T>*)a)->param;
    (((struct __container__<T>*)a)->function)(param);
  };

  attachInterruptParam(interruptNum, f, mode, cont);
}

template<typename T> void attachInterrupt(uint8_t interruptNum, voidTemplateFuncPtrParam<T*> userFunc, int mode, T* param) {
  attachInterruptParam(interruptNum, (voidFuncPtrParam)userFunc, mode, (void*)param);
}

#endif
#endif

#endif /* _WIRING_INTERRUPTS_ */
