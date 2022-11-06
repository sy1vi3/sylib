/**
 * \file sylib/env.hpp
 *
 * \brief Includes needed PROS or VEXcode header files, depending on which enviroment is being used
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <stdint.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

// #define SYLIB_SRC_PRESENT
#define SYLIB_ENV_PROS

// #define SYLIB_ENV_VEXCODE

#ifdef SYLIB_ENV_PROS
#ifdef SYLIB_SRC_PRESENT
#include "pros.h"
#else
#include "pros_includes.h"
#endif
#elif defined(SYLIB_ENV_VEXCODE)
#include "vex.h"
#endif

extern "C" {
int32_t vexAdiAddrLedSet(uint32_t index, uint32_t port, uint32_t* pData, uint32_t nOffset,
                         uint32_t nLength, uint32_t options);
int32_t vexDeviceAdiAddrLedSet(V5_DeviceT device, uint32_t port, uint32_t* pData, uint32_t nOffset,
                               uint32_t nLength, uint32_t options);
}