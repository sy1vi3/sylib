/**
 * \file sylib/env.hpp
 *
 * \brief Includes needed PROS or VEXcode header files, depending on which enviroment is being used
 */

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <deque>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <queue>
#include <algorithm>
#include <mutex>
#include <vector>
#include <cstdint>
#include <initializer_list>
#include <stdint.h>
#include <vector>

#define SYLIB_SRC_PRESENT

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
    int32_t  vexAdiAddrLedSet( uint32_t index, uint32_t port, uint32_t *pData, uint32_t nOffset, uint32_t nLength, uint32_t options );
    int32_t vexDeviceAdiAddrLedSet( V5_DeviceT device, uint32_t port, uint32_t *pData, uint32_t nOffset, uint32_t nLength, uint32_t options );
}