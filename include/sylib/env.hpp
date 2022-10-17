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

#define SYLIB_ENV_PROS
// #define SYLIB_ENV_VEXCODE



#ifdef SYLIB_ENV_PROS
#include "pros.h"
#elif defined(SYLIB_ENV_VEXCODE)
#include "vex.h"
#endif