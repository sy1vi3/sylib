#include "sylib/system.hpp"

#include "sylib/addrled.hpp"
#include "sylib/env.hpp"

namespace {
  std::vector<sylib::Device*> livingSubtasks{};
  sylib::Mutex livingSubtasksMutex{};
}

namespace sylib {
void delay_until(std::uint32_t* const prev_time, const std::uint32_t delta) {
#ifdef SYLIB_ENV_PROS
    pros::Task::delay_until(prev_time, delta);
#elif defined(SYLIB_ENV_VEXCODE)
    std::uint32_t time = *prev_time + delta;
    vex::this_thread::sleep_until(time);
    *prev_time = time;
#endif
}

void delay(uint32_t delay) {
#ifdef SYLIB_ENV_PROS
    pros::delay(delay);
#elif defined(SYLIB_ENV_VEXCODE)
    vex::this_thread::sleep_for(delay);
#endif
}

uint32_t millis() {
#ifdef SYLIB_ENV_PROS
    return pros::millis();
#elif defined(SYLIB_ENV_VEXCODE)
    return vex::timer::system();
#endif
}

uint64_t micros() { return vexSystemHighResTimeGet(); }
#ifdef SYLIB_ENV_PROS
bool Mutex::take() {
    lock();
    return 1;
}

bool Mutex::give() {
    unlock();
    return 1;
}
#else
bool Mutex::try_lock_for() { return try_lock(); }

bool Mutex::try_lock_until() { return try_lock(); }
#endif

Mutex sylib_port_mutexes[V5_MAX_DEVICE_PORTS];
Mutex sylib_controller_mutexes[2];

SylibDaemon::SylibDaemon() {}

// std::vector<sylib::Device*>& SylibDaemon::getLivingSubtasks() {
//     return livingSubtasks;
// }

void SylibDaemon::startSylibDaemon() {
    static bool daemonStarted = false;
    if (!daemonStarted) {
        managerTask = std::unique_ptr<sylib::Task>(new sylib::Task(managerTaskFunction));
#ifdef SYLIB_ENV_PROS
        managerTask->set_priority(15);
#endif
        daemonStarted = true;
    }
}

SylibDaemon::~SylibDaemon() {}

SylibDaemon& SylibDaemon::getInstanceUnsafe() {
    ;
    static SylibDaemon instance;
    return instance;
}

SylibDaemon& SylibDaemon::getInstance() {
    mutex_lock _lock(mutex);
    return getInstanceUnsafe();
}

int SylibDaemon::createSubTaskUnsafe(sylib::Device* objectPointerToSchedule) {
    subTasksCreated++;
    mutex_lock _lock_(livingSubtasksMutex);
    livingSubtasks.push_back(objectPointerToSchedule);
    return subTasksCreated;
}

int SylibDaemon::createSubTask(sylib::Device* objectPointerToSchedule) {
    mutex_lock _lock(mutex);
    createSubTaskUnsafe(objectPointerToSchedule);
    return subTasksCreated;
}

void SylibDaemon::removeSubTask(sylib::Device* objectPointerToSchedule) {
    mutex_lock _lock(mutex);
    mutex_lock _lock_(livingSubtasksMutex);
    livingSubtasks.erase(std::remove(livingSubtasks.begin(), livingSubtasks.begin(),
                                          objectPointerToSchedule));
}

void SylibDaemon::removeSubTaskByID(int idToKill) {
    mutex_lock _lock(mutex);
    mutex_lock _lock_(livingSubtasksMutex);
    livingSubtasks.erase(
        std::remove_if(livingSubtasks.begin(), livingSubtasks.end(),
                       [&](sylib::Device* x) { return (x->getSubTaskID() == idToKill); }));
}

[[noreturn]] int SylibDaemon::managerTaskFunction() {
    printf("\nstarted sylib daemon\n");
    /*

    THIS SECTION TAKES CARE OF DESYNCING SYLIB DAEMON WITH VEX BACKGROUND PROCESSING

    */

    // A 1ms loop will actually take around 1040 or 960 microseconds, always alternating.
    // Over 3ms, the total length of time in micros should be either around 3040 or 960
    // Daemon needs to start on a cycle to be directly opposite of vexBackgroundProcessing()
    // vexBackgroundProcessing always runs after a short cycle, meaning the sylib daemon needs to
    // start after a long cycle Values offset by 20 to give room for error, the groupings are very
    // tight so it shouldnt matter

    constexpr std::uint64_t LONG_MICROS_CYCLE_LENGTH = 1040 - 20;
    constexpr std::uint64_t AVERAGE_MICROS_CYCLE_LENGTH = 1000;
    constexpr std::uint64_t DIFFERENCE_BETWEEN_AVERAGE_AND_LONG =
        LONG_MICROS_CYCLE_LENGTH - AVERAGE_MICROS_CYCLE_LENGTH;

    uint32_t systemTime = sylib::millis();
    uint32_t detectorPreviousTime = sylib::millis();
    uint64_t systemTimeMicros = sylib::micros();
    uint64_t prevMicros = systemTimeMicros;
    uint32_t lastCenterControllerPressTime = 0;
    uint32_t centerControllerPressStartTime = 0;
    bool controllerCenterButtonPressDetected = false;

    do {
        systemTimeMicros = sylib::micros();
        detectorPreviousTime = systemTime;
        prevMicros = systemTimeMicros;
        sylib::delay_until(&systemTime, 3);
    } while ((sylib::micros() - prevMicros) >
             (((systemTime - detectorPreviousTime) * AVERAGE_MICROS_CYCLE_LENGTH) -
              DIFFERENCE_BETWEEN_AVERAGE_AND_LONG));
    /*

    NOW WE'RE TIMED CORRECTLY, STARTING DAEMON

    */
    while (1) {
        {
            mutex_lock _lock{mutex};
            mutex_lock _lock_(livingSubtasksMutex);
            frameCount++;
            for (auto& subTask : livingSubtasks) {
                if (!subTask->getSubTaskPaused() &&
                    ((frameCount + subTask->getUpdateOffset()) % subTask->getUpdateFrequency() ==
                     0)) {
                    subTask->update();
                }
            }
            if (frameCount % 5 == 0) {
                mutex_lock _lock{sylib_controller_mutexes[0]};
                if (vexControllerGet(kControllerMaster, static_cast<V5_ControllerIndex>(18))) {
                    if (!controllerCenterButtonPressDetected) {
                        centerControllerPressStartTime = systemTime;
                        controllerCenterButtonPressDetected = true;
                    }
                    if (systemTime > centerControllerPressStartTime + 500) {
                        sylib::Addrled::addrled_enabled = false;
                    }
                    lastCenterControllerPressTime = systemTime;
                } else {
                    controllerCenterButtonPressDetected = false;
                    if (systemTime > lastCenterControllerPressTime + 2500) {
                        sylib::Addrled::addrled_enabled = true;
                    }
                }
            }
            sylib::delay_until(&systemTime, 2);
        }
    }
}

int SylibDaemon::frameCount = 0;
sylib::Mutex SylibDaemon::mutex = sylib::Mutex();
int SylibDaemon::subTasksCreated = 0;

void Device::startSubTask() {
    if (sylib::millis() > 1) {
        mutex_lock _lock{mutex};
        static SylibDaemon& taskMgr = SylibDaemon::getInstance();
        subTaskID = taskMgr.createSubTask(this);
    } else {
        static SylibDaemon& taskMgr = SylibDaemon::getInstanceUnsafe();
        subTaskID = taskMgr.createSubTaskUnsafe(this);
    }
}

void Device::killSubTask() {
    mutex_lock _lock{mutex};
    static SylibDaemon& taskMgr = SylibDaemon::getInstance();
    taskMgr.removeSubTask(this);
}

Device::Device(int interval, int offset) : updateFrequency(interval), updateOffset(offset) {
    startSubTask();
}

Device::~Device() {
    mutex.lock();
    killSubTask();
}

void Device::setUpdateOffset(int offset) { updateOffset = offset; }

void Device::setUpdateFrequency(int interval) { updateFrequency = interval; }

int Device::getSubTaskID() {
    mutex_lock _lock(mutex);
    return subTaskID;
}

bool Device::getSubTaskPaused() {
    mutex_lock _lock(mutex);
    return subTaskPaused;
}

int Device::getUpdateFrequency() {
    mutex_lock _lock(mutex);
    return updateFrequency;
}

int Device::getUpdateOffset() {
    mutex_lock _lock(mutex);
    return updateOffset;
}

void initialize() { sylib::SylibDaemon::getInstance().startSylibDaemon(); }
}  // namespace sylib
