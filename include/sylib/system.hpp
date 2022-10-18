/**
 * \file sylib/system.hpp
 *
 * \brief Contains prototypes for functions relating to sylib's background
 * processes
 */

#pragma once

#include "env.hpp"
#include "system.hpp"

namespace sylib {
#ifdef SYLIB_ENV_PROS
typedef pros::Mutex PlatformMutex;
typedef pros::Task PlatformTask;
#elif defined(SYLIB_ENV_VEXCODE)
#define V5_MAX_DEVICE_PORTS 32
typedef vex::mutex PlatformMutex;
typedef vex::task PlatformTask;
#endif

/**
 * \brief Starts Sylib background processes. Called by the user in initialize()
 */
void initialize();

/**
 * \brief Delays the current task until a set time
 *
 * Updates the value at the pointer provided to the current time after the delay
 *ends
 *
 * This is a wrapper for pros::Task::delay_until() if running in a PROS
 *enviroment, and a wrapper for vex::this_thread::sleep_until() if in a VEXcode
 *enviroment
 *
 * \param prev_time
 *        A pointer to a value containing the current system time
 *
 * \param delta
 *        The number of milliseconds to pause the task for
 **/
void delay_until(std::uint32_t* const prev_time, const std::uint32_t delta);

/**
 * \brief Delays the current task for a set number of milliseconds
 *
 * This is a wrapper for pros::delay() if running in a PROS enviroment,
 * and a wrapper for vex::this_thread::sleep_for if in a VEXcode enviroment
 *
 * \param delay
 *        The number of milliseconds to pause the task for
 **/
void delay(std::uint32_t delay);

/**
 * \brief The current system time
 *
 * This is a wrapper for pros::millis() if running in a PROS enviroment,
 * and a wrapper for vex::timer::system() if in a VEXcode enviroment
 *
 * \return Milliseconds
 **/
uint32_t millis();

/**
 * \brief The current system time in microseconds
 *
 * This value drifts slightly from the system clock over time
 *
 * \return Microseconds
 **/
uint64_t micros();

/**
 * \brief Mutex
 *
 * This is a wrapper for pros::Mutex if running in a PROS enviroment,
 * and a wrapper for vex::mutex if running in a VEXcode enviroment
 */
class Mutex : public PlatformMutex {
   private:
    bool try_lock_for();
    bool try_lock_until();

   public:
    /**
     * \brief Locks the mutex
     **/
    bool take();

    /**
     * \brief Unlocks the mutex
     **/
    bool give();
};

/**
 * \brief Task
 *
 * This is a wrapper for pros::Task if running in a PROS enviroment,
 * and a wrapper for vex::task if running in a VEXcode enviroment
 */
class Task : public PlatformTask {
   public:
    template <class F>
    Task(F&& func) : PlatformTask(func) {}
};

extern Mutex sylib_port_mutexes[V5_MAX_DEVICE_PORTS];
extern Mutex sylib_controller_mutexes[2];
using mutex_lock = const std::lock_guard<sylib::Mutex>;

class Device;

/**
 * \brief Sylib System Daemon
 *
 * This is a singleton this is created when the first object that needs it is.
 *
 * Takes care of running update functions that need to happen periodically.
 */
class SylibDaemon {
   private:
    SylibDaemon();
    ~SylibDaemon();
    SylibDaemon(const SylibDaemon&);
    const SylibDaemon& operator=(const SylibDaemon&);
    std::unique_ptr<sylib::Task> managerTask = nullptr;
    static int subTasksCreated;
    static std::vector<sylib::Device*>& getLivingSubtasks();
    static int frameCount;

   public:
    /**
     * \brief Mutex to prevent multiple threads from performing operations
     * on important system things at once.
     */
    static sylib::Mutex mutex;

    /**
     * \brief Gets the single existing SylibDaemon object,
     * or creates it if it doesn't exist yet
     *
     * \return SylibDaemon object
     */
    static SylibDaemon& getInstance();

    /**
     * \brief Starts the Sylib daemon. This should be called in initialize() or
     * pre_auton()
     */
    void startSylibDaemon();

    /**
     * \brief Gets the single existing SylibDaemon object,
     * or creates it if it doesn't exist yet
     *
     * Doesn't lock the mutex. This is used when creating objects in a global
     * scope, before RTOS stuff has initialized and mutexes work.
     *
     * \return SylibDaemon object
     */
    static SylibDaemon& getInstanceUnsafe();

    /**
     * \brief The sole function that runs inside a task made specifically for
     * the Sylib daemon. Contains an infinite loop that will prevent anything
     * else from happening in the thread this is called in.
     *
     * \return Theoretically 1, but this function should never return on account
     * of the while(1) loop
     */
    static int managerTaskFunction();

    /**
     * \brief Adds a Device object or one of its child classes to the daemon's
     * list of objects which need periodic updating
     *
     * \param objectPointerToSchedule
     *        A pointer to a Device object
     *
     * \return The total number of subtasks created
     */
    static int createSubTask(sylib::Device* objectPointerToSchedule);

    /**
     * \brief Adds a Device object or one of its child classes to the daemon's
     * list of objects which need periodic updating
     *
     * Doesn't lock the mutex. This is used when creating objects in a global
     * scope, before RTOS stuff has initialized and mutexes work.
     *
     * \param objectPointerToSchedule
     *        A pointer to a Device object
     *
     * \return The total number of subtasks created
     */
    static int createSubTaskUnsafe(sylib::Device* objectPointerToSchedule);

    /**
     * \brief Removes a Device object or one of its child classes from the
     * daemon's list of objects which need periodic updating
     *
     * \param objectPointerToSchedule
     *        A pointer to a Device object
     */
    static void removeSubTask(sylib::Device* objectPointerToSchedule);

    /**
     * \brief Removes a subtask from the daemon's list
     * of objects which need periodic updating
     *
     * \param idToKill
     *        The ID number of the subtask to remove
     */
    static void removeSubTaskByID(int idToKill);

    /**
     * \brief Gets the number of ticks of the Sylib daemon update loop
     *
     * Should be approximately equal to the system timer divided by 2
     *
     * \return Frame count
     */
    static uint64_t getFrameCount();
};

/**
 * \brief Device Parent Class
 *
 * Motor and Addrled objects derive from this
 */
class Device {
   private:
    bool subTaskPaused = false;
    int updateFrequency;
    int updateOffset;
    int subTaskID;
    void startSubTask();
    bool idSet = false;

   public:
    /**
     * \brief Mutex to prevent multiple threads from performing operations
     * on important system things at once.
     */
    sylib::Mutex mutex;

    /**
     * \brief Creates a Device object
     *
     * \param interval
     *        The number of frames to wait between updates. A frame is 2ms, so
     * this value multiplied by 2 is the approximate time between updates. It
     * can be off by +/- 1ms
     *
     * \param offset
     *        The number of frames to shift the excecution of the update
     * function
     */
    Device(int interval = 2, int offset = 0);

    ~Device();

    /**
     * \brief Resumes the device update function
     */
    void resumeSubTask();

    /**
     * \brief Pauses the device update function
     */
    void pauseSubTask();

    /**
     * \brief Permanently ends the device update function
     */
    void killSubTask();

    /**
     * \brief Gets whether or not the device update function is running
     *
     * \return True or False
     */
    bool isSubTaskRunning();

    /**
     * \brief Gets the number of frames delayed between updates
     *
     * \return Sylib frame count
     */
    int getUpdateFrequency();

    /**
     * \brief Gets the number of frames the update function is offset by
     *
     * \return Sylib frame count
     */
    int getUpdateOffset();

    /**
     * \brief Sets the number of frames to wait between updates.
     *
     * \param interval
     *        Sylib frame count
     */
    void setUpdateFrequency(int interval);

    /**
     * \brief Sets the number of frames to shift the excecution of the update
     * function
     *
     * \param interval
     *        Sylib frame count
     */
    void setUpdateOffset(int offset);

    /**
     * \brief Gets the subtask ID of the device's update function
     *
     * \return ID number
     */
    int getSubTaskID();

    /**
     * \brief Default device update function. This is overridden by classes that
     * inherit from Device for their own specific needs.
     */
    virtual void update();

    /**
     * \brief Gets whether or not the device update function is paused
     *
     * \return True or False
     */
    bool getSubTaskPaused();
};
}  // namespace sylib