/**
 * \file sylib/math.hpp
 *
 * \brief Contains prototypes for various controllers and math utilities
 */

#include "env.hpp"

namespace sylib {
using kv_fn_t = std::function<double(double)>;

/**
 * \brief Exponential Moving Average Filter
 */
class EMAFilter {
   private:
    double inputkA;
    double ema;

   public:
    /**
     * \brief Creates an Exponential Moving Average filter
     */
    EMAFilter();

    /**
     * \brief Filters an input value
     *
     * \param rawValue
     *        The raw input value to filter
     * \param kA
     *        How much the filter will condider previous information. A value of 1
     *        will make the input the full value of the output, a value of 0 will
     *        make the input have no effect on the output.
     *
     * \return Filter output
     */
    double filter(double rawValue, double kA);

    /**
     * \brief Gets the most recently used kA value
     *
     * \return kA value
     */
    double getkA() const;

    /**
     * \brief Gets the current filter output
     *
     * \return Filter output
     */
    double getCurrentEMA() const;
};

/**
 * \brief Simple Moving Average Filter
 */
class SMAFilter {
   private:
    std::queue<double> rawInputValues;
    int sampleSize;
    double meanValue;
    double rawVelocityTotal;

   public:
    /**
     * \brief Creates a Simple Moving Average filter
     *
     * \param sampleSize
     *        The number of previous values to average.
     *        A higher number will result in a more stable value,
     *        but with more lag.
     */
    SMAFilter(int sampleSize);

    /**
     * \brief Filters an input value
     *
     * \param rawValue
     *        The raw input value to filter
     *
     * \return Filter output
     */
    double filter(double rawValue);

    /**
     * \brief Gets the current length of the queue of previous values
     *
     * \return Queue length
     */
    int getQueueLength() const;

    /**
     * \brief Gets the maximum length of the queue of previous values
     *
     * Equal to the sample size specifiied in the constructor
     *
     * \return Max sample size
     */
    int getQueueMaxLength() const;

    /**
     * \brief Gets the current sum of queue values
     *
     * \return Sum of values
     */
    double getCurrentTotal() const;

    /**
     * \brief Gets the current filter output without providing a new input value
     *
     * \return Filter output
     */
    double getCurrentValue() const;
};

/**
 * \brief Median Filter
 */
class MedianFilter {
   private:
    std::deque<double> rawInputValues;
    int sampleSize;
    int queueLength;
    double medianValue;
    int meanSizeEven;
    int meanSizeOdd;

   public:
    /**
     * \brief Creates a Median filter
     *
     * \param sampleSize
     *        The number of previous values to consider.
     *        A higher number will result in a more stable value,
     *        but with more lag.
     *
     * \param meanSizeEven
     *        The number of median values to average when the sample size is even
     *
     * \param meanSizeOdd
     *        The number of median values to average when the sample size is odd
     */
    MedianFilter(int sampleSize, int meanSizeEven, int meanSizeOdd);

    /**
     * \brief Filters an input value
     *
     * \param rawValue
     *        The raw input value to filter
     *
     * \return Filter output
     */
    double filter(double rawValue);

    /**
     * \brief ets the current length of the queue of previous values
     *
     * \return Queue length
     */
    int getQueueLength() const;

    /**
     * \brief Gets the current sum of queue values
     *
     * \return Sum of values
     */
    int getQueueMaxLength() const;

    /**
     * \brief Gets the number of median values to average when the sample size is even
     *
     * \return Center size
     */
    int getEvenCenterSize() const;

    /**
     * \brief Gets the number of median values to average when the sample size is odd
     *
     * \return Center size
     */
    int getOddCenterSize() const;

    /**
     * \brief Gets the current filter output without providing a new input value
     *
     * \return Filter output
     */
    double getCurrentValue() const;
};

/**
 * \brief Extrema Filter
 */
class RangeExtremaFilter {
   private:
    std::deque<double> rawInputValues;
    double maxValue;
    int queueLength;
    int sampleSize;

   public:
    /**
     * \brief Creates an Extrema filter that returns the largest absolute value of
     * recent inputs
     *
     * \param sampleSize
     *        The number of previous values to consider.
     *        A higher number will result in a more stable value,
     *        but with more lag.
     */
    RangeExtremaFilter(int sampleSize);

    /**
     * \brief Filters an input value
     *
     * \param rawValue
     *        The raw input value to filter
     *
     * \return Filter output
     */
    double filter(double rawValue);

    /**
     * \brief Gets the current length of the queue of previous values
     *
     * \return Queue length
     */
    int getQueueLength() const;

    /**
     * \brief Gets the number of median values to average when the sample size is even
     *
     * \return Center size
     */
    int getQueueMaxLength() const;

    /**
     * \brief Gets the current filter output without providing a new input value
     *
     * \return Filter output
     */
    double getCurrentValue() const;
};

/**
 * \brief Generic Derivative Solver
 */
class SympleDerivativeSolver {
   private:
    double currentInputFunctionValue;
    double previousInputFunctionValue;
    double deltaInputFunctionValue;
    uint32_t currentTime;
    uint32_t previousTime;
    uint32_t dT;
    double derivativeFunctionValue;

   public:
    /**
     * \brief Creates a generic derivative solver that outputs the dX/dT,
     * where X is the input value provided, and T is the time in
     * milliseconds since the last input. Keeps track of its own timer.
     *
     */
    SympleDerivativeSolver();

    /**
     * \brief Calculates the slope between the last input value and the current input value
     *
     * \param input
     *        The raw input value
     *
     * \return Derivative value
     */
    double solveDerivative(double input);

    /**
     * \brief Gets the current stored output value without adding any new input
     *
     * \return Derivative value
     */
    double getCurrentDerivative() const;

    /**
     * \brief Gets the most recent input value
     *
     * \return Input value
     */
    double getCurrentInputValue() const;
};

/**
 * \brief Feedforward Controller
 */
class VoltageEstimation {
   private:
    kv_fn_t kV;
    double voltageEstimate;
    double motorGearing;

   public:
    /**
     * \brief Creates a simple feedforward controller for a V5 motor
     *
     * \param kV
     *        A lambda function that takes double as an input for target RPM
     *        and returns a voltage constant to use for the estimation
     *
     * \param motorGearing
     *        The output speed of the motor, in RPM, if the motor internally
     *        is spinning at 3600 RPM. 200 is default for a green motor cartridge
     */
    VoltageEstimation(kv_fn_t kV, double motorGearing = 200);

    /**
     * \brief Outputs an estimated voltage value for acheiving the desired RPM based
     * on the kV function supplied in the constructor
     *
     * \param rpm
     *        The target velocity of the motor, in RPM
     */
    double estimate(double rpm);

    /**
     * \brief Gets the current kV function
     *
     * \return kV function
     */
    kv_fn_t getKv() const;

    /**
     * \brief Gets the set motor gearing for the estimator
     *
     * \return Output RPM of the motor at 100% velocity
     */
    double getMotorGearing() const;

    /**
     * \brief Sets the kV function
     *
     * \param value
     *        A lambda function that takes double as an input for target RPM
     *        and returns a voltage constant to use for the estimation
     */
    void setkV(kv_fn_t value);

    /**
     * \brief Gets the current stored output value without adding any new input
     *
     * \return Voltage estimate
     */
    double getOutput() const;
};

/**
 * \brief P Controller
 */
class ProportionalController {
   private:
    std::shared_ptr<double> error;
    double kP;
    double motorGearing;
    double proportional;
    bool maxRangeEnabled;
    double kP2;
    double maxRange;

   public:
    /**
     * \brief Creates a P controller for a V5 motor
     *
     * \param kP
     *        Proportional gain
     *
     * \param error
     *        A pointer to a double containing the error value to use for calculations
     *
     * \param motorGearing
     *        The output speed of the motor, in RPM, if the motor internally
     *        is spinning at 3600 RPM. 200 is default for a green motor cartridge
     *
     * \param maxRangeEnabled
     *        A flag that when true shifts the P controller to using kP2 as the kP constant when
     * error is less than <maxRange>
     *
     * \param kP2
     *        The secondary kP value that is used when maxRangeEnabled is on and the error is within
     * maxRange
     *
     * \param maxRange
     *        Error range from the target value that is used for switching between kP and kP2
     */
    ProportionalController(double kP, std::shared_ptr<double> error, double motorGearing = 200,
                           bool maxRangeEnabled = false, double kP2 = 0, double maxRange = 0);

    /**
     * \brief Calculates controller output based on the current error value
     *
     * \return Controller output
     */
    double update();

    /**
     * \brief Gets the kP constant
     *
     * \return kP value
     */
    double getkP() const;

    /**
     * \brief Gets the current stored output value without calculating a new value
     *
     * \return Controller output
     */
    double getOutput() const;

    /**
     * \brief Sets the kP constant
     *
     * \param gain
     *        Proportional gain
     */
    void setkP(double gain);

    /**
     * \brief Operator overload for getting the current filter ouput
     *
     * \return Controller output
     */
    double operator*();

    /**
     * \brief Sets whether the maxRangeEnabled flag is on or off
     *
     * \param enabled
     *        Flag value
     */
    void setMaxRangeEnabled(bool enabled);

    /**
     * \brief Sets the error range from the target value that is used for switching between kP and
     * kP2
     *
     * \param range
     *        Max error value
     */
    void setMaxRange(double range);

    /**
     * \brief Sets the secondary kP constant
     *
     * \param gain
     *        Proportional gain
     */
    void setkP2(double gain);
};

/**
 * \brief I Controller
 */
class IntegralController {
   private:
    std::shared_ptr<double> error;
    double kI;
    double integral;
    double motorGearing;
    uint32_t currentTime;
    uint32_t previousTime;
    uint32_t dT;
    bool antiWindupEnabled;
    double antiWindupRange;

   public:
    /**
     * \brief Creates an I controller for a V5 motor
     *
     * \param kI
     *        Integral gain
     *
     * \param error
     *        A pointer to a double containing the error value to use for calculations
     *
     * \param motorGearing
     *        The output speed of the motor, in RPM, if the motor internally
     *        is spinning at 3600 RPM. 200 is default for a green motor cartridge
     *
     * \param antiWindupEnabled
     *        A flag that when true enables anti-windup, setting the output value to 0 when
     *        error is more than <antiWindupRange>
     *
     * \param antiWindupRange
     *        Error range from the target value that is used for anti-windup
     */
    IntegralController(double kI, std::shared_ptr<double> error, double motorGearing = 200,
                       bool antiWindupEnabled = false, double antiWindupRange = 0);

    /**
     * \brief Calculates controller output based on the current error value
     *
     * \return Controller output
     */
    double update();

    /**
     * \brief Gets the kI constant
     *
     * \return kI value
     */
    double getkI() const;

    /**
     * \brief Resets the integral value to 0
     */
    void resetValue();

    /**
     * \brief Gets the current stored output value without calculating a new value
     *
     * \return Controller output
     */
    double getOutput() const;

    /**
     * \brief Gets the current time the controller is using
     *
     * \return Time in milliseconds
     */
    double getCurrentTime() const;

    /**
     * \brief Gets the most recent time difference between updates
     *
     * \return Time in milliseconds
     */
    uint32_t getdT() const;

    /**
     * \brief Sets the kI constant
     *
     * \param gain
     *        Integral gain
     */
    void setkI(double gain);

    /**
     * \brief Operator overload for getting the current filter ouput
     *
     * \return Controller output
     */
    double operator*();

    /**
     * \brief Sets whether the antiWindupEnabled flag is on or off
     *
     * \param enabled
     *        Flag value
     */
    void setAntiWindupEnabled(bool enabled);

    /**
     * \brief Sets the error range from the target value that is used for anti-windup
     *
     * \param range
     *        Max error value
     */
    void setAntiWindupRange(double range);
};

/**
 * \brief D Controller
 */
class DerivativeController {
   private:
    std::shared_ptr<double> error;
    double kD;
    double currentInput;
    double previousInput;
    double derivative;
    double motorGearing;
    uint32_t currentTime;
    uint32_t previousTime;
    uint32_t dT;

   public:
    /**
     * \brief Creates a D controller for a V5 motor
     *
     * \param kD
     *        Derivative gain
     *
     * \param error
     *        A pointer to a double containing the error value to use for calculations
     *
     * \param motorGearing
     *        The output speed of the motor, in RPM, if the motor internally
     *        is spinning at 3600 RPM. 200 is default for a green motor cartridge
     */
    DerivativeController(double kD, std::shared_ptr<double> error, double motorGearing = 200);

    /**
     * \brief Calculates controller output based on the current error value
     *
     * \return Controller output
     */
    double update();

    /**
     * \brief Gets the kD constant
     *
     * \return kD value
     */
    double getkD() const;

    /**
     * \brief Gets the current stored output value without calculating a new value
     *
     * \return Controller output
     */
    double getOutput() const;

    /**
     * \brief Gets the current time the controller is using
     *
     * \return Time in milliseconds
     */
    double getCurrentTime() const;

    /**
     * \brief Gets the most recent time difference between updates
     *
     * \return Time in milliseconds
     */
    uint32_t getdT() const;

    /**
     * \brief Sets the kD constant
     *
     * \param gain
     *        Derivative gain
     */
    void setkD(double gain);

    /**
     * \brief Operator overload for getting the current filter ouput
     *
     * \return Controller output
     */
    double operator*();
};

/**
 * \brief TBH Controller
 */
class TakeBackHalfController {
   private:
    std::shared_ptr<double> error;
    double kH;
    double output;
    double tbh;
    double previousError;
    uint32_t currentTime;
    uint32_t previousTime;
    uint32_t dT;

   public:
    /**
     * \brief Creates an Take Back Half controller for a V5 motor
     *
     * \param kH
     *        TBH gain
     *
     * \param error
     *        A pointer to a double containing the error value to use for calculations
     */
    TakeBackHalfController(double kH, std::shared_ptr<double> error);

    /**
     * \brief Calculates controller output based on the current error value
     *
     * \return Controller output
     */
    double update();

    /**
     * \brief Gets the current stored output value without calculating a new value
     *
     * \return Controller output
     */
    double getOutput() const;

    /**
     * \brief Gets the kH constant
     *
     * \return kH value
     */
    double getkH() const;

    /**
     * \brief Gets the current stored TBH value
     *
     * \return TBH value
     */
    double getTBH() const;

    /**
     * \brief Sets the kH constant
     *
     * \param gain
     *        TBH gain
     */
    void setkH(double gain);

    /**
     * \brief Gets the current time the controller is using
     *
     * \return Time in milliseconds
     */
    double getCurrentTime() const;

    /**
     * \brief Operator overload for getting the current filter ouput
     *
     * \return Controller output
     */
    double operator*() const;
};

/**
 * \brief Custom Velocity Controller
 *
 * To disable a particular controller, set its gain to 0.
 */
struct SpeedControllerInfo {
    /**
     * \brief Lambda function to use for feedforward kV calculation
     */
    kv_fn_t kV = [](double rpm) { return 0; };

    /**
     * \brief P controller gain
     */
    double kP = 0;

    /**
     * \brief I controller gain
     */
    double kI = 0;

    /**
     * \brief D controller gain
     */
    double kD = 0;

    /**
     * \brief TBH controller gain
     */
    double kH = 0;

    /**
     * \brief Whether or not to enable anti-windup on the I controller
     */
    bool antiWindupEnabled = false;

    /**
     * \brief Range to use for the anti-windup on the I controller, if enabled
     */
    double antiWindupRange = 0;

    /**
     * \brief Whether or not to change the P controller gain within a certain target range
     */
    bool pRangeEnabled = false;

    /**
     * \brief Range to use for changing P controller gain, if enabled
     */
    double pRange = 0;

    /**
     * \brief Secondary kP value that the P controller uses when inside the target range, if enabled
     */
    double kP2 = 0;

    /**
     * \brief Positive range from target above which the motor applies max voltage outside of
     */
    double maxVoltageRange = 0;

    /**
     * \brief Whether or not to enable coast-down to more quickly move to a lower velocity.
     */
    bool coastDownEnabled = false;

    /**
     * \brief Negative range from target below which the applied voltage is multiplied by the
     * coast-down modifier
     */
    double coastDownRange = 0;

    /**
     * \brief Constant to multiple the applied voltage by if error is below the coast-down range
     */
    double coastDownModifier = 1;

    SpeedControllerInfo(
        kv_fn_t kV = [](double rpm) { return 0; }, double kP = 0, double kI = 0, double kD = 0,
        double kH = 0, bool antiWindupEnabled = false, double antiWindupRange = 0,
        bool pRangeEnabled = false, double pRange = 0, double kP2 = 0, double maxVoltageRange = 0,
        bool coastDownEnabled = false, double coastDownRange = 0, double coastDownModifier = 1)
        : kV(kV),
          kP(kP),
          kI(kI),
          kD(kD),
          kH(kH),
          antiWindupEnabled(antiWindupEnabled),
          antiWindupRange(antiWindupRange),
          pRangeEnabled(pRangeEnabled),
          pRange(pRange),
          kP2(kP2),
          maxVoltageRange(maxVoltageRange),
          coastDownEnabled(coastDownEnabled),
          coastDownRange(coastDownRange),
          coastDownModifier(coastDownModifier) {}
};

}  // namespace sylib