/**
 * \file src/sylib/math.cpp
 *
 * \brief Contains definitions for various controllers and math utilities
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */


#include "sylib/math.hpp"
#include <vector>
#include "sylib/env.hpp"


namespace sylib {
EMAFilter::EMAFilter() {
    ema = 0;
    inputkA = 1;
}

double EMAFilter::filter(double rawValue, double kA) {
    inputkA = kA;
    ema = rawValue * inputkA + ema * (1 - inputkA);
    return ema;
}

double EMAFilter::getkA() const { return inputkA; }

double EMAFilter::getCurrentEMA() const { return ema; }

SMAFilter::SMAFilter(int sampleSize) : sampleSize(sampleSize) {
    meanValue = 0;
    rawVelocityTotal = 0;
}

double SMAFilter::filter(double rawValue) {
    rawVelocityTotal += rawValue;
    if (rawInputValues.size() >= sampleSize) {
        rawVelocityTotal -= rawInputValues.front();
        rawInputValues.pop();
    }
    rawInputValues.push(rawValue);
    meanValue = rawVelocityTotal / rawInputValues.size();
    return meanValue;
}

int SMAFilter::getQueueLength() const { return rawInputValues.size(); }

int SMAFilter::getQueueMaxLength() const { return sampleSize; }

double SMAFilter::getCurrentTotal() const { return rawVelocityTotal; }

double SMAFilter::getCurrentValue() const { return meanValue; }

MedianFilter::MedianFilter(int sampleSize, int meanSizeEven, int meanSizeOdd)
    : sampleSize(sampleSize), meanSizeEven(meanSizeEven), meanSizeOdd(meanSizeOdd) {
    queueLength = 0;
    medianValue = 0;
}

double MedianFilter::filter(double rawValue) {
    queueLength = rawInputValues.size();
    if (queueLength >= sampleSize) {
        rawInputValues.pop_front();
    }
    rawInputValues.push_back(rawValue);
    queueLength = rawInputValues.size();
    std::vector<double> rawInputValuesArray;
    rawInputValuesArray.resize(queueLength);
    for (int i = 0; i < queueLength; i++) {
        rawInputValuesArray[i] = rawInputValues[i];
    }
    std::sort(rawInputValuesArray.begin(), rawInputValuesArray.end());
    medianValue = 0;
    if (queueLength % 2 == 0) {
        for (int i = 0; i < meanSizeEven; i++) {
            if (i % 2 == 0) {
                int j = std::floor(queueLength / 2) + std::ceil(i / 2);
                medianValue += rawInputValuesArray[j];
            } else {
                int j = std::ceil(queueLength / 2) - std::ceil(i / 2);
                medianValue += rawInputValuesArray[j];
            }
        }
        medianValue = medianValue / meanSizeEven;
    } else {
        for (int i = 0; i < meanSizeOdd; i++) {
            if (i % 2 == 0) {
                int j = std::floor(queueLength / 2) + std::ceil(i / 2);
                medianValue += rawInputValuesArray[j];
            } else {
                int j = std::ceil(queueLength / 2) - std::ceil(i / 2);
                medianValue += rawInputValuesArray[j];
            }
        }
        medianValue = medianValue / meanSizeOdd;
    }
    return medianValue;
}

int MedianFilter::getQueueLength() const { return queueLength; }

int MedianFilter::getQueueMaxLength() const { return sampleSize; }

int MedianFilter::getEvenCenterSize() const { return meanSizeEven; }

int MedianFilter::getOddCenterSize() const { return meanSizeOdd; }

double MedianFilter::getCurrentValue() const { return medianValue; }

RangeExtremaFilter::RangeExtremaFilter(int sampleSize) : sampleSize(sampleSize) {
    queueLength = 0;
    maxValue = 0;
}

double RangeExtremaFilter::filter(double rawValue) {
    queueLength = rawInputValues.size();
    if (queueLength >= sampleSize) {
        rawInputValues.pop_front();
    }
    rawInputValues.push_back(rawValue);
    queueLength = rawInputValues.size();
    std::vector<double> rawInputValuesArray;
    rawInputValuesArray.resize(queueLength);
    for (int i = 0; i < queueLength; i++) {
        rawInputValuesArray[i] = std::abs(rawInputValues[i]);
    }
    std::sort(rawInputValuesArray.begin(), rawInputValuesArray.end(), std::greater<double>());
    maxValue = rawInputValuesArray[0];
    return maxValue;
}

double RangeExtremaFilter::getCurrentValue() const { return maxValue; }

int RangeExtremaFilter::getQueueLength() const { return queueLength; }

int RangeExtremaFilter::getQueueMaxLength() const { return sampleSize; }

SympleDerivativeSolver::SympleDerivativeSolver() {
    currentInputFunctionValue = 0;
    previousInputFunctionValue = 0;
    deltaInputFunctionValue = 0;
    derivativeFunctionValue = 0;
    previousTime = vexSystemTimeGet();
}

double SympleDerivativeSolver::solveDerivative(double input) {
    currentInputFunctionValue = input;
    deltaInputFunctionValue = currentInputFunctionValue - previousInputFunctionValue;
    previousInputFunctionValue = currentInputFunctionValue;
    currentTime = vexSystemTimeGet();
    dT = currentTime - previousTime;
    previousTime = currentTime;
    if (dT <= 0) {
        return derivativeFunctionValue;
    }
    derivativeFunctionValue = deltaInputFunctionValue / dT;
    return derivativeFunctionValue;
}

double SympleDerivativeSolver::getCurrentDerivative() const { return derivativeFunctionValue; }

double SympleDerivativeSolver::getCurrentInputValue() const { return currentInputFunctionValue; }

VoltageEstimation::VoltageEstimation(kv_fn_t kV, double motorGearing)
    : kV(kV), motorGearing(motorGearing) {
    voltageEstimate = 0;
}

double VoltageEstimation::estimate(double rpm) {
    voltageEstimate = rpm * kV(rpm);
    return voltageEstimate;
}

kv_fn_t VoltageEstimation::getKv() const { return kV; }

double VoltageEstimation::getMotorGearing() const { return motorGearing; }

void VoltageEstimation::setkV(kv_fn_t value) { kV = value; }

double VoltageEstimation::getOutput() const { return voltageEstimate; }

ProportionalController::ProportionalController(double kP, std::shared_ptr<double> error,
                                               double motorGearing, bool maxRangeEnabled,
                                               double kP2, double maxRange)
    : kP(kP),
      motorGearing(motorGearing),
      error(error),
      maxRange(maxRange),
      maxRangeEnabled(maxRangeEnabled),
      kP2(kP2) {
    proportional = 0;
}

double ProportionalController::update() {
    if (maxRangeEnabled) {
        if (std::abs(*error) < maxRange) {
            proportional = *error * kP2 * 3600 / motorGearing;
            return proportional;
        }
    }
    proportional = *error * kP * 3600 / motorGearing;
    return proportional;
}

double ProportionalController::getkP() const { return kP; }

double ProportionalController::getOutput() const { return proportional; }

void ProportionalController::setkP(double gain) { kP = gain; }

double ProportionalController::operator*() { return getOutput(); }

void ProportionalController::setMaxRangeEnabled(bool enabled) { maxRangeEnabled = enabled; }

void ProportionalController::setMaxRange(double range) { maxRange = range; }

void ProportionalController::setkP2(double gain) { kP2 = gain; }

IntegralController::IntegralController(double kI, std::shared_ptr<double> error,
                                       double motorGearing, bool antiWindupEnabled,
                                       double antiWindupRange)
    : kI(kI),
      motorGearing(motorGearing),
      error(error),
      antiWindupEnabled(antiWindupEnabled),
      antiWindupRange(antiWindupRange) {
    integral = 0;
    currentTime = vexSystemTimeGet();
    previousTime = currentTime;
    dT = 0;
}

double IntegralController::update() {
    currentTime = vexSystemTimeGet();
    dT = currentTime - previousTime;
    previousTime = currentTime;
    if (dT <= 0) {
        return integral * kI;
    }
    if (antiWindupEnabled) {
        if (std::abs(*error) > antiWindupRange) {
            integral = 0;
            return integral * kI;
        }
    }
    integral += (*error * dT) * 3600 / motorGearing;
    return integral * kI;
}

double IntegralController::getOutput() const { return integral * kI; }

double IntegralController::getkI() const { return kI; }

double IntegralController::getCurrentTime() const { return currentTime; }

uint32_t IntegralController::getdT() const { return dT; }

void IntegralController::setkI(double gain) { kI = gain; }

double IntegralController::operator*() { return getOutput(); }

void IntegralController::resetValue() { integral = 0; }

void IntegralController::setAntiWindupEnabled(bool enabled) { antiWindupEnabled = enabled; }

void IntegralController::setAntiWindupRange(double range) { antiWindupRange = range; }

DerivativeController::DerivativeController(double kD, std::shared_ptr<double> error,
                                           double motorGearing)
    : kD(kD), motorGearing(motorGearing), error(error) {
    derivative = 0;
    currentInput = *error;
    previousInput = currentInput;
    derivative = 0;
    currentTime = vexSystemTimeGet();
    previousTime = currentTime;
    dT = 0;
}

double DerivativeController::update() {
    currentTime = vexSystemTimeGet();
    currentInput = *error;
    dT = currentTime - previousTime;
    previousTime = currentTime;
    if (dT <= 0) {
        return derivative;
    }

    derivative = (currentInput - previousInput) / dT * 3600 / motorGearing;

    previousInput = currentInput;
    return derivative * kD;
}

double DerivativeController::getOutput() const { return derivative * kD; }

double DerivativeController::getCurrentTime() const { return currentTime; }

double DerivativeController::getkD() const { return kD; }

uint32_t DerivativeController::getdT() const { return dT; }

void DerivativeController::setkD(double gain) { kD = gain; }

double DerivativeController::operator*() { return getOutput(); }

TakeBackHalfController::TakeBackHalfController(double kH, std::shared_ptr<double> error)
    : error(error), kH(kH) {
    output = 0;
    previousError = 0;
    tbh = 0;
    currentTime = vexSystemTimeGet();
}

double TakeBackHalfController::update() {
    currentTime = vexSystemTimeGet();
    if (std::abs(*error) < 200) {
        output += *error * kH;
        if (*error * previousError <= 0) {
            output = 0.5 * (output + tbh);
            tbh = output;
        }
        previousError = *error;
        return output;
    } else {
        return 0;
    }
}

double TakeBackHalfController::getOutput() const { return output; }

double TakeBackHalfController::getkH() const { return kH; }

double TakeBackHalfController::getTBH() const { return tbh; }

double TakeBackHalfController::operator*() const { return getOutput(); }

void TakeBackHalfController::setkH(double gain) { kH = gain; }
}  // namespace sylib