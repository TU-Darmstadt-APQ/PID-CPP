/*    
 *  The PID library is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as 
 *  published by the Free Software Foundation, either version 3 of the 
 *  License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>   // abs()
#include "pid.h"

PID::PID(const uint32_t _setpoint, const double kp, const double ki, const double kd, uint8_t _qn, FeedbackDirection _feedbackDirection, ProportionalGain proportionalGain)
    : feedbackDirection(_feedbackDirection), qn(_qn), setpoint(_setpoint) {
    this->setTunings(kp, ki, kd, proportionalGain);
}

PID::PID(const uint32_t setpoint, const double kp, const double ki, const double kd, uint8_t qn, FeedbackDirection feedbackDirection)
    :PID::PID(setpoint, kp, ki, kd, qn, feedbackDirection, proportionalToError) {}

PID::PID(const uint32_t setpoint, const double kp, const double ki, const double kd, uint8_t qn)
    :PID::PID(setpoint, kp, ki, kd, qn, feedbackNegative) {}

/**
    Calculate a new output and store update all internal state variables. Note: The input is a simple number without
    any fixed point fractional precision. The reason is, that we need to scale the output according to the DAC
    precision anyway.
    
    QN should be scaled to that it is Q(DAC_resolution).(32-DAC_resolution).

    @param input The new input variable in Q32.0 format, that is with zero frational precision. The value is
      an unsigned integer in Offset Binary notation.
    @param *output The output in Q(DAC_resolution).0 format and is determinded by the ki, kp, kd
      parameters used in the multiplications. The encoding is Offset Binary, so it directly be fed to most DACs.
*/
const uint32_t PID::compute(const uint32_t input) {
      /*Compute all the working error variables*/
      double error = this->setPoint - input;
      double dInput = (input - lastInput);
      this->errorSum += (ki * error);

      this->errorSum = clamp(errorSum, outputMin, outputMax);

      /*Add Proportional on Error, if P_ON_E is specified*/
      double output = kp * error;

      /*Compute Rest of PID Output*/
      output += this->errorSum  - kd * dInput;
      output = clamp(output, outputMin, outputMax);

      /*Remember some variables for next time*/
      this->lastInput = input;
      return output;
}

/** Note: ki and kd must be normalized to the sampling time
 */
void PID::setTunings(const double kp, const double ki, const double kd, const ProportionalGain proportionalGain) {
    this->kp = (this->feedbackDirection == feedbackPositive) ? abs(kp) : -abs(kp);
    this->ki = (this->feedbackDirection == feedbackPositive) ? abs(ki) : -abs(ki);
    this->kd = (this->feedbackDirection == feedbackPositive) ? abs(kd) : -abs(kd);
    
    this->proportionalGain = proportionalGain;
}

void PID::setTunings(const double kp, const double ki, const double kd) {
   this->setTunings(kp, ki, kd, this->proportionalGain);
}

void PID::setOutputMin(const uint32_t value) {
    this->outputMin = value;    // Convert from Offset Binary to Integer
}

void PID::setOutputMax(const uint32_t value) {
    this->outputMax = value;    // Convert from Offset Binary to Integer
}

void PID::setSetpoint(const uint32_t value) {
    this->setpoint = value;
}

const uint32_t PID::getSetpoint() {
    return this->setpoint;
}

void PID::init(const uint32_t initialInput) {
   this->previousInput = initialInput;
}

void PID::updateOutput(const uint32_t value) {
    this->errorSum = clamp(value, this->outputMin, this->outputMax);
}

void PID::setControllerFeedback(const FeedbackDirection feedbackDirection) {
  this->feedbackDirection = feedbackDirection;

  this->setTunings(this->kp, this->ki, this->kd);
}

const double PID::getKp() {
    return this->kp;
}

const double PID::getKi() {
    return  this->ki;
}

const double PID::getKd() {
    return this->kd;
}

