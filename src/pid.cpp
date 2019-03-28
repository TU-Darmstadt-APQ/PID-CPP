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
#include <Arduino.h>		// Added this library to avoid using a precompiled library version on windows
#include "pid.h"

PID::PID(const uint32_t _setpoint, const double kp, const double ki, const double kd, FeedbackDirection _feedbackDirection, ProportionalGain proportionalGain)
    : feedbackDirection(_feedbackDirection), setpoint(_setpoint) {
    this->setTunings(kp, ki, kd, proportionalGain);
	PIDFsm::pid = this;
}

PID::PID(const uint32_t setpoint, const double kp, const double ki, const double kd, FeedbackDirection feedbackDirection)
    :PID::PID(setpoint, kp, ki, kd, feedbackDirection, proportionalToError) {PIDFsm::pid = this;}

PID::PID(const uint32_t setpoint, const double kp, const double ki, const double kd)
    :PID::PID(setpoint, kp, ki, kd, feedbackNegative) {PIDFsm::pid = this;}

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
const uint32_t PID::compute(const int32_t input) {
      double error = (double)(this->setpoint) - (double)input;
      double dInput = (double)input - (double)(this->previousInput);
      this->errorSum+= (this->ki * error);
      this->errorSum = clamp(this->errorSum, this->outputMin, this->outputMax);

      double output = this->kp * error + this->errorSum - this->kd * dInput;
      output = clamp(output, this->outputMin, this->outputMax);
      this->previousInput = input;
      return output;
}

const uint32_t PID::computeEMAFilter(const int32_t input){
	this->filteredValue += (input - this->filteredValue) * this->emaParameter;
	return this->compute((int32_t)this->filteredValue);
}

const double PID::getInput() {
    return this->previousInput;
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
    this->outputMin = value;
}

void PID::setOutputMax(const uint32_t value) {
    this->outputMax = value;
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

void PID::setKp(double newParameter){
	this->kp = newParameter;
}
void PID::setKi(double newParameter){
	this->ki = newParameter;
}

void PID::setKd(double newParameter){
	this->kd = newParameter;
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

void PID::setFeedbackDirection(bool directionOfFeedback){
	if(directionOfFeedback) this->feedbackDirection = feedbackPositive;
	else this->feedbackDirection = feedbackNegative;
	this->setTunings(this->kp, this->ki, this->kd);
}

void PID::setGain(bool gain){
	if(gain) this->proportionalGain = proportionalToError;
	else this->proportionalGain = proportionalToInput;
	this->setTunings(this->kp, this->ki, this->kd);
}

void PID::updateTunings(){
	this->setTunings(this->kp, this->ki, this->kd);
}
