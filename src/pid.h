#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "../../FSM/fsm_pid.hpp"
#include "../../FSM/interface.h"

#define LIKELY(x)       __builtin_expect(!!(x), 1)
#define UNLIKELY(x)     __builtin_expect(!!(x), 0)

static inline const double clamp(const double value, const double min, const double max) __attribute__((always_inline, unused));
static inline const double clamp(const double value, const double min, const double max) {
    return (value < min) ? min : (value > max) ? max : value;
}

enum FeedbackDirection {
    feedbackPositive = 0,
    feedbackNegative = 1,
};

enum ProportionalGain {
    proportionalToInput = 0,
    proportionalToError = 1,
};

class PID : public pidInterface {
    public:
        PID(const uint32_t setpoint, const double kp, const double ki, const double kd, FeedbackDirection feedbackDirection, ProportionalGain proportionalGain);
        PID(const uint32_t setpoint, const double kp, const double ki, const double kd, FeedbackDirection feedbackDirection);
        PID(const uint32_t setpoint, const double kp, const double ki, const double kd);

        const uint32_t compute(int32_t input);
        const double getInput();

        void setTunings(const double kp, const double ki, const double kd);
        void setTunings(const double kp, const double ki, const double kd, const ProportionalGain proportionalGain);
		void setKp(double newParameter);
		void setKi(double newParameter);
		void setKd(double newParameter);
        const double getKp();
        const double getKi();
        const double getKd();
        void setSetpoint(const uint32_t value);
        const uint32_t getSetpoint();
        void setControllerFeedback(const FeedbackDirection feedbackDirection);
        void setOutputMin(const uint32_t value);
        void setOutputMax(const uint32_t value);
        void updateOutput(const uint32_t value);
        void init(const uint32_t initialInput);
		
		void setFeedbackDirection(bool directionOfFeedback);
		void setGain(bool gain);
		void updateTunings();
		
		const uint32_t computeEMAFilter(int32_t input);
		
    private:
        double kp, ki, kd;
        FeedbackDirection feedbackDirection;
        ProportionalGain proportionalGain;
        int32_t outputMin = INT32_MIN;
        int32_t outputMax = INT32_MAX;
        uint32_t previousInput = 0;
        double errorSum = 0;
		double emaParameter = 1;
		double filteredValue;
    protected:
        uint32_t setpoint;
};
#endif

