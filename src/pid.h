#ifndef PID_H
#define PID_H

#include <stdint.h>

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

class PID {
    public:
        PID(const uint32_t setpoint, const double kp, const double ki, const double kd, uint8_t _qn, FeedbackDirection feedbackDirection, ProportionalGain proportionalGain);
        PID(const uint32_t setpoint, const double kp, const double ki, const double kd, uint8_t _qn, FeedbackDirection feedbackDirection);
        PID(const uint32_t setpoint, const double kp, const double ki, const double kd, uint8_t _qn);

        const uint32_t compute(int32_t input);

        void setTunings(const double kp, const double ki, const double kd);
        void setTunings(const double kp, const double ki, const double kd, const ProportionalGain proportionalGain);
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
    private:
        double kp, ki, kd;
        FeedbackDirection feedbackDirection;
        ProportionalGain proportionalGain;
        uint8_t qn;
        int32_t outputMin = INT32_MIN;
        int32_t outputMax = INT32_MAX;
        uint32_t previousInput = 0;
        double errorSum = 0;
    protected:
        uint32_t setpoint;
};
#endif

