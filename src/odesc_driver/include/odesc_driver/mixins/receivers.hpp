#ifndef ODESC_DRIVER_CAN_RECEIVERS_HPP
#define ODESC_DRIVER_CAN_RECEIVERS_HPP

#include <linux/can/raw.h>

#include <atomic>

#include "util.hpp"

using frame = struct can_frame;

namespace odesc_driver {

class HeartbeatReceiver {
   public:
    HeartbeatReceiver() : axisCurrentState(0) {}

    int getAxisCurrentState() {
        return static_cast<int>(axisCurrentState);
    }

    void updateHeartbeatData(uint64_t data) {
        axisCurrentState = data >> 32 & 0xFF;
    }

   private:
    std::atomic<uint8_t> axisCurrentState;
};

class GetEncoderEstimatesReceiver {
   public:
    GetEncoderEstimatesReceiver() : encoderPosEstimate(0.0), encoderVelEstimate(0.0) {}

    double getEncoderPosEstimate() {
        return static_cast<double>(encoderPosEstimate);
    }

    double getEncoderVelEstimate() {
        return static_cast<double>(encoderVelEstimate);
    }

    void updateEncoderEstimateData(uint64_t data) {
        float* estimates = reinterpret_cast<float*>(&data);
        encoderPosEstimate = estimates[0];
        encoderVelEstimate = estimates[1];
    }

   private:
    std::atomic<float> encoderPosEstimate;
    std::atomic<float> encoderVelEstimate;
};

};  // namespace odesc_driver

#endif  // ODESC_DRIVER_CAN_RECEIVERS_HPP