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
        axisError = data & 0xFFFFFFFF;
        axisCurrentState = data >> 32 & 0xFF;
        motorErrorFlag = data >> 40 & 0x1;
        encoderErrorFlag = data >> 48 & 0x1;
        controllerErrorFlag = data >> 56 & 0x1;
        trajectoryDoneFlag = data >> 63;
    }

   private:
    std::atomic<uint32_t> axisError;
    std::atomic<uint8_t> axisCurrentState;
    std::atomic<bool> motorErrorFlag;
    std::atomic<bool> encoderErrorFlag;
    std::atomic<bool> controllerErrorFlag;
    std::atomic<bool> trajectoryDoneFlag;
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