#ifndef ODESC_DRIVER_CAN_RECEIVERS_HPP
#define ODESC_DRIVER_CAN_RECEIVERS_HPP

#include <dbcppp/CApi.h>
#include <dbcppp/Network.h>
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

    void updateHeartbeatData(const dbcppp::IMessage* msg, frame& f) {
        const dbcppp::ISignal* mux_sig = msg->MuxSignal();
        for (const dbcppp::ISignal& sig : msg->Signals()) {
            if (isSig(sig, mux_sig, f)) {
                if (sig.Name() == "Axis_State") {
                    axisCurrentState = sig.RawToPhys(sig.Decode(f.data));
                }
            }
        }
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

    void updateEncoderEstimateData(const dbcppp::IMessage* msg, frame& f) {
        const dbcppp::ISignal* mux_sig = msg->MuxSignal();
        for (const dbcppp::ISignal& sig : msg->Signals()) {
            if (isSig(sig, mux_sig, f)) {
                if (sig.Name() == "Vel_Estimate") {
                    encoderVelEstimate = sig.RawToPhys(sig.Decode(f.data));
                } else if (sig.Name() == "Pos_Estimate") {
                    encoderPosEstimate = sig.RawToPhys(sig.Decode(f.data));
                }
            }
        }
    }

   private:
    std::atomic<float> encoderPosEstimate;
    std::atomic<float> encoderVelEstimate;
};

};  // namespace odesc_driver

#endif  // ODESC_DRIVER_CAN_RECEIVERS_HPP