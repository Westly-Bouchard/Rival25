#ifndef ODESC_DRIVER_MIXINS_UTIL_HPP
#define ODESC_DRIVER_MIXINS_UTIL_HPP

#include <linux/can/raw.h>

namespace odesc_driver {

enum MsgType { HEARTBEAT = 1, GET_ENCODER_ESTIMATES = 9, GET_ENCODER_COUNT = 10 };
};

#endif  // ODESC_DRIVER_MIXINS_UTIL_HPP