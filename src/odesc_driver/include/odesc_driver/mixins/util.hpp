#ifndef ODESC_DRIVER_MIXINS_UTIL_HPP
#define ODESC_DRIVER_MIXINS_UTIL_HPP

#include <dbcppp/CApi.h>
#include <dbcppp/Network.h>
#include <linux/can/raw.h>

using frame = struct can_frame;

bool isSig(const dbcppp::ISignal& sig, const dbcppp::ISignal* mux_sig, frame& f);

#endif // ODESC_DRIVER_MIXINS_UTIL_HPP