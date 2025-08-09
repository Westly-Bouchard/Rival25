#include "../../include/odesc_driver/mixins/util.hpp"

bool isSig(const dbcppp::ISignal& sig, const dbcppp::ISignal* mux_sig, frame& f) {
    return (sig.MultiplexerIndicator() != dbcppp::ISignal::EMultiplexer::MuxValue ||
            (mux_sig && mux_sig->Decode(f.data) == sig.MultiplexerSwitchValue()));
}