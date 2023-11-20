/*
 *    Copyright (c) 2023, The OpenThread Authors.
 *    All rights reserved.
 *
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted provided that the following conditions are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *    3. Neither the name of the copyright holder nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *    POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OTDAEMON_TELEMETRY_HPP_
#define OTDAEMON_TELEMETRY_HPP_

#include <openthread/instance.h>

namespace otbr {
namespace Android {

struct Telemetries {
    int32_t phy_rx;
    int32_t phy_tx = 2;
    int32_t mac_unicast_rx = 3;
    int32_t mac_unicast_tx = 4;
    int32_t mac_broadcast_rx = 5;
    int32_t mac_broadcast_tx = 6;
    int32_t mac_tx_ack_req = 7;
    int32_t mac_tx_no_ack_req = 8;
    int32_t mac_tx_acked = 9;
    int32_t mac_tx_data = 10;
    int32_t mac_tx_data_poll = 11;
    int32_t mac_tx_beacon = 12;
    int32_t mac_tx_beacon_req = 13;
    int32_t mac_tx_other_pkt = 14;
    int32_t mac_tx_retry = 15;
    int32_t mac_rx_data = 16;
    int32_t mac_rx_data_poll = 17;
    int32_t mac_rx_beacon = 18;
    int32_t mac_rx_beacon_req = 19;
    int32_t mac_rx_other_pkt = 20;
    int32_t mac_rx_filter_whitelist = 21;
    int32_t mac_rx_filter_dest_addr = 22;
    int32_t mac_tx_fail_cca = 23;
    int32_t mac_rx_fail_decrypt = 24;
    int32_t mac_rx_fail_no_frame = 25;
    int32_t mac_rx_fail_unknown_neighbor = 26;
    int32_t mac_rx_fail_invalid_src_addr = 27;
    int32_t mac_rx_fail_fcs = 28;
    int32_t mac_rx_fail_other = 29;
    int32_t ip_tx_success = 30;
    int32_t ip_rx_success = 31;
    int32_t ip_tx_failure = 32;
    int32_t ip_rx_failure = 33;
    uint32_t node_type = 34;
    uint32_t channel = 35;
    int32_t radio_tx_power = 36;
    float mac_cca_fail_rate = 37;
};

void ConvertAndPushAtoms(otInstance *otInstance/*, const TelemetryData &telemetryData*/);
} // namespace Android
} // namespace otbr
#endif // OTDAEMON_TELEMETRY_HPP_
