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

#include <vector>

#include <openthread/instance.h>

namespace otbr {
namespace Android {

struct ThreadnetworkTelemetryDataReported {
    int32_t phy_rx;
    int32_t phy_tx; 
    int32_t mac_unicast_rx; 
    int32_t mac_unicast_tx; 
    int32_t mac_broadcast_rx; 
    int32_t mac_broadcast_tx; 
    int32_t mac_tx_ack_req; 
    int32_t mac_tx_no_ack_req; 
    int32_t mac_tx_acked; 
    int32_t mac_tx_data; 
    int32_t mac_tx_data_poll; 
    int32_t mac_tx_beacon; 
    int32_t mac_tx_beacon_req; 
    int32_t mac_tx_other_pkt; 
    int32_t mac_tx_retry; 
    int32_t mac_rx_data; 
    int32_t mac_rx_data_poll; 
    int32_t mac_rx_beacon; 
    int32_t mac_rx_beacon_req; 
    int32_t mac_rx_other_pkt; 
    int32_t mac_rx_filter_whitelist; 
    int32_t mac_rx_filter_dest_addr; 
    int32_t mac_tx_fail_cca; 
    int32_t mac_rx_fail_decrypt; 
    int32_t mac_rx_fail_no_frame; 
    int32_t mac_rx_fail_unknown_neighbor; 
    int32_t mac_rx_fail_invalid_src_addr; 
    int32_t mac_rx_fail_fcs; 
    int32_t mac_rx_fail_other; 
    int32_t ip_tx_success; 
    int32_t ip_rx_success; 
    int32_t ip_tx_failure; 
    int32_t ip_rx_failure; 
    uint32_t node_type; 
    uint32_t channel; 
    int32_t radio_tx_power; 
    float mac_cca_fail_rate; 
    
    uint32_t rloc16; 
    uint32_t router_id; 
    uint32_t leader_router_id; 
    uint32_t leader_rloc16;  // replaced bytes leader_address; 
    uint32_t leader_weight; 
    uint32_t leader_local_weight; 
    uint32_t preferred_router_id; 
    uint32_t partition_id; 
    uint32_t child_table_size; 
    uint32_t neighbor_table_size; 
    int32_t instant_rssi; 
    bool has_extended_pan_id; 
    bool is_active_br; 
    bool is_active_srp_server; 
    uint32_t sum_on_link_prefix_changes; 

    // The number of Router Advertisement packets received by otbr-agent on the
    // infra link
    int64_t ra_rx; 

    // The number of Router Advertisement packets successfully transmitted by
    // otbr-agent on the infra link.
    int64_t ra_tx_success; 

    // The number of Router Advertisement packets failed to transmit by
    // otbr-agent on the infra link.
    int64_t ra_tx_failure; 

    // The number of Router Solicitation packets received by otbr-agent on the
    // infra link
    int64_t rs_rx; 

    // The number of Router Solicitation packets successfully transmitted by
    // otbr-agent on the infra link.
    int64_t rs_tx_success; 

    // The number of Router Solicitation packets failed to transmit by
    // otbr-agent on the infra link.
    int64_t rs_tx_failure; 

    // The counters for inbound unicast packets
    int64_t inbound_unicast_packet_count; 
    int64_t inbound_unicast_byte_count; 

    // The counters for inbound multicast packets
    int64_t inbound_multicast_packet_count; 
    int64_t inbound_multicast_byte_count; 

    // The counters for outbound unicast packets
    int64_t outbound_unicast_packet_count; 
    int64_t outbound_unicast_byte_count; 

    // The counters for outbound multicast packets
    int64_t outbound_multicast_packet_count; 
    int64_t outbound_multicast_byte_count; 

    // The inbound and outbound NAT64 traffic through the border router
    int64_t ipv4_to_ipv6_packets; 
    int64_t ipv4_to_ipv6_bytes; 
    int64_t ipv6_to_ipv4_packets; 
    int64_t ipv6_to_ipv4_bytes; 
    int64_t ipv4_to_ipv6_packets1; 
    int64_t ipv4_to_ipv6_bytes1; 
    int64_t ipv6_to_ipv4_packets1; 
    int64_t ipv6_to_ipv4_bytes1; 
    int64_t ipv4_to_ipv6_packets2; 
    int64_t ipv4_to_ipv6_bytes2; 
    int64_t ipv6_to_ipv4_packets2; 
    int64_t ipv6_to_ipv4_bytes2; 

    // Error counters for NAT64 translator on the border router
    int64_t err_ipv4_to_ipv6_packets; 
    int64_t err_ipv6_to_ipv4_packets; 
    int64_t err_ipv4_to_ipv6_packets1; 
    int64_t err_ipv6_to_ipv4_packets1; 
    int64_t err_ipv4_to_ipv6_packets2; 
    int64_t err_ipv6_to_ipv4_packets2; 
    int64_t err_ipv4_to_ipv6_packets3; 
    int64_t err_ipv6_to_ipv4_packets3; 
    int64_t err_ipv4_to_ipv6_packets4; 
    int64_t err_ipv6_to_ipv4_packets4; 

    uint8_t state; 

    // Listening port number
    uint32_t port; 
    // The address mode {unicast, anycast} of the SRP server
    uint8_t address_mode; 

    // The number of active hosts/services registered on the SRP server.
    uint32_t fresh_count; 

    // The number of hosts/services in 'Deleted' state on the SRP server.
    uint32_t deleted_count; 

    // The sum of lease time in milliseconds of all active hosts/services on the
    // SRP server.
    uint64_t lease_time_total_ms; 

    // The sum of key lease time in milliseconds of all active hosts/services on
    // the SRP server.
    uint64_t key_lease_time_total_ms; 

    // The sum of remaining lease time in milliseconds of all active
    // hosts/services on the SRP server.
    uint64_t remaining_lease_time_total_ms; 

    // The sum of remaining key lease time in milliseconds of all active
    // hosts/services on the SRP server.
    uint64_t remaining_key_lease_time_total_ms; 

    // The number of active hosts/services registered on the SRP server.
    uint32_t fresh_count2; 

    // The number of hosts/services in 'Deleted' state on the SRP server.
    uint32_t deleted_count2; 

    // The sum of lease time in milliseconds of all active hosts/services on the
    // SRP server.
    uint64_t lease_time_total_ms2; 

    // The sum of key lease time in milliseconds of all active hosts/services on
    // the SRP server.
    uint64_t key_lease_time_total_ms2; 

    // The sum of remaining lease time in milliseconds of all active
    // hosts/services on the SRP server.
    uint64_t remaining_lease_time_total_ms2; 

    // The sum of remaining key lease time in milliseconds of all active
    // hosts/services on the SRP server.
    uint64_t remaining_key_lease_time_total_ms2; 

    // The number of successful responses
    uint32_t success_count; 

    // The number of server failure responses
    uint32_t server_failure_count; 

    // The number of format error responses
    uint32_t format_error_count; 

    // The number of 'name exists' responses
    uint32_t name_exists_count; 

    // The number of refused responses
    uint32_t refused_count; 

    // The number of other responses
    uint32_t other_count; 

    // The number of successful responses
    uint32_t dns_response_success_count; 

    // The number of server failure responses
    uint32_t dns_response_server_failure_count; 

    // The number of format error responses
    uint32_t dns_response_format_error_count; 

    // The number of name error responses
    uint32_t dns_response_name_error_count; 

    // The number of 'not implemented' responses
    uint32_t dns_response_not_implemented_count; 

    // The number of other responses
    uint32_t dns_response_other_count; 

    // The number of DNS queries resolved at the local SRP server
    uint32_t resolved_by_local_srp_count; 

    // The number of successful responses
    uint32_t host_registration_success_count; 

    // The number of 'not found' responses
    uint32_t host_registration_not_found_count; 

    // The number of 'invalid arg' responses
    uint32_t host_registration_invalid_args_count; 

    // The number of 'duplicated' responses
    uint32_t host_registration_duplicated_count; 

    // The number of 'not implemented' responses
    uint32_t host_registration_not_implemented_count; 

    // The number of unknown error responses
    uint32_t host_registration_unknown_error_count; 

    // The number of aborted responses
    uint32_t host_registration_aborted_count; 

    // The number of invalid state responses
    uint32_t host_registration_invalid_state_count; 

    // The number of successful responses
    uint32_t service_registration_success_count; 

    // The number of 'not found' responses
    uint32_t service_registration_not_found_count; 

    // The number of 'invalid arg' responses
    uint32_t service_registration_invalid_args_count; 

    // The number of 'duplicated' responses
    uint32_t service_registration_duplicated_count; 

    // The number of 'not implemented' responses
    uint32_t service_registration_not_implemented_count; 

    // The number of unknown error responses
    uint32_t service_registration_unknown_error_count; 

    // The number of aborted responses
    uint32_t service_registration_aborted_count; 

    // The number of invalid state responses
    uint32_t service_registration_invalid_state_count; 

    // The number of successful responses
    uint32_t host_resolution_success_count; 

    // The number of 'not found' responses
    uint32_t host_resolution_not_found_count; 

    // The number of 'invalid arg' responses
    uint32_t host_resolution_invalid_args_count; 

    // The number of 'duplicated' responses
    uint32_t host_resolution_duplicated_count; 

    // The number of 'not implemented' responses
    uint32_t host_resolution_not_implemented_count; 

    // The number of unknown error responses
    uint32_t host_resolution_unknown_error_count; 

    // The number of aborted responses
    uint32_t host_resolution_aborted_count; 

    // The number of invalid state responses
    uint32_t host_resolution_invalid_state_count; 

    // The number of successful responses
    uint32_t service_resolution_success_count; 

    // The number of 'not found' responses
    uint32_t service_resolution_not_found_count; 

    // The number of 'invalid arg' responses
    uint32_t service_resolution_invalid_args_count; 

    // The number of 'duplicated' responses
    uint32_t service_resolution_duplicated_count; 

    // The number of 'not implemented' responses
    uint32_t service_resolution_not_implemented_count; 

    // The number of unknown error responses
    uint32_t service_resolution_unknown_error_count; 

    // The number of aborted responses
    uint32_t service_resolution_aborted_count; 

    // The number of invalid state responses
    uint32_t service_resolution_invalid_state_count; 

    // The EMA latency of host registrations in milliseconds
    uint32_t host_registration_ema_latency_ms; 

    // The EMA latency of service registrations in milliseconds
    uint32_t service_registration_ema_latency_ms; 

    // The EMA latency of host resolutions in milliseconds
    uint32_t host_resolution_ema_latency_ms; 

    // The EMA latency of service resolutions in milliseconds
    uint32_t service_resolution_ema_latency_ms;

    uint32_t prefix_manager_state = 1;
    uint32_t translator_state = 2;

    uint32_t rcp_timeout_count = 1;
    uint32_t rcp_reset_count = 2;
    uint32_t rcp_restoration_count = 3;
    uint32_t spinel_parse_error_count = 4;
    int32_t rcp_firmware_update_count = 5;
    uint32_t thread_stack_uptime = 6;

    uint32_t rcp_interface_type = 1;
    uint64_t transferred_frames_count = 2;
    uint64_t transferred_valid_frames_count = 3;
    uint64_t transferred_garbage_frames_count = 4;
    uint64_t rx_frames_count = 5;
    uint64_t rx_bytes_count = 6;
    uint64_t tx_frames_count = 7;
    uint64_t tx_bytes_count = 8;

    uint32_t count_tx_request = 1;
    uint32_t count_tx_grant_immediate = 2;
    uint32_t count_tx_grant_wait = 3;
    uint32_t count_tx_grant_wait_activated = 4;
    uint32_t count_tx_grant_wait_timeout = 5;
    uint32_t count_tx_grant_deactivated_during_request = 6;
    uint32_t tx_average_request_to_grant_time_us = 7;
    uint32_t count_rx_request = 8;
    uint32_t count_rx_grant_immediate = 9;
    uint32_t count_rx_grant_wait = 10;
    uint32_t count_rx_grant_wait_activated = 11;
    uint32_t count_rx_grant_wait_timeout = 12;
    uint32_t count_rx_grant_deactivated_during_request = 13;
    uint32_t count_rx_grant_none = 14;
    uint32_t rx_average_request_to_grant_time_us = 15;
};

struct ThreadnetworkTopoEntryRepeatedTopoEntry
{
    // 0~15: uint16_t rloc_16
    // 16~31: uint16_t version Thread version of the neighbor
    uint32_t combo_telemetry1 = 1;
    // 0~7: uint8_t link_quality_in
    // 8~15: int8_t average_rssi
    // 16~23: int8_t last_rssi
    // 24~31: uint8_t network_data_version
    uint32_t combo_telemetry2 = 2;
    uint32_t age_sec = 3;
    // Each bit on the flag represents a bool flag
    // 0: rx_on_when_idle
    // 1: full_function
    // 2: secure_data_request
    // 3: full_network_data
    // 4: is_child
    uint32_t topo_entry_flags = 4;
    uint32_t link_frame_counter = 5;
    uint32_t mle_frame_counter = 6;
    uint32_t timeout_sec = 7;
    // 0~15: uint16_t frame_error_rate. Frame error rate (0xffff->100%). Requires error tracking feature.
    // 16~31: uint16_t message_error_rate. (IPv6) msg error rate (0xffff->100%). Requires error tracking feature.
    uint32_t combo_telemetry3 = 8;
};

void RetrieveAndConvertTelemetries(otInstance *otInstance,
    ThreadnetworkTelemetryDataReported& telemetryDataReported,
    std::vector<ThreadnetworkTopoEntryRepeatedTopoEntry>& topoEntries);
} // namespace Android
} // namespace otbr
#endif // OTDAEMON_TELEMETRY_HPP_
