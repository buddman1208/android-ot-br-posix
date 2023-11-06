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
#include "android/otdaemon_telemetry.hpp"

#include <openthread/thread.h>

#include "statslog_threadnetwork.h"

namespace otbr {
namespace Android {

void convertTelemetryToAtom(const TelemetryData                &telemetryData,
                                   ThreadnetworkTelemetryDataReported &telemetryDataReported)
{
    auto wpanStats     = telemetryDataReported.mutable_wpan_stats();
    auto wpanStatsData = telemetryData.wpan_stats();

    wpanStats->set_phy_rx(wpanStatsData.phy_rx());
    wpanStats->set_phy_tx(wpanStatsData.phy_tx());
    wpanStats->set_mac_unicast_rx(wpanStatsData.mac_unicast_rx());
    wpanStats->set_mac_unicast_tx(wpanStatsData.mac_unicast_tx());
    wpanStats->set_mac_broadcast_rx(wpanStatsData.mac_broadcast_rx());
    wpanStats->set_mac_broadcast_tx(wpanStatsData.mac_broadcast_tx());
    wpanStats->set_mac_tx_ack_req(wpanStatsData.mac_tx_ack_req());
    wpanStats->set_mac_tx_no_ack_req(wpanStatsData.mac_tx_no_ack_req());
    wpanStats->set_mac_tx_acked(wpanStatsData.mac_tx_acked());
    wpanStats->set_mac_tx_data(wpanStatsData.mac_tx_data());
    wpanStats->set_mac_tx_data_poll(wpanStatsData.mac_tx_data_poll());
    wpanStats->set_mac_tx_beacon(wpanStatsData.mac_tx_beacon());
    wpanStats->set_mac_tx_beacon_req(wpanStatsData.mac_tx_beacon_req());
    wpanStats->set_mac_tx_other_pkt(wpanStatsData.mac_tx_other_pkt());
    wpanStats->set_mac_tx_retry(wpanStatsData.mac_tx_retry());
    wpanStats->set_mac_rx_data(wpanStatsData.mac_rx_data());
    wpanStats->set_mac_rx_data_poll(wpanStatsData.mac_rx_data_poll());
    wpanStats->set_mac_rx_beacon(wpanStatsData.mac_rx_beacon());
    wpanStats->set_mac_rx_beacon_req(wpanStatsData.mac_rx_beacon_req());
    wpanStats->set_mac_rx_other_pkt(wpanStatsData.mac_rx_other_pkt());
    wpanStats->set_mac_rx_filter_whitelist(wpanStatsData.mac_rx_filter_whitelist());
    wpanStats->set_mac_rx_filter_dest_addr(wpanStatsData.mac_rx_filter_dest_addr());
    wpanStats->set_mac_tx_fail_cca(wpanStatsData.mac_tx_fail_cca());
    wpanStats->set_mac_rx_fail_decrypt(wpanStatsData.mac_rx_fail_decrypt());
    wpanStats->set_mac_rx_fail_no_frame(wpanStatsData.mac_rx_fail_no_frame());
    wpanStats->set_mac_rx_fail_unknown_neighbor(wpanStatsData.mac_rx_fail_unknown_neighbor());
    wpanStats->set_mac_rx_fail_invalid_src_addr(wpanStatsData.mac_rx_fail_invalid_src_addr());
    wpanStats->set_mac_rx_fail_fcs(wpanStatsData.mac_rx_fail_fcs());
    wpanStats->set_mac_rx_fail_other(wpanStatsData.mac_rx_fail_other());
    wpanStats->set_ip_tx_success(wpanStatsData.ip_tx_success());
    wpanStats->set_ip_rx_success(wpanStatsData.ip_rx_success());
    wpanStats->set_ip_tx_failure(wpanStatsData.ip_tx_failure());
    wpanStats->set_ip_rx_failure(wpanStatsData.ip_rx_failure());
    wpanStats->set_node_type(wpanStatsData.node_type());
    wpanStats->set_channel(wpanStatsData.channel());
    wpanStats->set_radio_tx_power(wpanStatsData.radio_tx_power());
    wpanStats->set_mac_cca_fail_rate(wpanStatsData.mac_cca_fail_rate());

    auto wpanTopoFull     = telemetryDataReported.mutable_wpan_topo_full();
    auto wpanTopoFullData = telemetryData.wpan_topo_full();

    wpanTopoFull->set_rloc16(wpanTopoFullData.rloc16());
    wpanTopoFull->set_router_id(wpanTopoFullData.router_id());
    wpanTopoFull->set_leader_router_id(wpanTopoFullData.leader_router_id());

    wpanTopoFull->set_leader_weight(wpanTopoFullData.leader_weight());
    wpanTopoFull->set_leader_local_weight(wpanTopoFullData.leader_local_weight());
    wpanTopoFull->set_preferred_router_id(wpanTopoFullData.preferred_router_id());
    wpanTopoFull->set_child_table_size(wpanTopoFullData.child_table_size());
    wpanTopoFull->set_neighbor_table_size(wpanTopoFullData.neighbor_table_size());
    wpanTopoFull->set_instant_rssi(wpanTopoFullData.instant_rssi());
    wpanTopoFull->set_instant_rssi(wpanTopoFullData.instant_rssi());
    wpanTopoFull->set_has_extended_pan_id(wpanTopoFullData.extended_pan_id() != 0);
    // Note: use leader_router_id instead of leader_rloc16.
    // Note: Network level info (e.g., extended_pan_id, partition_id, is_active_br) is not logged.
    // TODO: populate is_active_srp_server, sum_on_link_prefix_changes.
    // TODO: populate preferred_router_id if needed.

    auto wpanBorderRouter          = telemetryDataReported.mutable_wpan_border_router();
    auto borderRoutingCounters     = wpanBorderRouter->mutable_border_routing_counters();
    auto borderRoutingCoutnersData = telemetryData.wpan_border_router().border_routing_counters();

    borderRoutingCounters->set_ra_rx(borderRoutingCoutnersData.ra_rx());
    borderRoutingCounters->set_ra_tx_success(borderRoutingCoutnersData.ra_tx_success());
    borderRoutingCounters->set_ra_tx_failure(borderRoutingCoutnersData.ra_tx_failure());
    borderRoutingCounters->set_rs_rx(borderRoutingCoutnersData.rs_rx());
    borderRoutingCounters->set_rs_tx_success(borderRoutingCoutnersData.rs_tx_success());
    borderRoutingCounters->set_rs_tx_failure(borderRoutingCoutnersData.rs_tx_failure());

    auto copyPacketsAndBytesFn =
        [](const threadnetwork::TelemetryData_PacketsAndBytes                                     &from,
           android::os::statsd::threadnetwork::ThreadnetworkTelemetryDataReported_PacketsAndBytes *to) {
            to->set_packet_count(from.packet_count());
            to->set_byte_count(from.byte_count());
        };

    copyPacketsAndBytesFn(borderRoutingCoutnersData.inbound_unicast(),
                          borderRoutingCounters->mutable_inbound_unicast());
    copyPacketsAndBytesFn(borderRoutingCoutnersData.inbound_multicast(),
                          borderRoutingCounters->mutable_inbound_multicast());
    copyPacketsAndBytesFn(borderRoutingCoutnersData.outbound_unicast(),
                          borderRoutingCounters->mutable_outbound_unicast());
    copyPacketsAndBytesFn(borderRoutingCoutnersData.outbound_multicast(),
                          borderRoutingCounters->mutable_outbound_multicast());

#if OTBR_ENABLE_NAT64
    auto copyNat64TrafficCountersFn =
        [](const threadnetwork::TelemetryData_Nat64TrafficCounters                                     &from,
           android::os::statsd::threadnetwork::ThreadnetworkTelemetryDataReported_Nat64TrafficCounters *to) {
            to->set_ipv4_to_ipv6_packets(from.ipv4_to_ipv6_packets());
            to->set_ipv4_to_ipv6_bytes(from.ipv4_to_ipv6_bytes());
            to->set_ipv6_to_ipv4_packets(from.ipv6_to_ipv4_packets());
            to->set_ipv6_to_ipv4_bytes(from.ipv6_to_ipv4_bytes());
        };

    copyNat64TrafficCountersFn(borderRoutingCoutnersData.nat64_protocol_counters().tcp(),
                               borderRoutingCounters->mutable_nat64_protocol_counters()->mutable_tcp());
    copyNat64TrafficCountersFn(borderRoutingCoutnersData.nat64_protocol_counters().udp(),
                               borderRoutingCounters->mutable_nat64_protocol_counters()->mutable_tcp());
    copyNat64TrafficCountersFn(borderRoutingCoutnersData.nat64_protocol_counters().icmp(),
                               borderRoutingCounters->mutable_nat64_protocol_counters()->mutable_tcp());

    auto copyNat64PacketCountersFn =
        [](const threadnetwork::TelemetryData_Nat64PacketCounters                                     &from,
           android::os::statsd::threadnetwork::ThreadnetworkTelemetryDataReported_Nat64PacketCounters *to) {
            to->set_ipv4_to_ipv6_packets(from.ipv4_to_ipv6_packets());
            to->set_ipv6_to_ipv4_packets(from.ipv6_to_ipv4_packets());
        };

    copyNat64PacketCountersFn(borderRoutingCoutnersData.nat64_error_counters().unknown(),
                              borderRoutingCounters->mutable_nat64_error_counters()->mutable_unknown());
    copyNat64PacketCountersFn(borderRoutingCoutnersData.nat64_error_counters().illegal_packet(),
                              borderRoutingCounters->mutable_nat64_error_counters()->mutable_illegal_packet());
    copyNat64PacketCountersFn(borderRoutingCoutnersData.nat64_error_counters().unsupported_protocol(),
                              borderRoutingCounters->mutable_nat64_error_counters()->mutable_unsupported_protocol());
    copyNat64PacketCountersFn(borderRoutingCoutnersData.nat64_error_counters().no_mapping(),
                              borderRoutingCounters->mutable_nat64_error_counters()->mutable_no_mapping());
#endif

#if OTBR_ENABLE_SRP_ADVERTISING_PROXY
    auto srpServer     = wpanBorderRouter->mutable_srp_server();
    auto srpServerData = telemetryData.wpan_border_router().srp_server();
    srpServer->set_state(
        static_cast<android::os::statsd::threadnetwork::ThreadnetworkTelemetryDataReported_SrpServerState>(
            srpServerData.state()));
    srpServer->set_port(srpServerData.port());
    srpServer->set_address_mode(
        static_cast<android::os::statsd::threadnetwork::ThreadnetworkTelemetryDataReported_SrpServerAddressMode>(
            srpServerData.address_mode()));
    srpServer->set_address_mode(
        static_cast<android::os::statsd::threadnetwork::ThreadnetworkTelemetryDataReported_SrpServerAddressMode>(
            srpServerData.address_mode()));

    auto copySrpServerRegistrationInfoFn =
        [](const threadnetwork::TelemetryData_SrpServerRegistrationInfo                                     &from,
           android::os::statsd::threadnetwork::ThreadnetworkTelemetryDataReported_SrpServerRegistrationInfo *to) {
            to->set_fresh_count(from.fresh_count());
            to->set_deleted_count(from.deleted_count());
            to->set_lease_time_total_ms(from.lease_time_total_ms());
            to->set_key_lease_time_total_ms(from.key_lease_time_total_ms());
            to->set_remaining_lease_time_total_ms(from.remaining_lease_time_total_ms());
            to->set_remaining_key_lease_time_total_ms(from.remaining_key_lease_time_total_ms());
        };

    copySrpServerRegistrationInfoFn(srpServerData.hosts(), srpServer->mutable_hosts());
    copySrpServerRegistrationInfoFn(srpServerData.services(), srpServer->mutable_services());

    auto srpServerResponseCounters     = srpServer->mutable_response_counters();
    auto srpServerResponseCountersData = srpServerData.response_counters();

    srpServerResponseCounters->set_success_count(srpServerResponseCountersData.success_count());
    srpServerResponseCounters->set_server_failure_count(srpServerResponseCountersData.server_failure_count());
    srpServerResponseCounters->set_format_error_count(srpServerResponseCountersData.format_error_count());
    srpServerResponseCounters->set_name_exists_count(srpServerResponseCountersData.name_exists_count());
    srpServerResponseCounters->set_refused_count(srpServerResponseCountersData.refused_count());
    srpServerResponseCounters->set_other_count(srpServerResponseCountersData.other_count());
#endif

#if OTBR_ENABLE_DNSSD_DISCOVERY_PROXY
    auto dnsServer            = wpanBorderRouter->mutable_dns_server();
    auto dnsServerData        = telemetryData.wpan_border_router().dns_server();
    auto responseCounters     = dnsServer->mutable_response_counters();
    auto responseCountersData = dnsServerData.response_counters();

    responseCounters->set_success_count(responseCountersData.success_count());
    responseCounters->set_server_failure_count(responseCountersData.server_failure_count());
    responseCounters->set_format_error_count(responseCountersData.format_error_count());
    responseCounters->set_name_error_count(responseCountersData.name_error_count());
    responseCounters->set_not_implemented_count(responseCountersData.not_implemented_count());
    responseCounters->set_other_count(responseCountersData.other_count());
    dnsServer->set_resolved_by_local_srp_count(dnsServerData.resolved_by_local_srp_count());
#endif

    // TODO: make the copy only when mPublisher is not nullptr.
    auto mdns     = wpanBorderRouter->mutable_mdns();
    auto mdnsData = telemetryData.wpan_border_router().mdns();
    auto copyMdnsResponseCountersFn =
        [](const threadnetwork::TelemetryData_MdnsResponseCounters                                     &from,
           android::os::statsd::threadnetwork::ThreadnetworkTelemetryDataReported_MdnsResponseCounters *to) {
            to->set_success_count(from.success_count());
            to->set_not_found_count(from.not_found_count());
            to->set_invalid_args_count(from.invalid_args_count());
            to->set_duplicated_count(from.duplicated_count());
            to->set_not_implemented_count(from.not_implemented_count());
            to->set_unknown_error_count(from.unknown_error_count());
            to->set_aborted_count(from.aborted_count());
            to->set_invalid_state_count(from.invalid_state_count());
        };

    copyMdnsResponseCountersFn(mdnsData.host_registration_responses(), mdns->mutable_host_registration_responses());
    copyMdnsResponseCountersFn(mdnsData.service_registration_responses(),
                               mdns->mutable_service_registration_responses());
    copyMdnsResponseCountersFn(mdnsData.host_resolution_responses(), mdns->mutable_host_resolution_responses());
    copyMdnsResponseCountersFn(mdnsData.service_resolution_responses(), mdns->mutable_service_resolution_responses());
    mdns->set_host_registration_ema_latency_ms(mdnsData.host_registration_ema_latency_ms());
    mdns->set_service_registration_ema_latency_ms(mdnsData.service_registration_ema_latency_ms());
    mdns->set_host_resolution_ema_latency_ms(mdnsData.host_resolution_ema_latency_ms());
    mdns->set_service_resolution_ema_latency_ms(mdnsData.service_resolution_ema_latency_ms());

#if OTBR_ENABLE_NAT64
    auto copyBorderRoutingNat64StateFn =
        [](const threadnetwork::TelemetryData_BorderRoutingNat64State                                     &from,
           android::os::statsd::threadnetwork::ThreadnetworkTelemetryDataReported_BorderRoutingNat64State *to) {
            to->set_prefix_manager_state(
                static_cast<android::os::statsd::threadnetwork::ThreadnetworkTelemetryDataReported_Nat64State>(
                    from.prefix_manager_state()));
            to->set_translator_state(
                static_cast<android::os::statsd::threadnetwork::ThreadnetworkTelemetryDataReported_Nat64State>(
                    from.translator_state()));
        };

    copyBorderRoutingNat64StateFn(telemetryData.wpan_border_router().nat64_state(),
                                  wpanBorderRouter->mutable_nat64_state());
#endif

    auto wpanRcp     = telemetryDataReported.mutable_wpan_rcp();
    auto wpanRcpData = telemetryData.wpan_rcp();

    wpanRcp->mutable_rcp_stability_statistics()->set_rcp_timeout_count(
        wpanRcpData.rcp_stability_statistics().rcp_timeout_count());
    wpanRcp->mutable_rcp_stability_statistics()->set_rcp_reset_count(
        wpanRcpData.rcp_stability_statistics().rcp_reset_count());
    wpanRcp->mutable_rcp_stability_statistics()->set_rcp_restoration_count(
        wpanRcpData.rcp_stability_statistics().rcp_restoration_count());
    wpanRcp->mutable_rcp_stability_statistics()->set_spinel_parse_error_count(
        wpanRcpData.rcp_stability_statistics().spinel_parse_error_count());
    wpanRcp->mutable_rcp_stability_statistics()->set_rcp_firmware_update_count(
        wpanRcpData.rcp_stability_statistics().rcp_firmware_update_count());
    wpanRcp->mutable_rcp_stability_statistics()->set_thread_stack_uptime(
        wpanRcpData.rcp_stability_statistics().thread_stack_uptime());
    wpanRcp->mutable_rcp_interface_statistics()->set_rcp_interface_type(
        wpanRcpData.rcp_interface_statistics().rcp_interface_type());
    wpanRcp->mutable_rcp_interface_statistics()->set_transferred_frames_count(
        wpanRcpData.rcp_interface_statistics().transferred_frames_count());
    wpanRcp->mutable_rcp_interface_statistics()->set_transferred_valid_frames_count(
        wpanRcpData.rcp_interface_statistics().transferred_valid_frames_count());
    wpanRcp->mutable_rcp_interface_statistics()->set_transferred_garbage_frames_count(
        wpanRcpData.rcp_interface_statistics().transferred_garbage_frames_count());
    wpanRcp->mutable_rcp_interface_statistics()->set_rx_frames_count(
        wpanRcpData.rcp_interface_statistics().rx_frames_count());
    wpanRcp->mutable_rcp_interface_statistics()->set_rx_bytes_count(
        wpanRcpData.rcp_interface_statistics().rx_bytes_count());
    wpanRcp->mutable_rcp_interface_statistics()->set_tx_frames_count(
        wpanRcpData.rcp_interface_statistics().tx_frames_count());
    wpanRcp->mutable_rcp_interface_statistics()->set_tx_bytes_count(
        wpanRcpData.rcp_interface_statistics().tx_bytes_count());

    auto coexMetrics     = telemetryDataReported.mutable_coex_metrics();
    auto coexMetricsData = telemetryData.coex_metrics();

    coexMetrics->set_count_tx_request(coexMetricsData.count_tx_request());
    coexMetrics->set_count_tx_grant_immediate(coexMetricsData.count_tx_grant_immediate());
    coexMetrics->set_count_tx_grant_wait(coexMetricsData.count_tx_grant_wait());
    coexMetrics->set_count_tx_grant_wait_activated(coexMetricsData.count_tx_grant_wait_activated());
    coexMetrics->set_count_tx_grant_wait_timeout(coexMetricsData.count_tx_grant_wait_timeout());
    coexMetrics->set_count_tx_grant_deactivated_during_request(
        coexMetricsData.count_tx_grant_deactivated_during_request());
    coexMetrics->set_tx_average_request_to_grant_time_us(coexMetricsData.tx_average_request_to_grant_time_us());
    coexMetrics->set_count_rx_request(coexMetricsData.count_rx_request());
    coexMetrics->set_count_rx_grant_immediate(coexMetricsData.count_rx_grant_immediate());
    coexMetrics->set_count_rx_grant_wait(coexMetricsData.count_rx_grant_wait());
    coexMetrics->set_count_rx_grant_wait_activated(coexMetricsData.count_rx_grant_wait_activated());
    coexMetrics->set_count_rx_grant_wait_timeout(coexMetricsData.count_rx_grant_wait_timeout());
    coexMetrics->set_count_rx_grant_deactivated_during_request(
        coexMetricsData.count_rx_grant_deactivated_during_request());
    coexMetrics->set_count_rx_grant_none(coexMetricsData.count_rx_grant_none());
    coexMetrics->set_rx_average_request_to_grant_time_us(coexMetricsData.rx_average_request_to_grant_time_us());
}

void convertTelemetryToAtom(const TelemetryData                &telemetryData,
                                   ThreadnetworkTopoEntryRepeated &topoEntryRepeated)
{
    for (int i = 0; i < telemetryData.topo_entries_size(); i++) {
        auto topoEntriesData = telemetryData.topo_entries(i);
        auto topoEntries = topoEntryRepeated.mutable_topo_entry_repeated()->add_topo_entries();

        // 0~15: uint16_t rloc_16
        // 16~31: uint16_t version Thread version of the neighbor
        uint32_t comboTelemetry1 = 0;
        comboTelemetry1 |= (topoEntriesData.rloc16() & 0x0000FFFF);
        comboTelemetry1 |= ((topoEntriesData.version() & 0x0000FFFF) << 16);
        topoEntries->set_combo_telemetry1(comboTelemetry1);

        // 0~7: uint8_t link_quality_in
        // 8~15: int8_t average_rssi
        // 16~23: int8_t last_rssi
        // 24~31: uint8_t network_data_version
        uint32_t comboTelemetry2 = 0;
        comboTelemetry2 |= (topoEntriesData.link_quality_in() & 0x000000FF);
        comboTelemetry2 |= ((topoEntriesData.average_rssi() & 0x000000FF) << 8);
        comboTelemetry2 |= ((topoEntriesData.last_rssi() & 0x000000FF) << 16);
        comboTelemetry2 |= ((topoEntriesData.network_data_version() & 0x000000FF) << 24);
        topoEntries->set_combo_telemetry2(comboTelemetry2);

        topoEntries->set_age_sec(topoEntriesData.age().seconds());

        // Each bit on the flag represents a bool flag
        // 0: rx_on_when_idle
        // 1: full_function
        // 2: secure_data_request
        // 3: full_network_data
        // 4: is_child
        uint32_t topoEntryFlags = 0;
        topoEntryFlags |= (topoEntriesData.rx_on_when_idle() ? 1 : 0);
        topoEntryFlags |= ((topoEntriesData.full_function() ? 1 : 0) << 1);
        topoEntryFlags |= ((topoEntriesData.secure_data_request() ? 1 : 0) << 2);
        topoEntryFlags |= ((topoEntriesData.full_network_data() ? 1 : 0) << 3);
        topoEntryFlags |= ((topoEntriesData.is_child() ? 1 : 0) << 4);
        topoEntries->set_topo_entry_flags(topoEntryFlags);

        topoEntries->set_link_frame_counter(topoEntriesData.link_frame_counter());
        topoEntries->set_mle_frame_counter(topoEntriesData.mle_frame_counter());
        topoEntries->set_timeout_sec(topoEntriesData.timeout().seconds());

        // 0~15: uint16_t mac_frame_error_rate. Frame error rate (0xffff->100%). Requires error tracking feature.
        // 16~31: uint16_t ip_message_error_rate. (IPv6) msg error rate (0xffff->100%). Requires error tracking feature.
        uint32_t comboTelemetry3 = 0;
        comboTelemetry3 |= (uint32_t)(topoEntriesData.mac_frame_error_rate() * 0xffff);
        comboTelemetry3 |= (uint32_t)(topoEntriesData.ip_message_error_rate() * 0xffff) << 16;
        topoEntries->set_combo_telemetry3(comboTelemetry3);
    }
}

void convertTelemetryToAtom(ThreadnetworkDeviceInfoReported &deviceInfoReported)
{
    deviceInfoReported.set_thread_version(otThreadGetVersion());
    // TODO: populate ot_host_version, ot_rcp_version, thread_daemon_version.
}

int pushAtom(const ThreadnetworkTelemetryDataReported &telemetryDataReported) {
    const std::string        &wpanStats        = telemetryDataReported.wpan_stats().SerializeAsString();
    const std::string        &wpanTopoFull     = telemetryDataReported.wpan_topo_full().SerializeAsString();
    const std::string        &wpanBorderRouter = telemetryDataReported.wpan_border_router().SerializeAsString();
    const std::string        &wpanRcp          = telemetryDataReported.wpan_rcp().SerializeAsString();
    const std::string        &coexMetrics      = telemetryDataReported.coex_metrics().SerializeAsString();
    threadnetwork::BytesField wpanStatsBytesField{wpanStats.c_str(), wpanStats.size()};
    threadnetwork::BytesField wpanTopoFullBytesField{wpanTopoFull.c_str(), wpanTopoFull.size()};
    threadnetwork::BytesField wpanBorderRouterBytesField{wpanBorderRouter.c_str(), wpanBorderRouter.size()};
    threadnetwork::BytesField wpanRcpBytesField{wpanRcp.c_str(), wpanRcp.size()};
    threadnetwork::BytesField coexMetricsBytesField{coexMetrics.c_str(), coexMetrics.size()};
    return threadnetwork::stats_write(threadnetwork::THREADNETWORK_TELEMETRY_DATA_REPORTED, wpanStatsBytesField,
                                      wpanTopoFullBytesField, wpanBorderRouterBytesField, wpanRcpBytesField,
                                      coexMetricsBytesField);
}

int pushAtom(const ThreadnetworkTopoEntryRepeated &topoEntryRepeated)
{
    const std::string        &topoEntryField        = topoEntryRepeated.topo_entry_repeated().SerializeAsString();
    threadnetwork::BytesField topoEntryFieldBytesField{topoEntryField.c_str(), topoEntryField.size()};
    return threadnetwork::stats_write(threadnetwork::THREADNETWORK_TOPO_ENTRY_REPEATED, topoEntryFieldBytesField);
}

int pushAtom(const ThreadnetworkDeviceInfoReported &deviceInfoReported)
{
    const std::string        &otHostVersion        = deviceInfoReported.ot_host_version();
    const std::string        &otRcpVersion     = deviceInfoReported.ot_rcp_version();
    const int32_t        &threadVersion =   deviceInfoReported.thread_version();
    const std::string        &threadDaemonVersion          = deviceInfoReported.thread_daemon_version();
    return threadnetwork::stats_write(threadnetwork::THREADNETWORK_DEVICE_INFO_REPORTED, otHostVersion.c_str(),
                                      otRcpVersion.c_str(), threadVersion, threadDaemonVersion.c_str());
}

void convertAndPushAtoms(const TelemetryData                &telemetryData) {
    ThreadnetworkTelemetryDataReported telemetryDataReported;
    convertTelemetryToAtom(telemetryData, telemetryDataReported);
    pushAtom(telemetryDataReported);

    ThreadnetworkTopoEntryRepeated topoEntryRepeated;
    convertTelemetryToAtom(telemetryData, topoEntryRepeated);
    pushAtom(topoEntryRepeated);

    ThreadnetworkDeviceInfoReported deviceInfoReported;
    convertTelemetryToAtom(deviceInfoReported);
    pushAtom(deviceInfoReported);
}
} // namespace Android
} // namespace otbr
