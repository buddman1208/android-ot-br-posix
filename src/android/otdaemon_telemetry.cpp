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

#if OTBR_ENABLE_DNSSD_DISCOVERY_PROXY
#include <openthread/dnssd_server.h>
#endif
#include <openthread/platform/radio.h>
#include <openthread/openthread-system.h>
#include <openthread/thread.h>
#include <openthread/thread_ftd.h>
#if OTBR_ENABLE_SRP_ADVERTISING_PROXY
#include <openthread/srp_server.h>
#endif

#include "common/code_utils.hpp"

namespace otbr {
namespace Android {

static uint32_t TelemetryNodeTypeFromRoleAndLinkMode(const otDeviceRole &aRole, const otLinkModeConfig &aLinkModeCfg)
{
    uint32_t nodeType;

    switch (aRole)
    {
    case OT_DEVICE_ROLE_DISABLED:
        nodeType = 6;
        break;
    case OT_DEVICE_ROLE_DETACHED:
        nodeType = 7;
        break;
    case OT_DEVICE_ROLE_ROUTER:
        nodeType = 1;
        break;
    case OT_DEVICE_ROLE_LEADER:
        nodeType = 0x40;
        break;
    case OT_DEVICE_ROLE_CHILD:
        if (!aLinkModeCfg.mRxOnWhenIdle)
        {
            nodeType = 3;
        }
        else if (!aLinkModeCfg.mDeviceType)
        {
            // If it's not an FTD, return as minimal end device.
            nodeType = 4;
        }
        else
        {
            nodeType = 2;
        }
        break;
    default:
        nodeType = 0;
    }

    return nodeType;
}

#if OTBR_ENABLE_SRP_ADVERTISING_PROXY
uint32_t SrpServerStateFromOtSrpServerState(otSrpServerState srpServerState)
{
    switch (srpServerState)
    {
    case OT_SRP_SERVER_STATE_DISABLED:
        return 1;
    case OT_SRP_SERVER_STATE_RUNNING:
        return 2;
    case OT_SRP_SERVER_STATE_STOPPED:
        return 3;
    default:
        return 0;
    }
}

uint32_t SrpServerAddressModeFromOtSrpServerAddressMode(
    otSrpServerAddressMode srpServerAddressMode)
{
    switch (srpServerAddressMode)
    {
    case OT_SRP_SERVER_ADDRESS_MODE_ANYCAST:
        return 2;
    case OT_SRP_SERVER_ADDRESS_MODE_UNICAST:
        return 1;
    default:
        return 0;
    }
}
#endif // OTBR_ENABLE_SRP_ADVERTISING_PROXY

#if OTBR_ENABLE_NAT64
threadnetwork::TelemetryDataReported_Nat64State Nat64StateFromOtNat64State(otNat64State nat64State)
{
    switch (nat64State)
    {
    case OT_NAT64_STATE_DISABLED:
        return 1;
    case OT_NAT64_STATE_NOT_RUNNING:
        return 2;
    case OT_NAT64_STATE_IDLE:
        return 3;
    case OT_NAT64_STATE_ACTIVE:
        return 4;
    default:
        return 0;
    }
}

void CopyNat64TrafficCounters(const otNat64Counters &from, ThreadnetworkTelemetryDataReported *to)
{
    // TODO: fields are not correctly mapped.
    to->ipv4_to_ipv6_packets = (from.m4To6Packets);
    to->ipv4_to_ipv6_bytes = (from.m4To6Bytes);
    to->ipv6_to_ipv4_packets = (from.m6To4Packets);
    to->ipv6_to_ipv4_bytes = (from.m6To4Bytes);
}
#endif // OTBR_ENABLE_NAT64

void CopyMdnsResponseCounters(const MdnsResponseCounters &from, ThreadnetworkTelemetryDataReported *to)
{
    // TODO: fields are not correctly mapped.
    to->host_registration_success_count = (from.mSuccess);
    to->host_registration_not_found_count = (from.mNotFound);
    to->host_registration_invalid_args_count = (from.mInvalidArgs);
    to->host_registration_duplicated_count = (from.mDuplicated);
    to->host_registration_not_implemented_count = (from.mNotImplemented);
    to->host_registration_unknown_error_count = (from.mUnknownError);
    to->host_registration_aborted_count = (from.mAborted);
    to->host_registration_invalid_state_count = (from.mInvalidState);
}

otError RetrieveAndConvertTelemetries(otInstance *otInstance,
    Mdns::Publisher *aPublisher,
    ThreadnetworkTelemetryDataReported& telemetryDataReported,
    std::vector<ThreadnetworkTopoEntryRepeatedTopoEntry>& topoEntries)
{
    otError                     error = OT_ERROR_NONE;
    std::vector<otNeighborInfo> neighborTable;

    // Begin of WpanStats section.

    {
        otDeviceRole     role  = otThreadGetDeviceRole(otInstance);
        otLinkModeConfig otCfg = otThreadGetLinkMode(otInstance);

        telemetryDataReported.node_type = (TelemetryNodeTypeFromRoleAndLinkMode(role, otCfg));
    }

    telemetryDataReported.channel = (otLinkGetChannel(otInstance));

    {
        uint16_t ccaFailureRate = otLinkGetCcaFailureRate(otInstance);

        telemetryDataReported.mac_cca_fail_rate = (static_cast<float>(ccaFailureRate) / 0xffff);
    }

    {
        int8_t radioTxPower;

        if (otPlatRadioGetTransmitPower(otInstance, &radioTxPower) == OT_ERROR_NONE)
        {
            telemetryDataReported.radio_tx_power = (radioTxPower);
        }
        else
        {
            error = OT_ERROR_FAILED;
        }
    }

    {
        const otMacCounters *linkCounters = otLinkGetCounters(otInstance);

        telemetryDataReported.phy_rx = (linkCounters->mRxTotal);
        telemetryDataReported.phy_tx = (linkCounters->mTxTotal);
        telemetryDataReported.mac_unicast_rx = (linkCounters->mRxUnicast);
        telemetryDataReported.mac_unicast_tx = (linkCounters->mTxUnicast);
        telemetryDataReported.mac_broadcast_rx = (linkCounters->mRxBroadcast);
        telemetryDataReported.mac_broadcast_tx = (linkCounters->mTxBroadcast);
        telemetryDataReported.mac_tx_ack_req = (linkCounters->mTxAckRequested);
        telemetryDataReported.mac_tx_no_ack_req = (linkCounters->mTxNoAckRequested);
        telemetryDataReported.mac_tx_acked = (linkCounters->mTxAcked);
        telemetryDataReported.mac_tx_data = (linkCounters->mTxData);
        telemetryDataReported.mac_tx_data_poll = (linkCounters->mTxDataPoll);
        telemetryDataReported.mac_tx_beacon = (linkCounters->mTxBeacon);
        telemetryDataReported.mac_tx_beacon_req = (linkCounters->mTxBeaconRequest);
        telemetryDataReported.mac_tx_other_pkt = (linkCounters->mTxOther);
        telemetryDataReported.mac_tx_retry = (linkCounters->mTxRetry);
        telemetryDataReported.mac_rx_data = (linkCounters->mRxData);
        telemetryDataReported.mac_rx_data_poll = (linkCounters->mRxDataPoll);
        telemetryDataReported.mac_rx_beacon = (linkCounters->mRxBeacon);
        telemetryDataReported.mac_rx_beacon_req = (linkCounters->mRxBeaconRequest);
        telemetryDataReported.mac_rx_other_pkt = (linkCounters->mRxOther);
        telemetryDataReported.mac_rx_filter_whitelist = (linkCounters->mRxAddressFiltered);
        telemetryDataReported.mac_rx_filter_dest_addr = (linkCounters->mRxDestAddrFiltered);
        telemetryDataReported.mac_tx_fail_cca = (linkCounters->mTxErrCca);
        telemetryDataReported.mac_rx_fail_decrypt = (linkCounters->mRxErrSec);
        telemetryDataReported.mac_rx_fail_no_frame = (linkCounters->mRxErrNoFrame);
        telemetryDataReported.mac_rx_fail_unknown_neighbor = (linkCounters->mRxErrUnknownNeighbor);
        telemetryDataReported.mac_rx_fail_invalid_src_addr = (linkCounters->mRxErrInvalidSrcAddr);
        telemetryDataReported.mac_rx_fail_fcs = (linkCounters->mRxErrFcs);
        telemetryDataReported.mac_rx_fail_other = (linkCounters->mRxErrOther);
    }

    {
        const otIpCounters *ipCounters = otThreadGetIp6Counters(otInstance);

        telemetryDataReported.ip_tx_success = (ipCounters->mTxSuccess);
        telemetryDataReported.ip_rx_success = (ipCounters->mRxSuccess);
        telemetryDataReported.ip_tx_failure = (ipCounters->mTxFailure);
        telemetryDataReported.ip_rx_failure = (ipCounters->mRxFailure);
    }
    // End of WpanStats section.

    {
        // Begin of WpanTopoFull section.
        uint16_t rloc16       = otThreadGetRloc16(otInstance);

        telemetryDataReported.rloc16 = (rloc16);

        {
            otRouterInfo info;

            if (otThreadGetRouterInfo(otInstance, rloc16, &info) == OT_ERROR_NONE)
            {
                telemetryDataReported.router_id = (info.mRouterId);
            }
            else
            {
                error = OT_ERROR_FAILED;
            }
        }

        otNeighborInfoIterator iter = OT_NEIGHBOR_INFO_ITERATOR_INIT;
        otNeighborInfo         neighborInfo;

        while (otThreadGetNextNeighborInfo(otInstance, &iter, &neighborInfo) == OT_ERROR_NONE)
        {
            neighborTable.push_back(neighborInfo);
        }
        telemetryDataReported.neighbor_table_size = (neighborTable.size());

        uint16_t                 childIndex = 0;
        otChildInfo              childInfo;
        std::vector<otChildInfo> childTable;

        while (otThreadGetChildInfoByIndex(otInstance, childIndex, &childInfo) == OT_ERROR_NONE)
        {
            childTable.push_back(childInfo);
            childIndex++;
        }
        telemetryDataReported.child_table_size = (childTable.size());

        {
            struct otLeaderData leaderData;

            if (otThreadGetLeaderData(otInstance, &leaderData) == OT_ERROR_NONE)
            {
                telemetryDataReported.leader_router_id = (leaderData.mLeaderRouterId);
                telemetryDataReported.leader_weight = (leaderData.mWeighting);
                // Do not log network_data_version.
            }
            else
            {
                error = OT_ERROR_FAILED;
            }
        }

        uint8_t weight = otThreadGetLocalLeaderWeight(otInstance);

        telemetryDataReported.leader_local_weight = (weight);

        uint32_t partitionId = otThreadGetPartitionId(otInstance);

        telemetryDataReported.partition_id = (partitionId);

        int8_t rssi = otPlatRadioGetRssi(otInstance);

        telemetryDataReported.instant_rssi = (rssi);

        const otExtendedPanId *extPanId = otThreadGetExtendedPanId(otInstance);
        uint64_t               extPanIdVal;

        extPanIdVal = ConvertOpenThreadUint64(extPanId->m8);
        telemetryDataReported.has_extended_pan_id = (extPanIdVal != 0);
        // Note: Used leader_router_id instead of leader_rloc16.
        // Note: Network level info (e.g., extended_pan_id, partition_id, is_active_br) is not logged.
        // TODO: populate is_active_srp_server, sum_on_link_prefix_changes, preferred_router_id
        // if needed.
        // End of WpanTopoFull section.

        // Begin of TopoEntry section.
        std::map<uint16_t, const otChildInfo *> childMap;

        for (const otChildInfo &childInfo : childTable)
        {
            auto pair = childMap.insert({childInfo.mRloc16, &childInfo});
            if (!pair.second)
            {
                // This shouldn't happen, so log an error. It doesn't matter which
                // duplicate is kept.
                otbrLogErr("Children with duplicate RLOC16 found: 0x%04x", static_cast<int>(childInfo.mRloc16));
            }
        }

        for (const otNeighborInfo &neighborInfo : neighborTable)
        {
            ThreadnetworkTopoEntryRepeatedTopoEntry topoEntry;

            // 0~15: uint16_t rloc_16
            // 16~31: uint16_t version Thread version of the neighbor
            uint32_t comboTelemetry1 = 0;
            comboTelemetry1 |= (((uint32_t)neighborInfo.mRloc16) & 0x0000FFFF);
            comboTelemetry1 |= ((((uint32_t)neighborInfo.mVersion) & 0x0000FFFF) << 16);
            topoEntry.combo_telemetry1 = (comboTelemetry1);

            topoEntry.age_sec = (neighborInfo.mAge);

            // 0~7: uint8_t link_quality_in
            // 8~15: int8_t average_rssi
            // 16~23: int8_t last_rssi
            // 24~31: uint8_t network_data_version
            uint32_t comboTelemetry2 = 0;
            comboTelemetry2 |= (((uint32_t)neighborInfo.mLinkQualityIn) & 0x000000FF);
            comboTelemetry2 |= ((((uint32_t)neighborInfo.mAverageRssi) & 0x000000FF) << 8);
            comboTelemetry2 |= ((((uint32_t)neighborInfo.mLastRssi) & 0x000000FF) << 16);
            // network_data_version is populated in the next section.
            topoEntry.combo_telemetry2 = (comboTelemetry2);

            // Each bit on the flag represents a bool flag
            // 0: rx_on_when_idle
            // 1: full_function
            // 2: secure_data_request
            // 3: full_network_data
            // 4: is_child
            uint32_t topoEntryFlags = 0;
            topoEntryFlags |= (neighborInfo.mRxOnWhenIdle ? 1 : 0);
            topoEntryFlags |= ((neighborInfo.mFullThreadDevice ? 1 : 0) << 1);
            topoEntryFlags |= ((/* secure_data_request */ true ? 1 : 0) << 2);
            topoEntryFlags |= ((neighborInfo.mFullNetworkData ? 1 : 0) << 3);
            topoEntry.topo_entry_flags = (topoEntryFlags);

            topoEntry.link_frame_counter = (neighborInfo.mLinkFrameCounter);
            topoEntry.mle_frame_counter = (neighborInfo.mMleFrameCounter);

            // 0~15: uint16_t mac_frame_error_rate. Frame error rate (0xffff->100%). Requires error tracking feature.
            // 16~31: uint16_t ip_message_error_rate. (IPv6) msg error rate (0xffff->100%). Requires error tracking feature.
            uint32_t comboTelemetry3 = 0;
            comboTelemetry3 |= ((uint32_t)(neighborInfo.mFrameErrorRate) & 0x0000FFFF);
            comboTelemetry3 |= ((((uint32_t)neighborInfo.mMessageErrorRate) & 0x0000FFFF) << 16);
            topoEntry.combo_telemetry3 = (comboTelemetry3);

            if (!neighborInfo.mIsChild)
            {
                topoEntries.push_back(topoEntry);
                continue;
            }

            auto it = childMap.find(neighborInfo.mRloc16);
            if (it == childMap.end())
            {
                topoEntries.push_back(topoEntry);
                otbrLogErr("Neighbor 0x%04x not found in child table", static_cast<int>(neighborInfo.mRloc16));
                continue;
            }
            const otChildInfo *childInfo = it->second;

            comboTelemetry2 |= ((((uint32_t)childInfo->mNetworkDataVersion) & 0x000000FF) << 24);
            topoEntry.combo_telemetry2 = (comboTelemetry2);

            topoEntryFlags |= ((/* is_child */true ? 1 : 0) << 4);
            topoEntry.topo_entry_flags = (topoEntryFlags);

            topoEntry.timeout_sec = (childInfo->mTimeout);
            topoEntries.push_back(topoEntry);
        }
        // End of TopoEntry section.
    }

    {
        // Begin of WpanBorderRouter section.
        // Begin of BorderRoutingCounters section.
        const otBorderRoutingCounters *otBorderRoutingCounters = otIp6GetBorderRoutingCounters(otInstance);

        telemetryDataReported.inbound_unicast_packet_count = (
            otBorderRoutingCounters->mInboundUnicast.mPackets);
        telemetryDataReported.inbound_unicast_byte_count = (
            otBorderRoutingCounters->mInboundUnicast.mBytes);
        telemetryDataReported.inbound_unicast_packet_count = (
            otBorderRoutingCounters->mInboundMulticast.mPackets);
        telemetryDataReported.inbound_unicast_byte_count = (
            otBorderRoutingCounters->mInboundMulticast.mBytes);
        telemetryDataReported.outbound_unicast_packet_count = (
            otBorderRoutingCounters->mOutboundUnicast.mPackets);
        telemetryDataReported.outbound_unicast_byte_count = (
            otBorderRoutingCounters->mOutboundUnicast.mBytes);
        telemetryDataReported.outbound_unicast_packet_count = (
            otBorderRoutingCounters->mOutboundMulticast.mPackets);
        telemetryDataReported.outbound_unicast_byte_count = (
            otBorderRoutingCounters->mOutboundMulticast.mBytes);
        telemetryDataReported.ra_rx = (otBorderRoutingCounters->mRaRx);
        telemetryDataReported.ra_tx_success = (otBorderRoutingCounters->mRaTxSuccess);
        telemetryDataReported.ra_tx_failure = (otBorderRoutingCounters->mRaTxFailure);
        telemetryDataReported.rs_rx = (otBorderRoutingCounters->mRsRx);
        telemetryDataReported.rs_tx_success = (otBorderRoutingCounters->mRsTxSuccess);
        telemetryDataReported.rs_tx_failure = (otBorderRoutingCounters->mRsTxFailure);

#if OTBR_ENABLE_NAT64
        {
            // auto nat64IcmpCounters = borderRoutingCouters->mutable_nat64_protocol_counters()->mutable_icmp();
            // auto nat64UdpCounters  = borderRoutingCouters->mutable_nat64_protocol_counters()->mutable_udp();
            // auto nat64TcpCounters  = borderRoutingCouters->mutable_nat64_protocol_counters()->mutable_tcp();
            otNat64ProtocolCounters otCounters;

            otNat64GetCounters(otInstance, &otCounters);
            telemetryDataReported.ipv4_to_ipv6_packets = (otCounters.mIcmp.m4To6Packets);
            telemetryDataReported.ipv4_to_ipv6_bytes = (otCounters.mIcmp.m4To6Bytes);
            telemetryDataReported.ipv6_to_ipv4_packets = (otCounters.mIcmp.m6To4Packets);
            telemetryDataReported.ipv6_to_ipv4_bytes = (otCounters.mIcmp.m6To4Bytes);
            // TODO: populate required fields.
            // nat64UdpCounters->set_ipv4_to_ipv6_packets(otCounters.mUdp.m4To6Packets);
            // nat64UdpCounters->set_ipv4_to_ipv6_bytes(otCounters.mUdp.m4To6Bytes);
            // nat64UdpCounters->set_ipv6_to_ipv4_packets(otCounters.mUdp.m6To4Packets);
            // nat64UdpCounters->set_ipv6_to_ipv4_bytes(otCounters.mUdp.m6To4Bytes);
            // nat64TcpCounters->set_ipv4_to_ipv6_packets(otCounters.mTcp.m4To6Packets);
            // nat64TcpCounters->set_ipv4_to_ipv6_bytes(otCounters.mTcp.m4To6Bytes);
            // nat64TcpCounters->set_ipv6_to_ipv4_packets(otCounters.mTcp.m6To4Packets);
            // nat64TcpCounters->set_ipv6_to_ipv4_bytes(otCounters.mTcp.m6To4Bytes);
        }

        {
            otNat64ErrorCounters otCounters;
            otNat64GetErrorCounters(otInstance, &otCounters);

            telemetryDataReported.err_ipv4_to_ipv6_packets = (
                otCounters.mCount4To6[OT_NAT64_DROP_REASON_UNKNOWN]);
            telemetryDataReported.err_ipv6_to_ipv4_packets(
                otCounters.mCount6To4[OT_NAT64_DROP_REASON_UNKNOWN]);
            // TODO: populate required fields.
            // errorCounters->mutable_illegal_packet()->set_ipv4_to_ipv6_packets(
            //     otCounters.mCount4To6[OT_NAT64_DROP_REASON_ILLEGAL_PACKET]);
            // errorCounters->mutable_illegal_packet()->set_ipv6_to_ipv4_packets(
            //     otCounters.mCount6To4[OT_NAT64_DROP_REASON_ILLEGAL_PACKET]);
            // errorCounters->mutable_unsupported_protocol()->set_ipv4_to_ipv6_packets(
            //     otCounters.mCount4To6[OT_NAT64_DROP_REASON_UNSUPPORTED_PROTO]);
            // errorCounters->mutable_unsupported_protocol()->set_ipv6_to_ipv4_packets(
            //     otCounters.mCount6To4[OT_NAT64_DROP_REASON_UNSUPPORTED_PROTO]);
            // errorCounters->mutable_no_mapping()->set_ipv4_to_ipv6_packets(
            //     otCounters.mCount4To6[OT_NAT64_DROP_REASON_NO_MAPPING]);
            // errorCounters->mutable_no_mapping()->set_ipv6_to_ipv4_packets(
            //     otCounters.mCount6To4[OT_NAT64_DROP_REASON_NO_MAPPING]);
        }
#endif // OTBR_ENABLE_NAT64
       // End of BorderRoutingCounters section.

#if OTBR_ENABLE_SRP_ADVERTISING_PROXY
        // Begin of SrpServerInfo section.
        {
            // auto                               srpServer = wpanBorderRouter->mutable_srp_server();
            otSrpServerLeaseInfo               leaseInfo;
            const otSrpServerHost             *host             = nullptr;
            const otSrpServerResponseCounters *responseCounters = otSrpServerGetResponseCounters(otInstance);

            telemetryDataReported.state = (SrpServerStateFromOtSrpServerState(otSrpServerGetState(otInstance)));
            telemetryDataReported.port = (otSrpServerGetPort(otInstance));
            telemetryDataReported.address_mode = (
                SrpServerAddressModeFromOtSrpServerAddressMode(otSrpServerGetAddressMode(otInstance)));

            // auto srpServerHosts            = srpServer->mutable_hosts();
            // auto srpServerServices         = srpServer->mutable_services();
            // auto srpServerResponseCounters = srpServer->mutable_response_counters();

            telemetryDataReported.deleted_count = 0;
            telemetryDataReported.fresh_count = 0;
            telemetryDataReported.lease_time_total_ms = 0;
            telemetryDataReported.key_lease_time_total_ms = 0;
            telemetryDataReported.remaining_lease_time_total_ms = 0;
            telemetryDataReported.remaining_key_lease_time_total_ms = 0;
            telemetryDataReported.deleted_count = 0;

            while ((host = otSrpServerGetNextHost(otInstance, host)))
            {
                const otSrpServerService *service = nullptr;

                if (otSrpServerHostIsDeleted(host))
                {
                    telemetryDataReported.deleted_count2 = (telemetryDataReported.deleted_count2 + 1);
                }
                else
                {
                    telemetryDataReported.fresh_count2 = (telemetryDataReported.fresh_count2 + 1);
                    otSrpServerHostGetLeaseInfo(host, &leaseInfo);
                    telemetryDataReported.lease_time_total_ms2 = (telemetryDataReported.lease_time_total_ms2 + leaseInfo.mLease);
                    telemetryDataReported.key_lease_time_total_ms2 = (telemetryDataReported.key_lease_time_total_ms2 +
                                                                leaseInfo.mKeyLease);
                    telemetryDataReported.remaining_lease_time_total_ms2 = (telemetryDataReported.remaining_lease_time_total_ms2 +
                                                                      leaseInfo.mRemainingLease);
                    telemetryDataReported.remaining_key_lease_time_total_ms2 = (
                        telemetryDataReported.remaining_key_lease_time_total_ms2 + leaseInfo.mRemainingKeyLease);
                }

                while ((service = otSrpServerHostGetNextService(host, service)))
                {
                    if (otSrpServerServiceIsDeleted(service))
                    {
                        telemetryDataReported.deleted_count2 = (telemetryDataReported.deleted_count2 + 1);
                    }
                    else
                    {
                        telemetryDataReported.fresh_count2 = telemetryDataReported.fresh_count2 + 1;
                        otSrpServerServiceGetLeaseInfo(service, &leaseInfo);
                        telemetryDataReported.lease_time_total_ms2 = (telemetryDataReported.lease_time_total_ms2 +
                                                                   leaseInfo.mLease);
                        telemetryDataReported.key_lease_time_total_ms2 = (telemetryDataReported.key_lease_time_total_ms2 +
                                                                       leaseInfo.mKeyLease);
                        telemetryDataReported.remaining_lease_time_total_ms2 =
                            telemetryDataReported.remaining_lease_time_total_ms2 + leaseInfo.mRemainingLease;
                        telemetryDataReported.remaining_key_lease_time_total_ms2 =
                            telemetryDataReported.remaining_key_lease_time_total_ms2 + leaseInfo.mRemainingKeyLease;
                    }
                }
            }

            telemetryDataReported.success_count = (responseCounters->mSuccess);
            telemetryDataReported.server_failure_count = (responseCounters->mServerFailure);
            telemetryDataReported.format_error_count = (responseCounters->mFormatError);
            telemetryDataReported.name_exists_count = (responseCounters->mNameExists);
            telemetryDataReported.refused_count = (responseCounters->mRefused);
            telemetryDataReported.other_count = (responseCounters->mOther);
        }
        // End of SrpServerInfo section.
#endif // OTBR_ENABLE_SRP_ADVERTISING_PROXY

#if OTBR_ENABLE_DNSSD_DISCOVERY_PROXY
        // Begin of DnsServerInfo section.
        {
            // auto            dnsServer                 = wpanBorderRouter->mutable_dns_server();
            // auto            dnsServerResponseCounters = dnsServer->mutable_response_counters();
            otDnssdCounters otDnssdCounters           = *otDnssdGetCounters(otInstance);

            telemetryDataReported.dns_response_success_count = (otDnssdCounters.mSuccessResponse);
            telemetryDataReported.dns_response_server_failure_count = (otDnssdCounters.mServerFailureResponse);
            telemetryDataReported.dns_response_format_error_count = (otDnssdCounters.mFormatErrorResponse);
            telemetryDataReported.dns_response_name_error_count = (otDnssdCounters.mNameErrorResponse);
            telemetryDataReported.dns_response_not_implemented_count = (otDnssdCounters.mNotImplementedResponse);
            telemetryDataReported.dns_response_other_count = (otDnssdCounters.mOtherResponse);

            telemetryDataReported.resolved_by_local_srp_count = (otDnssdCounters.mResolvedBySrp);
        }
        // End of DnsServerInfo section.
#endif // OTBR_ENABLE_DNSSD_DISCOVERY_PROXY

        // Start of MdnsInfo section.
        if (aPublisher != nullptr)
        {
            // auto                     mdns     = wpanBorderRouter->mutable_mdns();
            const MdnsTelemetryInfo &mdnsInfo = aPublisher->GetMdnsTelemetryInfo();

            CopyMdnsResponseCounters(mdnsInfo.mHostRegistrations, &telemetryDataReported);
            CopyMdnsResponseCounters(mdnsInfo.mServiceRegistrations, &telemetryDataReported);
            CopyMdnsResponseCounters(mdnsInfo.mHostResolutions, &telemetryDataReported);
            CopyMdnsResponseCounters(mdnsInfo.mServiceResolutions, &telemetryDataReported);

            telemetryDataReported.host_registration_ema_latency_ms = (mdnsInfo.mHostRegistrationEmaLatency);
            telemetryDataReported.service_registration_ema_latency_ms = (mdnsInfo.mServiceRegistrationEmaLatency);
            telemetryDataReported.host_resolution_ema_latency_ms = (mdnsInfo.mHostResolutionEmaLatency);
            telemetryDataReported.service_resolution_ema_latency_ms = (mdnsInfo.mServiceResolutionEmaLatency);
        }
        // End of MdnsInfo section.

#if OTBR_ENABLE_NAT64
        // Start of BorderRoutingNat64State section.
        {
            // auto nat64State = wpanBorderRouter->mutable_nat64_state();

            telemetryDataReported.prefix_manager_state = (Nat64StateFromOtNat64State(otNat64GetPrefixManagerState(otInstance)));
            telemetryDataReported.translator_state = (Nat64StateFromOtNat64State(otNat64GetTranslatorState(otInstance)));
        }
        // End of BorderRoutingNat64State section.

        // Start of Nat64Mapping section.
        {
            otNat64AddressMappingIterator iterator;
            otNat64AddressMapping         otMapping;
            Sha256::Hash                  hash;
            Sha256                        sha256;

            otNat64InitAddressMappingIterator(otInstance, &iterator);
            while (otNat64GetNextAddressMapping(otInstance, &iterator, &otMapping) == OT_ERROR_NONE)
            {
                // auto nat64Mapping         = wpanBorderRouter->add_nat64_mappings();
                // auto nat64MappingCounters = nat64Mapping->mutable_counters();

                telemetryDataReported.mapping_id = (otMapping.mId);
                CopyNat64TrafficCounters(otMapping.mCounters.mTcp, &telemetryDataReported);
                CopyNat64TrafficCounters(otMapping.mCounters.mUdp, &telemetryDataReported);
                CopyNat64TrafficCounters(otMapping.mCounters.mIcmp, &telemetryDataReported);

                // {
                //     uint8_t ipAddrShaInput[OT_IP6_ADDRESS_SIZE + kNat64SourceAddressHashSaltLength];
                //     memcpy(ipAddrShaInput, otMapping.mIp6.mFields.m8, sizeof(otMapping.mIp6.mFields.m8));
                //     memcpy(&ipAddrShaInput[sizeof(otMapping.mIp6.mFields.m8)], mNat64Ipv6AddressSalt,
                //            sizeof(mNat64Ipv6AddressSalt));

                //     sha256.Start();
                //     sha256.Update(ipAddrShaInput, sizeof(ipAddrShaInput));
                //     sha256.Finish(hash);

                //     telemetryDataReported.hashed_ipv6_address = (reinterpret_cast<const char *>(hash.GetBytes()),
                //                                                         sizeof(hash.GetBytes()));
                //     // Remaining time is not included in the telemetry
                // }
            }
        }
        // End of Nat64Mapping section.
#endif // OTBR_ENABLE_NAT64

        // End of WpanBorderRouter section.

        // Start of WpanRcp section.
        {
            // auto                        wpanRcp                = telemetryDataReported.mutable_wpan_rcp();
            const otRadioSpinelMetrics *otRadioSpinelMetrics   = otSysGetRadioSpinelMetrics();
            // auto                        rcpStabilityStatistics = wpanRcp->mutable_rcp_stability_statistics();

            if (otRadioSpinelMetrics != nullptr)
            {
                telemetryDataReported.rcp_timeout_count = (otRadioSpinelMetrics->mRcpTimeoutCount);
                telemetryDataReported.rcp_reset_count = (otRadioSpinelMetrics->mRcpUnexpectedResetCount);
                telemetryDataReported.rcp_restoration_count = (otRadioSpinelMetrics->mRcpRestorationCount);
                telemetryDataReported.spinel_parse_error_count = (otRadioSpinelMetrics->mSpinelParseErrorCount);
            }

            // TODO: provide rcp_firmware_update_count info.
            telemetryDataReported.thread_stack_uptime = (otInstanceGetUptime(otInstance));

            const otRcpInterfaceMetrics *otRcpInterfaceMetrics = otSysGetRcpInterfaceMetrics();

            if (otRcpInterfaceMetrics != nullptr)
            {
                // auto rcpInterfaceStatistics = wpanRcp->mutable_rcp_interface_statistics();

                telemetryDataReported.rcp_interface_type = (otRcpInterfaceMetrics->mRcpInterfaceType);
                telemetryDataReported.transferred_frames_count = (otRcpInterfaceMetrics->mTransferredFrameCount);
                telemetryDataReported.transferred_valid_frames_count = (
                    otRcpInterfaceMetrics->mTransferredValidFrameCount);
                telemetryDataReported.transferred_garbage_frames_count = (
                    otRcpInterfaceMetrics->mTransferredGarbageFrameCount);
                telemetryDataReported.rx_frames_count = (otRcpInterfaceMetrics->mRxFrameCount);
                telemetryDataReported.rx_bytes_count = (otRcpInterfaceMetrics->mRxFrameByteCount);
                telemetryDataReported.tx_frames_count = (otRcpInterfaceMetrics->mTxFrameCount);
                telemetryDataReported.tx_bytes_count = (otRcpInterfaceMetrics->mTxFrameByteCount);
            }
        }
        // End of WpanRcp section.

        // Start of CoexMetrics section.
        {
            // auto               coexMetrics = telemetryDataReported.mutable_coex_metrics();
            otRadioCoexMetrics otRadioCoexMetrics;

            if (otPlatRadioGetCoexMetrics(otInstance, &otRadioCoexMetrics) == OT_ERROR_NONE)
            {
                telemetryDataReported.count_tx_request = (otRadioCoexMetrics.mNumTxRequest);
                telemetryDataReported.count_tx_grant_immediate = (otRadioCoexMetrics.mNumTxGrantImmediate);
                telemetryDataReported.count_tx_grant_wait = (otRadioCoexMetrics.mNumTxGrantWait);
                telemetryDataReported.count_tx_grant_wait_activated = (otRadioCoexMetrics.mNumTxGrantWaitActivated);
                telemetryDataReported.count_tx_grant_wait_timeout = (otRadioCoexMetrics.mNumTxGrantWaitTimeout);
                telemetryDataReported.count_tx_grant_deactivated_during_request = (
                    otRadioCoexMetrics.mNumTxGrantDeactivatedDuringRequest);
                telemetryDataReported.tx_average_request_to_grant_time_us = (otRadioCoexMetrics.mAvgTxRequestToGrantTime);
                telemetryDataReported.count_rx_request = (otRadioCoexMetrics.mNumRxRequest);
                telemetryDataReported.count_rx_grant_immediate = (otRadioCoexMetrics.mNumRxGrantImmediate);
                telemetryDataReported.count_rx_grant_wait = (otRadioCoexMetrics.mNumRxGrantWait);
                telemetryDataReported.count_rx_grant_wait_activated = (otRadioCoexMetrics.mNumRxGrantWaitActivated);
                telemetryDataReported.count_rx_grant_wait_timeout = (otRadioCoexMetrics.mNumRxGrantWaitTimeout);
                telemetryDataReported.count_rx_grant_deactivated_during_request = (
                    otRadioCoexMetrics.mNumRxGrantDeactivatedDuringRequest);
                telemetryDataReported.count_rx_grant_none = (otRadioCoexMetrics.mNumRxGrantNone);
                telemetryDataReported.rx_average_request_to_grant_time_us = (otRadioCoexMetrics.mAvgRxRequestToGrantTime);
            }
            else
            {
                error = OT_ERROR_FAILED;
            }
        }
        // End of CoexMetrics section.
    }

    // deviceInfoReported.thread_version = (otThreadGetVersion());
    // deviceInfoReported.ot_rcp_version = (otGetRadioVersionString(otInstance));
    // TODO: populate ot_host_version, thread_daemon_version.

    return error;
}
} // namespace Android
} // namespace otbr
