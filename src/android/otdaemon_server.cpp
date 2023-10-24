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

#define OTBR_LOG_TAG "BINDER"

#include "android/otdaemon_server.hpp"

#include <net/if.h>
#include <string.h>

#include <android-base/file.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <openthread/border_router.h>
#include <openthread/ip6.h>
#include <openthread/openthread-system.h>
#include <openthread/platform/infra_if.h>

#include "statslog_threadnetwork.h"
#include "agent/vendor.hpp"
#include "common/code_utils.hpp"
#include "proto/thread_telemetry.pb.h"
#include "proto/threadnetwork_extension_atoms.pb.h"

#define BYTE_ARR_END(arr) ((arr) + sizeof(arr))

namespace otbr {

namespace vendor {

std::shared_ptr<VendorServer> VendorServer::newInstance(Application &aApplication)
{
    return ndk::SharedRefBase::make<Android::OtDaemonServer>(aApplication.GetNcp());
}

} // namespace vendor

} // namespace otbr

namespace otbr {
namespace Android {
using android::os::statsd::threadnetwork::ThreadnetworkTelemetryDataReported;
using threadnetwork::TelemetryData;

static const char       OTBR_SERVICE_NAME[] = "ot_daemon";
static constexpr size_t kMaxIp6Size         = 1280;

static void PropagateResult(otError                                   aError,
                            const std::string                        &aMessage,
                            const std::shared_ptr<IOtStatusReceiver> &aReceiver)
{
    if (aReceiver != nullptr)
    {
        (aError == OT_ERROR_NONE) ? aReceiver->onSuccess() : aReceiver->onError(aError, aMessage);
    }
}

static Ipv6AddressInfo ConvertToAddressInfo(const otIp6AddressInfo &aAddressInfo)
{
    Ipv6AddressInfo addrInfo;

    addrInfo.address.assign(aAddressInfo.mAddress->mFields.m8, BYTE_ARR_END(aAddressInfo.mAddress->mFields.m8));
    addrInfo.prefixLength = aAddressInfo.mPrefixLength;
    addrInfo.scope        = aAddressInfo.mScope;
    addrInfo.isPreferred  = aAddressInfo.mPreferred;
    return addrInfo;
}

OtDaemonServer::OtDaemonServer(otbr::Ncp::ControllerOpenThread &aNcp)
    : mNcp(aNcp)
{
    mClientDeathRecipient =
        ::ndk::ScopedAIBinder_DeathRecipient(AIBinder_DeathRecipient_new(&OtDaemonServer::BinderDeathCallback));
}

void OtDaemonServer::Init(void)
{
    binder_exception_t exp = AServiceManager_registerLazyService(asBinder().get(), OTBR_SERVICE_NAME);
    SuccessOrDie(exp, "Failed to register OT daemon binder service");

    assert(GetOtInstance() != nullptr);

    mNcp.AddThreadStateChangedCallback([this](otChangedFlags aFlags) { StateCallback(aFlags); });
    otIp6SetAddressCallback(GetOtInstance(), OtDaemonServer::AddressCallback, this);
    otIp6SetReceiveCallback(GetOtInstance(), OtDaemonServer::ReceiveCallback, this);

    otbrLogInfo("tonyzhou@ initialize post task.");
    mTaskRunner.Post(kTelemetryCheckInterval, [this]() { pushTelemetry(); });
}

void OtDaemonServer::BinderDeathCallback(void *aBinderServer)
{
    OtDaemonServer *thisServer = static_cast<OtDaemonServer *>(aBinderServer);

    otbrLogCrit("Client is died, removing callbacks...");
    thisServer->mCallback = nullptr;
    thisServer->mTunFd.set(-1); // the original FD will be closed automatically
}

void OtDaemonServer::StateCallback(otChangedFlags aFlags)
{
    assert(GetOtInstance() != nullptr);

    if (mCallback == nullptr)
    {
        otbrLogWarning("Ignoring OT state changes: callback is not set");
        ExitNow();
    }

    if (aFlags & OT_CHANGED_THREAD_NETIF_STATE)
    {
        mCallback->onInterfaceStateChanged(otIp6IsEnabled(GetOtInstance()));
    }

    if (aFlags & OT_CHANGED_THREAD_ROLE)
    {
        mCallback->onDeviceRoleChanged(otThreadGetDeviceRole(GetOtInstance()));

        if (!isAttached())
        {
            for (const auto &leaveCallback : mOngoingLeaveCallbacks)
            {
                leaveCallback();
            }
            mOngoingLeaveCallbacks.clear();
        }
    }

    if (aFlags & OT_CHANGED_THREAD_PARTITION_ID)
    {
        mCallback->onPartitionIdChanged(otThreadGetPartitionId(GetOtInstance()));
    }

    if (aFlags & OT_CHANGED_ACTIVE_DATASET)
    {
        std::vector<uint8_t>     result;
        otOperationalDatasetTlvs datasetTlvs;
        if (otDatasetGetActiveTlvs(GetOtInstance(), &datasetTlvs) == OT_ERROR_NONE)
        {
            result.assign(datasetTlvs.mTlvs, datasetTlvs.mTlvs + datasetTlvs.mLength);
        }
        mCallback->onActiveOperationalDatasetChanged(result);
    }

    if (aFlags & OT_CHANGED_PENDING_DATASET)
    {
        std::vector<uint8_t>     result;
        otOperationalDatasetTlvs datasetTlvs;
        if (otDatasetGetPendingTlvs(GetOtInstance(), &datasetTlvs) == OT_ERROR_NONE)
        {
            result.assign(datasetTlvs.mTlvs, datasetTlvs.mTlvs + datasetTlvs.mLength);
        }
        mCallback->onPendingOperationalDatasetChanged(result);
    }

exit:
    return;
}

void OtDaemonServer::AddressCallback(const otIp6AddressInfo *aAddressInfo, bool aIsAdded, void *aBinderServer)
{
    OtDaemonServer *thisServer = static_cast<OtDaemonServer *>(aBinderServer);

    if (thisServer->mCallback != nullptr)
    {
        thisServer->mCallback->onAddressChanged(ConvertToAddressInfo(*aAddressInfo), aIsAdded);
    }
    else
    {
        otbrLogWarning("OT daemon callback is not set");
    }
}

void OtDaemonServer::ReceiveCallback(otMessage *aMessage, void *aBinderServer)
{
    static_cast<OtDaemonServer *>(aBinderServer)->ReceiveCallback(aMessage);
}

// FIXME(wgtdkp): We should reuse the same code in openthread/src/posix/platform/netif.cp
// after the refactor there is done: https://github.com/openthread/openthread/pull/9293
void OtDaemonServer::ReceiveCallback(otMessage *aMessage)
{
    char     packet[kMaxIp6Size];
    uint16_t length = otMessageGetLength(aMessage);
    int      fd     = mTunFd.get();

    VerifyOrExit(fd != -1, otbrLogWarning("Ignoring egress packet: invalid tunnel FD"));

    if (otMessageRead(aMessage, 0, packet, sizeof(packet)) != length)
    {
        otbrLogWarning("Failed to read packet from otMessage");
        ExitNow();
    }

    if (write(fd, packet, length) != length)
    {
        otbrLogWarning("Failed to send packet over tunnel interface: %s", strerror(errno));
    }

exit:
    otMessageFree(aMessage);
}

// FIXME(wgtdkp): this doesn't support NAT64, we should use a shared library with ot-posix
// to handle packet translations between the tunnel interface and Thread.
void OtDaemonServer::TransmitCallback(void)
{
    char              packet[kMaxIp6Size];
    ssize_t           length;
    otMessage        *message = nullptr;
    otError           error   = OT_ERROR_NONE;
    otMessageSettings settings;
    int               fd = mTunFd.get();

    assert(GetOtInstance() != nullptr);

    VerifyOrExit(fd != -1);

    length = read(fd, packet, sizeof(packet));

    if (length == -1)
    {
        otbrLogWarning("Failed to read packet from tunnel interface: %s", strerror(errno));
        ExitNow();
    }
    else if (length == 0)
    {
        otbrLogWarning("Unexpected EOF on the tunnel FD");
        ExitNow();
    }

    VerifyOrExit(GetOtInstance() != nullptr, otbrLogWarning("Ignoring tunnel packet: OT is not initialized"));

    settings.mLinkSecurityEnabled = (otThreadGetDeviceRole(GetOtInstance()) != OT_DEVICE_ROLE_DISABLED);
    settings.mPriority            = OT_MESSAGE_PRIORITY_LOW;

    message = otIp6NewMessage(GetOtInstance(), &settings);
    VerifyOrExit(message != nullptr, error = OT_ERROR_NO_BUFS);

    SuccessOrExit(error = otMessageAppend(message, packet, length));

    error   = otIp6Send(GetOtInstance(), message);
    message = nullptr;

exit:
    if (message != nullptr)
    {
        otMessageFree(message);
    }

    if (error != OT_ERROR_NONE)
    {
        if (error == OT_ERROR_DROP)
        {
            otbrLogInfo("Dropped tunnel packet (length=%d)", length);
        }
        else
        {
            otbrLogWarning("Failed to transmit tunnel packet: %s", otThreadErrorToString(error));
        }
    }
}

otInstance *OtDaemonServer::GetOtInstance()
{
    return mNcp.GetInstance();
}

void OtDaemonServer::Update(MainloopContext &aMainloop)
{
    int fd = mTunFd.get();

    if (fd != -1)
    {
        FD_SET(fd, &aMainloop.mReadFdSet);
        aMainloop.mMaxFd = std::max(aMainloop.mMaxFd, fd);
    }
}

void OtDaemonServer::Process(const MainloopContext &aMainloop)
{
    int fd = mTunFd.get();

    if (fd != -1 && FD_ISSET(fd, &aMainloop.mReadFdSet))
    {
        TransmitCallback();
    }
}

Status OtDaemonServer::initialize(const ScopedFileDescriptor               &aTunFd,
                                  const std::shared_ptr<IOtDaemonCallback> &aCallback)
{
    otbrLogDebug("OT daemon is initialized by the binder client (tunFd=%d)", aTunFd.get());

    mTunFd    = aTunFd.dup();
    mCallback = aCallback;
    if (mCallback != nullptr)
    {
        AIBinder_linkToDeath(mCallback->asBinder().get(), mClientDeathRecipient.get(), this);
    }

    return Status::ok();
}

Status OtDaemonServer::join(bool                                      aDoForm,
                            const std::vector<uint8_t>               &aActiveOpDatasetTlvs,
                            const std::shared_ptr<IOtStatusReceiver> &aReceiver)
{
    otError                  error = OT_ERROR_NONE;
    std::string              message;
    otOperationalDatasetTlvs datasetTlvs;

    // TODO(b/273160198): check how we can implement join as a child
    (void)aDoForm;

    otbrLogInfo("Start joining...");

    VerifyOrExit(GetOtInstance() != nullptr, error = OT_ERROR_INVALID_STATE, message = "OT is not initialized");
    VerifyOrExit(!isAttached(), error = OT_ERROR_INVALID_STATE, message = "Cannot join when already attached");

    std::copy(aActiveOpDatasetTlvs.begin(), aActiveOpDatasetTlvs.end(), datasetTlvs.mTlvs);
    datasetTlvs.mLength = aActiveOpDatasetTlvs.size();
    SuccessOrExit(error   = otDatasetSetActiveTlvs(GetOtInstance(), &datasetTlvs),
                  message = "Failed to set Active Operational Dataset");

    // Shouldn't we have an equivalent `otThreadAttach` method vs `otThreadDetachGracefully`?
    SuccessOrExit(error = otIp6SetEnabled(GetOtInstance(), true), message = "Failed to bring up Thread interface");
    SuccessOrExit(error = otThreadSetEnabled(GetOtInstance(), true), message = "Failed to bring up Thread stack");

exit:
    PropagateResult(error, message, aReceiver);
    return Status::ok();
}

void OtDaemonServer::detachGracefully(const DetachCallback &aCallback)
{
    otError error;

    mOngoingLeaveCallbacks.push_back(aCallback);

    // The callback is already guarded by a timer inside OT, so the client side shouldn't need to
    // add a callback again.
    error = otThreadDetachGracefully(GetOtInstance(), OtDaemonServer::DetachGracefullyCallback, this);
    if (error == OT_ERROR_BUSY)
    {
        // There is already an ongoing detach request, do nothing but enqueue the callback
        otbrLogDebug("Reuse existing detach request");
        ExitNow(error = OT_ERROR_NONE);
    }

exit:;
}

Status OtDaemonServer::leave(const std::shared_ptr<IOtStatusReceiver> &aReceiver)
{
    if (GetOtInstance() == nullptr)
    {
        PropagateResult(OT_ERROR_INVALID_STATE, "OT is not initialized", aReceiver);
    }
    else
    {
        detachGracefully([=]() {
            if (aReceiver != nullptr)
            {
                aReceiver->onSuccess();
            }
        });
    }

    return Status::ok();
}

void OtDaemonServer::DetachGracefullyCallback(void *aBinderServer)
{
    OtDaemonServer *thisServer = static_cast<OtDaemonServer *>(aBinderServer);

    for (auto callback : thisServer->mOngoingLeaveCallbacks)
    {
        callback();
    }
    thisServer->mOngoingLeaveCallbacks.clear();
}

bool OtDaemonServer::isAttached()
{
    otDeviceRole role = otThreadGetDeviceRole(GetOtInstance());

    return role == OT_DEVICE_ROLE_CHILD || role == OT_DEVICE_ROLE_ROUTER || role == OT_DEVICE_ROLE_LEADER;
}

Status OtDaemonServer::scheduleMigration(const std::vector<uint8_t>               &aPendingOpDatasetTlvs,
                                         const std::shared_ptr<IOtStatusReceiver> &aReceiver)
{
    otError              error = OT_ERROR_NONE;
    std::string          message;
    otOperationalDataset emptyDataset;

    if (GetOtInstance() == nullptr)
    {
        message = "OT is not initialized";
        ExitNow(error = OT_ERROR_INVALID_STATE);
    }

    if (!isAttached())
    {
        message = "Cannot schedule migration when this device is detached";
        ExitNow(error = OT_ERROR_INVALID_STATE);
    }

    error = otDatasetSendMgmtPendingSet(GetOtInstance(), &emptyDataset, aPendingOpDatasetTlvs.data(),
                                        aPendingOpDatasetTlvs.size(), sendMgmtPendingSetCallback,
                                        /* aBinderServer= */ this);
    if (error != OT_ERROR_NONE)
    {
        message = "Failed to send MGMT_PENDING_SET.req";
    }

exit:
    PropagateResult(error, message, aReceiver);
    return Status::ok();
}

void OtDaemonServer::sendMgmtPendingSetCallback(otError aResult, void *aBinderServer)
{
    (void)aBinderServer;

    otbrLogDebug("otDatasetSendMgmtPendingSet callback: %d", aResult);
}

Status OtDaemonServer::getExtendedMacAddress(std::vector<uint8_t> *aExtendedMacAddress)
{
    Status              status = Status::ok();
    const otExtAddress *extAddress;

    if (aExtendedMacAddress == nullptr)
    {
        status =
            Status::fromServiceSpecificErrorWithMessage(OT_ERROR_INVALID_ARGS, "aExtendedMacAddress can not be null");
        ExitNow();
    }

    if (GetOtInstance() == nullptr)
    {
        status = Status::fromServiceSpecificErrorWithMessage(OT_ERROR_INVALID_STATE, "OT is not initialized");
        ExitNow();
    }

    extAddress = otLinkGetExtendedAddress(GetOtInstance());
    aExtendedMacAddress->assign(extAddress->m8, extAddress->m8 + sizeof(extAddress->m8));

exit:
    return status;
}

Status OtDaemonServer::getThreadVersion(int *aThreadVersion)
{
    Status status = Status::ok();

    if (aThreadVersion == nullptr)
    {
        status = Status::fromServiceSpecificErrorWithMessage(OT_ERROR_INVALID_ARGS, "aThreadVersion can not be null");
        ExitNow();
    }

    *aThreadVersion = otThreadGetVersion();

exit:
    return status;
}

binder_status_t OtDaemonServer::dump(int aFd, const char **aArgs, uint32_t aNumArgs)
{
    OT_UNUSED_VARIABLE(aArgs);
    OT_UNUSED_VARIABLE(aNumArgs);

    // TODO: Use ::android::base::WriteStringToFd to dump infomration.
    fsync(aFd);

    return STATUS_OK;
}

inline void convertTelemetryToAtom(const TelemetryData                &telemetryData,
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
    // TODO: populate leader_rloc16, preferred_router_id.
    wpanTopoFull->set_leader_weight(wpanTopoFullData.leader_weight());
    wpanTopoFull->set_leader_local_weight(wpanTopoFullData.leader_local_weight());
    wpanTopoFull->set_preferred_router_id(wpanTopoFullData.preferred_router_id());
    wpanTopoFull->set_partition_id(wpanTopoFullData.partition_id());
    wpanTopoFull->set_child_table_size(wpanTopoFullData.child_table_size());
    wpanTopoFull->set_neighbor_table_size(wpanTopoFullData.neighbor_table_size());
    wpanTopoFull->set_instant_rssi(wpanTopoFullData.instant_rssi());
    wpanTopoFull->set_instant_rssi(wpanTopoFullData.instant_rssi());
    wpanTopoFull->set_has_extended_pan_id(wpanTopoFullData.extended_pan_id() != 0);
    // TODO: populate is_active_br when thread network level data can be processed.
    // TODO: populate is_active_srp_server, sum_on_link_prefix_changes.

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

Status OtDaemonServer::pushTelemetry()
{
    Status status = Status::ok();
    auto   now    = Clock::now();

    if (now - lastTelemetryDataUpload < kTelemetryDataUploadInterval)
    {
        Milliseconds postDelay =
            kTelemetryDataUploadInterval - std::chrono::duration_cast<Milliseconds>(now - lastTelemetryDataUpload);

        mTaskRunner.Post(postDelay, [this]() { pushTelemetry(); });
        return status;
    }

    auto                               threadHelper = mNcp.GetThreadHelper();
    TelemetryData                      telemetryData;
    ThreadnetworkTelemetryDataReported telemetryDataReported;
    otError                            error = OT_ERROR_NONE;

    error = threadHelper->RetrieveTelemetryData(nullptr, telemetryData);
    if (error != OT_ERROR_NONE)
    {
        otbrLogWarning("Some metrics were not populated in RetrieveTelemetryData");
    }
    convertTelemetryToAtom(telemetryData, telemetryDataReported);

    {
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
        threadnetwork::stats_write(threadnetwork::THREADNETWORK_TELEMETRY_DATA_REPORTED, wpanStatsBytesField,
                                   wpanTopoFullBytesField, wpanBorderRouterBytesField, wpanRcpBytesField,
                                   coexMetricsBytesField);
        lastTelemetryDataUpload = now;
        mTaskRunner.Post(kTelemetryDataUploadInterval, [this]() { pushTelemetry(); });
    }

    return status;
}
} // namespace Android
} // namespace otbr
