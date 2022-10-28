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

#ifndef OTBR_ANDROID_BINDER_SERVER_HPP_
#define OTBR_ANDROID_BINDER_SERVER_HPP_

#include <functional>
#include <memory>
#include <vector>

#include <aidl/com/android/server/openthread/BnOtDaemon.h>
#include <openthread/instance.h>
#include <openthread/ip6.h>

#include "common/mainloop.hpp"
#include "ncp/ncp_openthread.hpp"

namespace otbr {
namespace Android {

using Status = ::ndk::ScopedAStatus;
using aidl::com::android::server::openthread::AddressInfo;
using aidl::com::android::server::openthread::BnOtDaemon;
using aidl::com::android::server::openthread::IBorderRouterPlatform;
using aidl::com::android::server::openthread::IEmptyResponseCallback;
using aidl::com::android::server::openthread::IOtbrCallback;

class BinderServer : public BnOtDaemon
{
public:
    static BinderServer &GetInstance();
    ~BinderServer() = default;

    // Disallow copy and assign.
    BinderServer(const BinderServer &) = delete;
    void operator=(const BinderServer &) = delete;

    // TODO(wgtdkp): dump service info for debugging.
    // status_t dump(int fd, const Vector<String16> &args) override;

    void InitOrDie(otbr::Ncp::ControllerOpenThread *aNcp);

    /** Returns the Border Router platform provider. */
    IBorderRouterPlatform &GetBorderRouterPlatform();

private:
    const Milliseconds kAttachTimeout = Milliseconds(10000);

    using DetachCallback = std::function<void()>;

    BinderServer() = default;

    otInstance *GetOtInstance();

    // Implements IOtbr.aidl
    Status      setOtbrCallback(const std::shared_ptr<IOtbrCallback> &aCallback) override;
    Status      getExtendedMacAddress(std::vector<uint8_t> *aExtendedMacAddress) override;
    Status      getStandardVersion(int *aStandardVersion) override;
    bool        isAttached();
    Status      attach(bool                                           aDoForm,
                       const std::vector<uint8_t> &                   aActiveOpDatasetTlvs,
                       const std::shared_ptr<IEmptyResponseCallback> &aCallback) override;
    Status      detach(const std::shared_ptr<IEmptyResponseCallback> &aCallback) override;
    void        detachGracefully(const DetachCallback &aCallback);
    Status      scheduleMigration(const std::vector<uint8_t> &                   aPendingOpDatasetTlvs,
                                  const std::shared_ptr<IEmptyResponseCallback> &aCallback) override;
    static void sendMgmtPendingSetCallback(otError aResult, void *aContext);
    Status      sendPacket(const std::vector<uint8_t> &aPacket) override;
    Status      setBorderRouterConfiguration(const std::string &                            aInfraIfName,
                                             const std::shared_ptr<IBorderRouterPlatform> & aBorderRouterPlatform,
                                             const std::shared_ptr<IEmptyResponseCallback> &aCallback) override;
    Status      notifyInfraIfState(const std::string &aInfraIfName, bool aIsRunning) override;

    void        StateCallback(otChangedFlags aFlags);
    static void AddressCallback(const otIp6AddressInfo *aAddressInfo, bool aIsAdded, void *aContext);
    static void ReceiveCallback(otMessage *aMessage, void *aContext);
    static void DetachGracefullyCallback(void *aContext);

    otbr::Ncp::ControllerOpenThread *      mNcp = nullptr;
    std::shared_ptr<IOtbrCallback>         mCallback;
    std::shared_ptr<IBorderRouterPlatform> mBorderRouterPlatform;
    bool                                   mIsBorderRoutingInitialized = false;

    std::vector<DetachCallback> mOngoingDetachCallbacks;
};

} // namespace Android
} // namespace otbr

#endif // OTBR_ANDROID_BINDER_SERVER_HPP_
