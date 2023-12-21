/*
 *    Copyright (c) 2024, The OpenThread Authors.
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

#ifndef OTBR_ANDROID_MDNS_PUBLISHER_HPP_
#define OTBR_ANDROID_MDNS_PUBLISHER_HPP_

#include "mdns/mdns.hpp"

#include <aidl/com/android/server/thread/openthread/BnOtDaemon.h>
#include <aidl/com/android/server/thread/openthread/DnsTxtAttribute.h>
#include <aidl/com/android/server/thread/openthread/IMdnsPublisher.h>

namespace otbr {
namespace Android {

using aidl::com::android::server::thread::openthread::DnsTxtAttribute;
using aidl::com::android::server::thread::openthread::IMdnsPublisher;
using aidl::com::android::server::thread::openthread::IMdnsStatusReceiver;

class MdnsPublisher : public Mdns::Publisher
{
public:
    explicit MdnsPublisher(Publisher::StateCallback aCallback)
    {
        mNextListenerId = 0;
        mStateCallback  = std::move(aCallback);
    }

    void SetIMdnsPublisher(std::shared_ptr<IMdnsPublisher> aIMdnsPublisher)
    {
        otbrLogInfo("Set IOtDaemonCallback");
        mNsdPublisher = std::move(aIMdnsPublisher);
        mStateCallback(Mdns::Publisher::State::kReady);
    }

    void ExecuteCallback(int32_t aListenerId, int aError);

    otbrError Start(void) override { return OTBR_ERROR_NONE; }

    void Stop(void) override {}

    bool IsStarted(void) const override { return mNsdPublisher != nullptr; }

    void UnpublishService(const std::string &aName, const std::string &aType, ResultCallback &&aCallback) override;

    void UnpublishHost(const std::string &aName, ResultCallback &&aCallback) override;

    void SubscribeService(const std::string &aType, const std::string &aInstanceName) override;

    void UnsubscribeService(const std::string &aType, const std::string &aInstanceName) override;

    void SubscribeHost(const std::string &aHostName) override;

    void UnsubscribeHost(const std::string &aHostName) override;

protected:
    otbrError PublishServiceImpl(const std::string &aHostName,
                                 const std::string &aName,
                                 const std::string &aType,
                                 const SubTypeList &aSubTypeList,
                                 uint16_t           aPort,
                                 const TxtData     &aTxtData,
                                 ResultCallback   &&aCallback) override;

    otbrError PublishHostImpl(const std::string &aName, const AddressList &aAddresses, ResultCallback &&aCallback);

    void OnServiceResolveFailedImpl(const std::string &aType, const std::string &aInstanceName, int32_t aErrorCode);

    void OnHostResolveFailedImpl(const std::string &aHostName, int32_t aErrorCode);

    otbrError DnsErrorToOtbrError(int32_t aError);

private:
    class NsdServiceRegistration : public ServiceRegistration
    {
    public:
        NsdServiceRegistration(const std::string              &aHostName,
                               const std::string              &aName,
                               const std::string              &aType,
                               const SubTypeList              &aSubTypeList,
                               uint16_t                        aPort,
                               const TxtData                  &aTxtData,
                               ResultCallback                &&aCallback,
                               MdnsPublisher                  *aPublisher,
                               std::shared_ptr<IMdnsPublisher> aIMdnsPublisher,
                               int32_t                         aListenerId)
            : ServiceRegistration(aHostName,
                                  aName,
                                  aType,
                                  aSubTypeList,
                                  aPort,
                                  aTxtData,
                                  std::move(aCallback),
                                  aPublisher)
            , mNsdPublisher(std::move(aIMdnsPublisher))
            , mListenerId(aListenerId)
        {
        }

        ~NsdServiceRegistration(void) override;

    private:
        std::shared_ptr<IMdnsPublisher> mNsdPublisher;

    public:
        const int32_t mListenerId;
    };

    int32_t AllocateListenerId(void);

    StateCallback                                      mStateCallback;
    int32_t                                            mNextListenerId;
    std::shared_ptr<IMdnsPublisher>                    mNsdPublisher = nullptr;
    std::map<int32_t, std::unique_ptr<ResultCallback>> mCallbacks;
};

} // namespace Android
} // namespace otbr

#endif // OTBR_ANDROID_MDNS_PUBLISHER_HPP_
