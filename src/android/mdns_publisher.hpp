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

#ifndef OTBR_ANDROID_MDNS_PUBLISHER_HPP_
#define OTBR_ANDROID_MDNS_PUBLISHER_HPP_

#include "mdns/mdns.hpp"

#include <aidl/com/android/server/thread/openthread/BnOtDaemon.h>

namespace otbr {
namespace Android {

using aidl::com::android::server::thread::openthread::DnsTxtAttribute;
using aidl::com::android::server::thread::openthread::IOtDaemonCallback;
using aidl::com::android::server::thread::openthread::IOtStatusReceiver;

class MdnsPublisher : public Mdns::Publisher
{
public:
    explicit MdnsPublisher(Publisher::StateCallback aCallback)
    {
        mNextListenerId = 0;
        mStateCallback  = std::move(aCallback);
    }

    void SetIOtDaemonCallback(std::shared_ptr<IOtDaemonCallback> aIOtDaemonCallback)
    {
        otbrLogInfo("Set IOtDaemonCallback");
        mIOtDaemonCallback = std::move(aIOtDaemonCallback);
        mStateCallback(Mdns::Publisher::State::kReady);
    }

    void ExecuteCallback(int32_t aListenerId, int aError)
    {
        auto it = mCallbacks.find(aListenerId);

        VerifyOrExit(it != mCallbacks.end());

        otbrLogInfo("ExecuteCallback %d %d", aListenerId, aError);

        if (!it->second->IsNull())
        {
            std::move (*it->second)(aError ? OTBR_ERROR_MDNS : OTBR_ERROR_NONE);
        }
        mCallbacks.erase(it);

    exit:
        return;
    }

    otbrError Start(void) override { return OTBR_ERROR_NONE; }

    void Stop(void) override {}

    bool IsStarted(void) const override { return true; }

    void UnpublishService(const std::string &aName, const std::string &aType, ResultCallback &&aCallback) override
    {
        (void)aName;
        (void)aType;
        (void)aCallback;

        NsdServiceRegistration *serviceRegistration =
            static_cast<NsdServiceRegistration *>(FindServiceRegistration(aName, aType));
        int32_t unpublishListenerId;

        VerifyOrExit(serviceRegistration != nullptr, std::move(aCallback)(OTBR_ERROR_NONE));

        VerifyOrExit(mIOtDaemonCallback != nullptr, std::move(aCallback)(OTBR_ERROR_MDNS));

        unpublishListenerId = serviceRegistration->mUnpublishListenerId;

        assert(!mCallbacks.count(unpublishListenerId));

        mCallbacks[unpublishListenerId] = MakeUnique<ResultCallback>(std::move(aCallback));

        otbrLogInfo("Unpublish %s %s", aName.c_str(), aType.c_str());

        RemoveServiceRegistration(aName, aType, OTBR_ERROR_NONE);

    exit:
        return;
    }

    virtual void UnpublishHost(const std::string &aName, ResultCallback &&aCallback)
    {
        (void)aName;
        (void)aCallback;
    }

    virtual void UnpublishKey(const std::string &aName, ResultCallback &&aCallback)
    {
        (void)aName;
        (void)aCallback;
    }

    void SubscribeService(const std::string &aType, const std::string &aInstanceName) override
    {
        (void)aType;
        (void)aInstanceName;
    }

    void UnsubscribeService(const std::string &aType, const std::string &aInstanceName)
    {
        (void)aType;
        (void)aInstanceName;
    }

    void SubscribeHost(const std::string &aHostName) { (void)aHostName; }

    void UnsubscribeHost(const std::string &aHostName) { (void)aHostName; }

protected:
    otbrError PublishServiceImpl(const std::string &aHostName,
                                 const std::string &aName,
                                 const std::string &aType,
                                 const SubTypeList &aSubTypeList,
                                 uint16_t           aPort,
                                 const TxtData     &aTxtData,
                                 ResultCallback   &&aCallback)
    {
        (void)aHostName;
        (void)aName;
        (void)aType;
        (void)aSubTypeList;
        (void)aPort;
        (void)aTxtData;
        (void)aCallback;

        int32_t   publishListenerId   = AllocateListenerId();
        int32_t   unpublishListenerId = AllocateListenerId();
        TxtList   txtList;
        otbrError error = OTBR_ERROR_NONE;

        std::vector<DnsTxtAttribute> txtAttributes;

        if (mIOtDaemonCallback == nullptr)
        {
            std::move(aCallback)(OTBR_ERROR_MDNS);
            return OTBR_ERROR_MDNS;
        }

        assert(!mCallbacks.count(publishListenerId));
        //        otbrLogInfo("111111111111 acallback %d", aCallback.IsNull());
        aCallback = HandleDuplicateServiceRegistration(aHostName, aName, aType, aSubTypeList, aPort, aTxtData,
                                                       std::move(aCallback));
        //        otbrLogInfo("111111111111 acallback %d", aCallback.IsNull());
        VerifyOrExit(!aCallback.IsNull(), error = OTBR_ERROR_INVALID_STATE);

        SuccessOrExit(error = DecodeTxtData(txtList, aTxtData.data(), aTxtData.size()));

        mCallbacks[publishListenerId] = MakeUnique<ResultCallback>(std::move(aCallback));
        otbrLogInfo("222222222222");

        for (const auto &txtEntry : txtList)
        {
            txtAttributes.emplace_back(txtEntry.mKey, txtEntry.mValue);
        }
        AddServiceRegistration(MakeUnique<NsdServiceRegistration>(
            aHostName, aName, aType, aSubTypeList, aPort, aTxtData, std::move(aCallback), this, mIOtDaemonCallback,
            publishListenerId, unpublishListenerId));
        otbrLogInfo("333333333333");

        mIOtDaemonCallback->publishService(aHostName, aName, aType, aSubTypeList, aPort, txtAttributes,
                                           publishListenerId, unpublishListenerId);
        otbrLogInfo("444444444444");

    exit:
        return error;
    }

    otbrError PublishHostImpl(const std::string &aName, const AddressList &aAddresses, ResultCallback &&aCallback)
    {
        (void)aName;
        (void)aAddresses;
        (void)aCallback;

        return OTBR_ERROR_NONE;
    }

    otbrError PublishKeyImpl(const std::string &aName, const KeyData &aKeyData, ResultCallback &&aCallback)
    {
        (void)aName;
        (void)aKeyData;
        (void)aCallback;

        return OTBR_ERROR_NONE;
    }

    void OnServiceResolveFailedImpl(const std::string &aType, const std::string &aInstanceName, int32_t aErrorCode)
    {
        (void)aType;
        (void)aInstanceName;
        (void)aErrorCode;
    }

    void OnHostResolveFailedImpl(const std::string &aHostName, int32_t aErrorCode)
    {
        (void)aHostName;
        (void)aErrorCode;
    }

    otbrError DnsErrorToOtbrError(int32_t aError)
    {
        (void)aError;
        return OTBR_ERROR_NONE;
    }

private:
    class NsdServiceRegistration : public ServiceRegistration
    {
    public:
        NsdServiceRegistration(const std::string                 &aHostName,
                               const std::string                 &aName,
                               const std::string                 &aType,
                               const SubTypeList                 &aSubTypeList,
                               uint16_t                           aPort,
                               const TxtData                     &aTxtData,
                               ResultCallback                   &&aCallback,
                               MdnsPublisher                     *aPublisher,
                               std::shared_ptr<IOtDaemonCallback> aIOtDaemonCallback,
                               int32_t                            aPublishListenerId,
                               int32_t                            aUnpublishListenerId)
            : ServiceRegistration(aHostName,
                                  aName,
                                  aType,
                                  aSubTypeList,
                                  aPort,
                                  aTxtData,
                                  std::move(aCallback),
                                  aPublisher)
            , mIOtDaemonCallback(std::move(aIOtDaemonCallback))
            , mPublishListenerId(aPublishListenerId)
            , mUnpublishListenerId(aUnpublishListenerId)
        {
        }

        ~NsdServiceRegistration(void) override
        {
            static_cast<MdnsPublisher *>(mPublisher)->ExecuteCallback(mPublishListenerId, 0);
            mIOtDaemonCallback->unpublishService(mPublishListenerId);
        }

    private:
        std::shared_ptr<IOtDaemonCallback> mIOtDaemonCallback;

    public:
        const int32_t mPublishListenerId;
        const int32_t mUnpublishListenerId;
    };

    int32_t AllocateListenerId(void)
    {
        if (mNextListenerId == std::numeric_limits<int32_t>::max())
        {
            mNextListenerId = 0;
        }
        return mNextListenerId++;
    }

    StateCallback                                      mStateCallback;
    int32_t                                            mNextListenerId;
    std::shared_ptr<IOtDaemonCallback>                 mIOtDaemonCallback = nullptr;
    std::map<int32_t, std::unique_ptr<ResultCallback>> mCallbacks;
};

} // namespace Android
} // namespace otbr

#endif // OTBR_ANDROID_MDNS_PUBLISHER_HPP_
