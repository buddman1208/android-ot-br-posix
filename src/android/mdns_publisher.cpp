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

#include "android/mdns_publisher.hpp"

#include <aidl/com/android/server/thread/openthread/BnMdnsStatusReceiver.h>

namespace otbr {

Mdns::Publisher *Mdns::Publisher::Create(Mdns::Publisher::StateCallback aCallback)
{
    return new Android::MdnsPublisher(std::move(aCallback));
}

namespace Android {

using aidl::com::android::server::thread::openthread::BnMdnsStatusReceiver;
using Status = ::ndk::ScopedAStatus;

otbrError DnsErrorToOtbrErrorImpl(int32_t aError)
{
    return aError == 0 ? OTBR_ERROR_NONE : OTBR_ERROR_MDNS;
}

otbrError MdnsPublisher::DnsErrorToOtbrError(int32_t aError)
{
    return DnsErrorToOtbrErrorImpl(aError);
}

class MdnsStatusReceiver : public BnMdnsStatusReceiver
{
public:
    explicit MdnsStatusReceiver(Mdns::Publisher::ResultCallback aCallback)
        : mCallback(std::move(aCallback))
    {
    }

    Status onSuccess(void) override
    {
        if (!mCallback.IsNull())
        {
            std::move(mCallback)(OTBR_ERROR_NONE);
        }

        return Status::ok();
    }

    Status onError(int aError) override
    {
        if (!mCallback.IsNull())
        {
            std::move(mCallback)(DnsErrorToOtbrErrorImpl(aError));
        }

        return Status::ok();
    }

private:
    Mdns::Publisher::ResultCallback mCallback;
};

std::shared_ptr<MdnsStatusReceiver> CreateReceiver(Mdns::Publisher::ResultCallback aCallback)
{
    return ndk::SharedRefBase::make<MdnsStatusReceiver>(std::move(aCallback));
}

void DieForNotImplemented(const char *aFuncName)
{
    VerifyOrDie(false, (std::string(aFuncName) + " is not implemented").c_str());
}

void MdnsPublisher::UnpublishService(const std::string &aName, const std::string &aType, ResultCallback &&aCallback)
{
    NsdServiceRegistration *serviceRegistration =
        static_cast<NsdServiceRegistration *>(FindServiceRegistration(aName, aType));
    int listenerId;

    VerifyOrExit(serviceRegistration != nullptr, std::move(aCallback)(OTBR_ERROR_NONE));
    VerifyOrExit(mNsdPublisher != nullptr, std::move(aCallback)(OTBR_ERROR_MDNS));

    listenerId = serviceRegistration->mListenerId;

    assert(!mCallbacks.count(listenerId));
    mCallbacks[listenerId] = MakeUnique<ResultCallback>(std::move(aCallback));

    RemoveServiceRegistration(aName, aType, OTBR_ERROR_NONE);

exit:
    return;
}

void MdnsPublisher::UnpublishHost(const std::string &aName, ResultCallback &&aCallback)
{
    OTBR_UNUSED_VARIABLE(aName);
    OTBR_UNUSED_VARIABLE(aCallback);

    DieForNotImplemented(__func__);
}

void MdnsPublisher::SubscribeService(const std::string &aType, const std::string &aInstanceName)
{
    OTBR_UNUSED_VARIABLE(aType);
    OTBR_UNUSED_VARIABLE(aInstanceName);

    DieForNotImplemented(__func__);
}

void MdnsPublisher::UnsubscribeService(const std::string &aType, const std::string &aInstanceName)
{
    OTBR_UNUSED_VARIABLE(aType);
    OTBR_UNUSED_VARIABLE(aInstanceName);

    DieForNotImplemented(__func__);
}

void MdnsPublisher::SubscribeHost(const std::string &aHostName)
{
    OTBR_UNUSED_VARIABLE(aHostName);

    DieForNotImplemented(__func__);
}

void MdnsPublisher::UnsubscribeHost(const std::string &aHostName)
{
    OTBR_UNUSED_VARIABLE(aHostName);

    DieForNotImplemented(__func__);
}

otbrError MdnsPublisher::PublishServiceImpl(const std::string &aHostName,
                                            const std::string &aName,
                                            const std::string &aType,
                                            const SubTypeList &aSubTypeList,
                                            uint16_t           aPort,
                                            const TxtData     &aTxtData,
                                            ResultCallback   &&aCallback)
{
    int32_t   listenerId = AllocateListenerId();
    TxtList   txtList;
    otbrError error = OTBR_ERROR_NONE;

    std::vector<DnsTxtAttribute> txtAttributes;

    if (mNsdPublisher == nullptr)
    {
        std::move(aCallback)(OTBR_ERROR_MDNS);
        return OTBR_ERROR_MDNS;
    }

    assert(!mCallbacks.count(listenerId));
    aCallback = HandleDuplicateServiceRegistration(aHostName, aName, aType, aSubTypeList, aPort, aTxtData,
                                                   std::move(aCallback));
    VerifyOrExit(!aCallback.IsNull(), error = OTBR_ERROR_INVALID_STATE);

    SuccessOrExit(error = DecodeTxtData(txtList, aTxtData.data(), aTxtData.size()));

    for (const auto &txtEntry : txtList)
    {
        txtAttributes.emplace_back(txtEntry.mKey, txtEntry.mValue);
    }
    AddServiceRegistration(MakeUnique<NsdServiceRegistration>(aHostName, aName, aType, aSubTypeList, aPort, aTxtData,
                                                              /* aCallback= */ nullptr, this, mNsdPublisher,
                                                              listenerId));

    mNsdPublisher->registerService(aHostName, aName, aType, aSubTypeList, aPort, txtAttributes,
                                   CreateReceiver(std::move(aCallback)), listenerId);

exit:
    return error;
}

otbrError MdnsPublisher::PublishHostImpl(const std::string &aName,
                                         const AddressList &aAddresses,
                                         ResultCallback   &&aCallback)
{
    OTBR_UNUSED_VARIABLE(aName);
    OTBR_UNUSED_VARIABLE(aAddresses);
    OTBR_UNUSED_VARIABLE(aCallback);

    DieForNotImplemented(__func__);

    return OTBR_ERROR_NOT_IMPLEMENTED;
}

void MdnsPublisher::OnServiceResolveFailedImpl(const std::string &aType,
                                               const std::string &aInstanceName,
                                               int32_t            aErrorCode)
{
    OTBR_UNUSED_VARIABLE(aType);
    OTBR_UNUSED_VARIABLE(aInstanceName);
    OTBR_UNUSED_VARIABLE(aErrorCode);

    DieForNotImplemented(__func__);
}

void MdnsPublisher::OnHostResolveFailedImpl(const std::string &aHostName, int32_t aErrorCode)
{
    OTBR_UNUSED_VARIABLE(aHostName);
    OTBR_UNUSED_VARIABLE(aErrorCode);

    DieForNotImplemented(__func__);
}

int32_t MdnsPublisher::AllocateListenerId(void)
{
    if (mNextListenerId == std::numeric_limits<int32_t>::max())
    {
        mNextListenerId = 0;
    }
    return mNextListenerId++;
}

MdnsPublisher::NsdServiceRegistration::~NsdServiceRegistration(void)
{
    auto receiver = CreateReceiver([](int aError) { OTBR_UNUSED_VARIABLE(aError); });
    mNsdPublisher->unregisterService(receiver, mListenerId);
}

} // namespace Android
} // namespace otbr
