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

/**
 * @file
 * Provides Android implementation of the openthread/include/platform/infra_if.h APIs.
 */

#include <vector>

#include <net/if.h>

#include <openthread/platform/infra_if.h>

#include "android/binder_server.hpp"

using otbr::Android::BinderServer;

bool otPlatInfraIfHasAddress(uint32_t aInfraIfIndex, const otIp6Address *aAddress)
{
    (void)aInfraIfIndex;
    (void)aAddress;

    // Returns false so that the Routing Manager won't learn the RA header from
    // the received RA messages. This is desired because the Thread system service
    // will simply discard the header of egress RA messages.
    return false;
}

otError otPlatInfraIfSendIcmp6Nd(uint32_t            aInfraIfIndex,
                                 const otIp6Address *aDestAddress,
                                 const uint8_t *     aBuffer,
                                 uint16_t            aBufferLength)
{
    bool                 result;
    char                 destAddress[OT_IP6_ADDRESS_STRING_SIZE];
    char                 infraIfName[IF_NAMESIZE];
    std::vector<uint8_t> ndMessage;

    if (if_indextoname(aInfraIfIndex, infraIfName) != nullptr)
    {
        otIp6AddressToString(aDestAddress, destAddress, sizeof(destAddress));
        ndMessage.assign(aBuffer, aBuffer + aBufferLength);
        BinderServer::GetInstance().GetBorderRouterPlatform().sendIcmp6Nd(infraIfName, destAddress, ndMessage, &result);
    }
    else
    {
        result = false;
    }

    return result ? OT_ERROR_NONE : OT_ERROR_FAILED;
}
