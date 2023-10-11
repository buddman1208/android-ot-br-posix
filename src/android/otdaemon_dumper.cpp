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
#include "android/otdaemon_dumper.hpp"

#include "ncp/ncp_openthread.hpp"
#include <android-base/file.h>
#include <android-base/stringprintf.h>
#include <openthread/border_router.h>

#include "common/code_utils.hpp"
#include "proto/thread_telemetry.pb.h"

namespace otbr {
namespace Android {

OtDaemonDumper::OtDaemonDumper(otInstance *aInstance, int aFd, const char **aArgs, uint32_t aNumArgs)
    : mInstance(aInstance), mFd(aFd)
{
    OTBR_UNUSED_VARIABLE(aArgs);
    OTBR_UNUSED_VARIABLE(aNumArgs);
}

void OtDaemonDumper::OutputFormatV(const char *aFormat, va_list aArguments) {
  android::base::WriteStringToFd(android::base::StringPrintf(aFormat, aArguments), mFd);
}

void OtDaemonDumper::OutputFormat(const char *aFormat, ...)
{
    va_list args;

    va_start(args, aFormat);
    OutputFormatV(aFormat, args);
    va_end(args);
}

void OtDaemonDumper::OutputFormat(uint8_t aIndentSize, const char *aFormat, ...)
{
    va_list args;

    OutputSpaces(aIndentSize);

    va_start(args, aFormat);
    OutputFormatV(aFormat, args);
    va_end(args);
}

void OtDaemonDumper::OutputLine(const char *aFormat, ...)
{
    va_list args;

    va_start(args, aFormat);
    OutputFormatV(aFormat, args);
    va_end(args);

    OutputNewLine();
}

void OtDaemonDumper::OutputLine(uint8_t aIndentSize, const char *aFormat, ...)
{
    va_list args;

    OutputSpaces(aIndentSize);

    va_start(args, aFormat);
    OutputFormatV(aFormat, args);
    va_end(args);

    OutputNewLine();
}

void OtDaemonDumper::OutputNewLine(void) { OutputFormat("\r\n"); }

void OtDaemonDumper::OutputSpaces(uint8_t aCount) { OutputFormat("%*s", aCount, ""); }

void OtDaemonDumper::OutputBytes(const uint8_t *aBytes, uint16_t aLength)
{
    for (uint16_t i = 0; i < aLength; i++)
    {
        OutputFormat("%02x", aBytes[i]);
    }
}

void OtDaemonDumper::OutputBytesLine(const uint8_t *aBytes, uint16_t aLength)
{
    OutputBytes(aBytes, aLength);
    OutputNewLine();
}

void OtDaemonDumper::OutputIp6Address(const otIp6Address &aAddress)
{
    char string[OT_IP6_ADDRESS_STRING_SIZE];

    otIp6AddressToString(&aAddress, string, sizeof(string));

    return OutputFormat("%s", string);
}

void OtDaemonDumper::OutputIp6Prefix(const otIp6Prefix &aPrefix)
{
    char string[OT_IP6_PREFIX_STRING_SIZE];

    otIp6PrefixToString(&aPrefix, string, sizeof(string));

    OutputFormat("%s", string);
}

inline unsigned long ToUlong(uint32_t aUint32) { return static_cast<unsigned long>(aUint32); }

otError OtDaemonDumper::GetNextPrefix(otNetworkDataIterator *aIterator, otBorderRouterConfig *aConfig, bool aLocal)
{
    otError error;

    if (aLocal)
    {
        error = otBorderRouterGetNextOnMeshPrefix(GetInstancePtr(), aIterator, aConfig);
    }
    else
    {
        error = otNetDataGetNextOnMeshPrefix(GetInstancePtr(), aIterator, aConfig);
    }

    return error;
}

void OtDaemonDumper::PrefixFlagsToString(const otBorderRouterConfig &aConfig, FlagsString &aString)
{
    char *flagsPtr = &aString[0];

    if (aConfig.mPreferred)
    {
        *flagsPtr++ = 'p';
    }

    if (aConfig.mSlaac)
    {
        *flagsPtr++ = 'a';
    }

    if (aConfig.mDhcp)
    {
        *flagsPtr++ = 'd';
    }

    if (aConfig.mConfigure)
    {
        *flagsPtr++ = 'c';
    }

    if (aConfig.mDefaultRoute)
    {
        *flagsPtr++ = 'r';
    }

    if (aConfig.mOnMesh)
    {
        *flagsPtr++ = 'o';
    }

    if (aConfig.mStable)
    {
        *flagsPtr++ = 's';
    }

    if (aConfig.mNdDns)
    {
        *flagsPtr++ = 'n';
    }

    if (aConfig.mDp)
    {
        *flagsPtr++ = 'D';
    }

    *flagsPtr = '\0';
}

const char *OtDaemonDumper::PreferenceToString(signed int aPreference)
{
    const char *str = "";

    switch (aPreference)
    {
    case OT_ROUTE_PREFERENCE_LOW:
        str = "low";
        break;

    case OT_ROUTE_PREFERENCE_MED:
        str = "med";
        break;

    case OT_ROUTE_PREFERENCE_HIGH:
        str = "high";
        break;

    default:
        break;
    }

    return str;
}

void OtDaemonDumper::OutputPrefixes(bool aLocal)
{
    otNetworkDataIterator iterator = OT_NETWORK_DATA_ITERATOR_INIT;
    otBorderRouterConfig  config;

    OutputLine("Prefixes:");

    while (GetNextPrefix(&iterator, &config, aLocal) == OT_ERROR_NONE)
    {
        OutputPrefix(config);
    }
}

void OtDaemonDumper::OutputPrefix(const otBorderRouterConfig &aConfig)
{
    FlagsString flagsString;

    OutputIp6Prefix(aConfig.mPrefix);

    PrefixFlagsToString(aConfig, flagsString);

    if (flagsString[0] != '\0')
    {
        OutputFormat(" %s", flagsString);
    }

    OutputLine(" %s %04x", PreferenceToString(aConfig.mPreference), aConfig.mRloc16);
}

otError OtDaemonDumper::GetNextRoute(otNetworkDataIterator *aIterator, otExternalRouteConfig *aConfig, bool aLocal)
{
    otError error;

    if (aLocal)
    {
        error = otBorderRouterGetNextRoute(GetInstancePtr(), aIterator, aConfig);
    }
    else
    {
        error = otNetDataGetNextRoute(GetInstancePtr(), aIterator, aConfig);
    }

    return error;
}

void OtDaemonDumper::RouteFlagsToString(const otExternalRouteConfig &aConfig, FlagsString &aString)
{
    char *flagsPtr = &aString[0];

    if (aConfig.mStable)
    {
        *flagsPtr++ = 's';
    }

    if (aConfig.mNat64)
    {
        *flagsPtr++ = 'n';
    }

    if (aConfig.mAdvPio)
    {
        *flagsPtr++ = 'a';
    }

    *flagsPtr = '\0';
}

void OtDaemonDumper::OutputRoutes(bool aLocal)
{
    otNetworkDataIterator iterator = OT_NETWORK_DATA_ITERATOR_INIT;
    otExternalRouteConfig config;

    OutputLine("Routes:");

    while (GetNextRoute(&iterator, &config, aLocal) == OT_ERROR_NONE)
    {
        OutputRoute(config);
    }
}

void OtDaemonDumper::OutputRoute(const otExternalRouteConfig &aConfig)
{
    FlagsString flagsString;

    OutputIp6Prefix(aConfig.mPrefix);

    RouteFlagsToString(aConfig, flagsString);

    if (flagsString[0] != '\0')
    {
        OutputFormat(" %s", flagsString);
    }

    OutputLine(" %s %04x", PreferenceToString(aConfig.mPreference), aConfig.mRloc16);
}

otError OtDaemonDumper::GetNextService(otNetworkDataIterator *aIterator, otServiceConfig *aConfig, bool aLocal)
{
    otError error;

    if (aLocal)
    {
#if OPENTHREAD_CONFIG_TMF_NETDATA_SERVICE_ENABLE
        error = otServerGetNextService(GetInstancePtr(), aIterator, aConfig);
#else
        error = OT_ERROR_NOT_FOUND;
#endif
    }
    else
    {
        error = otNetDataGetNextService(GetInstancePtr(), aIterator, aConfig);
    }

    return error;
}

void OtDaemonDumper::OutputService(const otServiceConfig &aConfig)
{
    OutputFormat("%lu ", ToUlong(aConfig.mEnterpriseNumber));
    OutputBytes(aConfig.mServiceData, aConfig.mServiceDataLength);
    OutputFormat(" ");
    OutputBytes(aConfig.mServerConfig.mServerData, aConfig.mServerConfig.mServerDataLength);

    if (aConfig.mServerConfig.mStable)
    {
        OutputFormat(" s");
    }

    OutputLine(" %04x", aConfig.mServerConfig.mRloc16);
}

void OtDaemonDumper::OutputServices(bool aLocal)
{
    otNetworkDataIterator iterator = OT_NETWORK_DATA_ITERATOR_INIT;
    otServiceConfig       config;

    OutputLine("Services:");

    while (GetNextService(&iterator, &config, aLocal) == OT_ERROR_NONE)
    {
        OutputService(config);
    }
}

void OtDaemonDumper::OutputLowpanContexts(bool aLocal)
{
    otNetworkDataIterator iterator = OT_NETWORK_DATA_ITERATOR_INIT;
    otLowpanContextInfo   info;

    VerifyOrExit(!aLocal);

    OutputLine("Contexts:");

    while (otNetDataGetNextLowpanContextInfo(GetInstancePtr(), &iterator, &info) == OT_ERROR_NONE)
    {
        OutputIp6Prefix(info.mPrefix);
        OutputLine(" %u %c", info.mContextId, info.mCompressFlag ? 'c' : '-');
    }

exit:
    return;
}

otError OtDaemonDumper::OutputBinaryNetdata(bool aLocal)
{
    otError error;
    uint8_t data[255];
    uint8_t len = sizeof(data);

    if (aLocal)
    {
        error = otBorderRouterGetNetData(GetInstancePtr(), false, data, &len);
    }
    else
    {
        error = otNetDataGet(GetInstancePtr(), false, data, &len);
    }
    SuccessOrExit(error);

    OutputLine("Binary:");
    OutputBytesLine(data, static_cast<uint8_t>(len));

exit:
    return error;
}

threadnetwork::TelemetryData_SrpServerState SrpServerStateFromOtSrpServerState(otSrpServerState srpServerState)
{
    switch (srpServerState)
    {
    case OT_SRP_SERVER_STATE_DISABLED:
        return threadnetwork::TelemetryData::SRP_SERVER_STATE_DISABLED;
    case OT_SRP_SERVER_STATE_RUNNING:
        return threadnetwork::TelemetryData::SRP_SERVER_STATE_RUNNING;
    case OT_SRP_SERVER_STATE_STOPPED:
        return threadnetwork::TelemetryData::SRP_SERVER_STATE_STOPPED;
    default:
        return threadnetwork::TelemetryData::SRP_SERVER_STATE_UNSPECIFIED;
    }
}

void OtDaemonDumper::OutputHostAddresses(const otSrpServerHost *aHost)
{
    const otIp6Address *addresses;
    uint8_t             addressesNum;

    addresses = otSrpServerHostGetAddresses(aHost, &addressesNum);

    OutputFormat("[");
    for (uint8_t i = 0; i < addressesNum; ++i)
    {
        if (i != 0)
        {
            OutputFormat(", ");
        }

        OutputIp6Address(addresses[i]);
    }
    OutputFormat("]");
}

void OtDaemonDumper::OutputDnsTxtData(const uint8_t *aTxtData, uint16_t aTxtDataLength)
{
    otDnsTxtEntry         entry;
    otDnsTxtEntryIterator iterator;
    bool                  isFirst = true;

    otDnsInitTxtEntryIterator(&iterator, aTxtData, aTxtDataLength);

    OutputFormat("[");

    while (otDnsGetNextTxtEntry(&iterator, &entry) == OT_ERROR_NONE)
    {
        if (!isFirst)
        {
            OutputFormat(", ");
        }

        if (entry.mKey == nullptr)
        {
            // A null `mKey` indicates that the key in the entry is
            // longer than the recommended max key length, so the entry
            // could not be parsed. In this case, the whole entry is
            // returned encoded in `mValue`.

            OutputFormat("[");
            OutputBytes(entry.mValue, entry.mValueLength);
            OutputFormat("]");
        }
        else
        {
            OutputFormat("%s", entry.mKey);

            if (entry.mValue != nullptr)
            {
                OutputFormat("=");
                OutputBytes(entry.mValue, entry.mValueLength);
            }
        }

        isFirst = false;
    }

    OutputFormat("]");
}

void OtDaemonDumper::OutputSrpServerService() {
  const otSrpServerHost *host  = nullptr;

    while ((host = otSrpServerGetNextHost(GetInstancePtr(), host)) != nullptr)
    {
        const otSrpServerService *service = nullptr;

        while ((service = otSrpServerHostGetNextService(host, service)) != nullptr)
        {
            bool                 isDeleted = otSrpServerServiceIsDeleted(service);
            const uint8_t       *txtData;
            uint16_t             txtDataLength;
            bool                 hasSubType = false;
            otSrpServerLeaseInfo leaseInfo;

            OutputLine("%s", otSrpServerServiceGetInstanceName(service));
            OutputLine(kIndentSize, "deleted: %s", isDeleted ? "true" : "false");

            if (isDeleted)
            {
                continue;
            }

            otSrpServerServiceGetLeaseInfo(service, &leaseInfo);

            OutputFormat(kIndentSize, "subtypes: ");

            for (uint16_t index = 0;; index++)
            {
                char        subLabel[OT_DNS_MAX_LABEL_SIZE];
                const char *subTypeName = otSrpServerServiceGetSubTypeServiceNameAt(service, index);

                if (subTypeName == nullptr)
                {
                    break;
                }

                // IgnoreError
                OTBR_UNUSED_VARIABLE(otSrpServerParseSubTypeServiceName(subTypeName, subLabel, sizeof(subLabel)));
                OutputFormat("%s%s", hasSubType ? "," : "", subLabel);
                hasSubType = true;
            }

            OutputLine(hasSubType ? "" : "(null)");

            OutputLine(kIndentSize, "port: %u", otSrpServerServiceGetPort(service));
            OutputLine(kIndentSize, "priority: %u", otSrpServerServiceGetPriority(service));
            OutputLine(kIndentSize, "weight: %u", otSrpServerServiceGetWeight(service));
            OutputLine(kIndentSize, "ttl: %lu", ToUlong(otSrpServerServiceGetTtl(service)));
            OutputLine(kIndentSize, "lease: %lu", ToUlong(leaseInfo.mLease / 1000));
            OutputLine(kIndentSize, "key-lease: %lu", ToUlong(leaseInfo.mKeyLease / 1000));

            txtData = otSrpServerServiceGetTxtData(service, &txtDataLength);
            OutputFormat(kIndentSize, "TXT: ");
            OutputDnsTxtData(txtData, txtDataLength);
            OutputNewLine();

            OutputLine(kIndentSize, "host: %s", otSrpServerHostGetFullName(host));

            OutputFormat(kIndentSize, "addresses: ");
            OutputHostAddresses(host);
            OutputNewLine();
        }
    }
}

void OtDaemonDumper::OutputSrpServerHost()
{
  const otSrpServerHost *host;

  host = nullptr;
  while ((host = otSrpServerGetNextHost(GetInstancePtr(), host)) != nullptr)
  {
      const otIp6Address *addresses;
      uint8_t             addressesNum;
      bool                isDeleted = otSrpServerHostIsDeleted(host);

      OutputLine("%s", otSrpServerHostGetFullName(host));
      OutputLine(kIndentSize, "deleted: %s", isDeleted ? "true" : "false");
      if (isDeleted)
      {
          continue;
      }

      OutputSpaces(kIndentSize);
      OutputFormat("addresses: [");

      addresses = otSrpServerHostGetAddresses(host, &addressesNum);

      for (uint8_t i = 0; i < addressesNum; ++i)
      {
          OutputIp6Address(addresses[i]);
          if (i < addressesNum - 1)
          {
              OutputFormat(", ");
          }
      }

      OutputLine("]");
  }
}

binder_status_t OtDaemonDumper::Dump()
{
    if (GetInstancePtr() == nullptr) {
        OutputLine("Error: OtInstance is null");
        return STATUS_OK;
    }

    OutputLine("[thread version]");
    OutputLine("%u", otThreadGetVersion());

    OutputLine("[netdata]");
    const bool local = false;
    OutputPrefixes(local);
    OutputRoutes(local);
    OutputServices(local);
    OutputLowpanContexts(local);
    OTBR_UNUSED_VARIABLE(OutputBinaryNetdata(local));

    OutputLine("[state (device role)]");
    OutputLine(otThreadDeviceRoleToString(otThreadGetDeviceRole(GetInstancePtr())));

    OutputLine("[srp server state]");
    OutputLine(
        TelemetryData_SrpServerState_Name(SrpServerStateFromOtSrpServerState(otSrpServerGetState(GetInstancePtr()))).c_str());
    OutputLine("[srp server service]");
    OutputSrpServerService();

    fsync(mFd);
    return STATUS_OK;
}
}
}
