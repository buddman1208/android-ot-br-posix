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

#ifndef OTDAEMON_DUMPER_HPP_
#define OTDAEMON_DUMPER_HPP_

#include <android/binder_status.h>
#include <openthread/instance.h>
#include <openthread/netdata.h>
#include <openthread/srp_server.h>

#include <stdarg.h>

namespace otbr {
namespace Android {
class OtDaemonDumper {
public:
    explicit OtDaemonDumper(otInstance *aInstance, int aFd, const char **aArgs, uint32_t aNumArgs);
    virtual ~OtDaemonDumper(void) = default;

    binder_status_t Dump();

private:
  /**
     * This constant specifies the string size for representing Network Data prefix/route entry flags.
     *
     * BorderRouter (OnMeshPrefix) TLV uses `uint16_t` for its flags and ExternalRoute uses `uint8_t`, though some of
     * the bits are not currently used and reserved for future, so 17 chars string (16 flags plus null char at end of
     * string) covers current and future flags.
     *
     */
    static constexpr uint16_t kFlagsStringSize = 17;

    typedef char FlagsString[kFlagsStringSize]; ///< Flags String type (char array of `kFlagsStringSize`).

    otInstance *GetInstancePtr(void) { return mInstance; }
    void OutputFormatV(const char *aFormat, va_list aArguments);
    void OutputFormat(const char *aFormat, ...);
    void OutputFormat(uint8_t aIndentSize, const char *aFormat, ...);
    void OutputLine(const char *aFormat, ...);
    void OutputLine(uint8_t aIndentSize, const char *aFormat, ...);
    void OutputNewLine(void);
    void OutputSpaces(uint8_t aCount);
    void OutputBytes(const uint8_t *aBytes, uint16_t aLength);
    void OutputBytesLine(const uint8_t *aBytes, uint16_t aLength);

    void OutputIp6Address(const otIp6Address &aAddress);
    void OutputIp6Prefix(const otIp6Prefix &aPrefix);

    // netdata
    otError GetNextPrefix(otNetworkDataIterator *aIterator, otBorderRouterConfig *aConfig, bool aLocal);
    static void PrefixFlagsToString(const otBorderRouterConfig &aConfig, FlagsString &aString);
    static const char *PreferenceToString(signed int aPreference);
    void OutputPrefixes(bool aLocal);
    void OutputPrefix(const otBorderRouterConfig &aConfig);
    otError GetNextRoute(otNetworkDataIterator *aIterator, otExternalRouteConfig *aConfig, bool aLocal);
    void RouteFlagsToString(const otExternalRouteConfig &aConfig, FlagsString &aString);
    void OutputRoutes(bool aLocal);
    void OutputRoute(const otExternalRouteConfig &aConfig);
    otError GetNextService(otNetworkDataIterator *aIterator, otServiceConfig *aConfig, bool aLocal);
    void OutputService(const otServiceConfig &aConfig);
    void OutputServices(bool aLocal);
    void OutputLowpanContexts(bool aLocal);
    otError OutputBinaryNetdata(bool aLocal);

    // srp server
    void OutputHostAddresses(const otSrpServerHost *aHost);
    void OutputDnsTxtData(const uint8_t *aTxtData, uint16_t aTxtDataLength);
    void OutputSrpServerService();
    void OutputSrpServerHost();

    otInstance *mInstance;
    int mFd;
    const int kIndentSize = 4;
};

} // namespace Android
} // namespace otbr

#endif  // OTDAEMON_DUMPER_HPP_
