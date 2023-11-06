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

#ifndef OTDAEMON_TELEMETRY_HPP_
#define OTDAEMON_TELEMETRY_HPP_

#include "proto/thread_telemetry.pb.h"
#include "proto/threadnetwork_extension_atoms.pb.h"

namespace otbr {
namespace Android {
using android::os::statsd::threadnetwork::ThreadnetworkTelemetryDataReported;
using android::os::statsd::threadnetwork::ThreadnetworkTopoEntryRepeated;
using android::os::statsd::threadnetwork::ThreadnetworkDeviceInfoReported;
using threadnetwork::TelemetryData;

void convertTelemetryToAtom(const TelemetryData                &telemetryData,
                                   ThreadnetworkTelemetryDataReported &telemetryDataReported);
void convertTelemetryToAtom(const TelemetryData                &telemetryData,
                                   ThreadnetworkTopoEntryRepeated &topoEntryRepeated);
void convertTelemetryToAtom(ThreadnetworkDeviceInfoReported &deviceInfoReported);

int pushAtom(const ThreadnetworkTelemetryDataReported &telemetryDataReported);
int pushAtom(const ThreadnetworkTopoEntryRepeated &topoEntryRepeated);
int pushAtom(const ThreadnetworkDeviceInfoReported &deviceInfoReported);

void convertAndPushAtoms(const TelemetryData                &telemetryData);
} // namespace Android
} // namespace otbr
#endif // OTDAEMON_TELEMETRY_HPP_
