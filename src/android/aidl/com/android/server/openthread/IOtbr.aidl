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

package com.android.server.openthread;

import com.android.server.openthread.AddressInfo;
import com.android.server.openthread.IBorderRouterPlatform;
import com.android.server.openthread.IEmptyResponseCallback;
import com.android.server.openthread.IOtbrCallback;

/**
 * The OpenThread Border Router (OTBR) service which provides access to the core Thread stack for
 * system_server.
 */
interface IOtbr {
    /**
     * The Thread tunnel interface name. This interface MUST be created before
     * starting this {@link IOtbr} service.
     */
    const String TUN_IF_NAME = "thread-wpan";

    /**
     * Sets the callback for receiving Thread events. This method should be called before any other
     * APIs.
     */
    // Okay to be blocking API, this doesn't call into OT stack
    void setOtbrCallback(in IOtbrCallback callback);

    /** Returns the Extended MAC Address of this Thread device. */
    // Okay to be blocking, this is already cached in memory
    byte[] getExtendedMacAddress();

    /** Returns the Thread standard version that this Thread device is running. */
    // Okay to be blocking, this is in-memory-only value
    int getStandardVersion();

    /**
     * Attaches this device to the network specified by {@code activeOpDatasetTlvs}.
     *
     * @sa com.android.threadnetwork.ThreadNetworkController#attach
     * @sa com.android.threadnetwork.ThreadNetworkController#attachOrForm
     */
    oneway void attach(
        in boolean doForm, in byte[] activeOpDatasetTlvs, in IEmptyResponseCallback callback);

    /**
     * Detaches from current network.
     *
     * 1. It returns success immediately if this device is already detached or disabled
     * 2. Else if there is already an onging {@code detach} request, no action will be taken but
     *    the {@code callback} will be invoked after the previous request is completed
     * 3. Otherwise, OTBR sends Address Release Notification (i.e. ADDR_REL.ntf) to grcefully
     *    detach from current network and it takes 1 second to finish
     *
     * @sa com.android.threadnetwork.ThreadNetworkController#detach
     */
    oneway void detach(in IEmptyResponseCallback callback);

    /** Migrates to the new network {@code pendingOpDatasetTlvs}.
     *
     * @sa com.android.threadnetwork.ThreadNetworkController#scheduleMigration
     */
    oneway void scheduleMigration(
        in byte[] pendingOpDatasetTlvs, in IEmptyResponseCallback callback);

    /** Sends packet into the Thread network. */
    oneway void sendPacket(in byte[] packet);

    /** Enables the Border Router function. */
    oneway void setBorderRouterConfiguration(
            in String infraIfName,
            in IBorderRouterPlatform platform,
            in IEmptyResponseCallback callback);

    /**
     * Notifies the Thread stack that the infrastructure network interface state has changed.
     */
    oneway void notifyInfraIfState(in String infraIfName, in boolean isRunning);
}
