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

package com.android.server.thread.openthread;

import com.android.server.thread.openthread.DnsTxtAttribute;
import com.android.server.thread.openthread.INsdStatusReceiver;

/**
 * The service which supports mDNS advertising and discovery by {@link NsdManager}.
 */
oneway interface INsdPublisher {
    /**
     * Registers an mDNS service.
     *
     * <p>When the hostname is set to null, register the service with the default host. Otherwise,
     * register the servcie with the specified hostname.
     *
     * <p>The listenerId is an integer ID generated by the caller. It's used by the caller and
     * the NsdPublisher service to uniquely identify a registration.
     *
     * @param hostname the hostname
     * @param name the service instance name
     * @param type the service type
     * @param subtypeList the list of subtypes
     * @param port the port number of the service
     * @param txt the entries of the TXT record
     * @param receiver the receiver of the register callback
     * @param listenerId the listener ID of the 'unregister' opreation
     */
    void registerService(in @nullable String hostname,
                        in String name,
                        in String type,
                        in List<String> subtypeList,
                        int port,
                        in List<DnsTxtAttribute> txt,
                        in INsdStatusReceiver receiver,
                        int listenerId);

    /**
     * Registers an mDNS host.
     *
     * @param name the hostname like "my-host"
     * @param addresses the IPv6 addresses of the host. Each String represents an address.
     * @param receiver the receiver of the register callback
     * @param listenerId the listener ID of the 'unregister' opreation which is used to
     *                             identify the callback of the service unregistration
     */
    void registerHost(in String name,
                      in List<String> addresses,
                      in INsdStatusReceiver receiver,
                      int listenerId);

    /**
     * Unregisters an mDNS service.
     *
     * <p>To unregister a previously registered service/host/key, the caller must pass in the same
     * listener which was used when registering the service/host/key.
     *
     * @param receiver the receiver of the unregister callback
     * @param listenerId the listenerId of the 'unregister' operation
     */
    void unregister(in INsdStatusReceiver receiver, int listenerId);
}