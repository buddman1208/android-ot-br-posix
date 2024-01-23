/*
 *    copyright (c) 2024, the openthread authors.
 *    all rights reserved.
 *
 *    redistribution and use in source and binary forms, with or without
 *    modification, are permitted provided that the following conditions are met:
 *    1. redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *    3. neither the name of the copyright holder nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *    this software is provided by the copyright holders and contributors "as is"
 *    and any express or implied warranties, including, but not limited to, the
 *    implied warranties of merchantability and fitness for a particular purpose
 *    are disclaimed. in no event shall the copyright holder or contributors be
 *    liable for any direct, indirect, incidental, special, exemplary, or
 *    consequential damages (including, but not limited to, procurement of
 *    substitute goods or services; loss of use, data, or profits; or business
 *    interruption) however caused and on any theory of liability, whether in
 *    contract, strict liability, or tort (including negligence or otherwise)
 *    arising in any way out of the use of this software, even if advised of the
 *    possibility of such damage.
 */

package com.android.server.thread.openthread;

import com.android.server.thread.openthread.NsdResolvedHostInfo;

/**
 * Receives the result of an mDNS host resolution event. The resolution may fail with an error code
 * 'ERROR_*' from {@link DnsResolver}.
 */
oneway interface INsdResolvedHostReceiver {
    void onResolved(in NsdResolvedHostInfo hostInfo);
    void onResolveFailed(in String hostname, int errorCode);
}