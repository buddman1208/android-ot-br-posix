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

package com.android.server.thread.openthread.testing;

import android.annotation.NonNull;
import android.annotation.Nullable;
import android.os.IBinder;
import android.os.IBinder.DeathRecipient;
import android.os.ParcelFileDescriptor;
import android.os.RemoteException;

import com.android.server.thread.openthread.BorderRouterConfigurationParcel;
import com.android.server.thread.openthread.IOtDaemon;
import com.android.server.thread.openthread.IOtDaemonCallback;
import com.android.server.thread.openthread.IOtStatusReceiver;

import java.util.NoSuchElementException;

/** A fake implementation of the {@link IOtDaemon} AIDL API for testing. */
public final class FakeOtDaemon extends IOtDaemon.Stub {
    @Nullable private DeathRecipient mDeathRecipient;

    @Nullable private RemoteException mRemoteFailure;

    @Nullable private ParcelFileDescriptor mTunFd;

    @Nullable private IOtDaemonCallback mStateCallback;

    @Nullable private Long mStateCallbackListenerId;

    @Nullable private RemoteException mJoinException;

    @Nullable private byte[] mActiveDataset;

    @Nullable private IOtStatusReceiver mJoinReceiver;

    private int mDeviceRole = 0;

    /**
     * Sets the failure result for all operations. Use {@code null} to clear/reset the failure.
     *
     * <p>Once set, all operations which can throw a {@link RemoteException} will fail immediately
     * with {@code remoteFailure}.
     */
    public void setRemoteFailure(@Nullable RemoteException remoteFailure) {
        mRemoteFailure = remoteFailure;
    }

    @Override
    public IBinder asBinder() {
        return this;
    }

    @Override
    public void linkToDeath(DeathRecipient recipient, int flags) {
        if (mDeathRecipient != null && recipient != null) {
            throw new IllegalStateException("IOtDaemon death recipient is already linked!");
        }

        mDeathRecipient = recipient;
    }

    @Override
    public boolean unlinkToDeath(@NonNull DeathRecipient recipient, int flags) {
        if (mDeathRecipient == null || recipient != mDeathRecipient) {
            throw new NoSuchElementException("recipient is not linked! " + recipient);
        }

        mDeathRecipient = null;
        return true;
    }

    @Override
    public void initialize(ParcelFileDescriptor tunFd) throws RemoteException {
        if (mRemoteFailure != null) {
            throw mRemoteFailure;
        }

        mTunFd = tunFd;
    }

    /**
     * Returns the Thread tunnel interface FD sent to OT daemon or {@code null} if {@link
     * initialize} is never called.
     */
    @Nullable
    public ParcelFileDescriptor getTunFd() {
        return mTunFd;
    }

    @Override
    public void registerStateCallback(IOtDaemonCallback callback, long listenerId)
            throws RemoteException {
        if (mRemoteFailure != null) {
            throw mRemoteFailure;
        }

        mStateCallback = callback;
        mStateCallbackListenerId = listenerId;
    }

    @Nullable
    public IOtDaemonCallback getStateCallback() {
        return mStateCallback;
    }

    @Override
    public void join(byte[] activeDataset, IOtStatusReceiver receiver) throws RemoteException {
        if (mJoinException != null) {
            throw mJoinException;
        }

        mActiveDataset = activeDataset;
        mJoinReceiver = receiver;
    }

    /** Sets the {@link RemoteException} which should be thrown from {@link #join}. */
    public void setJoinException(RemoteException exception) {
        mJoinException = exception;
    }

    @Override
    public void leave(IOtStatusReceiver receiver) throws RemoteException {
        if (mRemoteFailure != null) {
            throw mRemoteFailure;
        }
    }

    @Override
    public void configureBorderRouter(
            BorderRouterConfigurationParcel config, IOtStatusReceiver receiver)
            throws RemoteException {
        if (mRemoteFailure != null) {
            throw mRemoteFailure;
        }
    }

    @Override
    public void scheduleMigration(byte[] pendingDataset, IOtStatusReceiver receiver)
            throws RemoteException {
        if (mRemoteFailure != null) {
            throw mRemoteFailure;
        }
    }
}
