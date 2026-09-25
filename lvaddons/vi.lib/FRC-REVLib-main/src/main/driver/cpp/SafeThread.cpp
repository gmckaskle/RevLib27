/*
 * Copyright (c) 2025 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "SafeThread.h"

#include <atomic>
#include <memory>
#include <utility>

using namespace rev;

// thread start/stop notifications for bindings that need to set up
// per-thread state

static void* DefaultOnThreadStart() { return nullptr; }
static void DefaultOnThreadEnd(void*) {}

using OnThreadStartFn = void* (*)();
using OnThreadEndFn = void (*)(void*);
static std::atomic<int> gSafeThreadRefcount;
static std::atomic<OnThreadStartFn> gOnSafeThreadStart{DefaultOnThreadStart};
static std::atomic<OnThreadEndFn> gOnSafeThreadEnd{DefaultOnThreadEnd};

namespace rev::impl {
void SetSafeThreadNotifiers(OnThreadStartFn OnStart, OnThreadEndFn OnEnd) {
    if (gSafeThreadRefcount != 0) {
        throw std::runtime_error(
            "cannot set notifier while safe threads are running");
    }
    // Note: there's a race here, but if you're not calling this function on
    // the main thread before you start anything else, you're using this
    // function incorrectly
    gOnSafeThreadStart = OnStart ? OnStart : DefaultOnThreadStart;
    gOnSafeThreadEnd = OnEnd ? OnEnd : DefaultOnThreadEnd;
}
}  // namespace rev::impl

void SafeThread::Stop() {
    m_active = false;
    m_cond.notify_all();
}

detail::SafeThreadProxyBase::SafeThreadProxyBase(
    std::shared_ptr<SafeThreadBase> thr)
    : m_thread(std::move(thr)) {
    if (!m_thread) {
        return;
    }
    m_lock = std::unique_lock<std::mutex>(m_thread->m_mutex);
    if (!m_thread->m_active) {
        m_lock.unlock();
        m_thread = nullptr;
        return;
    }
}

detail::SafeThreadOwnerBase::~SafeThreadOwnerBase() {
    if (m_joinAtExit) {
        Join();
    } else {
        Stop();
    }
}

void detail::SafeThreadOwnerBase::Start(std::shared_ptr<SafeThreadBase> thr) {
    std::scoped_lock lock(m_mutex);
    if (auto thr = m_thread.lock()) {
        return;
    }
    m_stdThread = std::thread([=] {
        gSafeThreadRefcount++;
        void* opaque = (gOnSafeThreadStart.load())();
        thr->Main();
        (gOnSafeThreadEnd.load())(opaque);
        gSafeThreadRefcount--;
    });
    thr->m_threadId = m_stdThread.get_id();
    m_thread = thr;
}

void detail::SafeThreadOwnerBase::Stop() {
    std::scoped_lock lock(m_mutex);
    if (auto thr = m_thread.lock()) {
        thr->Stop();
        m_thread.reset();
    }
    if (m_stdThread.joinable()) {
        m_stdThread.detach();
    }
}

void detail::SafeThreadOwnerBase::Join() {
    std::unique_lock lock(m_mutex);
    if (auto thr = m_thread.lock()) {
        auto stdThread = std::move(m_stdThread);
        m_thread.reset();
        lock.unlock();
        thr->Stop();
        stdThread.join();
    } else if (m_stdThread.joinable()) {
        m_stdThread.detach();
    }
}

void detail::swap(SafeThreadOwnerBase& lhs, SafeThreadOwnerBase& rhs) noexcept {
    using std::swap;
    if (&lhs == &rhs) {
        return;
    }
    std::scoped_lock lock(lhs.m_mutex, rhs.m_mutex);
    std::swap(lhs.m_stdThread, rhs.m_stdThread);
    std::swap(lhs.m_thread, rhs.m_thread);
}

detail::SafeThreadOwnerBase::operator bool() const {
    std::scoped_lock lock(m_mutex);
    return !m_thread.expired();
}

std::thread::native_handle_type
detail::SafeThreadOwnerBase::GetNativeThreadHandle() {
    std::scoped_lock lock(m_mutex);
    return m_stdThread.native_handle();
}

std::shared_ptr<SafeThreadBase>
detail::SafeThreadOwnerBase::GetThreadSharedPtr() const {
    std::scoped_lock lock(m_mutex);
    return m_thread.lock();
}
