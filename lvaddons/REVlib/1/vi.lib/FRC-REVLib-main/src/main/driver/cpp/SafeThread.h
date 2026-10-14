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

// This is WPI's SafeThread with the removal of SafeEvent and changing
// all of the wpi:: concurrency primitives with the associated std:: ones.

#ifndef FRC_REVLIB_SRC_MAIN_DRIVER_CPP_SAFETHREAD_H_
#define FRC_REVLIB_SRC_MAIN_DRIVER_CPP_SAFETHREAD_H_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

namespace rev {

/**
 * Base class for SafeThreadOwner threads.
 */
class SafeThreadBase {
public:
    virtual ~SafeThreadBase() = default;
    virtual void Main() = 0;
    virtual void Stop() = 0;

    mutable std::mutex m_mutex;
    std::atomic_bool m_active{true};
    std::thread::id m_threadId;
};

class SafeThread : public SafeThreadBase {
public:
    void Stop() override;

    std::condition_variable m_cond;
};

namespace detail {

/**
 * Non-template proxy base class for common proxy code.
 */
class SafeThreadProxyBase {
public:
    explicit SafeThreadProxyBase(std::shared_ptr<SafeThreadBase> thr);
    explicit operator bool() const { return m_thread != nullptr; }
    std::unique_lock<std::mutex>& GetLock() { return m_lock; }

protected:
    std::shared_ptr<SafeThreadBase> m_thread;
    std::unique_lock<std::mutex> m_lock;
};

/**
 * A proxy for SafeThread.
 *
 * Also serves as a scoped lock on SafeThread::m_mutex.
 */
template <typename T>
class SafeThreadProxy : public SafeThreadProxyBase {
public:
    explicit SafeThreadProxy(std::shared_ptr<SafeThreadBase> thr)
        : SafeThreadProxyBase(std::move(thr)) {}
    T& operator*() const { return *static_cast<T*>(m_thread.get()); }
    T* operator->() const { return static_cast<T*>(m_thread.get()); }
};

/**
 * Non-template owner base class for common owner code.
 */
class SafeThreadOwnerBase {
public:
    void Stop();
    void Join();

    SafeThreadOwnerBase() noexcept = default;
    SafeThreadOwnerBase(const SafeThreadOwnerBase&) = delete;
    SafeThreadOwnerBase& operator=(const SafeThreadOwnerBase&) = delete;
    SafeThreadOwnerBase(SafeThreadOwnerBase&& other) noexcept
        : SafeThreadOwnerBase() {
        swap(*this, other);
    }
    SafeThreadOwnerBase& operator=(SafeThreadOwnerBase&& other) noexcept {
        swap(*this, other);
        return *this;
    }
    ~SafeThreadOwnerBase();

    friend void swap(SafeThreadOwnerBase& lhs,
                     SafeThreadOwnerBase& rhs) noexcept;

    explicit operator bool() const;

    std::thread::native_handle_type GetNativeThreadHandle();

    void SetJoinAtExit(bool joinAtExit) { m_joinAtExit = joinAtExit; }

protected:
    void Start(std::shared_ptr<SafeThreadBase> thr);
    std::shared_ptr<SafeThreadBase> GetThreadSharedPtr() const;

private:
    mutable std::mutex m_mutex;
    std::thread m_stdThread;
    std::weak_ptr<SafeThreadBase> m_thread;
    std::atomic_bool m_joinAtExit{true};
};

void swap(SafeThreadOwnerBase& lhs, SafeThreadOwnerBase& rhs) noexcept;

}  // namespace detail

template <typename T>
class SafeThreadOwner : public detail::SafeThreadOwnerBase {
public:
    template <typename... Args>
    void Start(Args&&... args) {
        detail::SafeThreadOwnerBase::Start(
            std::make_shared<T>(std::forward<Args>(args)...));
    }

    using Proxy = typename detail::SafeThreadProxy<T>;
    Proxy GetThread() const {
        return Proxy(detail::SafeThreadOwnerBase::GetThreadSharedPtr());
    }

    std::shared_ptr<T> GetThreadSharedPtr() const {
        return std::static_pointer_cast<T>(
            detail::SafeThreadOwnerBase::GetThreadSharedPtr());
    }
};

}  // namespace rev

#endif  // FRC_REVLIB_SRC_MAIN_DRIVER_CPP_SAFETHREAD_H_
