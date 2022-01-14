/*
 * (c) 2021 Copyright, Real-Time Innovations, Inc. (RTI) All rights reserved.
 *
 * RTI grants Licensee a license to use, modify, compile, and create derivative
 * works of the software solely for use with RTI Connext DDS.  Licensee may
 * redistribute copies of the software provided that all such copies are
 * subject to this license. The software is provided "as is", with no warranty
 * of any type, including any warranty for fitness for any purpose. RTI is
 * under no obligation to maintain or support the software.  RTI shall not be
 * liable for any incidental or consequential damages arising out of the use or
 * inability to use the software.
 */

#ifndef SAFE_QUEUE_H
#define SAFE_QUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>
//#include <stdexcept>
#include <chrono>

namespace SafeQueue {

    // enum to return to check whether could push or pop or queue was closed
    enum QueueResult { OK, TIMEOUT = 1, CLOSED = -1 };

    // a thread-safe queue with a maximal size based on std::list<>::splice()
    template <typename T>
    class SafeQueue
    {
    private:
        enum class State { OPEN, CLOSED = -1 }; // check whether the queue is running or closed
        State state_; // The current state
        std::condition_variable cond_; // The condition variable to use for popping
        std::mutex mutex_; // The mutex for locking the queue
        std::queue<T> queue_; // The queue that holds the data

    public:
        // create and initialize the Q with maximum size
        explicit SafeQueue() : state_(State::OPEN), cond_(), mutex_(), queue_() {}

        // Push data to Q, if queue is full then  blocks
        void push(T const& data) {
            // Push with lock
            std::unique_lock<std::mutex> lk(mutex_);
            // Check whether the Q is closed or not and pushing is allowed
            if (State::CLOSED == state_)
                throw std::runtime_error("The queue is closed and trying to push.");
            // Pushing to Q
            queue_.push(data);
            // popping thread to wake up
            cond_.notify_all();
        }
        // Push data to Q with rvalue reference
        void push(T&& data) {
            // Push with lock
            std::unique_lock<std::mutex> lk(mutex_);
            // Check whether the Q is closed or not and pushing is allowed
            if (State::CLOSED == state_)
                throw std::runtime_error("The queue is closed and trying to push.");
            // Pushing to Q
            queue_.push(data);
            // popping thread to wake up
            cond_.notify_all();
        }
        // Poping value from Q and write to var
        // If successful, OK is returned, else if the Q is empty and was closed, then CLOSED is returned
        QueueResult pop(T& var, std::chrono::milliseconds timeout = 100ms)
        {
            std::unique_lock<std::mutex> lk(mutex_);
            // wait until there is data, if there is no data
            if (cond_.wait_for(lk, timeout, [&] {return !queue_.empty(); })) {
                var = queue_.front();
                queue_.pop();
                return OK;
            }
            else if (queue_.empty() && State::CLOSED == state_)
                return CLOSED;
            else {
                return TIMEOUT;
            }
        }
        // No pushing data when the queue is closed
        void close() noexcept
        {
            std::unique_lock<std::mutex> lk(mutex_);
            state_ = State::CLOSED;
            // all consumers notify
            cond_.notify_all();
        }
    };
}
#endif
