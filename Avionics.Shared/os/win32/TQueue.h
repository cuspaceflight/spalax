#pragma once
#include <queue>
#include <mutex>

template <class T> 
class TQueue {
public:
    void enqueue(T val) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(val);
        condition_.notify_one();
    }

    T dequeue() {
        std::unique_lock<std::mutex> lock(mutex_);
        while (queue_.empty()) {
            condition_.wait(lock);
        }
        T val = queue_.front();
        queue_.pop();
        return val;
    }

    bool isEmpty() {
        std::unique_lock<std::mutex> lock(mutex_);
        return queue_.empty();
    }

private:
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
};