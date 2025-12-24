#ifndef THREADSAFEQUEUE_H
#define THREADSAFEQUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <iostream>

template<typename T>
class ThreadSafeQueue {
public:
    explicit ThreadSafeQueue(size_t maxSize = 100)
        : maxSize_(maxSize) {}

    // 推入元素（如队列满，移除最旧的元素）
    void push(const T& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (queue_.size() >= maxSize_) {
            std::cout << "[Queue] Full: removing oldest element to make room.\n";
            queue_.pop();  // 丢弃最旧的元素
        }
        queue_.push(value);
        std::cout << "[Queue] Pushed. Current size = " << queue_.size() << "\n";
        cond_.notify_one();
    }

    // 推入右值引用（支持 std::move）
    void push(T&& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (queue_.size() >= maxSize_) {
            std::cout << "[Queue] Full: removing oldest element to make room.\n";
            queue_.pop();
        }
        queue_.push(std::move(value));
        std::cout << "[Queue] Pushed (moved). Current size = " << queue_.size() << "\n";
        cond_.notify_one();
    }

    // 阻塞直到取出一个元素
    T wait_and_pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this] { return !queue_.empty(); });
        T value = std::move(queue_.front());
        queue_.pop();
        std::cout << "[Queue] Popped. Current size = " << queue_.size() << "\n";
        return value;
    }

    size_t size() {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
    size_t maxSize_;
};

#endif
