#pragma once

#include <iostream>
#include <mutex>
#include <condition_variable>

//https://stackoverflow.com/questions/4792449/c0x-has-no-semaphores-how-to-synchronize-threads
class Semaphore {
    std::mutex mutex_;
    std::condition_variable condition_;
    unsigned long count_;
    const unsigned long maxVal;

public:
    Semaphore(int initVal, int maxVal): maxVal(maxVal), count_(initVal){}

    void release() {
        std::lock_guard<decltype(mutex_)> lock(mutex_);
        count_ = std::min(maxVal, count_ + 1);
        condition_.notify_one();
        std::cout << count_ << std::endl;
    }

    void acquire() {
        std::unique_lock<decltype(mutex_)> lock(mutex_);
        while(!count_){ // Handle spurious wake-ups.        
            condition_.wait(lock);
        }
        --count_;
        // netreba count_ = std::max((unsigned long) 0, count_ - 1); lebo while nepusti ak count nieje vacsie ako 0
        std::cout << count_ << std::endl;
    }
};