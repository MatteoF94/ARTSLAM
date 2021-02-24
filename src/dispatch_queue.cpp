/** @file DispatchQueue.cpp
 * @brief Definition of class DispatchQueue
 */

#include <dispatch_queue.h>
#include <iostream>
#include <sstream>

DispatchQueue::DispatchQueue(const std::string& name, size_t thread_count) : name(name), threads(thread_count){
    this->quit = false;
    std::ostringstream to_print;
    to_print << "[DispatchQueue] creating dispatch queue: " << name << "\n";
    to_print << "[DispatchQueue (" << name << ")] number of dispatched threads: " << thread_count << "\n";
    std::cout << to_print.str();

    for (auto & i : this->threads) {
        i = std::thread(&DispatchQueue::dispatch_thread_handler, this);
    }
}

DispatchQueue::~DispatchQueue() {
    std::ostringstream to_print;
    to_print << "[DispatchQueue (" << this->name << ")] destroying dispatched threads" << "\n";
    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    // signal to dispatch threads that it's time to wrap up
    std::unique_lock<std::mutex> new_lock(this->lock);
    this->quit = true;
    new_lock.unlock();
    this->cv.notify_all();

    // wait for threads to finish before the exit
    for (int i = 0; i < this->threads.size(); i++) {
        if (this->threads[i].joinable()) {
            this->threads[i].join();
        }
    }

    to_print << "[DispatchQueue (" << this->name << ")] finished destroying dispatched threads" << "\n";
    std::cout << to_print.str();
}

/**
 * @brief Prematurely stops the threads and performs cleaning operations, like the destructor.
 */
void DispatchQueue::stop_threads() {
    std::ostringstream to_print;
    to_print << "[DispatchQueue (" << this->name << ")][stop_threads] destroying dispatched threads" << "\n";
    std::cout << to_print.str();
    to_print.str("");
    to_print.clear();

    // signal to dispatch threads that it's time to wrap up
    std::unique_lock<std::mutex> new_lock(this->lock);
    this->quit = true;
    new_lock.unlock();
    this->cv.notify_all();

    // wait for threads to finish before the exit
    for (int i = 0; i < this->threads.size(); i++) {
        if (this->threads[i].joinable()) {
            this->threads[i].join();
        }
    }

    to_print << "[DispatchQueue (" << this->name << ")][stop_threads] finished destroying dispatched threads" << "\n";
    std::cout << to_print.str();
}

/**
 * @brief Adds an element to the queue of operations to execute.
 */
void DispatchQueue::dispatch(const fp_t &op) {
    std::unique_lock<std::mutex> new_lock(this->lock);
    this->op_queue.push(op);

    // manual unlocking is done before notifying, to avoid waking up threads unecessarily
    new_lock.unlock();
    this->cv.notify_one();
}

/**
 * @brief Adds an element to the queue of operations to execute (r-value).
 */
void DispatchQueue::dispatch(fp_t &&op) {
    std::unique_lock<std::mutex> new_lock(this->lock);
    this->op_queue.push(std::move(op));

    // manual unlocking is done before notifying, to avoid waking up threads unecessarily
    new_lock.unlock();
    this->cv.notify_one();
}

/**
 * @brief Handles the threads, assigning operations to execute, depending on their availability.
 */
void DispatchQueue::dispatch_thread_handler() {
    std::unique_lock<std::mutex> new_lock(this->lock);

    do {
        // wait until data is available or a quit signal
        this->cv.wait(new_lock, [this]{return (!this->op_queue.empty() || this->quit);});

        // after wait, take the lock
        if (!this->quit && !this->op_queue.empty()) {
            auto op = std::move(this->op_queue.front());
            this->op_queue.pop();

            new_lock.unlock();

            op();

            new_lock.lock();
        }

        if(this->quit) {
            int micio = 2;
        }
    } while (!this->quit);

    int miao = 2;
}
