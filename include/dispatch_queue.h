/** @file dispatch_queue.h
* @brief Declaration of class DispatchQueue
*/

#ifndef ARTSLAM_DISPATCH_QUEUE_H
#define ARTSLAM_DISPATCH_QUEUE_H


#include <functional>
#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>

/**
 * @class DispatchQueue
 * @brief This class is used to handle dynamic requests to objects, allowing them to queue operations and execute in a FIFO manner.
 * @author Matteo Frosi
 */
class DispatchQueue {
public:
    /*
     * std::function<void()> does allow to bind any function that returns void as long as the arguments are pre-bound
     */
    typedef std::function<void()> fp_t;

    /**
     * @brief Class constructor
     * @param name The name of the dispatch queue.
     * @param thread_count Number of threads used to execute operations, default is 1.
     */
    DispatchQueue(const std::string& name, size_t thread_count = 1);

    /**
     * @brief Class destructor, it stops the threads and performs cleaning operations.
     */
    ~DispatchQueue();

    /**
     * @brief Prematurely stops the threads and performs cleaning operations, like the destructor.
     */
    void stop_threads();

    /**
     * @brief Adds an element to the queue of operations to execute.
     * @param op The operation to be added.
     */
    void dispatch(const fp_t& op);

    /**
     * @brief Adds an element to the queue of operations to execute.
     * @param op The operation to be added, passed as r-value.
     */
    void dispatch(fp_t&& op);

    /**
     * @brief Deletes the copy constructor.
     */
    DispatchQueue(const DispatchQueue& rhs) = delete;

    /**
     * @brief Deletes the copy assignment operator.
     */
    DispatchQueue& operator = (const DispatchQueue& rhs) = delete;

    /**
     * @brief Deletes the move constructor.
     */
    DispatchQueue(DispatchQueue&& rhs) = delete;

    /**
     * @brief Deletes the move assignment operator.
     */
    DispatchQueue& operator = (DispatchQueue&& rhs) = delete;

private:
    /**
     * @brief Handles the threads, assigning operations to execute, depending on their availability.
     */
    void dispatch_thread_handler();

    // ---------------------------------------------------------------
    // ------------------- PARAMETERS AND VARIABLES ------------------
    //----------------------------------------------------------------
    std::string name;                   /**< Name of the dispatch queue */
    std::mutex lock;                    /**< Lock used in the dispatch queue */
    std::vector<std::thread> threads;   /**< Vector of threads used by the dispatch queue */
    std::queue<fp_t> op_queue;          /**< Queue of operations to execute */
    std::condition_variable cv;         /**< Condition variable used to wake or stop a thread */
    bool quit;                          /**< Whether or not the threads should stop processing new operations */
};


#endif //ARTSLAM_DISPATCH_QUEUE_H
