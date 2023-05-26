#pragma once

#include<thread>
#include<mutex>
#include<condition_variable>
#include<deque>
#include<queue>
#include<atomic>
#include<optional>

#include "mission_msgs/MissionMsg.h"

namespace mission_manager{

template <typename T>
class DoubleEndedQueue{

private:
    std::deque<T> queue_;
    std::mutex mutex_;

public:
    std::atomic<bool> isStop;

    /**
     * @brief Construct a new Double Ended Queue object
     * 
     */
    DoubleEndedQueue(): isStop(false)
    {}

    /**
     * @brief Function to push element to the back of the queue.
     * 
     * @param data mission/feedback message  
     */
    void enqueue(const T& data)
    {
        // Acquire lock on queue
        std::unique_lock<std::mutex> lock(mutex_);

        // Add data to queue
        queue_.push_back(data);

    } // lock is released when it goes out of scope

    /**
     * @brief   Function to get element from the front of the queue
     *          The data can be accessed from the "result" member variable
     * 
     * @return std::optional<T> 
     */
    std::optional<T> dequeue()
    {
        // Acquire lock on queue
        std::unique_lock<std::mutex> lock(mutex_);
        std::optional<T> result;
      
        // retrieve and return data from the front of queue
        if(queue_.size()>0)
        {
            result = queue_.front();
            queue_.pop_front();
        }

        return result;
    }

    /**
     * @brief Move all element of queue to the vector passed as reference
     * 
     * @param msgs Reference to the vector to store messages
     */
    void dequeueAll(std::vector<T>& msgs)
    {
        // Acquire lock on queue
        std::unique_lock<std::mutex> lock(mutex_);

        while (!queue_.empty())
        {
            msgs.push_back(queue_.front());
            queue_.pop_front();
        }
    }

    /**
     * @brief   Stop the processing of queue. Cancel all waiting 
     *          tasks so the threads can join main thread.
     *          All the element of the queue are destroyed.
     */
    void stopQueue()
    {
        isStop = true;
        clearAll();
    }

    /**
     * @brief   Delete message with a certain mission_id from the queue 
     * 
     * @param mission_id Mission ID of the message to be deleted.
     * @return true : successful in deleting 
     * @return false: unsuccessful/mission id not found 
     */
    bool deleteEntry(uint64_t mission_id)
    {
        // Acquire lock on queue
        std::unique_lock<std::mutex> lock(mutex_);

        bool status = false;

        if(queue_.empty())
        {
            return status;
        }

        // check if T.mission_id exists
        typename std::deque<T>::iterator iter = queue_.begin();
        try
        {
            if (iter->mission_id>0)
            {
                while (iter != queue_.end())
                {
                    if(iter->mission_id == mission_id)
                    {
                        iter = queue_.erase(iter);
                        status = true;
                    }
                    else
                        ++iter;
                }
            }
        }
        catch(const std::exception& e)
        {
            status = false;
            ROS_WARN("Error deleting element from queue. ");
            ROS_WARN("%s",e.what());
        }
        
        return status;
    }

    /**
     * @brief   Delete all the elements in the queue.
     * 
     */
    void clearAll()
    {
        // Acquire lock on queue
        std::unique_lock<std::mutex> lock(mutex_);

        queue_.clear();
    }

};  //DoubleEndedQueue

// Comparison function object type for std::priority_queue
struct Compare{
public:
    bool operator() (mission_msgs::MissionMsg lhs, 
                        mission_msgs::MissionMsg rhs )
    {
        return lhs.priority < rhs.priority;
    }
};  // Compare

class PriorityBasedQueue{
private:
    std::priority_queue<mission_msgs::MissionMsg, 
                std::vector<mission_msgs::MissionMsg>, Compare> queue_;
    std::mutex mutex_;

public:
    std::atomic<bool> isStop;

    /**
     * @brief Construct a new Priority Based Queue object
     * 
     */
    PriorityBasedQueue():isStop(false)
    {}

    /**
     * @brief       Function to push element to the priority queue.
     * 
     * @param data  mission(command type) message with priority 
     */
    void enqueue(const mission_msgs::MissionMsg& data)
    {
        // Acquire lock
        std::unique_lock<std::mutex> lock(mutex_);

        // Push element
        queue_.push(data);
    }

    /**
     * @brief   Function to get element from the front of the queue
     *          The data can be accessed from the "result" member variable
     * 
     * @return  std::optional<mission_msgs::MissionMsg> 
     */
    std::optional<mission_msgs::MissionMsg> dequeue()
    {
        // Acquire lock
        std::unique_lock<std::mutex> lock(mutex_);
        std::optional<mission_msgs::MissionMsg> result;

        if(queue_.size()>0)
        {
            // retrieve element
            result = queue_.top();
            queue_.pop();
        }
        return result;
    }

    /**
     * @brief   Delete all the elements in the queue.
     * 
     */
    void clearAll()
    {
        // Acquire lock on queue
        std::unique_lock<std::mutex> lock(mutex_);

        while (!queue_.empty())
        {
            queue_.pop();
        }
    }

    /**
     * @brief   Stop the processing of queue. Cancel all waiting 
     *          tasks so the threads can join main thread.
     *          All the element of the queue are destroyed.
     */
    void stopQueue()
    {
        isStop = true;
        clearAll();
    }
}; // PriorityBasedQueue 

} // namespace
