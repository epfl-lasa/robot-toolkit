#ifndef MOVINGAVERAGE_H
#define MOVINGAVERAGE_H

//#include <vector>
#include <assert.h>
//#include <cstring>
#include <iostream>
#include <boost/circular_buffer.hpp>
#include <numeric>

template <class T>
class MovingAverage
{
public:
    MovingAverage(int winlen);
    ~MovingAverage();
    T filter(const T&);
private:
    boost::circular_buffer<T> *data_history_;
    bool is_initialized_;
};


template <class T>
MovingAverage<T>::MovingAverage(int winlen)
{
    is_initialized_ = false;
    assert(winlen>0);
    data_history_ = new boost::circular_buffer<T>(winlen);
}

template <class T>
MovingAverage<T>::~MovingAverage()
{
    delete data_history_;
}


template <class T>
T MovingAverage<T>::filter(const T& newdata)
{
    if(!is_initialized_){
        for (int i = 0; i < data_history_->capacity(); ++i) {
            data_history_->push_back(newdata);
        }
        is_initialized_=true;
    }
    T result = std::accumulate(data_history_->begin()+1,data_history_->end(),newdata);
    result /= static_cast<float>(data_history_->capacity());
    data_history_->push_back(newdata);
    return result;
}



#endif // MOVINGAVERAGE_H
