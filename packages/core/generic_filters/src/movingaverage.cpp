#include "movingaverage.h"


template <class data_type>
MovingAverage<data_type>::MovingAverage(int winlen)
{
    is_initialized_ = false;
    winlen_ = winlen;
    tail_ = 0;
    assert(winlen_>0);
    data_history_ = new data_type[winlen];
}

template <class data_type>
data_type MovingAverage<data_type>::filter(const data_type& newdata)
{
    if(!is_initialized_){
        for (int i = 0; i < winlen_; ++i) {
            data_history_[i] = newdata;
        }
        is_initialized_=true;
    }else{
        data_history_[tail_] = newdata;
        ++tail_;
        if(tail_>winlen_-1)
            tail_=0;
    }

    result_ *= 0.0;
    for (int i = 0; i < winlen_; ++i) {
        result_ += data_history_[i];
    }
    result_ /= static_cast<float>(winlen_);
}

