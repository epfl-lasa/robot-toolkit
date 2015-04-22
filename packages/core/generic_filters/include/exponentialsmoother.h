#ifndef EXPONENTIALSMOOTHER_H
#define EXPONENTIALSMOOTHER_H

#include <iostream>

template <class T>
class ExponentialSmoother{
public:
    ExponentialSmoother(const float &alpha){
        is_initialized_ = false;
        alpha_ = alpha;
        //std::cout<<old_average_<<std::endl;
    }
    ExponentialSmoother(const float &alpha,const T& starting_value){
        new_output_ = starting_value;
        is_initialized_ = true;
        alpha_ = alpha;
        //std::cout<<old_average_<<std::endl;
    }
    ~ExponentialSmoother(){}
    void set_initialized(bool init){
        is_initialized_ = init;
    }

    const T& filter(const T& inp){
        if(!is_initialized_){
            new_output_ = inp;
            is_initialized_ = true;
        }
        //new_output_.Print();
        //exit(1);
        new_output_ *= (1.0-alpha_);
        new_output_ += (inp*alpha_);
        return new_output_;
    }

private:
    T new_output_;
    float alpha_;
    bool is_initialized_;
};

//template<class T>
//ExponentialSmoother<T>::ExponentialSmoother(const float& alpha){
//    is_initialized_ = false;
//    alpha_ = alpha;
//    //std::cout<<old_average_<<std::endl;
//};

//template<class T>
//ExponentialSmoother<T>::ExponentialSmoother(const float& alpha,const T& starting_value){
//    new_output_ = starting_value;
//    is_initialized_ = true;
//    alpha_ = alpha;
//    //std::cout<<old_average_<<std::endl;
//};

//template<class T>
//const T& ExponentialSmoother<T>::ExponentialSmoother::filter(const T& inp){
//    if(!is_initialized_){
//        new_output_ = inp;
//        is_initialized_ = true;
//    }
//    //new_output_.Print();
//    //exit(1);
//    new_output_ *= (1.0-alpha_);
//    new_output_ += (inp*alpha_);
//    return new_output_;
//}

template<class T>
class ApproximateMovingAverage : public ExponentialSmoother<T>{
public:
    ApproximateMovingAverage(const int& winlen) : ExponentialSmoother<T>(1.0/static_cast<float>(winlen)){};
    ApproximateMovingAverage(const int& winlen, const T& starting_value) : ExponentialSmoother<T>(1.0/static_cast<float>(winlen),starting_value){};
};

#endif // EXPONENTIALSMOOTHER_H
