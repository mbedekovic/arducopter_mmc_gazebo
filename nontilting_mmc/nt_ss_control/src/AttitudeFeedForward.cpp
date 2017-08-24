#include <AttitudeFeedForward.h>

AttitudeFeedForward::AttitudeFeedForward()
{
    u_ff_ = 0.0;      // ff control value
    u_ff_old_ = 0.0; // old ff control value
    ud_old_ = 0.0; // old disturbance value

    Kff_ = 0.002; // feed forward gain
    //Kff_ = 1;

    // disturbance is modeled as discrete dynamic element:
    // yd = a1_ * yd(k-1) + b1_ * ud_(k-1)
    // the following coefficient correspond to the gas motor tf
    a1_ =  0.428571428571;
    b1_ = 1 - a1_;

    ff_ctl_msg_.state = std::vector<float>(1);
    ff_ctl_msg_.meas = std::vector<float>(1);
    ff_ctl_msg_.est = std::vector<float>(1);


}

AttitudeFeedForward::~AttitudeFeedForward()
{
    //delete ss_ctl_msg_;
}

float AttitudeFeedForward::feedForwardCompute(float ud)
{
    float uff;

    uff = a1_ * u_ff_old_ + Kff_ * b1_ * ud_old_;

    u_ff_ = uff;
    u_ff_old_ = u_ff_;
    ud_old_ = ud;

    return uff;
}

void AttitudeFeedForward::setFeedForwardGain(float kff) {
    Kff_ = kff;

    std::cout << "Feedforward gain set to " << Kff_ << std::endl;
}

void AttitudeFeedForward::setFeedForwardModel(float timeConstant, float sampleTime, float modelFasteFactor) {
    Tm_ = timeConstant / (float)modelFasteFactor;
    Ts_ = sampleTime;

    std::cout<<"Disturbance model params Tm, Ts = "<< Tm_<< ", " <<  Ts_ << std::endl;

    a1_ = Tm_ / (Tm_ + Ts_);
    b1_ = 1 - a1_;
    
    std::cout<<"PT1 model constants a1,  b1 = "<< a1_<< ", " <<  b1_ << std::endl;

}



custom_msgs::SSController AttitudeFeedForward::generateMsg()
{   //std::cout << "Generating messages" << std::endl;
    ff_ctl_msg_.ref = ud_old_;
    ff_ctl_msg_.meas[0] = u_ff_old_ / Kff_;
    ff_ctl_msg_.header.stamp = ros::Time::now();

    return ff_ctl_msg_;
}


