#ifndef ATTITUDEFEEDFORWARD_H
#define ATTITUDEFEEDFORWARD_H

#include "custom_msgs/SSController.h"

class AttitudeFeedForward
{

public :
    float u_ff_;
    float u_ff_old_;
    float ud_old_;
    float Kff_;
    float b1_, a1_;
    float Tm_, Ts_; 
    custom_msgs::SSController ff_ctl_msg_;

    AttitudeFeedForward();
    ~AttitudeFeedForward();
    float feedForwardCompute(float ud);
    void setFeedForwardGain(float kff);
    void setFeedForwardModel(float timeConstant, float sampleTime, float modelFasteFactor);
    custom_msgs::SSController generateMsg();
};

#endif // ATTITUDEFEEDFORWARD_H
