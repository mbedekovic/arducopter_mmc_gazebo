#ifndef ATTITUDECONTROLSS_H
#define ATTITUDECONTROLSS_H
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rosgraph_msgs/Clock.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "custom_msgs/SSController.h"
#include <sstream>
#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <float.h>

using namespace Eigen;

class AttitudeControlSS
{

public :
    u_int64_t clk_sec_;
    Eigen::MatrixXf A_;
    Eigen::MatrixXf B_;
    Eigen::MatrixXf C_;
    Eigen::MatrixXf D_;
    Eigen::MatrixXf states_, y_est_;
    Eigen::MatrixXf Kctl_;
    Eigen::MatrixXf Kobs_;
    custom_msgs::SSController ss_ctl_msg_;
    float Ki_;
    float y_sp_;
    float y_mv_[2];
    float ui_old_;
    float u_, ui_, us_;
    int n_, m_, l_;
    float Td_; // discretization time;
    float dead_zone_;
    float saturation_;

    AttitudeControlSS(int n, int m, int l, float Td);
    ~AttitudeControlSS();
    void initParams();
    void observerCompute(float u, float* y);
    float stateSpaceCtlCompute(float y_sp, float y_mv, float u_ff);
    custom_msgs::SSController generateMsg();
    void run();
    void setAMatrix(std::vector<double> double_list);
    void setBMatrix(std::vector<double> double_list);
    void setCMatrix(std::vector<double> double_list);
    void setDMatrix(std::vector<double> double_list);
    void setKctlMatrix(std::vector<double> double_list);
    void setKiGain(double Ki);
    void setDeadZone(double dz);
    void setSaturation(double dz);
    void setKobsMatrix(std::vector<double> double_list);

};

#endif // ATTITUDECONTROLSS_H
