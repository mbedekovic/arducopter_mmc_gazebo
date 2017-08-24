#ifndef ATTITUDE_CTL_NODE_H
#define ATTITUDE_CTL_NODE_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <AttitudeControlSS.h>
#include <AttitudeFeedForward.h>
#include <custom_msgs/PIDController.h>
#include <custom_msgs/SSController.h>
#include <rosgraph_msgs/Clock.h>
#include <cmath>
#include <signal.h>

#define GM_FRONT 1
#define GM_BACK 3

class AttitudeCtlNode
{
public:
    AttitudeCtlNode();
    ~AttitudeCtlNode();
    void run();
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void refCallback(const geometry_msgs::Vector3::ConstPtr &msg);
    void clkCallback(const rosgraph_msgs::Clock::ConstPtr &msg);
    void refPosCallback(const geometry_msgs::Vector3::ConstPtr &msg);
    void pidvxCallback(const custom_msgs::PIDController::ConstPtr &msg);
    void pidvyCallback(const custom_msgs::PIDController::ConstPtr &msg);
    float mmPos2Angle(float pos, int mm_id);

    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber euler_ref_sub_;
    ros::Subscriber clock_sub_;
    ros::Subscriber pos_ref_sub_;
    ros::Subscriber pid_vx_sub_;
    ros::Subscriber pid_vy_sub_;

    ros::Publisher mass1_pub_;
    ros::Publisher mass2_pub_;
    ros::Publisher mass3_pub_;
    ros::Publisher euler_ahrs_pub_;
    ros::Publisher mass4_pub_;

    //ros::Publisher roll_obs_pub_;
    //ros::Publisher roll_ctl_pub_;
    //ros::Publisher pitch_obs_pub_;
    //ros::Publisher pitch_ctl_pub_;
    ros::Publisher roll_ss_pub_;
    ros::Publisher pitch_ss_pub_;
    ros::Publisher pitch_ff_pub_;
    ros::Publisher roll_ff_pub_;

    geometry_msgs::Vector3 euler_mv_;
    geometry_msgs::Vector3 euler_sp_;
    geometry_msgs::Vector3 pos_sp_;
    geometry_msgs::Vector3 euler_rate_mv_, euler_rate_old_;
    //AttitudeControlSS roll_ctl_ss_;
    //AttitudeFeedForward roll_ff_;
    AttitudeControlSS *pitch_ctl_ss_;
    AttitudeFeedForward *pitch_ff_;
    AttitudeControlSS *roll_ctl_ss_;
    AttitudeFeedForward *roll_ff_;

    bool first_meas_;
    float u_roll_;
    float u_pitch_;
    float dwx_, dwy_;
    float w0_;
    double clk_sec_old;
    double clk_sec_now;
    float gyro_pt1_const;


    int ppm_f_mv, ppm_b_mv;

};


#endif // ATTITUDE_CTL_NODE_H
