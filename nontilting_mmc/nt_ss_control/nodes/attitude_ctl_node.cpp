#include <attitude_ctl_node.h>


AttitudeCtlNode::AttitudeCtlNode()
{
    ros::NodeHandle nh_;
    first_meas_ = false;
    u_roll_ = 0.0;
    u_pitch_ = 0.0;
    w0_ = 456.21577;
    dwx_ = 0;
    mm_path = 0.21;
    ppm_f_mv = 1000;
    ppm_b_mv = 1000;
    gyro_pt1_const = 0.9;
    int n, m, l;
    float Td;
    double Ki;
    double dead_zone;
    float Kff, tm, modelFasterFactor;
    
    euler_rate_mv_.y = 0;
    euler_rate_old_.y = 0;

    std::string param;

    usleep(1000000);

    mass1_pub_ = nh_.advertise<std_msgs::Float64>("mm_front_controller/command", 2);
    mass3_pub_ = nh_.advertise<std_msgs::Float64>("mm_back_controller/command", 2);

    imu_sub_ =  nh_.subscribe("ahrs", 2, &AttitudeCtlNode::imuCallback, this);
    euler_ref_sub_ = nh_.subscribe("euler_ref", 2, &AttitudeCtlNode::refCallback, this);
    gm_front_status_sub_ = nh_.subscribe("/gm_front_status", 2,&AttitudeCtlNode::gmFrontCallback, this);
    gm_back_status_sub_ = nh_.subscribe("/gm_back_status",2, &AttitudeCtlNode::gmBackCallback, this);

    //roll_ss_pub_ = nh_.advertise<custom_msgs::SSController>("/morus/roll_ctl", 1);
    pitch_ss_pub_ = nh_.advertise<custom_msgs::SSController>("pitch_ctl", 1);
    pitch_ff_pub_ = nh_.advertise<custom_msgs::SSController>("pitch_ff", 1);
    euler_ahrs_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("euler_ahrs", 1);
    //roll_ff_pub_ = nh_.advertise<custom_msgs::SSController>("/morus/roll_ff", 1);

    std::vector<double> my_double_list;

    if (ros::param::has("~mm_f_fv")) {
        ros::param::get("~mm_f_fv", mm_f_fv);    
    }
    else {
        ROS_INFO("Could not load a moving mass param.");
        ros::shutdown();
    }
    
    if (ros::param::has("~mm_f_bv")) {
        ros::param::get("~mm_f_bv", mm_f_bv);    
    }
    else {
        ROS_INFO("Could not load a moving mass param.");
        ros::shutdown();
    }
    
    if (ros::param::has("~mm_b_fv")) {
        ros::param::get("~mm_b_fv", mm_b_fv);    
    }
    else {
        ROS_INFO("Could not load a moving mass param.");
        ros::shutdown();
    }
    
    if (ros::param::has("~mm_b_bv")) {
        ros::param::get("~mm_b_bv", mm_b_bv);    
    }
    else {
        ROS_INFO("Could not load a moving mass param.");
        ros::shutdown();
    }

    //compute linear tranformation from position to angles
    //angle = a * pos + b
    mm_f_a = (mm_f_fv - mm_f_bv) / (2.0 * mm_path);
    mm_f_b = mm_f_fv - mm_f_a * mm_path;

    mm_b_a = (mm_b_fv - mm_b_bv) / (2.0 * mm_path);
    mm_b_b = mm_b_fv - mm_b_a * mm_path;

    ROS_INFO("Front mass a, b = %.3f, %.3f \n", mm_f_a, mm_f_b);
    ROS_INFO("Back mass a, b = %.3f, %.3f \n", mm_b_a, mm_b_b);

    if (ros::param::has("~pt1_const")) {
        ros::param::get("~pt1_const", gyro_pt1_const);    
    }
    else {
        ROS_INFO("Could not load pt1 filter constant.");
        ros::shutdown();
    }
    
    if (ros::param::has("~Td")) {
        ros::param::get("~Td", Td);    
    }
    else {
        ROS_INFO("Could not load discretization time from param server.");
        ros::shutdown();
    }
    
    if (ros::param::has("~n")) {
       ros::param::get("~n", n);    
    }
    else {
        ROS_INFO("Could not load the number of states N from param server.");
        ros::shutdown();
    }

    if (ros::param::has("~m")) {
        ros::param::get("~m", m);    
    }
    else {
        ROS_INFO("Could not load the number of inputs M from param server.");
        ros::shutdown();
    }

    if (ros::param::has("~l")) {
        ros::param::get("~l", l);    
    }
    else {
        ROS_INFO("Could not load the number of outputs L from param server.");
        ros::shutdown();
    }

    if (ros::param::has("~A")) {
        ros::param::get("~A", my_double_list);    
    }
    else {
        ROS_INFO("Could not load system matrix A from param server.");
        ros::shutdown();
    }

    if (ros::param::has("~dead_zone")) {
        ros::param::get("~dead_zone", dead_zone);    
    }
    else {
        ROS_INFO("Could not load dead zone param from param server.");
        ros::shutdown();
    }

    pitch_ctl_ss_ = new AttitudeControlSS(n,m,l, Td);

    pitch_ctl_ss_->setDeadZone(dead_zone);

    pitch_ctl_ss_->setAMatrix(my_double_list);

    if (ros::param::has("~B")) {
        ros::param::get("~B", my_double_list);    
    }
    else {
        ROS_INFO("Could not load system matrix B from param server.");
        ros::shutdown();
    }

    pitch_ctl_ss_->setBMatrix(my_double_list);

    if (ros::param::has("~C")) {
        ros::param::get("~C", my_double_list);    
    }
    else {
        ROS_INFO("Could not load C matrix from param server.");
        ros::shutdown();
    }

    pitch_ctl_ss_->setCMatrix(my_double_list);

    if (ros::param::has("~D")) {
        ros::param::get("~D", my_double_list);    
    }
    else {
        ROS_INFO("Could not load D matrix from param server.");
        ros::shutdown();
    }

    pitch_ctl_ss_->setDMatrix(my_double_list);

    if (ros::param::has("~Kctl")) {
        ros::param::get("~Kctl", my_double_list);    
    }
    else {
        ROS_INFO("Could not load KCtl matrix from param server.");
        ros::shutdown();
    }

    pitch_ctl_ss_->setKctlMatrix(my_double_list);

    if (ros::param::has("~Kobs")) {
        ros::param::get("~Kobs", my_double_list);    
    }
    else {
        ROS_INFO("Could not load Kobs matrix from param server.");
        ros::shutdown();
    }

    pitch_ctl_ss_->setKobsMatrix(my_double_list);

    if (ros::param::has("~Ki")) {
        ros::param::get("~Ki", Ki);    
    }
    else {
        ROS_INFO("Could not load Ki gain from param server.");
        ros::shutdown();
    }

    pitch_ctl_ss_->setKiGain(Ki);

    std::cout << "A=" << std::endl << pitch_ctl_ss_->A_ << std::endl;
    std::cout << "B=" << std::endl << pitch_ctl_ss_->B_ << std::endl;
    std::cout << "C=" << std::endl << pitch_ctl_ss_->C_ << std::endl;
    std::cout << "D=" << std::endl << pitch_ctl_ss_->D_ << std::endl;
    std::cout << "Kctl=" << std::endl << pitch_ctl_ss_->Kctl_ << std::endl;
    std::cout << "Kobs=" << std::endl << pitch_ctl_ss_->Kobs_ << std::endl;
    std::cout << "Ki=" << std::endl << pitch_ctl_ss_->Ki_ << std::endl;

    if (ros::param::has("~Kff")) {
        ros::param::get("~Kff", Kff);    
    }
    else {
        ROS_INFO("Could not load feed forward gain.");
        ros::shutdown();
    }
    
    if (ros::param::has("~Tm")) {
        ros::param::get("~Tm", tm);    
    }
    else {
        ROS_INFO("Could not load feed forward model dynamics constant.");
        ros::shutdown();
    }

    if (ros::param::has("~modelFaster")) {
        ros::param::get("~modelFaster", modelFasterFactor);    
    }
    else {
        ROS_INFO("Could not load the param which determines how much faster is the feedforward model.");
        ros::shutdown();
    }

    pitch_ff_ = new AttitudeFeedForward();
    pitch_ff_->setFeedForwardGain(Kff);
    pitch_ff_->setFeedForwardModel(tm, Td, modelFasterFactor);

}

AttitudeCtlNode::~AttitudeCtlNode() {
    if (pitch_ctl_ss_)
        delete pitch_ctl_ss_;
    if (pitch_ff_)
        delete pitch_ff_;
}


void AttitudeCtlNode::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    float qx, qy, qz, qw, p, q, r, sx, cx, cy, ty;

    /*
    qx = msg->orientation.x;
    qy = msg->orientation.y;
    qz = msg->orientation.z;
    qw = msg->orientation.w;

    euler_mv_.x = atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz);
    euler_mv_.y = asin(2 * (qw * qy - qx * qz));
    euler_mv_.z = atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz);
    */
    
    euler_mv_.x = msg->orientation.x;
    euler_mv_.y = -msg->orientation.y;
    euler_mv_.z = -msg->orientation.z;
    
    p = msg->angular_velocity.x;
    q = -msg->angular_velocity.y;
    r = -msg->angular_velocity.z;

    sx = sin(euler_mv_.x);       // sin(roll)
    cx = cos(euler_mv_.x);      // cos(roll)
    cy = cos(euler_mv_.y);      // cos(pitch)
    ty = tan(euler_mv_.y);      // cos(pitch)

    //conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
    euler_rate_mv_.x = p + sx * ty * q + cx * ty * r;
    euler_rate_mv_.y = cx * q - sx * r;
    euler_rate_mv_.z = sx / cy * q + cx / cy * r;

    // filter pitch rate
    euler_rate_mv_.y = gyro_pt1_const * euler_rate_old_.y + (1-gyro_pt1_const) * euler_rate_mv_.y;

    euler_rate_old_.y = euler_rate_mv_.y;

    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.stamp = ros::Time::now();
    twist_msg.twist.linear = euler_mv_;
    twist_msg.twist.angular = euler_rate_mv_;
    euler_ahrs_pub_.publish(twist_msg);
    
    
    if (first_meas_ == false)
        first_meas_ = true;
}

void AttitudeCtlNode::refCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
    euler_sp_.x = msg->x;
    euler_sp_.y = msg->y;
    euler_sp_.z = msg->z;
}

void AttitudeCtlNode::clkCallback(const rosgraph_msgs::Clock::ConstPtr &msg)
{
    clk_sec_now = msg->clock.toSec();
}

void AttitudeCtlNode::refPosCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
    pos_sp_.x = msg->x;
    pos_sp_.y = msg->y;
}

void AttitudeCtlNode::pidvxCallback(const custom_msgs::PIDController::ConstPtr &msg)
{
    dwx_ = msg->U;
}

void AttitudeCtlNode::gmFrontCallback(const morus_uav_ros_msgs::GmStatus::ConstPtr &msg) {
    ppm_f_mv = msg->starter_ppm_S;
}

void AttitudeCtlNode::gmBackCallback(const morus_uav_ros_msgs::GmStatus::ConstPtr &msg) {
    ppm_b_mv = msg->starter_ppm_S;
}

void AttitudeCtlNode::run()
{

    std_msgs::String msg;
    std::stringstream ss;

    /*
    while (ros::Time::now().toNSec() == 0)
    {
        ss << "Waiting for the first clock tick" << std::endl;
        ROS_INFO("%s", ss.str().c_str());
        sleep(1);
    }
    */


    while (first_meas_ == false) {
        ROS_INFO("Waiting for the first measurement");
        sleep(1);
        ros::spinOnce();
    }

    // set initial states
    pitch_ctl_ss_->states_ << 0.0, 0.0, euler_rate_mv_.y, euler_mv_.y;

    float rate = 1.0 / pitch_ctl_ss_->Td_;
    ROS_INFO("Ros rate %f", rate);
    ros::Rate ros_rate(rate);
    
    clk_sec_now = ros::Time::now().toSec();
    clk_sec_old = clk_sec_now;
    ros_rate.sleep();

    std_msgs::Float64 mass1_ref_msg;
    //std_msgs::Float64 mass2_ref_msg;
    std_msgs::Float64 mass3_ref_msg;
    //std_msgs::Float64 mass4_ref_msg;
    custom_msgs::PIDController pid_msg;
    custom_msgs::SSController ss_msg;

    float dt;
    float y_obs[2] = {0.0};
    int count = 0;
    float u_roll_ff;
    float u_pitch_ff;

    ROS_INFO("Starting control.\n");

    while (ros::ok())
    {
        ros::spinOnce();
        clk_sec_now = ros::Time::now().toSec();
        dt = clk_sec_now - clk_sec_old;
        clk_sec_old = clk_sec_now;

        //ROS_INFO("Time (sec) : %.3f", clk_sec_now);

        if (dt > 0.015)
        {
            count++;
            ROS_INFO("Sporija petlja %.3f - %d", dt, count);
        }
        else if (dt < 0.005) {
            count++;
            ROS_INFO("Brza petlja %.3f - %d", dt, count);
        }
        // compute feed forward
        u_pitch_ff = pitch_ff_->feedForwardCompute(ppm_f_mv - ppm_b_mv);

        //ROS_INFO("pitch ff value %.5f", u_pitch_ff);
      
        y_obs[0] = euler_rate_mv_.y;
        y_obs[1] = euler_mv_.y;
        //ROS_INFO("Computing observer");
        pitch_ctl_ss_->observerCompute(u_pitch_, y_obs);
        //ROS_INFO("Computing control law");
        u_pitch_ = pitch_ctl_ss_->stateSpaceCtlCompute(euler_sp_.y, euler_mv_.y, u_pitch_ff);

        mass1_ref_msg.data =  mmPos2Angle(u_pitch_, GM_FRONT);;
        mass1_pub_.publish(mass1_ref_msg);

        mass3_ref_msg.data =  mmPos2Angle(u_pitch_, GM_BACK);;
        mass3_pub_.publish(mass3_ref_msg);
        ros::spinOnce();

        //roll_ss_pub_.publish(roll_ctl_ss_.generateMsg());
        pitch_ss_pub_.publish(pitch_ctl_ss_->generateMsg());

        pitch_ff_pub_.publish(pitch_ff_->generateMsg());
        //roll_ff_pub_.publish(roll_ff_.generateMsg());


        ros::spinOnce();
        ros_rate.sleep();
    }
}

float AttitudeCtlNode::mmPos2Angle(float pos, int mm_id) {
    float angle = 0.0;
    if (mm_id == GM_FRONT) {
        angle = mm_f_a * pos + mm_f_b;
    }
    else if (mm_id == GM_BACK) {
        angle = mm_b_a * pos + mm_b_b;

    }
    else {
        ROS_INFO("Received unexpected mm id: %d \n", mm_id);
    }

    return angle;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "attitute_ctl");
    //ros::NodeHandle private_node_handle("~");



    AttitudeCtlNode *attitude_ctl;
    attitude_ctl = new AttitudeCtlNode();
    attitude_ctl->run();

    delete attitude_ctl;
    return 0;
}
