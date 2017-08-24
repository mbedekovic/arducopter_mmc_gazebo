#include <AttitudeControlSS.h>

AttitudeControlSS::AttitudeControlSS(int n, int m, int l, float Td)
{

    n_ = n;         // number of states
    m_ = m;         // number of inputs
    l_ = l;         // number of measurements (outputs())
    Td_ = Td;

    std::cout << n_ << ", " << m_ << ", " << l_ << std::endl;
    std::cout << "Initializing matrices" << std::endl;

    A_.resize(n_, n_);
    B_.resize(n_, m_);
    C_.resize(l_, n_);
    D_.resize(l_, m_);
    Kctl_.resize(m_, n_);
    Kobs_.resize(n_, l_);

    states_.resize(n,1);
    states_.setZero();

    y_est_.resize(l,1);
    y_est_.setZero();

    ui_old_ = 0.0;
    dead_zone_ = 0.0;
    saturation_ = FLT_MAX; // float max

    std::cout << "Initializing vectors" << std::endl;

    ss_ctl_msg_.state = std::vector<float>(n);
    ss_ctl_msg_.meas = std::vector<float>(l);
    ss_ctl_msg_.est = std::vector<float>(l); // states that are measured and estimated. used for evaluation of the model
   // std::cout << "Exiting constructor" << std::endl;
}

AttitudeControlSS::~AttitudeControlSS()
{
    //delete ss_ctl_msg_;
}

void AttitudeControlSS::setAMatrix(std::vector<double> double_list) {
    std::cout << "Setting A matrix."<< std::endl;
    for (int i = 0; i < n_; i++) {
        for(int j = 0; j < n_; j++){
            //ROS_INFO("(%d, %d) = %f", i,j, double_list[i*n_ + j]);
            A_(i,j) = double_list[i*n_ + j];
        } 
    }

}

void AttitudeControlSS::setBMatrix(std::vector<double> double_list) {
    std::cout << "Setting B matrix."<< std::endl;
    for (int i = 0; i < n_; i++) {
        for(int j = 0; j < m_; j++){
            //ROS_INFO("(%d, %d) = %f", i,j, double_list[i*n_ + j]);
            B_(i,j) = double_list[i*m_ + j];
        } 
    }

}

void AttitudeControlSS::setCMatrix(std::vector<double> double_list) {
    std::cout << "Setting C matrix."<< std::endl;
    for (int i = 0; i < l_; i++) {
        for(int j = 0; j < n_; j++){
            //ROS_INFO("(%d, %d) = %f", i,j, double_list[i*n_ + j]);
            C_(i,j) = double_list[i*n_ + j];
        } 
    }

}

void AttitudeControlSS::setDMatrix(std::vector<double> double_list) {
    std::cout << "Setting D matrix."<< std::endl;
    for (int i = 0; i < l_; i++) {
        for(int j = 0; j < m_; j++){
            //ROS_INFO("(%d, %d) = %f", i,j, double_list[i*n_ + j]);
            D_(i,j) = double_list[i*m_ + j];
        } 
    }

}

void AttitudeControlSS::setKctlMatrix(std::vector<double> double_list) {
    std::cout << "Setting Kctl matrix."<< std::endl;
    for (int i = 0; i < m_; i++) {
        for(int j = 0; j < n_; j++){
            //ROS_INFO("(%d, %d) = %f", i,j, double_list[i*n_ + j]);
            Kctl_(i,j) = double_list[i*n_ + j];
        } 
    }

}

void AttitudeControlSS::setKobsMatrix(std::vector<double> double_list) {
    std::cout << "Setting Kobs matrix."<< std::endl;
    for (int i = 0; i < n_; i++) {
        for(int j = 0; j < l_; j++){
            //ROS_INFO("(%d, %d) = %f", i,j, double_list[i*n_ + j]);
            Kobs_(i,j) = double_list[i*l_ + j];
        } 
    }

}

void AttitudeControlSS::setKiGain(double Ki) {
    std::cout << "Setting Ki gain."<< std::endl;
    Ki_ = Ki;
}

void AttitudeControlSS::setDeadZone(double dz) {
    std::cout << "Setting dead zone gain."<< std::endl;
    dead_zone_ = dz;
}

void AttitudeControlSS::setSaturation(double sat) {
    std::cout << "Setting saturation."<< std::endl;
    saturation_ = sat;
}


void AttitudeControlSS::observerCompute(float u, float *y)
{
    //std::cout << "Entering observer computation" << std::endl;
    MatrixXf y_meas;
    y_meas.resize(2,1);
    y_meas << y[0], y[1];
    y_mv_[0] = y[0];
    y_mv_[1] = y[1];
    // D = 0 -> y(k) = C*x
    y_est_ = C_ * states_;
    //std::cout << "Computed y_est for the first time." << std::endl;
    // x(k+1) = A*x + B*u + Kobs*(y_m - y_est)
    states_ = A_ * states_ + B_ * u + Kobs_ * (y_meas - y_est_);
    //std::cout << "Computed state equ." << std::endl;
    //y_est_ = C_ * states_;
    //std::cout << "Computed y_est for the second time." << std::endl;
    //std::cout << "Assigned result." << std::endl;
}

float AttitudeControlSS::stateSpaceCtlCompute(float y_sp, float y_mv, float u_ff)
{
    float ui, kui, ux, u, error;
    MatrixXf u_st;

    //std::cout << "Compute integral part" << std::endl;
    // compute integral part
    error = y_sp - y_mv;
    if (error > 0) {
        if (error < dead_zone_)
            error = 0.0;
        else
            error = error - dead_zone_;
    }
    else {
        if (error > -dead_zone_)
            error = 0.0;
        else
            error = error + dead_zone_;
    }
    ui = ui_old_ + error;
    kui = -Ki_ * ui;

    //std::cout << "Compute state part" << std::endl;
    // classic state space ctl
    u_st = -Kctl_ * states_;
    ux = u_st(0,0);

    //std::cout << "Compute total part" << std::endl;
    // total signal
    u = kui + ux + u_ff;

    //std::cout << "Check saturation" << std::endl;
    // saturation
    if (u > saturation_)
    {
        u = saturation_;
        ui = ui_old_;  // anti-wind up
    }
    else if (u < -saturation_)
    {
        u = -saturation_;
        ui = ui_old_; // anti-wind up
    }

    //std::cout << "Save old values" << std::endl;
    ui_old_ = ui;
    u_ = u;
    ui_ = kui;
    us_ = ux;
    y_sp_ = y_sp;
    return u;
}

custom_msgs::SSController AttitudeControlSS::generateMsg()
{
    //std::cout << "Generating messages" << std::endl;
    ss_ctl_msg_.ref = y_sp_;
    ss_ctl_msg_.meas[0] = y_mv_[0];
    ss_ctl_msg_.meas[1] = y_mv_[1];
    //std::cout << "Generating messages - y_est" << std::endl;
    ss_ctl_msg_.est[0] = y_est_(0,0);
    ss_ctl_msg_.est[1] = y_est_(1,0);
    //std::cout << "Generating messages - state" << std::endl;
    ss_ctl_msg_.state[0] = states_(0,0);
    ss_ctl_msg_.state[1] = states_(1,0);
    ss_ctl_msg_.state[2] = states_(2,0);
    ss_ctl_msg_.state[3] = states_(3,0);
    ss_ctl_msg_.I = ui_;
    ss_ctl_msg_.S = us_;
    ss_ctl_msg_.U = u_;
    ss_ctl_msg_.header.stamp = ros::Time::now();

    return ss_ctl_msg_;

}

void AttitudeControlSS::initParams()
{

}

void AttitudeControlSS::run()
{

}
