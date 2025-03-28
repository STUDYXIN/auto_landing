#include "optimize_estimation/imu_ekf.h"

IMUEKF::IMUEKF(ros::NodeHandle &nh)
{
    nh_=nh;
    ROS_INFO("Start initialize!!");
    Initialize();
}

void IMUEKF::Initialize()
{
    system_state_ = SystemState::INITIAL;

    //ROS
    //in callback function , we cannot use ConstPtr be the function value.
    // cmd_sub_ = nh_.subscribe<mavros_msgs::AttitudeTarget>("/airsim_node/drone_1/angle_rate_throttle_enu",1,&IMUEKF::CmdCallbackFunc,this);
    // t_a_sub_ = nh_.subscribe<std_msgs::Float64>("/airsim_node/rone_d1/t_a",1,&IMUEKF::TaCallbackFunc,this);
    // odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/odom_local_enu",1,&IMUEKF::odomCallbackFunc,this);
    // imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/airsim_node/drone_1/imu/imu_enu",1,&IMUEKF::IMUCallbackFunc,this);

    cmd_sub_ = nh_.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",1,&IMUEKF::CmdCallbackFunc,this);
    // t_a_sub_ = nh_.subscribe<std_msgs::Float64>("/airsim_node/drone_1/t_a",1,&IMUEKF::TaCallbackFunc,this);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom",1,&IMUEKF::odomCallbackFunc,this);
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data",1,&IMUEKF::IMUCallbackFunc,this);

    ekf_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/imu_ekf_pose",1);
    p_error_pub_ = nh_.advertise<std_msgs::Float64>("/p_error",1);
    q_error_pub_ = nh_.advertise<std_msgs::Float64>("/q_error",1);
    only_imu_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/imu_pose",1);
    is_pub_pose_ = true;

    //init value
    state_pv_ = Eigen::VectorXd::Zero(6);
    state_pv_imu_ = Eigen::VectorXd::Zero(6);
    state_q_=Eigen::Vector4d::Zero();
    state_q_(0)=1; // wxyz
    state_q_imu_ = Eigen::Vector4d::Zero();
    state_q_imu_(0) = 1;

    acc_last_ = Eigen::Vector3d::Zero();
    gyro_last_ = Eigen::Vector3d::Zero();
    curr_tw_ = Eigen::Vector4d::Zero();
    last_tw_ = Eigen::Vector4d::Zero();
    T_a_=1.0;
    t_last_=0.0;
    have_ta_ = true;
    have_plot_ = false;

    init_p_ = Eigen::Vector3d::Zero();
    init_v_ = Eigen::Vector3d::Zero();
    init_q_=Eigen::Vector4d::Zero();
    init_w_=Eigen::Vector3d::Zero();
    have_init_odom_ = false;

    acc_meas_ = Eigen::Vector3d::Zero();
    gyro_meas_ = Eigen::Vector3d::Zero();
    have_imu_ = false;
    gw_<<0.0,0.0,-9.78;

    double sigma2 = 1.0;
    acc_meas_covirance_(0,0) = sigma2;
    acc_meas_covirance_(1,1) = sigma2;
    acc_meas_covirance_(2,2) = sigma2;

    p_errors_.clear();
    q_errors_.clear();
    T_a_list_.clear();

    //Kalman
    P_last_  = Eigen::Matrix4d::Identity()*0;
}

void IMUEKF::CmdCallbackFunc(const mavros_msgs::AttitudeTarget msg)
{
    curr_tw_(0) = msg.thrust * T_a_;
    curr_tw_(1) = msg.body_rate.x;
    curr_tw_(2) = msg.body_rate.y;
    curr_tw_(3) = msg.body_rate.z;
    if(system_state_ == SystemState::INITIAL)
    {
        if(have_init_odom_==false||have_ta_==false||have_imu_==false)
        {
            //no init odom, cannot initilaze
            ROS_WARN("%d, %d, %d",have_init_odom_,have_ta_,have_imu_);
            return;
        }
        // initilize something i dont no
        system_state_ = SystemState::NORMAL;
        // initilize imu
        acc_last_ = acc_meas_;
        gyro_last_ = gyro_meas_;
    }
    else if(system_state_ == SystemState::NORMAL)
    {
        t_now_ = msg.header.stamp.toSec();
        //Do EKF
        ROS_INFO("Lets do some ekf!!"); 
        EKF_Pose_Estimation();
            // ROS_WARN("111111111111111111111");
        PublishEKFPose();
        record_error();
        // if(is_pub_pose_)
        // {

        // }

        if((ros::Time::now().toSec() - start_time_) > 120.0){
            if(!have_plot_){
                have_plot_ = true;
                plt::figure();
                plt::plot(time_, p_errors_, {{"color", "black"}, {"marker", "o"}});
                // plt::plot(time_, q_errors_);
                // plt::plot(time_, T_a_list_);
                plt::ylabel("油门归一化系数");
                plt::axis("equal");
                plt::legend();
                plt::grid(true);
                plt::xlabel("t(s)");

                plt::show();
            }
            
        }
        
        // t_last_ = t_now_;
    }
    else if(system_state_ == SystemState::ERROR)
    {
        ROS_ERROR("ERROR!!!!!!!!!!!!!!!!");
    }
    last_tw_ = curr_tw_;
    t_last_ = msg.header.stamp.toSec();
}

void IMUEKF::IMUCallbackFunc(const sensor_msgs::Imu msg)
{
    have_imu_ = true;
    acc_meas_(0) = msg.linear_acceleration.x;
    acc_meas_(1) = msg.linear_acceleration.y;
    acc_meas_(2) = msg.linear_acceleration.z;

    gyro_meas_(0) = msg.angular_velocity.x;
    gyro_meas_(1) = msg.angular_velocity.y;
    gyro_meas_(2) = msg.angular_velocity.z;
}

void IMUEKF::odomCallbackFunc(const nav_msgs::Odometry msg)
{
    init_p_(0) = msg.pose.pose.position.x;
    init_p_(1) = msg.pose.pose.position.y;
    init_p_(2) = msg.pose.pose.position.z;

    init_v_(0) = msg.twist.twist.linear.x;
    init_v_(1) = msg.twist.twist.linear.y;
    init_v_(2) = msg.twist.twist.linear.z;

    init_q_(0) = msg.pose.pose.orientation.w;
    init_q_(1) = msg.pose.pose.orientation.x;
    init_q_(2) = msg.pose.pose.orientation.y;
    init_q_(3) = msg.pose.pose.orientation.z;

    init_w_(0) = msg.twist.twist.angular.x;
    init_w_(1) = msg.twist.twist.angular.y;
    init_w_(2) = msg.twist.twist.angular.z;
    state_z_ = init_p_(2);
    if(have_init_odom_) 
    {
        return;
    }
    have_init_odom_ = true;
    
    start_time_ = ros::Time::now().toSec();
    state_pv_.segment(0, 3) = init_p_;
    state_pv_.segment(3, 3) = init_v_;
    state_q_ = init_q_;
    gyro_last_ = init_w_;
    state_pv_imu_.segment(0,3) = init_p_;
    state_pv_imu_.segment(3,3) = init_v_;
    state_q_imu_ = init_q_;
}



void IMUEKF::EKF_Pose_Estimation()
{
    // UpdateStatePV();
    UpdateStatePV_dyna();
    UpdateStatePV();
    //TODO EKF

    //Predicit
    Eigen::Matrix4d P_pred;
    Eigen::Vector4d x_pred;
    //TODO : calc F and Q'
    //TODO : calc x_pred (x_pred should be normlized)
    double dt = t_now_-t_last_;
    Eigen::Matrix4d Fq = Eigen::Matrix4d::Identity();
    Eigen::MatrixXd Gq(4, 3);
    Eigen::Matrix4d Qq = Eigen::Matrix4d::Identity() * 1e-9;
    double qw = state_q_(0), qx = state_q_(1), qy = state_q_(2), qz = state_q_(3);
    Gq << -qx, -qy, -qz, qw, -qz, qy, qz, qw, -qx, -qy, qx, qw;
    Gq = Gq * dt / 2;
    x_pred = Fq * state_q_ + Gq * last_tw_.segment(1, 3);
    x_pred.normalize();

    //P_pred = Fq*P_last_*Fq.transpose()+Gq*Qq*Gq.transpose();//Gq*Qq*Gq.transpose();
    P_pred = Fq*P_last_*Fq.transpose()+Qq;//Gq*Qq*Gq.transpose();

    //Kalman Update
    //calc H :
    double q0,q1,q2,q3;// wxyz
    Eigen::Matrix<double,3,4> H;
    q0 = state_q_(0);
    q1 = state_q_(1);
    q2 = state_q_(2);
    q3 = state_q_(3);
    // H << -q2 , q3 , -q0 , q1,
    //            q1 , q0 , q3 , q2,
    //            q0 , -q1 , -q2 , q3;
    H << -q2 , q3 , -q0 , q1,
               q1 , q0 , q3 , q2,
               0 , -2*q1 , -2*q2 , 0;
    H=2*H;
    //Kalman zengyi K
    Eigen::Matrix<double,4,3> K;
    Eigen::Matrix<double,3,3> T;

    T = H*P_pred*H.transpose()+acc_meas_covirance_;
    K=P_pred*H.transpose()*T.inverse();

    //update
    Eigen::Matrix4d P_est;
    Eigen::Vector4d x_est;
    Eigen::Vector3d y,gk;
    //y : the real measurement
    y(0) = acc_meas_(0);
    y(1) = acc_meas_(1);
    y(2) = acc_meas_(2);
    //gk : in the measurement function with x_pred
    gk(0) = 2*x_pred(1)*x_pred(3) - 2*x_pred(0)*x_pred(2);
    gk(1) = 2*x_pred(2)*x_pred(3) - 2*x_pred(0)*x_pred(1);
    gk(2) = x_pred(0)*x_pred(0) - x_pred(1)*x_pred(1) - x_pred(2)*x_pred(2) + x_pred(3)*x_pred(3);
    P_est=(Eigen::Matrix4d::Identity() - K*H)*P_pred;
    x_est=x_pred + K*(y - gk);
    x_est.normalize();
    //update the optimize estimation
    state_q_ = x_est;
    P_last_ = P_est;
    
}

void IMUEKF::UpdateStatePV()
{
    //update pv by imu
    double dt = t_now_ - t_last_;

    Eigen::Quaterniond delta_q;
    Eigen::Vector3d gyro_now_ = 0.5*(gyro_last_+gyro_meas_);
    gyro_last_ = gyro_now_;
    delta_q.w()=1.0;
    delta_q.x()=gyro_now_.x()*dt*0.5;
    delta_q.y()=gyro_now_.y()*dt*0.5;
    delta_q.z()=gyro_now_.z()*dt*0.5;

    Eigen::Quaterniond Qwb(state_q_imu_(0),state_q_imu_(1),state_q_imu_(2),state_q_imu_(3));

    Eigen::Matrix3d R_now = Qwb.normalized().toRotationMatrix();
    Eigen::Vector3d acc_w_now = R_now*acc_meas_+gw_;

    Eigen::Vector3d acc = 0.5*(acc_w_now+acc_last_);
    acc_last_ = acc_w_now;
    //Eigen::Vector3d acc = 0.5*(R_now*(acc_last+g)+R_last*(acc_meas+g));

    Qwb = Qwb.normalized()*delta_q.normalized();
    state_pv_imu_.segment(3,3) = state_pv_imu_.segment(3,3)+ acc* dt;
    state_pv_imu_.segment(0,3) = state_pv_imu_.segment(0,3) +state_pv_imu_.segment(3,3) * dt + 0.5* acc *dt*dt;
    state_q_imu_(0)=Qwb.w();
    state_q_imu_(1)=Qwb.x();
    state_q_imu_(2)=Qwb.y();
    state_q_imu_(3)=Qwb.z();
}

void IMUEKF::UpdateStatePV_dyna()
{
    //update pv by dyna
    double dt = t_now_ - t_last_;
    ROS_WARN("%f",dt);

    Eigen::MatrixXd Fp = Eigen::MatrixXd::Identity(6, 6);
    Fp.block(0, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * dt;
    Eigen::MatrixXd Gp = Eigen::MatrixXd::Zero(6, 2);
    Eigen::Quaterniond q(state_q_(0),state_q_(1),state_q_(2),state_q_(3));
    Gp.block(3, 0, 3, 1) = q.normalized().toRotationMatrix().block(0, 2, 3, 1);
    Gp(5, 1) = -1;
    Gp = Gp * dt;
    Eigen::Vector2d u_pv(last_tw_(0), 9.805);
    state_pv_ = Fp * state_pv_ + Gp * u_pv;
}

void IMUEKF::PublishEKFPose()
{
    geometry_msgs::PoseStamped pubpose;
    pubpose.header.frame_id = "odom";
    pubpose.header.stamp = ros::Time::now();

    pubpose.pose.orientation.w = state_q_(0);
    pubpose.pose.orientation.x = state_q_(1);
    pubpose.pose.orientation.y = state_q_(2);
    pubpose.pose.orientation.z = state_q_(3);

    pubpose.pose.position.x = state_pv_(0);
    pubpose.pose.position.y = state_pv_(1);
    // pubpose.pose.position.z = state_pv_(2);
    pubpose.pose.position.z = state_z_;

    ekf_pose_pub_.publish(pubpose);

    //pub  imu pose
    geometry_msgs::PoseStamped pubimupose;
    pubimupose.header.frame_id = "odom";
    pubimupose.header.stamp = ros::Time::now();

    pubimupose.pose.orientation.w = state_q_imu_(0);
    pubimupose.pose.orientation.x = state_q_imu_(1);
    pubimupose.pose.orientation.y = state_q_imu_(2);
    pubimupose.pose.orientation.z = state_q_imu_(3);

    pubimupose.pose.position.x = state_pv_imu_(0);
    pubimupose.pose.position.y = state_pv_imu_(1);
    //pubpose.pose.position.z = state_pv_(2);
    pubimupose.pose.position.z = state_z_;

    only_imu_pose_pub_.publish(pubimupose);

    
}

void IMUEKF::record_error()
{
    double pe = (state_pv_.segment(0, 3) - init_p_).norm();
    Eigen::Quaterniond q_true(init_q_(0), init_q_(1), init_q_(2), init_q_(3));
    Eigen::Quaterniond q_est(state_q_(0), state_q_(1), state_q_(2), state_q_(3));
    Eigen::Matrix3d diff_r = q_true.toRotationMatrix() * q_est.toRotationMatrix().transpose();
    double qe = acos((diff_r.trace() - 1) / 2) * 180 / M_PI;
    p_errors_.push_back(pe);
    q_errors_.push_back(qe);
    T_a_list_.push_back(T_a_);
    time_.push_back(ros::Time::now().toSec() - start_time_);

    std_msgs::Float64 msg;
    msg.data = pe;
    p_error_pub_.publish(msg);
    msg.data = qe;
    q_error_pub_.publish(msg);
}
