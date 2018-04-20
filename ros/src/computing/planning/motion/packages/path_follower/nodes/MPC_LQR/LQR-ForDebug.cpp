#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
#include "LQR.h"

using namespace std;
using namespace Eigen;

vlr::MPCController* mpc=NULL;

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "controller");  
  try {
    mpc = new vlr::MPCController;
  }
  catch(vlr::Ex<>& e) {
    std::cout << e.what() << std::endl;
  }

  try {
    mpc->run();
  }
  catch(vlr::Ex<>& e) {
    std::cout << e.what() << std::endl;
  }
  delete mpc;
  return 0;
}


namespace vlr {
Matrix<double, NUM_STATES, 1> s_close;
MPCController::MPCController() : nh_(), received_applanix_state_(false),received_trajectory_(false), 
        start_time_flag_(true), flag(0), s_flag(0),mpc_flag(true), plan_flag(0), next_flag(0), mpc_pub_flag(0),
        run_controller_(false) {
p_hertz = 20;
//p_horizon = 20;

p_Q.setZero();
p_R.setZero();
p_R_delta.setZero();
p_vs.set_mkz_params();
changeParams();
des_states_.resize(NUM_STATES, p_horizon + 1);
controls_.resize(NUM_CONTROLS, p_horizon);
controls_.setZero();
//cout<<"controls_(0,0) = "<<controls_(0,0)<<"controls_(1,0) = "<<controls_(1,0)<<endl;

errors_.setZero();
ctl_err_vel_.setZero();
ctl_err_int_.setZero();
ctl_err_.setZero();
vel_err_int_ = 0;
last_dtheta_ = 0;

current_state_sub_ = nh_.subscribe("state_estimate", 5, &MPCController::currentstateHandler, this);
trajectory_sub_ = nh_.subscribe("final_trajectory", 5, &MPCController::trajectoryHandler, this);
steering_sub_ = nh_.subscribe("/vehicle/steering_report", 5, &MPCController::steeringHandler, this);

controller_target_pub_  = nh_.advertise<path_follower_msgs::ControllerTarget>("ControllerTarget", 1);
steering_pub_ = nh_.advertise<dbw_mkz_msgs::SteeringCmd>("vehicle/steering_cmds",1);
twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("twist_cmds", 1);
s_next_pub_ = nh_.advertise<path_follower_msgs::ApplanixPose>("s_next_", 30);
s_close_pub_ = nh_.advertise<path_follower_msgs::ApplanixPose>("s_close_",30);
des_traj_pub_ =nh_.advertise<path_follower_msgs::traj_plan>("des_traj_",30);
applanix_pub_ = nh_.advertise<path_follower_msgs::ApplanixPose>("Applanix_",30);
time_pub_ = nh_.advertise<path_follower_msgs::MpcTime>("Time_",30);
//err_states_pub_ = nh_.advertise<path_follower::ApplanixPose>("err_states_",10);
}

MPCController::~MPCController() {
}

void MPCController::run() {
  ros::Rate r(p_hertz);
  while (ros::ok()) {
    controlLoop();
    ros::spinOnce();
    r.sleep();
  }
}

// parameter change callback (not invoked)
void MPCController::changeParams() {
 /* p_q_lon = 13;
  p_q_lat = 13;
  p_q_theta = 2.0;
  p_q_u = 0.1;
  p_q_v = 0.1;
  p_q_theta_dot = 0.01;

  p_r_udot = 0.50;
  p_r_delta =1.50;
  p_rd_udot =2.00;
  p_rd_delta =2.00;*/
  p_q_lon = 1.5;
  p_q_lat = 1.5;
  p_q_theta = 0.01;
  p_q_u = 0.01;
  p_q_v = 0.01;
  p_q_theta_dot = 1;

  p_r_udot = 0.01;
  p_r_delta =0.5;
  p_rd_udot =0.01;
  p_rd_delta =0.01;

  p_Q(0, 0) = p_q_lon;
  p_Q(1, 1) = p_q_lat;
  p_Q(2, 2) = p_q_theta;
  p_Q(3, 3) = p_q_u;
  p_Q(4, 4) = p_q_v;
  p_Q(5, 5) = p_q_theta_dot;

  p_R(0, 0) = p_r_udot;
  p_R(1, 1) = p_r_delta;
  p_R_delta(0, 0) = p_rd_udot;
  p_R_delta(1, 1) = p_rd_delta;
}

void MPCController::currentstateHandler(const path_follower_msgs::state_Dynamic current_state){
  applanix_.vel_north  = current_state.vx;
  applanix_.vel_east   = current_state.vy;
  applanix_.smooth_x   = current_state.X;
  applanix_.smooth_y   = current_state.Y;
  applanix_.yaw        = current_state.psi; 
  applanix_.rate_yaw   = current_state.wz;
 
  received_applanix_state_ = true;

  if (start_time_flag_ && received_applanix_state_){
    start_time = Time::current();
    cout.precision(8);
//    cout<<"initial_state="<<applanix_<<endl;
    start_time_flag_ = false;
  }


}

void MPCController::steeringHandler(const dbw_mkz_msgs::SteeringReport msg){
  steering_current_ = msg.steering_wheel_angle;
}


void MPCController::trajectoryHandler(const path_follower_msgs::Trajectory2D trajectory){
  traj_ = trajectory;
  received_trajectory_ = true;
}


// main controller loop
void MPCController::controlLoop() {
  if (!received_applanix_state_ || !received_trajectory_) {
    return;
  }
double t_start = Time::current();
// if we haven't gotten anything for a while, reset
  if (traj_.header.stamp.toSec() < applanix_.timestamp - 10.0) {
    controls_.setZero();
    ctl_err_.setZero();
    ctl_err_vel_.setZero();
    ctl_err_int_.setZero();
    vel_err_int_ = 0;
    last_dtheta_ = 0;
    received_applanix_state_ = false;
    received_trajectory_ = false;
    cout << "No commands in a while, resetting controller..." << endl;
    return;
  }
  double mpc_start;
  double mpc_end;
// compute desired control (delta, velocity)
  getState();
    applanix_.timestamp  = Time::current();
    applanix_pub_.publish(applanix_);
  getDesiredStates();

// shift controls and run MPC
  Matrix<double, NUM_CONTROLS, 1> u_prev = controls_.col(0);
  Matrix<double, NUM_STATES, Dynamic> s_pred(NUM_STATES, p_horizon + 1);
  Matrix<double, NUM_STATES, 1> s;

  controls_.block(0, 0, NUM_CONTROLS, p_horizon - 1) = controls_.block(0, 1, NUM_CONTROLS, p_horizon - 1);
  
  if(mpc_flag == true ){
    for(int i = 0; i < 1; i++) {
      //mpcLQR(state_, controls_, des_states_, u_prev, &controls_);
      //mpcLQR(err_states_, controls_, des_states_, u_prev, &controls_);
      mpcLQR(s_close, controls_, des_states_, u_prev, &controls_);  
      //controls_pub_(Controls_);
    }
    mpc_pub_flag++; 
    //publish steering cmd 
    double dt = 1.0 / p_hertz;
    //cout<<"controls_(1,0)="<<controls_(1,0);
    double steering_angle = controls_(1, 0) * p_vs.param.steering_ratio;
    //cout<<",steering_angle="<<steering_angle<<endl;
    steering_cmd_.enable = true;
    //steering_cmd_.steering_wheel_angle_cmd = min(steering_angle, p_vs.param.max_wheel_angle);
    double temp = min(steering_angle, p_vs.param.max_wheel_angle);
    steering_cmd_.steering_wheel_angle_cmd = max(temp, -p_vs.param.max_wheel_angle);
    //steering_cmd_.steering_wheel_angle_velocity = min((steering_angle - steering_current_)/dt, p_vs.param.max_wheel_rate);
    temp = min((steering_angle - steering_current_)/dt, p_vs.param.max_wheel_rate);
    steering_cmd_.steering_wheel_angle_velocity = max(temp, -p_vs.param.max_wheel_rate);
    steering_pub_.publish(steering_cmd_);

    // publish velocity cmd to compute throttle 
    twist_.twist.linear.x = des_states_(3);
    twist_pub_.publish(twist_);
  }

  else{
  // publish steering cmd 
    double dt = 1.0 / p_hertz;
    double steering_angle = 0;
  // cout<<"steering_cmd ON"<<steering_angle<<endl;
  // cout<<"controls_(1,0) = "<<controls_(1,0)<<endl;
    steering_cmd_.enable = true;
    steering_cmd_.steering_wheel_angle_cmd = steering_angle;
    steering_cmd_.steering_wheel_angle_velocity = 0;
    steering_pub_.publish(steering_cmd_);

  // publish velocity cmd to compute throttle 
    twist_.twist.linear.x = state_(3)-dt*2;
    twist_pub_.publish(twist_);
  }

// publish errors
  controller_target_.target_velocity = des_states_(3, 0);
  controller_target_.target_steering_angle = controls_(1, 0) * p_vs.param.steering_ratio;
  controller_target_.cross_track_error = -sin(des_states_(2, 0)) * (state_(0) - des_states_(0, 0)) + cos(des_states_(2, 0)) * (state_(1) - des_states_(1, 0));
  controller_target_.heading_error = (state_(2) - des_states_(2, 0));
  controller_target_.timestamp = Time::current();
  controller_target_pub_.publish(controller_target_);  

  double t_end = Time::current();
  time_.t_start = t_start;
  time_.t_end = t_end;
  time_.mpc_start = mpc_start;
  time_.mpc_end = mpc_end;
  time_pub_.publish(time_);
}

//---------------------------------------------------------------------------------
// get current state of the car (really a delta state from desired state)
void MPCController::getState() 
{  
  double cos_th = cos(state_(2)), sin_th = sin(state_(2));
  double imu_to_cg = p_vs.param.b;
  state_(0) = applanix_.smooth_x + cos(applanix_.yaw) * imu_to_cg;
  state_(1) = applanix_.smooth_y + sin(applanix_.yaw) * imu_to_cg;  
  state_(2) = applanix_.yaw;
  state_(5) = applanix_.rate_yaw;
  state_(3) = applanix_.vel_north;
  state_(4) = applanix_.vel_east + state_(5) * imu_to_cg;
}

// get desired states from trajecotry and controls using Newton's method
void MPCController::getDesiredStates() {
  double t = applanix_.timestamp, alpha;
  double ra_to_cg = p_vs.param.b;
  int j = 0;
  // find all desired positions
  // path_follower::TrajectoryPoint2D *p1, *p2;
  double dt = 1.0 / p_hertz;
  for (int i = 0; i < p_horizon + 1; i++) {
    if(i==0 && ( traj_.point[(int)traj_.point.size()-1].t + start_time) < t)
      mpc_flag = false;
    while ( (traj_.point[j + 1].t + start_time) < t && j < (int)traj_.point.size() - 2)
       j++;
//    if(flag<20){
//      cout.precision(15);
//      cout<<"tra_time="<< traj_.point[j + 1].t <<endl;
//      cout<<"start_time="<<start_time << endl;
//      cout<<"Total time="<<traj_.point[j + 1].t + start_time <<endl;
//      cout<<"t="<<t<<endl;
//      flag++;
//      cout<<"i="<<i<<",j="<<j<<endl;
//    }
    path_follower_msgs::TrajectoryPoint2D& p1 = traj_.point[j];       // TODO: Check if copy is better than reference
    path_follower_msgs::TrajectoryPoint2D& p2 = traj_.point[j + 1];
      
    while (p2.theta - p1.theta > M_PI)
      p2.theta -= 2 * M_PI;
    while (p2.theta - p1.theta < -M_PI)
      p2.theta += 2 * M_PI;
  // integrate to create a smoothed trajectory   
    alpha = (t - p1.t -start_time) / (p2.t - p1.t);
    if (alpha > 1) alpha = 1;
    if (alpha < 0) alpha = 0;
    des_states_(0, i) = (1 - alpha) * p1.x + alpha * p2.x;
    des_states_(1, i) = (1 - alpha) * p1.y + alpha * p2.y;
    des_states_(2, i) = (1 - alpha) * p1.theta + alpha * p2.theta;
    des_states_(0, i) += cos(des_states_(2, i)) * ra_to_cg;
    des_states_(1, i) += sin(des_states_(2, i)) * ra_to_cg;
    des_states_(3, i) = (1 - alpha) * p1.v + alpha * p2.v;
    des_states_(4, i) = 0.0;
    des_states_(5, i) =  des_states_(3,i)*((1 - alpha) * p1.kappa + alpha * p2.kappa);
    //  des_states_(5, i) =  (1 - alpha) * p1.kappa + alpha * p2.kappa;    //when traj.mat stores 'yaw_rate' instead of 'kappa'
    t += dt;

    //For close_traj mpc : get the initial s_close
    if(flag <1  ){
      s_close = des_states_.col(0);
      s_close(3) = des_states_(3,0);
      flag ++;
    } 

    //For initial position error debug:
    if(i==0){
      err_states_(0,0)= des_states_(0, i) - 7;
      err_states_(1,0)= des_states_(1, i) - 20;
      err_states_(2,0)= des_states_(2, i);
      err_states_(3,0)= des_states_(3, i);
      err_states_(4,0)= des_states_(4, i);
      err_states_(5,0)= des_states_(5, i);
    } 
    //

    des_traj_.x=des_states_(0,i);
    des_traj_.y=des_states_(1,i);
    des_traj_.theta=des_states_(2,i);
    des_traj_.v=des_states_(3,i);
    des_traj_.rate_yaw=des_states_(5,i);
  //if(plan_flag < 20 && mpc_pub_flag >= 1){
    des_traj_pub_.publish(des_traj_); 
  //plan_flag ++;
  //}
  }
  
  // normalize desired angles properly
  while (des_states_(2, 0) - state_(2) > M_PI)
    des_states_(2, 0) -= 2 * M_PI;
  while (des_states_(2, 0) - state_(2) < -M_PI)
    des_states_(2, 0) += 2 * M_PI;
  for (int i = 1; i < p_horizon + 1; i++) {
    while (des_states_(2, i) - des_states_(2, i - 1) > M_PI)
      des_states_(2, i) -= 2 * M_PI;
    while (des_states_(2, i) - des_states_(2, i - 1) < -M_PI)
      des_states_(2, i) += 2 * M_PI;
  }
  errors_ += 0.05 * (Rotation2D<double> (-state_(2)).toRotationMatrix() * (state_.block(0, 0, 2, 1) - des_states_.block(0, 0, 2, 1)) - errors_);
}



// model predictive control using LQR for optimization
#define NUM_EXT_STATES (NUM_STATES+2*NUM_CONTROLS)

void MPCController::mpcLQR( const Matrix<double, NUM_STATES, 1> &s0, 
                            const Matrix<double, NUM_CONTROLS, Dynamic> &u0,
                            const Matrix<double, NUM_STATES, Dynamic> &s_star, 
                            const Matrix<double, NUM_CONTROLS, 1> &u_prev, 
                            Matrix<double, NUM_CONTROLS, Dynamic> *u_out) {
  Matrix<double, NUM_STATES, NUM_STATES> A[p_horizon];
  Matrix<double, NUM_STATES, NUM_CONTROLS> B[p_horizon];
  Matrix<double, NUM_EXT_STATES, NUM_EXT_STATES> Ae, P, Q;
  Matrix<double, NUM_EXT_STATES, NUM_CONTROLS> Be;
  Matrix<double, NUM_CONTROLS, NUM_EXT_STATES> K[p_horizon];
  Matrix<double, NUM_CONTROLS, 1> g[p_horizon], u_opt;
  Matrix<double, NUM_STATES, Dynamic> s(NUM_STATES, p_horizon + 1);
  Matrix<double, NUM_EXT_STATES, 1> s_opt, q, ds;
  Matrix<double, NUM_STATES, 1> s_next;
  Matrix<double, NUM_CONTROLS, NUM_CONTROLS> Z;
  Matrix2d R;

  s.col(0) = s0;
  // initialize cost and dynamics matrices
  for (int i = 0; i < p_horizon; i++) {
    simulateRK4(s.col(i), u0.col(i), &s_next, &A[i], &B[i]);
    s.col(i + 1) = s_next;
  //  cout<<"A[]="<<endl<<A[i]<<endl;
  }

  // initialize extended dynamics matrices
  Ae.setZero();
  Ae.block(NUM_STATES + NUM_CONTROLS, NUM_STATES, NUM_CONTROLS, NUM_CONTROLS) = Matrix<double, NUM_CONTROLS, NUM_CONTROLS>::Identity();
  Be.setZero();
  Be.block(NUM_STATES, 0, NUM_CONTROLS, NUM_CONTROLS) = Matrix<double, NUM_CONTROLS, NUM_CONTROLS>::Identity();

  Q.setZero();
  Q.block(0, 0, NUM_STATES, NUM_STATES) = p_Q;
  R = Rotation2D<double> (s_star(2, p_horizon)).toRotationMatrix();
  Q.block(0, 0, 2, 2) = R.transpose() * p_Q.block(0, 0, 2, 2) * R;
  Q.block(NUM_STATES, NUM_STATES, NUM_CONTROLS, NUM_CONTROLS) = p_R_delta;
  Q.block(NUM_STATES + NUM_CONTROLS, NUM_STATES, NUM_CONTROLS, NUM_CONTROLS) = -p_R_delta;
  Q.block(NUM_STATES, NUM_STATES + NUM_CONTROLS, NUM_CONTROLS, NUM_CONTROLS) = -p_R_delta;
  Q.block(NUM_STATES + NUM_CONTROLS, NUM_STATES + NUM_CONTROLS, NUM_CONTROLS, NUM_CONTROLS) = p_R_delta;

  s_opt.block(0, 0, NUM_STATES, 1) = s_star.col(p_horizon) - s.col(p_horizon);
  s_opt.block(NUM_STATES, 0, NUM_CONTROLS, 1) = -u0.col(p_horizon - 1);
  s_opt.block(NUM_STATES + NUM_CONTROLS, 0, NUM_CONTROLS, 1) = -u0.col(p_horizon - 2);

  // Ricatti recursion
  P = Q;
  q = -Q * s_opt;
  for (int i = p_horizon - 1; i >= 0; i--) {
    R = Rotation2D<double> (s_star(2, i)).toRotationMatrix();
    Q.block(0, 0, 2, 2) = R.transpose() * p_Q.block(0, 0, 2, 2) * R;

    s_opt.block(0, 0, NUM_STATES, 1) = s_star.col(i) - s.col(i);
    if (i >= 1) 
      s_opt.block(NUM_STATES, 0, NUM_CONTROLS, 1) = -u0.col(i - 1);
    else 
      s_opt.block(NUM_STATES, 0, NUM_CONTROLS, 1) = -u_prev;
    if (i >= 2) 
      s_opt.block(NUM_STATES + NUM_CONTROLS, 0, NUM_CONTROLS, 1) = -u0.col(i - 2);
    else 
      s_opt.block(NUM_STATES + NUM_CONTROLS, 0, NUM_CONTROLS, 1) = -u_prev;
    
    u_opt = -u0.col(i);
    Ae.block(0, 0, NUM_STATES, NUM_STATES) = A[i];
    Be.block(0, 0, NUM_STATES, NUM_CONTROLS) = B[i];
    Z = (p_R + Be.transpose() * P * Be).inverse();
    K[i] = -Z * Be.transpose() * P * Ae;
    g[i] = -Z * (Be.transpose() * q - p_R * u_opt);
    P = Q + Ae.transpose() * P * Ae + Ae.transpose() * P * Be * K[i];
    P = 0.5 * (P + P.transpose());
    q = (Ae + Be * K[i]).transpose() * q - K[i].transpose() * p_R * u_opt - Q * s_opt;
  }
 
  // simulate forward
  s_next = s0;
  for (int i = 0; i < p_horizon; i++) {
    s_next_.smooth_x  = s_next(0,0);
    s_next_.smooth_y  = s_next(1,0);
    s_next_.yaw       = s_next(2,0);  
    s_next_.rate_yaw  = s_next(5,0);
    s_next_.vel_north = s_next(3,0);
    s_next_.vel_east  = s_next(4,0);
    s_next_.timestamp = Time::current();
//    if(next_flag<20 && mpc_pub_flag >= 10){
    s_next_pub_.publish(s_next_);
//    next_flag++;}

    ds.block(0, 0, NUM_STATES, 1) = s_next - s.col(i);
    if (i >= 1) 
      ds.block(NUM_STATES, 0, NUM_CONTROLS, 1) = u_out->col(i - 1) - u0.col(i - 1);
    else 
      ds.block(NUM_STATES, 0, NUM_CONTROLS, 1).setZero();
    if (i >= 2) 
      ds.block(NUM_STATES + NUM_CONTROLS, 0, NUM_CONTROLS, 1) = u_out->col(i - 2) - u0.col(i - 2);
    else 
      ds.block(NUM_STATES + NUM_CONTROLS, 0, NUM_CONTROLS, 1).setZero();

    u_out->col(i) = u0.col(i) + K[i] * ds + g[i];

    (*u_out)(1, i) = max((*u_out)(1, i), -p_vs.param.max_wheel_angle);
    (*u_out)(1, i) = min((*u_out)(1, i), p_vs.param.max_wheel_angle);
    simulateRK4(s_next, u_out->col(i), &s_next);
    if(mpc_pub_flag<1){
 //     cout<<"uout="<<endl<<u_out->col(i)<<endl;
    }
    
    if(i == 0){
    s_close = s_next;
    s_close_.smooth_x  = s_next(0,0);
    s_close_.smooth_y  = s_next(1,0);
    s_close_.yaw       = s_next(2,0);  
    s_close_.vel_north = s_next(3,0);
    s_close_.vel_east  = s_next(4,0);
    s_close_.rate_yaw  = s_next(5,0);
    s_close_.timestamp = Time::current();
    s_close_pub_.publish(s_close_);
    }
  }
}

#define EPSILON 1e-5

// dynamics of the car (bicycle model with velocity/steering input)
void MPCController::dynamics( const Matrix<double, NUM_STATES, 1> &s, 
                              const Matrix<double, NUM_CONTROLS, 1> &u_, 
                              Matrix<double, NUM_STATES, 1> *s_dot, 
                              Matrix<double, NUM_STATES, NUM_STATES> *A, 
                              Matrix<double, NUM_STATES, NUM_CONTROLS> *B){
  double u = s(3), v = s(4), cos_th = cos(s(2)), sin_th = sin(s(2));
  double th_dot = s(5), u_dot = u_(0);
  double del = u_(1), tan_del = tan(u_(1)), cos_del = cos(u_(1));
  double Fyf, Fyr, Caf = p_vs.param.ftire_stiffness, 
  Car = p_vs.param.rtire_stiffness, m = p_vs.param.mass, 
  a = p_vs.param.a, b = p_vs.param.b, I = p_vs.param.iz;
// compute slip angles and lateral forces
  Fyf = -Caf * (atan2(v + th_dot * a, max(u, dgc::dgc_mph2ms(5))) - del);
  Fyr = -Car * atan2(v - th_dot * b, max(u, dgc::dgc_mph2ms(5)));

// compute derivatives
  (*s_dot)(0) = u * cos_th - v * sin_th;
  (*s_dot)(1) = u * sin_th + v * cos_th;
  (*s_dot)(2) = th_dot;

  (*s_dot)(3) = u_dot;
  (*s_dot)(4) = tan_del * (u_dot - th_dot * v) + (Fyf / cos_del + Fyr) / m - th_dot * u;
  (*s_dot)(5) = m * a / I * tan_del * (u_dot - th_dot * v) + a * Fyf / (I * cos_del) - b * Fyr / I;

// compute Jacobians (numerically) if desired
  if (A != 0) {
    Matrix<double, NUM_STATES, 1> s2 = s;
    Matrix<double, NUM_STATES, 1> s_dot1, s_dot2;
    for (int i = 0; i < NUM_STATES; i++) {
      s2(i) += EPSILON;
      dynamics(s2, u_, &s_dot1, 0, 0);
      s2(i) -= 2 * EPSILON;
      dynamics(s2, u_, &s_dot2, 0, 0);
      s2(i) += EPSILON;
      A->col(i) = (s_dot1 - s_dot2) / (2 * EPSILON);
    }
  }
  if (B != 0) {
    Matrix<double, NUM_CONTROLS, 1> u2 = u_;
    Matrix<double, NUM_STATES, 1> s_dot1, s_dot2;
    for (int i = 0; i < NUM_CONTROLS; i++) {
      u2(i) += EPSILON;
      dynamics(s, u2, &s_dot1, 0, 0);
      u2(i) -= 2 * EPSILON;
      dynamics(s, u2, &s_dot2, 0, 0);
      u2(i) += EPSILON;
      B->col(i) = (s_dot1 - s_dot2) / (2 * EPSILON);
    }
  }
}


void MPCController::simulateEuler(const Matrix<double, NUM_STATES, 1> &s, const Matrix<double, NUM_CONTROLS, 1> &u, Matrix<double, NUM_STATES, 1> *s_next,
    Matrix<double, NUM_STATES, NUM_STATES> *A, Matrix<double, NUM_STATES, NUM_CONTROLS> *B) {
  Matrix<double, NUM_STATES, 1> s_dot;
  dynamics(s, u, &s_dot, A, B);
  (*s_next) = s + s_dot / p_hertz;

  if (A) {
    (*A) /= p_hertz;
    (*A) += Matrix<double, NUM_STATES, NUM_STATES>::Identity();
  }

  if (B) (*B) /= p_hertz;
}


void MPCController::simulateRK4(const Matrix<double, NUM_STATES, 1> &s, const Matrix<double, NUM_CONTROLS, 1> &u, Matrix<double, NUM_STATES, 1> *s_next,
    Matrix<double, NUM_STATES, NUM_STATES> *A, Matrix<double, NUM_STATES, NUM_CONTROLS> *B) {
  Matrix<double, NUM_STATES, 1> k1, k2, k3, k4;
  double dt = 1 / p_hertz;

  dynamics(s, u, &k1);
  dynamics(s + 0.5 * dt * k1, u, &k2);
  dynamics(s + 0.5 * dt * k2, u, &k3);
  dynamics(s + dt * k3, u, &k4);
  (*s_next) = s + dt * (k1 / 6.0 + k2 / 3.0 + k3 / 3.0 + k4 / 6.0);

// compute Jacobians (numerically) if desired
  if (A != 0) {
    Matrix<double, NUM_STATES, 1> s2 = s;
    Matrix<double, NUM_STATES, 1> sn1, sn2;
    for (int i = 0; i < NUM_STATES; i++) {
      s2(i) += EPSILON;
      simulateRK4(s2, u, &sn1, 0, 0);
      s2(i) -= 2 * EPSILON;
      simulateRK4(s2, u, &sn2, 0, 0);
      s2(i) += EPSILON;
      A->col(i) = (sn1 - sn2) / (2 * EPSILON);
    }
  }

  if (B != 0) {
    Matrix<double, NUM_CONTROLS, 1> u2 = u;
    Matrix<double, NUM_STATES, 1> sn1, sn2;
    for (int i = 0; i < NUM_CONTROLS; i++) {
      u2(i) += EPSILON;
      simulateRK4(s, u2, &sn1, 0, 0);
      u2(i) -= 2 * EPSILON;
      simulateRK4(s, u2, &sn2, 0, 0);
      u2(i) += EPSILON;
      B->col(i) = (sn1 - sn2) / (2 * EPSILON);
    }
  }
}


template <class T> void MPCController::getParam(std::string key, T& var) {
   if(!nh_.getParam(key, var)) {
    throw VLRException("Cannot read parameter " + key + std::string("."));
  }
}
} //namespace vlr