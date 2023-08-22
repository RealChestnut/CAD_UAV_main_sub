//2022.05.16 Coaxial-Octorotor version
//2022.06.23 Ground Station Application
//2022.08.XX DOB (Disturbance Observer) Application
//2022.09.05 ESC (Extremum Seeking Control) Application
//2022.09.21 Controller mode selection Application

#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <std_msgs/String.h>
#include <vector>
#include <cmath>
#include <cstdio>
#include <chrono>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/Imu.h>
//#include "FAC_MAV/FAC_MAV_ctrler.h"

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "FAC_MAV/ArmService.h" //ASDF
#include "FAC_MAV/KillService.h" //ASDF
#include "FAC_MAV/PosCtrlService.h" //ASDF
#include "FAC_MAV/HoverService.h" //ASDF
#include "FAC_MAV/FAC_HoverService.h" //ASDF

#include "nav_msgs/Odometry.h"

double freq=200;//controller loop frequency
double pwm_freq=417.3;//pwm signal frequency

std::chrono::duration<double> delta_t;
int16_t Sbus[8];
int16_t PWM_d;
int16_t loop_time;
std_msgs::Int16MultiArray PWMs_cmd;
std_msgs::Int32MultiArray PWMs_val;
std_msgs::Float32MultiArray Force_comb;

sensor_msgs::JointState rac_servo_value;
sensor_msgs::Imu imu;
geometry_msgs::Quaternion imu_quaternion;
geometry_msgs::Vector3 imu_rpy;
geometry_msgs::Vector3 imu_ang_vel;
geometry_msgs::Vector3 imu_lin_acc;
geometry_msgs::Vector3 angle_d;
geometry_msgs::Vector3 pos;
geometry_msgs::Vector3 t265_lin_vel;
geometry_msgs::Vector3 t265_ang_vel;
geometry_msgs::Vector3 lin_vel;
geometry_msgs::Vector3 prev_lin_vel;
geometry_msgs::Vector3 ang_vel;
geometry_msgs::Quaternion t265_quat;
geometry_msgs::Quaternion rot;
geometry_msgs::Quaternion desired_value;
geometry_msgs::Vector3 desired_pos;
geometry_msgs::Vector3 F_total;
geometry_msgs::Vector3 torque_d;
geometry_msgs::Vector3 force_d;
std_msgs::Float32MultiArray force_cmd;
geometry_msgs::Vector3 desired_lin_vel;
geometry_msgs::Vector3 t265_att;
geometry_msgs::Vector3 filtered_angular_rate;
geometry_msgs::Vector3 lin_acl;
std_msgs::Float32 altitude_d;
std_msgs::Float32 battery_voltage_msg;
std_msgs::Float32 battery_real_voltage;
std_msgs::Float32 dt;

//
std_msgs::Float32MultiArray Force_cmd; 
std_msgs::Float32MultiArray Servo_sub_cmd;
std_msgs::Float32MultiArray Servo_main_cmd;
std_msgs::Int32MultiArray Sbus_data_plot;
std_msgs::Bool kill_sub;
std_msgs::Bool tilt_sub;
//for time checker
//
bool servo_sw=false;
double theta11_command, theta12_command;
bool start_flag=false;
bool tilting_flag=false;

//Mode selection flag
bool attitude_mode = false;
bool velocity_mode = false;
bool position_mode = false;
bool kill_mode = true;
bool combined_mode = true; //2022_10_26
bool altitude_mode = false;
bool tilt_mode = false;
bool ESC_control = false;
//Thruster_cmd
float F1 = 0;//desired propeller 1 force
double F2 = 0;//desired propeller 2 force
double F3 = 0;//desired propeller 3 force
double F4 = 0;//desired propeller 4 force
double F5 = 0;//desired propeller 5 force
double F6 = 0;//desired propeller 6 force
double F7 = 0;//desired propeller 7 force
double F8 = 0;//desired propeller 8 force

//Global : XYZ  Body : xyz
double e_r_i = 0;//roll error integration
double e_p_i = 0;//pitch error integration
double e_y_i = 0;//yaw error integration
double e_X_i = 0;//X position error integration
double e_Y_i = 0;//Y position error integration
double e_Z_i = 0;//Z position error integration
double e_X_dot_i = 0;//X velocity error integration
double e_Y_dot_i = 0;//Y velocity error integration
double e_Z_dot_i = 0;//Z velocity error integration

double tau_r_d = 0;//roll  desired torque (N.m)
double tau_p_d = 0;//pitch desired torque(N.m)
double tau_y_d = 0;//yaw desired torque (N.m)

double Thrust_d = 0;//altitiude desired thrust(N)
//ud_cmd

double r_d = 0;//desired roll angle
double p_d = 0;//desired pitch angle
double y_d = 0;//desired yaw angle
double y_d_tangent = 0;//yaw increment tangent
double T_d = 0;//desired thrust

//Desired Global position
double X_d = 0;//desired X position
double Y_d = 0;//desired Y position
double Z_d = 0;//desired altitude
double X_d_base = 0;//initial desired X position
double Y_d_base = 0;//initial desired Y position
double Z_d_base = 0;//initial desired Z position

//Global Desired Global velocity
double X_dot_d = 0;
double Y_dot_d = 0;
double Z_dot_d = 0;

//Global desired acceleration
double X_ddot_d = 0;
double Y_ddot_d = 0;
double Z_ddot_d = 0;

double alpha = 0;
double beta = 0;

//Body desired force
double F_xd = 0;
double F_yd = 0;
double F_zd = 0;

//Yaw safety
double yaw_prev = 0;
double yaw_now = 0;
double base_yaw = 0;
int yaw_rotate_count = 0;
//--------------------------------------------------------

//General dimensions

static double r_arm = 0.109;// m // diagonal length between thruster : 218mm;
static double l_servo = 0.015;
static double mass = 6; //combined drone mass //4.8
static double m1 =3.3; //master drone mass 22.10.27
static double m2 =2.7; //slave drone mass  22.10.27
static double frame_l = 0.32; //(cm) frame length 22.10.27
static double diag = 0.32/sqrt(2); // (cm) diagonal frame length 22.10.27
static double m_a = m1/(m1+m2); // mass ratio for master 22.10.29
static double m_b = m2/(m1+m2); // mass ratio for slave 22.10.29

//Propeller constants(DJI E800(3510 motors + 620S ESCs))
static double b_over_k_ratio = 0.01;//F_i=k*(omega_i)^2, M_i=b*(omega_i)^2
//--------------------------------------------------------

//General parameters======================================

static double pi = 3.141592;//(rad)
static double g = 9.80665;//(m/s^2)

static double rp_limit = 0.3;//(rad)
static double rp_limit_tilt = 0.1;//(rad)
static double y_vel_limit = 0.01;//(rad/s)
static double y_d_tangent_deadzone = (double)0.05 * y_vel_limit;//(rad/s)
static double T_limit = 58;//(N) 
static double altitude_limit = 1;//(m)
static double XY_limit = 0.5;
static double XYZ_dot_limit=0.5;
static double XYZ_ddot_limit=1.0; //4
static double XYZ_ddot_limit_tilt=1.2;
static double alpha_beta_limit=1;
static double hardware_servo_limit=0.225; //0.3
static double servo_command_limit = 0.2;
static double tau_y_limit = 0.45; //0.5
static double deg2rad= pi/180.0;
static double rad2deg = 180.0/pi;
double x_c_hat=0.0;
double y_c_hat=0.0;
double z_c_hat=0.0;
//--------------------------------------------------------

//Combined Drone parameter 

float theta21_command, theta22_command;
float theta21=0, theta22=0;
float F11 = 0;//desired propeller 1 force
float F12 = 0;//desired propeller 2 force
float F13 = 0;//desired propeller 3 force
float F14 = 0;//desired propeller 4 force
float F21 = 0;//desired propeller 5 force
float F22 = 0;//desired propeller 6 force
float F23 = 0;//desired propeller 7 force
float F24 = 0;//desired propeller 8 force
//

//Control gains===========================================

//integratior(PID) limitation
double integ_limit=10;
double z_integ_limit=100;
double pos_integ_limit=10;
double vel_integ_limit=10;

//Roll, Pitch PID gains
double Par=3.5;
double Iar=0.4;
double Dar=0.5;

double Pap=3.5;
double Iap=0.4;
double Dap=0.5;

double Pa=3.5;
double Ia=0.4;
double Da=0.5;

//Yaw PID gains
double Py=2.0;
double Dy=0.1;

//Z Velocity PID gains
double Pz=16.0;
double Iz=5.0;
double Dz=15.0;

//XY Velocity PID gains
double Pv=5.0;
double Iv=0.1;
double Dv=5.0;

//Position PID gains
double Pp=3.0;
double Ip=0.1;
double Dp=5.0;

//Conventional Flight Mode Control Gains
double conv_Par, conv_Iar, conv_Dar;
double conv_Pap, conv_Iap, conv_Dap;
double conv_Py, conv_Dy;
double conv_Pz, conv_Iz, conv_Dz;
double conv_Pv, conv_Iv, conv_Dv;
double conv_Pp, conv_Ip, conv_Dp;


//Tilt Flight Mode Control Gains
double tilt_Par, tilt_Iar, tilt_Dar;
double tilt_Pap, tilt_Iap, tilt_Dap;
double tilt_Py, tilt_Dy;
double tilt_Pz, tilt_Iz, tilt_Dz;
double tilt_Pv, tilt_Iv, tilt_Dv;
double tilt_Pp, tilt_Ip, tilt_Dp;
//--------------------------------------------------------

//Servo angle=============================================
double theta11=0,theta12=0;
//--------------------------------------------------------

//Voltage=================================================
double voltage=16.0;
double voltage_old=16.0;
//--------------------------------------------------------


//-DOB----------------------------------------------------
geometry_msgs::Vector3 dhat;
double fq_cutoff=0.5;//Q filter Cut-off frequency

// Nominal MoI
double J_x = 0.01;
double J_y = 0.01;
double J_z = 0.1;

//Roll DOB
double x_r1=0;
double x_r2=0;
double x_r3=0;
double x_dot_r1=0;
double x_dot_r2=0;
double x_dot_r3=0;

double y_r1=0;
double y_r2=0;
double y_r3=0;
double y_dot_r1=0;
double y_dot_r2=0;
double y_dot_r3=0;

double dhat_r = 0;
double tautilde_r_d=0;

//Pitch DOB
double x_p1=0;
double x_p2=0;
double x_p3=0;
double x_dot_p1=0;
double x_dot_p2=0;
double x_dot_p3=0;

double y_p1=0;
double y_p2=0;
double y_p3=0;
double y_dot_p1=0;
double y_dot_p2=0;
double y_dot_p3=0;

double dhat_p=0;
double tautilde_p_d=0;

//Yaw DOB
double x_y1=0;
double x_y2=0;
double x_y3=0;
double x_dot_y1=0;
double x_dot_y2=0;
double x_dot_y3=0;

double y_y1=0;
double y_y2=0;
double y_y3=0;
double y_dot_y1=0;
double y_dot_y2=0;
double y_dot_y3=0;

double tautilde_y_d=0;
//--------------------------------------------------------
//Function------------------------------------------------
template <class T>
T map(T x, T in_min, T in_max, T out_min, T out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void rpyT_ctrl();
void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des);
float Force_to_PWM(float F);
void jointstateCallback(const sensor_msgs::JointState& msg);
void jointstate_sub_Callback(const sensor_msgs::JointState& msg);
void imu_Callback(const sensor_msgs::Imu& msg);
sensor_msgs::JointState servo_msg_create(float rr1, float rp1/*,float rr2, float rp2*/);
void sbusCallback(const std_msgs::Int16MultiArray::ConstPtr& array);
void batteryCallback(const std_msgs::Int16& msg);
void posCallback(const geometry_msgs::Vector3& msg);
void rotCallback(const geometry_msgs::Quaternion& msg);
void filterCallback(const sensor_msgs::Imu& msg);
void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void setCM();
void setCM_comb();//combined Control matrix
void publisherSet();
int32_t pwmMapping(int32_t pwm);
void pwm_Command(int16_t pwm11, int16_t pwm12, int16_t pwm13, int16_t pwm14, int16_t pwm15, int16_t pwm16, int16_t pwm17, int16_t pwm18,int16_t pwm21, int16_t pwm22, int16_t pwm23, int16_t pwm24, int16_t pwm25, int16_t pwm26, int16_t pwm27, int16_t pwm28 );
void pwm_Kill();
void pwm_Max();
void pwm_Arm();
void pwm_Calibration();
void kalman_Filtering();
void pid_Gain_Setting();
void disturbance_Observer();
void sine_wave_vibration();
//-------------------------------------------------------

//Publisher Group--------------------------------------
ros::Publisher PWMs;
ros::Publisher goal_dynamixel_position_;

//
//

ros::Publisher euler;
ros::Publisher desired_angle;
ros::Publisher Forces;
ros::Publisher desired_torque;
ros::Publisher linear_velocity;
ros::Publisher angular_velocity;
ros::Publisher PWM_generator;
ros::Publisher desired_position;
ros::Publisher position;
ros::Publisher kalman_angular_vel;
ros::Publisher kalman_angular_accel;
ros::Publisher desired_force;
ros::Publisher battery_voltage;
ros::Publisher force_command;
ros::Publisher delta_time;
ros::Publisher desired_velocity;
ros::Publisher Center_of_Mass;
ros::Publisher angular_Acceleration;
ros::Publisher sine_wave_data;
ros::Publisher disturbance;
ros::Publisher linear_acceleration;
ros::Publisher sbus_data;
//----------------------------------------------------

// Combine drone set
ros::Publisher master2slave_force_pub;
ros::Publisher master2slave_servo_pub;
ros::Publisher master_servo_cmd_pub;
ros::Publisher kill_switch_sub;
ros::Publisher tilt_switch_sub;
Eigen::MatrixXd CM_comb(4,8);
Eigen::VectorXd F_cmd_comb(8);
Eigen::MatrixXd invCM_comb(4,8);
Eigen::VectorXd u_comb(8);
//Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>::COD(4,8);
//Control Matrix---------------------------------------
Eigen::MatrixXd CM(4,4);
//Eigen::Vector4d u;
Eigen::VectorXd u(4);
Eigen::VectorXd F_cmd(4);

Eigen::MatrixXd invCM(4,4);
//-----------------------------------------------------

//Linear_velocity--------------------------------------
Eigen::Vector3d cam_v;
Eigen::Matrix3d R_v;
Eigen::Vector3d v;
//-----------------------------------------------------

//Attitude--------------------------------------
Eigen::Vector3d cam_att;
//-----------------------------------------------------

//Kalman_Filter----------------------------------------
Eigen::MatrixXd F(6,6);
Eigen::MatrixXd P(6,6);
Eigen::MatrixXd H(3,6);
Eigen::MatrixXd Q(6,6);
Eigen::MatrixXd R(3,3);
Eigen::MatrixXd K(6,3);
Eigen::VectorXd x(6);
Eigen::Vector3d z;
Eigen::MatrixXd predicted_P(6,6);
Eigen::VectorXd predicted_x(6);
Eigen::Matrix3d gain_term;
Eigen::MatrixXd covariance_term;
geometry_msgs::Vector3 filtered_Angular_vel;
geometry_msgs::Vector3 filtered_Angular_accel;
//-----------------------------------------------------
//for delay check



//Timer------------------------------------------------
auto end  =std::chrono::high_resolution_clock::now();
auto start=std::chrono::high_resolution_clock::now();
//-----------------------------------------------------

//Extremum Seeking Control-----------------------------
geometry_msgs::Vector3 prev_angular_Vel;
geometry_msgs::Vector3 angular_Accel;
geometry_msgs::Vector3 CoM;
geometry_msgs::Vector3 sine_wave;

double MoI_x_hat = 0.01;
double MoI_y_hat = 0.01;
double G_XY = 0.2;
double G_Z = 0.5;

double bias_x_c = 0;
double bias_y_c = 0;
double bias_z_c = 0;
double x_c_limit = 0.04;
double y_c_limit = 0.04;
double z_c_limit = 0.1;

//Bandpass filter parameter
double Q_factor=10;
double pass_freq1=5.0;
double pass_freq2=5.0;

//Filter1
double x_11=0;
double x_12=0;
double x_dot_11=0;
double x_dot_12=0;
double y_11=0;

//Filter2
double x_21=0;
double x_22=0;
double x_dot_21=0;
double x_dot_22=0;
double y_21=0;

//Filter3
double x_31=0;
double x_32=0;
double x_dot_31=0;
double x_dot_32=0;
double y_31=0;

double vibration1=0;
double vibration2=0;
double time_count=0;
double Amp_XY=0.5;
double Amp_Z=1.0;
//-----------------------------------------------------
//Accelerometer LPF------------------------------------
double x_ax_dot = 0;
double x_ay_dot = 0;
double x_ax = 0;
double x_ay = 0;
double accel_cutoff_freq = 5.0;
//-----------------------------------------------------
int main(int argc, char **argv){
	
    	ros::init(argc, argv,"fac_mav_main");//22.10.27

    	std::string deviceName;
    	ros::NodeHandle params("~");
    	params.param<std::string>("device", deviceName, "/gx5");

    	ros::NodeHandle nh;


	//Loading gains from the "t3_mav_controller.launch" file
		//integratior(PID) limitation
		integ_limit=nh.param<double>("attitude_integ_limit",10);
		z_integ_limit=nh.param<double>("altitude_integ_limit",100);
		pos_integ_limit=nh.param<double>("position_integ_limit",10);

		//Center of Mass
		x_c_hat=nh.param<double>("x_center_of_mass",0.0);
		y_c_hat=nh.param<double>("y_center_of_mass",0.0);
		z_c_hat=nh.param<double>("z_center_of_mass",0.0);
		CoM.x = x_c_hat;
		CoM.y = y_c_hat;
		CoM.z = z_c_hat;

		//Conventional Flight Mode Control Gains
			//Roll, Pitch PID gains
			conv_Par=nh.param<double>("conv_attitude_r_P_gain",3.5);
			conv_Iar=nh.param<double>("conv_attitude_r_I_gain",0.4);
			conv_Dar=nh.param<double>("conv_attitude_r_D_gain",0.5);
			
			conv_Pap=nh.param<double>("conv_attitude_p_P_gain",3.5);
                        conv_Iap=nh.param<double>("conv_attitude_p_I_gain",0.4);
                        conv_Dap=nh.param<double>("conv_attitude_p_D_gain",0.5);

			//Yaw PID gains
			conv_Py=nh.param<double>("conv_attitude_y_P_gain",2.0);
			conv_Dy=nh.param<double>("conv_attitude_y_D_gain",0.1);

			//Altitude PID gains
			conv_Pz=nh.param<double>("conv_altitude_P_gain",16.0);
			conv_Iz=nh.param<double>("conv_altitude_I_gain",5.0);
			conv_Dz=nh.param<double>("conv_altitude_D_gain",15.0);

			//Velocity PID gains
			conv_Pv=nh.param<double>("conv_velocity_P_gain",5.0);
			conv_Iv=nh.param<double>("conv_velocity_I_gain",1.0);
			conv_Dv=nh.param<double>("conv_velocity_D_gain",5.0);

			//Position PID gains
			conv_Pp=nh.param<double>("conv_position_P_gain",3.0);
			conv_Ip=nh.param<double>("conv_position_I_gain",0.1);
			conv_Dp=nh.param<double>("conv_position_D_gain",5.0);

		//Tilt Flight Mode Control Gains
			//Roll, Pitch PID gains
			tilt_Par=nh.param<double>("tilt_attitude_r_P_gain",3.5);
			tilt_Iar=nh.param<double>("tilt_attitude_r_I_gain",0.4);
			tilt_Dar=nh.param<double>("tilt_attitude_r_D_gain",0.5);

			tilt_Pap=nh.param<double>("tilt_attitude_p_P_gain",3.5);
                        tilt_Iap=nh.param<double>("tilt_attitude_p_I_gain",0.4);
                        tilt_Dap=nh.param<double>("tilt_attitude_p_D_gain",0.5);


			//Yaw PID gains
			tilt_Py=nh.param<double>("tilt_attitude_y_P_gain",5.0);
			tilt_Dy=nh.param<double>("tilt_attitude_y_D_gain",0.3);

			//Altitude PID gains
			tilt_Pz=nh.param<double>("tilt_altitude_P_gain",15.0);
			tilt_Iz=nh.param<double>("tilt_altitude_I_gain",5.0);
			tilt_Dz=nh.param<double>("tilt_altitude_D_gain",10.0);

			//Velocity PID gains
			tilt_Pv=nh.param<double>("tilt_velocity_P_gain",5.0);
			tilt_Iv=nh.param<double>("tilt_velocity_I_gain",0.1);
			tilt_Dv=nh.param<double>("tilt_velocity_D_gain",5.0);

			//Position PID gains
			tilt_Pp=nh.param<double>("tilt_position_P_gain",3.0);
			tilt_Ip=nh.param<double>("tilt_position_I_gain",0.1);
			tilt_Dp=nh.param<double>("tilt_position_D_gain",5.0);


	//----------------------------------------------------------
	
	//Kalman initialization-------------------------------------
		x << 0, 0, 0, 0, 0, 0;
		P << Eigen::MatrixXd::Identity(6,6);
		H << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);
		Q << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3),
		     Eigen::MatrixXd::Zero(3,3), 100.*Eigen::MatrixXd::Identity(3,3);
		R << 0.0132*Eigen::MatrixXd::Identity(3,3);
        //----------------------------------------------------------
	//Set Control Matrix----------------------------------------
		//setCM();
		setCM_comb();
                F_cmd << 0, 0, 0, 0;
		F_cmd_comb << 0,0,0,0,0,0,0,0;
	//----------------------------------------------------------
    PWMs = nh.advertise<std_msgs::Int16MultiArray>("PWMs", 1); // PWM 1,2,3,4
    PWM_generator = nh.advertise<std_msgs::Int32MultiArray>("/command",1);  // publish to pca9685

    goal_dynamixel_position_  = nh.advertise<sensor_msgs::JointState>("goal_dynamixel_position",10); // desired theta1,2
    //need to compare with sub drone's servo angle 1, 2

	euler = nh.advertise<geometry_msgs::Vector3>("angle",1); // roll, pitch, yaw
	desired_angle = nh.advertise<geometry_msgs::Vector3>("desired_angle",10);
	Forces = nh.advertise<std_msgs::Float32MultiArray>("Force_comb",10); // F 1,2,3,4
	desired_torque = nh.advertise<geometry_msgs::Vector3>("torque_d",10);
	linear_velocity = nh.advertise<geometry_msgs::Vector3>("lin_vel",10);
	angular_velocity = nh.advertise<geometry_msgs::Vector3>("gyro",10);
	desired_position = nh.advertise<geometry_msgs::Vector3>("pos_d",10);
	position = nh.advertise<geometry_msgs::Vector3>("pos",10);
	desired_force = nh.advertise<geometry_msgs::Vector3>("force_d",10);
	battery_voltage = nh.advertise<std_msgs::Float32>("battery_voltage",10);
	force_command = nh.advertise<std_msgs::Float32MultiArray>("force_cmd",10);
	delta_time = nh.advertise<std_msgs::Float32>("delta_t",10);
	desired_velocity = nh.advertise<geometry_msgs::Vector3>("lin_vel_d",10);
	angular_Acceleration = nh.advertise<geometry_msgs::Vector3>("ang_accel",10);
	linear_acceleration = nh.advertise<geometry_msgs::Vector3>("lin_acl",10);
	sbus_data = nh.advertise<std_msgs::Int32MultiArray>("Sbus_data_plot",10);
	
	
	master_servo_cmd_pub = nh.advertise<std_msgs::Float32MultiArray>("master_servo_cmd_pub",10);
	master2slave_force_pub =nh.advertise<std_msgs::Float32MultiArray>("master2slave_force_pub",100);
	master2slave_servo_pub =nh.advertise<std_msgs::Float32MultiArray>("master2slave_servo_pub",100);
	
	/*
	kill_switch_sub = nh.advertise<std_msgs::Bool>("kill_switch_sub",1);
	tilt_switch_sub = nh.advertise<std_msgs::Bool>("tilt_switch_sub",1);
	*/

	//


	//

    ros::Subscriber dynamixel_state = nh.subscribe("joint_states",10,jointstateCallback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber Servo_angle_sub = nh. subscribe("servo_angle_sub",10,jointstate_sub_Callback,ros::TransportHints().tcpNoDelay()); 
    ros::Subscriber att = nh.subscribe("/imu/data",1,imu_Callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber rc_in = nh.subscribe("/sbus",10,sbusCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber battery_checker = nh.subscribe("/battery",10,batteryCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_pos=nh.subscribe("/t265_pos",10,posCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_rot=nh.subscribe("/t265_rot",10,rotCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_odom=nh.subscribe("/rs_t265/odom/sample",10,t265OdomCallback,ros::TransportHints().tcpNoDelay());
	
	ros::Timer timerPublish = nh.createTimer(ros::Duration(1.0/200.0),std::bind(publisherSet));
    ros::spin();
    return 0;
}

void publisherSet(){
	
	
	

	//for time delay check//	
	end=std::chrono::high_resolution_clock::now();
	delta_t=end-start; 
	dt.data=delta_t.count();
	start=std::chrono::high_resolution_clock::now();
	///////////////////////
	// F << Eigen::MatrixXd::Identity(3,3), delta_t.count()*Eigen::MatrixXd::Identity(3,3),
	// 	     Eigen::MatrixXd::Zero(3,3),                 Eigen::MatrixXd::Identity(3,3);
	//sine_wave_vibration();
	//setCM();
	setCM_comb();
	angular_Accel.x = (imu_ang_vel.x-prev_angular_Vel.x)/delta_t.count();
	angular_Accel.y = (imu_ang_vel.y-prev_angular_Vel.y)/delta_t.count();
	angular_Accel.z = (imu_ang_vel.z-prev_angular_Vel.z)/delta_t.count();
	if(!position_mode){
		X_d_base=pos.x;
		Y_d_base=pos.y;
		X_d = X_d_base;
		Y_d = Y_d_base;
		e_X_i=0;
		e_Y_i=0;
		if(attitude_mode){
			e_X_dot_i=0;
			e_Y_dot_i=0;	
		}	
	}

	if(kill_mode){	
		y_d=imu_rpy.z;	//[J]This line ensures that yaw desired right after disabling the kill switch becomes current yaw attitude
		Z_d_base=pos.z;
		e_r_i = 0;
		e_p_i = 0;
		e_Z_i = 0;
		e_Z_dot_i=0;
		e_X_i=0;
		e_X_dot_i=0;
		e_Y_i=0;
		e_Y_dot_i=0;
		start_flag=false;
		theta11_command=0.0;
		theta12_command=0.0;
		//-----------2023.01.05----------------//
		theta21_command=0.0;
		theta22_command=0.0;
		//------------------------------------//
		pwm_Kill();
		//ROS_INFO("Kill mode");	
		Force_cmd.data={0,0,0,0};
	}
	else{
		//pwm_Command(Sbus[2],Sbus[2],Sbus[2],Sbus[2],Sbus[2],Sbus[2],Sbus[2],Sbus[2]);
		//pwm_Command(1000,1000,1000,Sbus[2],1000,1000,1000,1000);

		rpyT_ctrl();
		//ROS_INFO("F11 : %f, F12 : %f, F13 : %f, F14 : %f",F11,F12,F13,F14);
        	//ROS_INFO("F21 : %f, F22 : %f, F23 : %f, F24 : %f",F21,F22,F23,F24);		

	}


	//ROS_INFO("test : %f, %f, %f",lin_vel.x,lin_vel.y,lin_vel.z);
	//ROS_INFO("test : %f, %f, %f",desired_lin_vel.x,desired_lin_vel.y,desired_lin_vel.z);


//	pwm_Calibration();	
	angle_d.x=r_d;
	angle_d.y=p_d;
	angle_d.z=y_d;
	desired_pos.x = X_d;
	desired_pos.y = Y_d;
	desired_pos.z = Z_d;

	Force_comb.data.resize(8);
	Force_comb.data[0] = F11;
	Force_comb.data[1] = F12;
	Force_comb.data[2] = F13;
	Force_comb.data[3] = F14;
	Force_comb.data[4] = F21;
	Force_comb.data[5] = F22;
        Force_comb.data[6] = F23;
        Force_comb.data[7] = F24;

	Force_cmd.data.resize(4);
/*
        Force_cmd.data[0] = F21;
        Force_cmd.data[1] = F22;
        Force_cmd.data[2] = F23;
        Force_cmd.data[3] = F24;
*/
        Servo_sub_cmd.data.resize(2);
/*        Servo_sub_cmd.data[0] = theta21_command;
        Servo_sub_cmd.data[1] = theta22_command; */
	
	//ROS_INFO("th_21_cmd :%lf, th_22_cmd :%lf \n",theta21_command,theta22_command);
	//ROS_INFO("th_11_cmd :%lf, th_12_cmd :%lf \n",theta11_command,theta12_command);
	//ROS_INFO("F_xd, F_yd :: %lf | %lf",F_xd,F_yd);

	Servo_main_cmd.data.resize(2);
/*	Servo_main_cmd.data[0]= theta11_command;
	Servo_main_cmd.data[1]= theta12_command;*/
	master_servo_cmd_pub.publish(Servo_main_cmd);
	
	Sbus_data_plot.data.resize(8);
	Sbus_data_plot.data[0]=Sbus[0];
	Sbus_data_plot.data[1]=Sbus[1];
	Sbus_data_plot.data[2]=Sbus[2];
	Sbus_data_plot.data[3]=Sbus[3];
	Sbus_data_plot.data[4]=Sbus[4];
	Sbus_data_plot.data[5]=Sbus[5];
	Sbus_data_plot.data[6]=Sbus[6];
	Sbus_data_plot.data[7]=Sbus[7];
	sbus_data.publish(Sbus_data_plot);

	CoM.x = x_c_hat;
	CoM.y = y_c_hat;
	CoM.z = z_c_hat;
	PWMs.publish(PWMs_cmd);// PWMs_d value
        //22.10.27 master to slave data
        master2slave_force_pub.publish(Force_cmd);
        master2slave_servo_pub.publish(Servo_sub_cmd);
	//22.11.01
        //kill_switch_sub.publish(kill_sub);
	//tilt_switch_sub.publish(tilt_sub);
        //
        euler.publish(imu_rpy);//rpy_act value
        desired_angle.publish(angle_d);//rpy_d value
        Forces.publish(Force_comb);// force conclusion
        goal_dynamixel_position_.publish(servo_msg_create(theta11_command,-theta12_command)); // desired theta
        desired_torque.publish(torque_d); // torque desired
        linear_velocity.publish(lin_vel); // actual linear velocity
        PWM_generator.publish(PWMs_val);
        desired_position.publish(desired_pos);//desired position
        position.publish(pos); // actual position
        desired_force.publish(force_d); // desired force it need only tilt mode
        battery_voltage.publish(battery_voltage_msg);
        delta_time.publish(dt);
        desired_velocity.publish(desired_lin_vel);
        angular_Acceleration.publish(angular_Accel);
        linear_acceleration.publish(lin_acl);
	prev_angular_Vel = imu_ang_vel;
	prev_lin_vel = lin_vel;
	
	
	//data checker
}

void setCM(){
	//Co-rotating type
	/*
	CM <<          (l_servo+z_c_hat)*sin(theta1)+y_c_hat*cos(theta1),  (r_arm+y_c_hat)*cos(theta2)+b_over_k_ratio*sin(theta2),       (l_servo+z_c_hat)*sin(theta1)+y_c_hat*cos(theta1), -(r_arm-y_c_hat)*cos(theta2)+b_over_k_ratio*sin(theta2),
              (r_arm-x_c_hat)*cos(theta1)+b_over_k_ratio*sin(theta1),      (-l_servo+z_c_hat)*sin(theta2)-x_c_hat*cos(theta2), -(r_arm+x_c_hat)*cos(theta1)+b_over_k_ratio*sin(theta1),      (-l_servo+z_c_hat)*sin(theta2)-x_c_hat*cos(theta2),
              (r_arm-x_c_hat)*sin(theta1)-b_over_k_ratio*cos(theta1), -(r_arm+y_c_hat)*sin(theta2)+b_over_k_ratio*cos(theta2), -(r_arm+x_c_hat)*sin(theta1)-b_over_k_ratio*cos(theta1),  (r_arm-y_c_hat)*sin(theta2)+b_over_k_ratio*cos(theta2),
                                                        -cos(theta1),                                            -cos(theta2),                                            -cos(theta1),                                            -cos(theta2);
    	invCM = CM.inverse();*/
}
// set Combined control matrix 22.10.27
// [ M1 | M2 ]
void setCM_comb(){

	CM_comb <<        (l_servo+z_c_hat)*sin(theta11)+y_c_hat*cos(theta11),  (r_arm+y_c_hat)*cos(theta12)+b_over_k_ratio*sin(theta12),       (l_servo+z_c_hat)*sin(theta11)+y_c_hat*cos(theta11), -(r_arm-y_c_hat)*cos(theta12)+b_over_k_ratio*sin(theta12),   (l_servo+z_c_hat)*sin(theta21)+(-diag+y_c_hat)*cos(theta21),    (r_arm-diag+y_c_hat)*cos(theta22)+b_over_k_ratio*sin(theta22),    (l_servo+z_c_hat)*sin(theta21)+(-diag+y_c_hat)*cos(theta21), -(r_arm+diag-y_c_hat)*cos(theta22)+b_over_k_ratio*sin(theta22),
              (r_arm-x_c_hat)*cos(theta11)+b_over_k_ratio*sin(theta11),      (-l_servo+z_c_hat)*sin(theta12)-x_c_hat*cos(theta12), -(r_arm+x_c_hat)*cos(theta11)+b_over_k_ratio*sin(theta11),      (-l_servo+z_c_hat)*sin(theta12)-x_c_hat*cos(theta12), (r_arm-diag-x_c_hat)*cos(theta21)+b_over_k_ratio*sin(theta21),      (-l_servo+z_c_hat)*sin(theta22)-(diag+x_c_hat)*cos(theta22), -(r_arm+diag+x_c_hat)*cos(theta21)+b_over_k_ratio*sin(theta21),    (-l_servo+z_c_hat)*sin(theta22)-(diag+x_c_hat)*cos(theta22),
              (r_arm-x_c_hat)*sin(theta11)-b_over_k_ratio*cos(theta11), -(r_arm+y_c_hat)*sin(theta12)+b_over_k_ratio*cos(theta12), -(r_arm+x_c_hat)*sin(theta11)-b_over_k_ratio*cos(theta11),  (r_arm-y_c_hat)*sin(theta12)+b_over_k_ratio*cos(theta12), (r_arm-diag-x_c_hat)*sin(theta21)-b_over_k_ratio*cos(theta21),   -(r_arm-diag+y_c_hat)*sin(theta22)+b_over_k_ratio*cos(theta22), -(r_arm+diag+x_c_hat)*sin(theta21)-b_over_k_ratio*cos(theta21),  (r_arm+diag-y_c_hat)*sin(theta22)+b_over_k_ratio*cos(theta22),
                                                        -cos(theta11),                                            -cos(theta12),                                            -cos(theta11),                                            -cos(theta12),                                                -cos(theta21),                                                   -cos(theta22),                                                 -cos(theta21),                                                 -cos(theta22);


  	Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> COD(CM_comb);	
	invCM_comb = COD.pseudoInverse();

}

void rpyT_ctrl() {
	pid_Gain_Setting();
	y_d_tangent=y_vel_limit*(((float)Sbus[0]-(float)1500)/(float)500);
	if(fabs(y_d_tangent)<y_d_tangent_deadzone || fabs(y_d_tangent)>y_vel_limit) y_d_tangent=0;
	y_d+=y_d_tangent;
	
	Z_d = -altitude_limit*(((float)Sbus[2]-(float)1500)/(float)500)-altitude_limit;
	T_d = -T_limit*(((float)Sbus[2]-(float)1500)/(float)500)-T_limit;

	float e_r = 0;
	float e_p = 0;
	float e_y = 0;
	float e_X = 0;
	float e_Y = 0;
	float e_Z = 0;
	float e_X_dot = 0;
	float e_Y_dot = 0;
		
	float global_X_ddot = (lin_vel.x - prev_lin_vel.x)/delta_t.count();
	float global_Y_ddot = (lin_vel.y - prev_lin_vel.y)/delta_t.count();
	x_ax_dot=-accel_cutoff_freq*x_ax+global_X_ddot;
	x_ax+=x_ax_dot*delta_t.count();
	lin_acl.x=accel_cutoff_freq*x_ax;
	x_ay_dot=-accel_cutoff_freq*x_ay+global_Y_ddot;
	x_ay+=x_ay_dot*delta_t.count();
	lin_acl.y=accel_cutoff_freq*x_ay;
	//ROS_INFO("%lf",time_count);

	if(position_mode || velocity_mode){
		if(position_mode){
			X_d = X_d_base - XY_limit*(((float)Sbus[1]-(float)1500)/(float)500);
			Y_d = Y_d_base + XY_limit*(((float)Sbus[3]-(float)1500)/(float)500);
			
			e_X = X_d - pos.x;
			e_Y = Y_d - pos.y;
			e_X_i += e_X * delta_t.count();
			if (fabs(e_X_i) > pos_integ_limit) e_X_i = (e_X_i / fabs(e_X_i)) * pos_integ_limit;
			e_Y_i += e_Y * delta_t.count();
			if (fabs(e_Y_i) > pos_integ_limit) e_Y_i = (e_Y_i / fabs(e_Y_i)) * pos_integ_limit;
	
			X_dot_d = Pp * e_X + Ip * e_X_i - Dp * lin_vel.x;
			Y_dot_d = Pp * e_Y + Ip * e_Y_i - Dp * lin_vel.y;
		}
		if(velocity_mode){
		  X_dot_d = -XYZ_dot_limit*(((double)Sbus[1]-(double)1500)/(double)500);
		  Y_dot_d = XYZ_dot_limit*(((double)Sbus[3]-(double)1500)/(double)500);
		  }
		
		
		
		if(fabs(X_dot_d) > XYZ_dot_limit) X_dot_d = (X_dot_d/fabs(X_dot_d))*XYZ_dot_limit;
                if(fabs(Y_dot_d) > XYZ_dot_limit) Y_dot_d = (Y_dot_d/fabs(Y_dot_d))*XYZ_dot_limit;

                desired_lin_vel.x = X_dot_d;
                desired_lin_vel.y = Y_dot_d;

                e_X_dot = X_dot_d - lin_vel.x;
                e_Y_dot = Y_dot_d - lin_vel.y;
                e_X_dot_i += e_X_dot * delta_t.count();
                if (fabs(e_X_dot_i) > vel_integ_limit) e_X_dot_i = (e_X_dot_i / fabs(e_X_dot_i)) * vel_integ_limit;
                e_Y_dot_i += e_Y_dot * delta_t.count();
                if (fabs(e_Y_dot_i) > vel_integ_limit) e_Y_dot_i = (e_Y_dot_i / fabs(e_Y_dot_i)) * vel_integ_limit;

                X_ddot_d = Pv * e_X_dot + Iv * e_X_dot_i - Dv * lin_acl.x;
                Y_ddot_d = Pv * e_Y_dot + Iv * e_Y_dot_i - Dv * lin_acl.y;
		
		
		if(tilt_mode)
		{
                if(fabs(X_ddot_d) > XYZ_ddot_limit_tilt) X_ddot_d = (X_ddot_d/fabs(X_ddot_d))*XYZ_ddot_limit_tilt;
                if(fabs(Y_ddot_d) > XYZ_ddot_limit_tilt) Y_ddot_d = (Y_ddot_d/fabs(Y_ddot_d))*XYZ_ddot_limit_tilt;

		}	

		else
		{

		
		if(fabs(X_ddot_d) > XYZ_ddot_limit) X_ddot_d = (X_ddot_d/fabs(X_ddot_d))*XYZ_ddot_limit;
		if(fabs(Y_ddot_d) > XYZ_ddot_limit) Y_ddot_d = (Y_ddot_d/fabs(Y_ddot_d))*XYZ_ddot_limit;
		}


		if(tilt_mode){
			
			r_d = 0.0;
			p_d = 0.0;
			//if(fabs(r_d)>rp_limit_tilt) r_d = (r_d/fabs(r_d))*rp_limit_tilt;
                        //if(fabs(p_d)>rp_limit_tilt) p_d = (p_d/fabs(p_d))*rp_limit_tilt;
			F_xd = mass*(X_ddot_d*cos(imu_rpy.z)*cos(imu_rpy.y)+Y_ddot_d*sin(imu_rpy.z)*cos(imu_rpy.y)-(Z_ddot_d-g)*sin(imu_rpy.y));
			F_yd = mass*(-X_ddot_d*(cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.x)*sin(imu_rpy.y))+Y_ddot_d*(cos(imu_rpy.x)*cos(imu_rpy.z)+sin(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d-g)*cos(imu_rpy.y)*sin(imu_rpy.x));
			
			//if(position_mode) ROS_INFO("Position & Tilt !!!");
			//else ROS_INFO("Velocity & Tilt !!!");
		}
		else{
			alpha=(-sin(imu_rpy.z)*X_ddot_d+cos(imu_rpy.z)*Y_ddot_d)/g;
			beta=(-cos(imu_rpy.z)*X_ddot_d-sin(imu_rpy.z)*Y_ddot_d)/g;
			if(fabs(alpha) > alpha_beta_limit) alpha = (alpha/fabs(alpha))*alpha_beta_limit;
			if(fabs(beta) > alpha_beta_limit) beta = (beta/fabs(beta))*alpha_beta_limit;
			r_d = asin(alpha);
			p_d = asin(beta/cos(imu_rpy.x));
			if(fabs(r_d)>rp_limit) r_d = (r_d/fabs(r_d))*rp_limit;
			if(fabs(p_d)>rp_limit) p_d = (p_d/fabs(p_d))*rp_limit;
			
			//if(position_mode) ROS_INFO("Position & Conventional !!!");
			//else ROS_INFO("Velocity & Conventional !!!");
		}
	}
	if(attitude_mode){
		if(tilt_mode){
			
			r_d = 0.0;
			p_d = 0.0;
			/*
			theta11_command= hardware_servo_limit*(((float)Sbus[3]-(float)1500)/(float)500);
                        theta12_command=hardware_servo_limit*(((float)Sbus[1]-(float)1500)/(float)500);
			theta21_command= hardware_servo_limit*(((float)Sbus[3]-(float)1500)/(float)500);
                        theta22_command=hardware_servo_limit*(((float)Sbus[1]-(float)1500)/(float)500);
			*/
			
			F_xd=-mass*XYZ_ddot_limit_tilt*(((float)Sbus[1]-(float)1500)/(float)500);
			F_yd=mass*XYZ_ddot_limit_tilt*(((float)Sbus[3]-(float)1500)/(float)500);
		
			/*
			F_xd=0.0;
                        F_yd=0.0;*/
			//ROS_INFO("Attitude & Tilt !!!");

		}
		else{
			r_d=rp_limit*(((float)Sbus[3]-(float)1500)/(float)500);
			p_d=rp_limit*(((float)Sbus[1]-(float)1500)/(float)500);
			//ROS_INFO("Attidue Control!!");
			F_xd=0.0;
			F_yd=0.0;
			//ROS_INFO("Attitude & Conventional");
		}
	}
	
	e_r = r_d - imu_rpy.x;
	e_p = p_d - imu_rpy.y;
	e_y = y_d - imu_rpy.z;

	e_Z = Z_d - pos.z;
	
	e_r_i += e_r * delta_t.count();
	if (fabs(e_r_i) > integ_limit)	e_r_i = (e_r_i / fabs(e_r_i)) * integ_limit;
	e_p_i += e_p * delta_t.count();
	if (fabs(e_p_i) > integ_limit)	e_p_i = (e_p_i / fabs(e_p_i)) * integ_limit;
	e_Z_i += e_Z * delta_t.count();	
	if (fabs(e_Z_i) > z_integ_limit) e_Z_i = (e_Z_i / fabs(e_Z_i)) * z_integ_limit;

	tau_r_d = Par * e_r + Iar * e_r_i + Dar * (-imu_ang_vel.x);//- (double)0.48;
	tau_p_d = Pap * e_p + Iap * e_p_i + Dap * (-imu_ang_vel.y);//+ (double)0.18; 
	tau_y_d = Py * e_y + Dy * (-imu_ang_vel.z);
	if(fabs(tau_y_d) > tau_y_limit) tau_y_d = tau_y_d/fabs(tau_y_d)*tau_y_limit;
	
	if(altitude_mode){
		Z_ddot_d = Pz * e_Z + Iz * e_Z_i - Dz * lin_vel.z;
		desired_lin_vel.z = Z_ddot_d; // But this is desired acceleration
		if(Sbus[6]>1500) F_zd = mass*(X_ddot_d*(sin(imu_rpy.x)*sin(imu_rpy.z)+cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y))-Y_ddot_d*(cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d-g)*cos(imu_rpy.x)*cos(imu_rpy.y));
		else F_zd = mass*(Z_ddot_d-g);
		//ROS_INFO("Altitude");
	}
	else{
		e_Z_i = 0;
		e_Z_dot_i = 0;
		F_zd=T_d;
		//ROS_INFO("Manual Thrust!!");
	}
	if(F_zd > -0.5*mass*g) F_zd = -0.5*mass*g;
	if(F_zd <= -1.5*mass*g) F_zd = -1.5*mass*g; //1.5
	
	//ROS_INFO("x : %lf,dx : %lf,ddx : %lf, \n",X_d,X_dot_d,X_ddot_d);	

	//DOB-----------------------------------------------------
		//disturbance_Observer();
	//--------------------------------------------------------
	
	//tautilde_r_d = tau_r_d - dhat_r;
	//tautilde_p_d = tau_p_d - dhat_p;
	//u << tau_r_d, tau_p_d, tau_y_d, F_zd;
	//u << tautilde_r_d, tautilde_p_d, tau_y_d, F_zd;
	torque_d.x = tau_r_d;
	torque_d.y = tau_p_d;
	torque_d.z = tau_y_d;
	force_d.x = F_xd;
	force_d.y = F_yd;
	force_d.z = F_zd;
	
	u << tau_r_d, tau_p_d, tau_y_d, F_zd;
	prev_angular_Vel = imu_ang_vel;
	ud_to_PWMs(tau_r_d, tau_p_d, tau_y_d, Thrust_d); //but not use 22.10.12
	//ud_to_PWMs(tautilde_r_d, tautilde_p_d, tautilde_y_d, Thrust_d);
}

 

void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des) {	
 	
	//Co-rotating coaxial
	//Conventional type
//	F_cmd = invCM*u;
	F_cmd_comb = invCM_comb*u;
	if(Sbus[7]<=1500){
		//---------------2023.01.05----------------//

		theta11_command = 0.0;
        theta12_command = 0.0;
	theta21_command = 0.0;
	theta22_command = 0.0;
	//--------------------------------------------------//
	}
	//Tilting type
	else {
		//22.10.27 should be changed
		theta11_command = asin((F_yd)/(F_cmd_comb(0)+F_cmd_comb(2)+F_cmd_comb(4)+F_cmd_comb(6)));
		theta12_command = -asin((F_xd)/(F_cmd_comb(1)+F_cmd_comb(3)+F_cmd_comb(5)+F_cmd_comb(7)));
		theta21_command = asin((F_yd)/(F_cmd_comb(0)+F_cmd_comb(2)+F_cmd_comb(4)+F_cmd_comb(6)));
		theta22_command = -asin((F_xd)/(F_cmd_comb(1)+F_cmd_comb(3)+F_cmd_comb(5)+F_cmd_comb(7)));
 		if(fabs(theta11_command)>hardware_servo_limit) theta11_command = (theta11_command/fabs(theta11_command))*hardware_servo_limit;
		if(fabs(theta12_command)>hardware_servo_limit) theta12_command = (theta12_command/fabs(theta12_command))*hardware_servo_limit;
		if(fabs(theta21_command)>hardware_servo_limit) theta21_command = (theta21_command/fabs(theta21_command))*hardware_servo_limit;
                if(fabs(theta22_command)>hardware_servo_limit) theta22_command = (theta22_command/fabs(theta22_command))*hardware_servo_limit;
	}
	
	Servo_sub_cmd.data[0] = theta21_command;
        Servo_sub_cmd.data[1] = theta22_command;

	Servo_main_cmd.data[0]= theta11_command;
        Servo_main_cmd.data[1]= theta12_command;	
	
	F11 = (F_cmd_comb(0));
	F12 = (F_cmd_comb(1));
	F13 = (F_cmd_comb(2));
	F14 = (F_cmd_comb(3));
	F21 = (F_cmd_comb(4));
	F22 = (F_cmd_comb(5));
	F23 = (F_cmd_comb(6));
	F24 = (F_cmd_comb(7));
/*
	Force_cmd.data[0] = F21;
        Force_cmd.data[1] = F22;
        Force_cmd.data[2] = F23;
        Force_cmd.data[3] = F24;
*/

	pwm_Command(Force_to_PWM(F11),Force_to_PWM(F12), Force_to_PWM(F13), Force_to_PWM(F14), Force_to_PWM(F11), Force_to_PWM(F12), Force_to_PWM(F13), Force_to_PWM(F14),Force_to_PWM(F23),Force_to_PWM(F24), Force_to_PWM(F21), Force_to_PWM(F22), Force_to_PWM(F23), Force_to_PWM(F24), Force_to_PWM(F21), Force_to_PWM(F22));
	
	// ROS_INFO("11:%d, 12:%d, 13:%d, 14:%d, 21:%d, 22:%d, 23:%d, 24:%d",PWMs_cmd.data[0], PWMs_cmd.data[1], PWMs_cmd.data[2], PWMs_cmd.data[3],PWMs_cmd.data[8], PWMs_cmd.data[9], PWMs_cmd.data[10], PWMs_cmd.data[11]);
	// ROS_INFO("%f 1:%d, 2:%d, 3:%d, 4:%d",z_d,PWMs_cmd.data[0], PWMs_cmd.data[1], PWMs_cmd.data[2], PWMs_cmd.data[3]);
}

 

float Force_to_PWM(float F) {
	float pwm;
	float A = -9.8*pow(10.0,-8.0)*pow(voltage,2.0)+3.23*pow(10.0,-6.0)*voltage-1.8*pow(10.0,-5.0);
	float B = 0.000243*pow(voltage,2.0)-0.00663*voltage+0.03723;
	float C = -0.11063*pow(voltage,2.0)+2.332691*voltage-10.885;
	float param1 = -B/(2.0*A);
	float param2 = 1.0/A;
	float param3 = (pow(B,2.0)-4*A*C)/(4*pow(A,2.0));
	//Force=A*pwm^2+B*pwm+C
	if(param2*F+param3>0){
		pwm = param1 + sqrt(param2 * F + param3);
	}
	else pwm = param1;

	if(Sbus[5]>1500){//altitude mode
                if(Z_d_base<=0){
                        if(Z_d>Z_d_base && !start_flag) {
                                pwm=param1;
                        }
                        else if(Z_d<Z_d_base) start_flag=true;
                }
                else pwm=param1;
        }
	if (pwm > 1900)	pwm = 1900;
	if(pwm < 1100) pwm = 1100;
	return pwm;
}

void jointstateCallback(const sensor_msgs::JointState& msg){
    	rac_servo_value=msg;
	theta11=msg.position[0];
	theta12=-msg.position[1];

	theta21=msg.position[2];
	theta22=-msg.position[3];

   	//ROS_INFO("theta11:%lf,   theta12:%lf,theta21:%lf, theta22:%lf",theta11, theta12,theta21,theta22);
}


void jointstate_sub_Callback(const sensor_msgs::JointState& msg){
        rac_servo_value=msg;
        theta21=msg.position[0];
        theta22=msg.position[1];
        //ROS_INFO("theta21:%lf   theta22:%lf",theta21, theta22);
}
ros::Time imuTimer;

void imu_Callback(const sensor_msgs::Imu& msg){
    	imu=msg;
    
    	// TP attitude - Quaternion representation
    	imu_quaternion=msg.orientation;
    	imu_ang_vel=msg.angular_velocity;
    	// ROS_INFO("R:%lf\tP:%lf\tY:%lf",imu_ang_vel.x,imu_ang_vel.y,imu_ang_vel.z);
    	imu_lin_acc=msg.linear_acceleration;

    	tf::Quaternion quat;
    	tf::quaternionMsgToTF(imu_quaternion,quat);

    	// TP attitude - Euler representation
    	
	
	tf::Matrix3x3(quat).getRPY(imu_rpy.x,imu_rpy.y,imu_rpy.z);
	
	/*
	base_yaw = cam_att(2);
    	if(base_yaw - yaw_prev < -pi) yaw_rotate_count++;
	else if(base_yaw - yaw_prev > pi) yaw_rotate_count--;
	yaw_now = base_yaw+2*pi*yaw_rotate_count;
	//ROS_INFO("now : %lf / prev : %lf / count : %d",yaw_now, yaw_prev, yaw_rotate_count);
	
	imu_rpy.z = yaw_now;
	yaw_prev = base_yaw;
	*/
	// ROS_INFO("imuCallback time : %f",(((double)ros::Time::now().sec-(double)imuTimer.sec)+((double)ros::Time::now().nsec-(double)imuTimer.nsec)/1000000000.));
	imuTimer = ros::Time::now();
}

void filterCallback(const sensor_msgs::Imu& msg){
	filtered_angular_rate=msg.angular_velocity;
}

sensor_msgs::JointState servo_msg_create(float rr1, float rp1/*, float rr2,float rp2*/){
	sensor_msgs::JointState servo_msg;

	servo_msg.header.stamp=ros::Time::now();

	servo_msg.name.resize(2);
	servo_msg.name[0]="id_11";
	servo_msg.name[1]="id_12";

	/*
	servo_msg.name[2]="id_21";
        servo_msg.name[3]="id_22";		
	*/

	servo_msg.position.resize(2);
	servo_msg.position[0]=rr1;
	servo_msg.position[1]=rp1;
	/*
	servo_msg.position[2]=rr2;
        servo_msg.position[3]=rp2;*/
	//ROS_INFO("rr: %lf, rp: %lf",rr,rp);
	return servo_msg;
}

void sbusCallback(const std_msgs::Int16MultiArray::ConstPtr& array){
	for(int i=0;i<8;i++){
		Sbus[i]=map<int16_t>(array->data[i], 352, 1696, 1000, 2000);
	}
	
	if(Sbus[4]<1500) kill_mode=true;
	else kill_mode=false;
	kill_sub.data=kill_mode;
	
	if(Sbus[5]>1500) altitude_mode=true;
	else altitude_mode=false;

	if(Sbus[6]<1300){
		attitude_mode=true;
		velocity_mode=false;
		position_mode=false;
	}
	else if(Sbus[6]<1700){
		attitude_mode=false;
		velocity_mode=true;
		position_mode=false;
	}
	else{
		attitude_mode=false;
		velocity_mode=false;
		position_mode=true;
	}

	if(Sbus[7]>1500) tilt_mode=true;
	else tilt_mode=false;
	tilt_sub.data=tilt_mode;


	//ROS_INFO("%d, %d, %d, %d, %d, %d, %d, %d",Sbus[0],Sbus[1],Sbus[2],Sbus[3],Sbus[4],Sbus[5],Sbus[6],Sbus[7]);
	//if(Sbus[9]>1500) ESC_control=true;
	//else ESC_control=false;
}


void batteryCallback(const std_msgs::Int16& msg){
	int16_t value=msg.data;
	voltage=value*5.0/(double)1024/(7440./(30000.+7440.)); //4096
	double kv=0.08;
	voltage=kv*voltage+(1-kv)*voltage_old;
	voltage_old=voltage;
	//ROS_INFO("%f",voltage);
	if(voltage>16.8) voltage=16.8;
	if(voltage<13.0) voltage=13.0;
	battery_voltage_msg.data=voltage;
	
}

ros::Time posTimer;
void posCallback(const geometry_msgs::Vector3& msg){

	pos.x=msg.x;
	pos.y=msg.y;
	pos.z=msg.z;
}

void rotCallback(const geometry_msgs::Quaternion& msg){
	rot.x=msg.x;
	rot.y=msg.y;
	rot.z=msg.z;
	rot.w=msg.w;
	
	tf::Quaternion quat;
	tf::quaternionMsgToTF(rot,quat);
	tf::Matrix3x3(quat).getRPY(imu_rpy.x,imu_rpy.y,imu_rpy.z); //t265_att.x,t265_att.y,t265_att.z
	
	base_yaw = cam_att(2);
        if(base_yaw - yaw_prev < -pi) yaw_rotate_count++;
        else if(base_yaw - yaw_prev > pi) yaw_rotate_count--;
        yaw_now = base_yaw+2*pi*yaw_rotate_count;
        //ROS_INFO("now : %lf / prev : %lf / count : %d",yaw_now, yaw_prev, yaw_rotate_count);
        
        imu_rpy.z = yaw_now;
        yaw_prev = base_yaw;	

	//ROS_INFO("%f, %f, %f",imu_rpy.x,imu_rpy.y,imu_rpy.z);

	//ROS_INFO("%f, %f, %f",t265_att.x,t265_att.y,t265_att.z);	
}

void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	t265_lin_vel=msg->twist.twist.linear;
	t265_ang_vel=msg->twist.twist.angular;
	t265_quat=msg->pose.pose.orientation;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(rot,quat);
	tf::Matrix3x3(quat).getRPY(cam_att(0),cam_att(1),cam_att(2));
	cam_v << t265_lin_vel.x, t265_lin_vel.y, t265_lin_vel.z;
	R_v << cos(-pi/4.), -sin(-pi/4.),  0.,
 	       sin(-pi/4.),  cos(-pi/4.),  0.,
	                0.,           0.,  1.;
	

	v = R_v*cam_v;

	double global_X_dot = v(2)*(sin(imu_rpy.x)*sin(imu_rpy.z)+cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y))-v(1)*(cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.x)*sin(imu_rpy.y))+v(0)*cos(imu_rpy.z)*cos(imu_rpy.y);
	double global_Y_dot = v(1)*(cos(imu_rpy.x)*cos(imu_rpy.z)+sin(imu_rpy.x)*sin(imu_rpy.z)*sin(imu_rpy.y))-v(2)*(cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.z)*sin(imu_rpy.y))+v(0)*cos(imu_rpy.y)*sin(imu_rpy.z);
	double global_Z_dot = -v(0)*sin(imu_rpy.y)+v(2)*cos(imu_rpy.x)*cos(imu_rpy.y)+v(1)*cos(imu_rpy.y)*sin(imu_rpy.x);

	lin_vel.x=global_X_dot; 
	lin_vel.y=global_Y_dot;
	lin_vel.z=global_Z_dot;
	//ROS_INFO("Attitude - [r: %f  p: %f  y:%f]",cam_att(0),cam_att(1),cam_att(2));
	//ROS_INFO("Rotate Linear_velocity - [x: %f  y: %f  z:%f]",v(0),v(1),v(2));
	//ROS_INFO("Linear_velocity - [x: %f  y: %f  z:%f]",cam_v(0),cam_v(1),cam_v(2));
	//ROS_INFO("Angular_velocity - [x: %f  y: %f  z:%f]",w(0),w(1),w(2));
}

int32_t pwmMapping(int32_t pwm){
	return (int32_t)(65535.*pwm/(1./pwm_freq*1000000.));
}

void pwm_Command(int16_t pwm11, int16_t pwm12, int16_t pwm13, int16_t pwm14, int16_t pwm15, int16_t pwm16, int16_t pwm17, int16_t pwm18,int16_t pwm21, int16_t pwm22, int16_t pwm23, int16_t pwm24, int16_t pwm25, int16_t pwm26, int16_t pwm27, int16_t pwm28){
	PWMs_cmd.data.resize(16);
	PWMs_cmd.data[0] = pwm11;
	PWMs_cmd.data[1] = pwm12;
	PWMs_cmd.data[2] = pwm13;
	PWMs_cmd.data[3] = pwm14;
	PWMs_cmd.data[4] = pwm15;
	PWMs_cmd.data[5] = pwm16;
	PWMs_cmd.data[6] = pwm17;
	PWMs_cmd.data[7] = pwm18;
	PWMs_cmd.data[8] = pwm21;
        PWMs_cmd.data[9] = pwm22;
        PWMs_cmd.data[10] = pwm23;
        PWMs_cmd.data[11] = pwm24;
        PWMs_cmd.data[12] = pwm25;
        PWMs_cmd.data[13] = pwm26;
        PWMs_cmd.data[14] = pwm27;
        PWMs_cmd.data[15] = pwm28;
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(pwm11);
	PWMs_val.data[1] = pwmMapping(pwm12);
	PWMs_val.data[2] = pwmMapping(pwm13);
	PWMs_val.data[3] = pwmMapping(pwm14);
	PWMs_val.data[4] = pwmMapping(pwm15);
	PWMs_val.data[5] = pwmMapping(pwm16);
	PWMs_val.data[6] = pwmMapping(pwm17);
	PWMs_val.data[7] = pwmMapping(pwm18);
	PWMs_val.data[8] = pwmMapping(pwm21);
	PWMs_val.data[9] = pwmMapping(pwm22);
	PWMs_val.data[10] = pwmMapping(pwm23);
	PWMs_val.data[11] = pwmMapping(pwm24);
	PWMs_val.data[12] = pwmMapping(pwm25);
	PWMs_val.data[13] = pwmMapping(pwm26);
	PWMs_val.data[14] = pwmMapping(pwm27);
	PWMs_val.data[15] = pwmMapping(pwm28);

}

void pwm_Max(){
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(2000.);
	PWMs_val.data[1] = pwmMapping(2000.);
	PWMs_val.data[2] = pwmMapping(2000.);
	PWMs_val.data[3] = pwmMapping(2000.);
	PWMs_val.data[4] = pwmMapping(2000.);
	PWMs_val.data[5] = pwmMapping(2000.);
	PWMs_val.data[6] = pwmMapping(2000.);
	PWMs_val.data[7] = pwmMapping(2000.);
	PWMs_val.data[8] = -1;
	PWMs_val.data[9] = -1;
	PWMs_val.data[10] = -1;
	PWMs_val.data[11] = -1;
	PWMs_val.data[12] = -1;
	PWMs_val.data[13] = -1;
	PWMs_val.data[14] = -1;
	PWMs_val.data[15] = -1;
}

void pwm_Kill(){
	PWMs_cmd.data.resize(16);
	PWMs_cmd.data[0] = 1000;
	PWMs_cmd.data[1] = 1000;
	PWMs_cmd.data[2] = 1000;
	PWMs_cmd.data[3] = 1000;
	PWMs_cmd.data[4] = 1000;
	PWMs_cmd.data[5] = 1000;
	PWMs_cmd.data[6] = 1000;
	PWMs_cmd.data[7] = 1000;
	PWMs_cmd.data[8] = 1000;
        PWMs_cmd.data[9] = 1000;
        PWMs_cmd.data[10] = 1000;
        PWMs_cmd.data[11] = 1000;
        PWMs_cmd.data[12] = 1000;
        PWMs_cmd.data[13] = 1000;
        PWMs_cmd.data[14] = 1000;
        PWMs_cmd.data[15] = 1000;
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(1000.);
	PWMs_val.data[1] = pwmMapping(1000.);
	PWMs_val.data[2] = pwmMapping(1000.);
	PWMs_val.data[3] = pwmMapping(1000.);
	PWMs_val.data[4] = pwmMapping(1000.);
	PWMs_val.data[5] = pwmMapping(1000.);
	PWMs_val.data[6] = pwmMapping(1000.);
	PWMs_val.data[7] = pwmMapping(1000.);
	PWMs_val.data[8] = pwmMapping(1000.);
	PWMs_val.data[9] = pwmMapping(1000.);
	PWMs_val.data[10] = pwmMapping(1000.);
	PWMs_val.data[11] = pwmMapping(1000.);
	PWMs_val.data[12] = pwmMapping(1000.);
	PWMs_val.data[13] = pwmMapping(1000.);
	PWMs_val.data[14] = pwmMapping(1000.);
	PWMs_val.data[15] = pwmMapping(1000.);

}

void pwm_Arm(){
	PWMs_cmd.data.resize(8);
	PWMs_cmd.data[0] = 1200;
	PWMs_cmd.data[1] = 1200;
	PWMs_cmd.data[2] = 1200;
	PWMs_cmd.data[3] = 1200;
	PWMs_cmd.data[4] = 1200;
	PWMs_cmd.data[5] = 1200;
	PWMs_cmd.data[6] = 1200;
	PWMs_cmd.data[7] = 1200;
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(2000.);
	PWMs_val.data[1] = pwmMapping(2000.);
	PWMs_val.data[2] = pwmMapping(2000.);
	PWMs_val.data[3] = pwmMapping(2000.);
	PWMs_val.data[4] = pwmMapping(2000.);
	PWMs_val.data[5] = pwmMapping(2000.);
	PWMs_val.data[6] = pwmMapping(2000.);
	PWMs_val.data[7] = pwmMapping(2000.);
	PWMs_val.data[8] = pwmMapping(2000.);
	PWMs_val.data[9] = pwmMapping(2000.);
	PWMs_val.data[10] = pwmMapping(2000.);
	PWMs_val.data[11] = pwmMapping(2000.);
	PWMs_val.data[12] = pwmMapping(2000.);
	PWMs_val.data[13] = pwmMapping(2000.);
	PWMs_val.data[14] = pwmMapping(2000.);
	PWMs_val.data[15] = pwmMapping(2000.);

}
void pwm_Calibration(){
	//if(Sbus[4]>1500) pwm_Arm();
	//else pwm_Kill();
	if(kill_mode) pwm_Kill();
	else pwm_Arm();
//ROS_INFO("Sbus[6] : %d", Sbus[6]);	
}

void kalman_Filtering(){
	z << imu_ang_vel.x, imu_ang_vel.y, imu_ang_vel.z;
	predicted_x = F*x;
	predicted_P = F*P*F.transpose()+Q;
	gain_term = H*predicted_P*H.transpose()+R;
	K=predicted_P*H.transpose()*gain_term.inverse();
	x=predicted_x+K*(z-H*predicted_x);
	covariance_term = Eigen::MatrixXd::Identity(6,6)-K*H;
	P=covariance_term*predicted_P*covariance_term.transpose()+K*R*K.transpose();
	filtered_Angular_vel.x=x(0);
	filtered_Angular_vel.y=x(1);
	filtered_Angular_vel.z=x(2);
	filtered_Angular_accel.x=x(3);
	filtered_Angular_accel.y=x(4);
	filtered_Angular_accel.z=x(5);
}

void pid_Gain_Setting(){
	if(Sbus[7]<=1500){
		Par = conv_Par;
		Iar = conv_Iar;
		Dar = conv_Dar;

		Pap = conv_Pap;
                Iap = conv_Iap;
                Dap = conv_Dap;

		Py = conv_Py;
		Dy = conv_Dy;

		Pz = conv_Pz;
		Iz = conv_Iz;
		Dz = conv_Dz;
		
		Pv = conv_Pv;
		Iv = conv_Iv;
		Dv = conv_Dv;

		Pp = conv_Pp;
		Ip = conv_Ip;
		Dp = conv_Dp;
	}
	else{
		Par = tilt_Par;
		Iar = tilt_Iar;
		Dar = tilt_Dar;

		Pap = tilt_Pap;
                Iap = tilt_Iap;
                Dap = tilt_Dap;

		Py = tilt_Py;
		Dy = tilt_Dy;

		Pz = tilt_Pz;
		Iz = tilt_Iz;
		Dz = tilt_Dz;
		
		Pv = tilt_Pv;
		Iv = tilt_Iv;
		Dv = tilt_Dv;

		Pp = tilt_Pp;
		Ip = tilt_Ip;
		Dp = tilt_Dp;
	}
	//ROS_INFO("%.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf ",Pa, Ia, Da, Py, Dy, Pz, Iz, Dz, Pp, Ip, Dp);
}

void disturbance_Observer(){
	//DOB------------------------------------------------------------------------------
	//Nominal transfer function : q/tau = 1/Js^2    Q - 3rd order butterworth filter
	//Roll
	//Q*(Js^2) transfer function to state space 
	x_dot_r1 = -2*fq_cutoff*x_r1-2*pow(fq_cutoff,2)*x_r2-pow(fq_cutoff,3)*x_r3+imu_rpy.x;
	x_dot_r2 = x_r1;
	x_dot_r3 = x_r2;
    x_r1 += x_dot_r1*delta_t.count(); 
	x_r2 += x_dot_r2*delta_t.count(); 
	x_r3 += x_dot_r3*delta_t.count(); 
	double tauhat_r = J_x*pow(fq_cutoff,3)*x_r1;

	//Q transfer function to state space
	y_dot_r1 = -2*fq_cutoff*y_r1-2*pow(fq_cutoff,2)*y_r2-pow(fq_cutoff,3)*y_r3+tautilde_r_d;
	y_dot_r2 = y_r1;
	y_dot_r3 = y_r2;
	y_r1 += y_dot_r1*delta_t.count();
	y_r2 += y_dot_r2*delta_t.count();
	y_r3 += y_dot_r3*delta_t.count();
	double Qtautilde_r = pow(fq_cutoff,3)*y_r3;

	dhat_r = tauhat_r - Qtautilde_r;


	//Pitch
	//Q*(Js^2) transfer function to state space 
	x_dot_p1 = -2*fq_cutoff*x_p1-2*pow(fq_cutoff,2)*x_p2-pow(fq_cutoff,3)*x_p3+imu_rpy.y;
	x_dot_p2 = x_p1;
	x_dot_p3 = x_p2;
	x_p1 += x_dot_p1*delta_t.count(); 
	x_p2 += x_dot_p2*delta_t.count(); 
	x_p3 += x_dot_p3*delta_t.count(); 
	double tauhat_p = J_y*pow(fq_cutoff,3)*x_p1;

	//Q transfer function to state space
	y_dot_p1 = -2*fq_cutoff*y_p1-2*pow(fq_cutoff,2)*y_p2-pow(fq_cutoff,3)*y_p3+tautilde_p_d;
	y_dot_p2 = y_p1;
	y_dot_p3 = y_p2;
	y_p1 += y_dot_p1*delta_t.count();
	y_p2 += y_dot_p2*delta_t.count();
	y_p3 += y_dot_p3*delta_t.count();
	double Qtautilde_p = pow(fq_cutoff,3)*y_p3;

	dhat_p = tauhat_p - Qtautilde_p;


	//Yaw
	//Q*(Js^2) transfer function to state space 
	x_dot_y1 = -2*fq_cutoff*x_y1-2*pow(fq_cutoff,2)*x_y2-pow(fq_cutoff,3)*x_y3+imu_rpy.z;
	x_dot_y2 = x_y1;
	x_dot_y3 = x_y2;
    x_y1 += x_dot_y1*delta_t.count(); 
	x_y2 += x_dot_y2*delta_t.count(); 
	x_y3 += x_dot_y3*delta_t.count(); 
	double tauhat_y = J_z*pow(fq_cutoff,3)*x_y1;

	//Q transfer function to state space
	y_dot_y1 = -2*fq_cutoff*y_y1-2*pow(fq_cutoff,2)*y_y2-pow(fq_cutoff,3)*y_y3+tautilde_y_d;
	y_dot_y2 = y_y1;
	y_dot_y3 = y_y2;
	y_y1 += y_dot_y1*delta_t.count();
	y_y2 += y_dot_y2*delta_t.count();
	y_y3 += y_dot_y3*delta_t.count();
	double Qtautilde_y = pow(fq_cutoff,3)*y_y3;

	double dhat_y = tauhat_y - Qtautilde_y;
	dhat.x = dhat_r;
	dhat.y = dhat_p;
	dhat.z = dhat_y;

	//tautilde_y_d = tau_y_d - dhat_y;
    tautilde_y_d = tau_y_d;
	//--------------------------------------------------------------------------------------
}

void sine_wave_vibration(){
	vibration1 = Amp_Z*sin(pass_freq1*time_count);
	vibration2 = Amp_XY*sin(pass_freq2*time_count);
	sine_wave.x = vibration1;
	sine_wave.y = vibration2;
	time_count += delta_t.count();
}
