#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <std_msgs/String.h>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <vector>

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

#include <string>


double freq=200;//controller loop frequency
double pwm_freq=417.3;//pwm signal frequency

std::chrono::duration<double> delta_t;
int16_t Sbus[10];
int16_t PWM_d;
int16_t loop_time;
std_msgs::Int16MultiArray PWMs_cmd;
std_msgs::Int32MultiArray PWMs_val;
std_msgs::Float32MultiArray Force;
std_msgs::Float32 force_alpha;
std_msgs::Float32MultiArray kill_tilt_switch;
std_msgs::String kill_tilt_switch_str;

sensor_msgs::JointState rac_servo_value;
sensor_msgs::Imu imu;
geometry_msgs::Quaternion imu_quaternion;
geometry_msgs::Vector3 imu_rpy;
geometry_msgs::Vector3 imu_ang_vel;
geometry_msgs::Vector3 imu_lin_acc;
geometry_msgs::Vector3 angle_d;
geometry_msgs::Vector3 angular_acc_d;
Eigen::Vector3d rpy_ddot_d; // angular acceleration desired vector for cal.
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
geometry_msgs::Vector3 desired_lin_vel;
geometry_msgs::Vector3 t265_att;
geometry_msgs::Vector3 filtered_angular_rate;
geometry_msgs::Vector3 lin_acl;
std_msgs::Float32 altitude_d;
std_msgs::Float32 battery_voltage_msg;
std_msgs::Float32 battery_real_voltage;
std_msgs::Float32 dt;
geometry_msgs::Vector3 external_force;
geometry_msgs::Vector3 desired_position_change;
geometry_msgs::Vector3 reference_position;

bool servo_sw=false;
double theta1_command, theta2_command, theta3_command, theta4_command;
bool start_flag=false;
bool tilting_flag=false;

//Mode selection flag
bool attitude_mode = false;
bool velocity_mode = false;
bool position_mode = false;
bool kill_mode = true;
bool altitude_mode = false;
bool tilt_mode = false;
bool ESC_control = false;
bool admittance_mode = false;
//Thruster_cmd
double F1 = 0;//desired propeller 1 force
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
double tau_y_sin = 0; //yaw sine term torque (N.m)

double r_ddot_d = 0; //roll angular acceleration
double p_ddot_d = 0; //pitch angular acceleration
double y_ddot_d = 0; //yaw angular acceleration

double Thrust_d = 0;//altitude desired thrust(N)

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

//Admittance control value
double X_e = 0;
double Y_e = 0;
double X_r = 0;
double Y_r = 0;
double Z_r = 0;
double X_e_x1=0;
double X_e_x2=0;
double X_e_x1_dot=0;
double X_e_x2_dot=0;
double Y_e_x1=0;
double Y_e_x2=0;
double Y_e_x1_dot=0;
double Y_e_x2_dot=0;
double M=0.5;
double D=20.0;
double K=0;
double external_force_deadzone=3.5; //N

//Force estimation lpf
double Fe_x_x_dot = 0;
double Fe_y_x_dot = 0;
double Fe_z_x_dot = 0;
double Fe_x_x = 0;
double Fe_y_x = 0;
double Fe_z_x = 0;
double Fe_cutoff_freq = 1.0;
//--------------------------------------------------------

//General dimensions

static double r_arm = 0.3025;// m // diagonal length between thruster x2
static double l_servo = 0.035;
static double mass = 5.6;//2.9;//3.8; 2.365;//(Kg)
static double mass_sub1 = 0;
static double mass_sub2 = 0;

static double r2=sqrt(2);
static double l_module = 0.50; //(m) module horizontal body length

//Propeller constants(DJI E800(3510 motors + 620S ESCs))
static double xi = 0.01;//F_i=k*(omega_i)^2, M_i=b*(omega_i)^2
//--------------------------------------------------------

//General parameters======================================

static double pi = 3.141592;//(rad)
static double g = 9.80665;//(m/s^2)

static double rp_limit = 0.25;//(rad)
static double y_vel_limit = 0.01;//(rad/s)
static double y_d_tangent_deadzone = (double)0.05 * y_vel_limit;//(rad/s)
static double T_limit = 80;//(N)
static double altitude_limit = 1;//(m)
static double XY_limit = 1.0;
static double XYZ_dot_limit=1;
static double XYZ_ddot_limit=2;
static double alpha_beta_limit=1;
static double hardware_servo_limit=0.3;
static double servo_command_limit = 0.3;
static double tau_y_limit = 1.0;

double x_c_hat=0.0;
double y_c_hat=0.0;
double z_c_hat=0.0;

//  Inertia Tensor elements


//Original inertia tensor diagonal
double J_xx = 0.01;
double J_yy = 0.01;
double J_zz = 0.1;
//Original inertia tensor off-diagonal
double J_xy = 0;
double J_xz = 0;

double J_yx = 0;
double J_yz = 0;

double J_zx = 0;
double J_zy = 0;

//


//--------------------------------------------------------

//Control gains===========================================

//integratior(PID) limitation
double integ_limit=10;
double z_integ_limit=100;
double pos_integ_limit=10;
double vel_integ_limit=10;

//Roll, Pitch PID gains
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
double conv_Pa, conv_Ia, conv_Da;
double conv_Py, conv_Dy;
double conv_Pz, conv_Iz, conv_Dz;
double conv_Pv, conv_Iv, conv_Dv;
double conv_Pp, conv_Ip, conv_Dp;

//Tilt Flight Mode Control Gains
double tilt_Pa, tilt_Ia, tilt_Da;
double tilt_Py, tilt_Dy;
double tilt_Pz, tilt_Iz, tilt_Dz;
double tilt_Pv, tilt_Iv, tilt_Dv;
double tilt_Pp, tilt_Ip, tilt_Dp;
//--------------------------------------------------------

//Servo angle=============================================
double theta1=0,theta2=0,theta3=0,theta4=0;
//--------------------------------------------------------

//Voltage=================================================
double voltage=22.4;
double voltage_old=22.4;
//--------------------------------------------------------


//-DOB----------------------------------------------------
geometry_msgs::Vector3 dhat;
double fq_cutoff=0.5;//Q filter Cut-off frequency

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


//Publisher Group--------------------------------------
ros::Publisher PWMs;
ros::Publisher goal_dynamixel_position_;
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
ros::Publisher delta_time;
ros::Publisher desired_velocity;
ros::Publisher Center_of_Mass;
ros::Publisher angular_Acceleration;

ros::Publisher disturbance;
ros::Publisher linear_acceleration;
ros::Publisher External_force_data;
ros::Publisher reference_desired_pos_error;
ros::Publisher reference_pos;

ros::Publisher force_alpha_pub; //for force distributor
ros::Publisher kill_tilt_switch_pub; // for sub module
ros::Publisher kill_tilt_switch_str_pub;


//Control Matrix---------------------------------------
//Eigen::MatrixXd CM(4,8);
Eigen::MatrixXd CM(4,4); //Thrust Allocation
Eigen::MatrixXd SA(3,4); //Servo Allocation
//Eigen::Vector4d u;
Eigen::VectorXd u(4);
Eigen::VectorXd F_cmd(4);
Eigen::VectorXd control_by_theta(3);
Eigen::MatrixXd invCM(4,4);
Eigen::MatrixXd pinvSA(4,3);

Eigen::MatrixXd CM_Xc_p2(4,4); // Thrust Allocation :: Xc_p2
Eigen::MatrixXd invCM_Xc_p2(4,4); 
//-----------------------------------------------------

//desired torque commend

Eigen::Vector3d tau_cmd;

//Linear_velocity--------------------------------------
Eigen::Vector3d cam_v;
Eigen::Matrix3d R_v;
Eigen::Vector3d v;
//-----------------------------------------------------

//Attitude--------------------------------------
Eigen::Vector3d cam_att;
//-----------------------------------------------------

//Rotation_Matrix--------------------------------------
Eigen::Matrix3d Rotz;
Eigen::Matrix3d Roty;
Eigen::Matrix3d Rotx;
//-----------------------------------------------------

//Hat_Moment_of_Inertia_Tensor_Group ----------------------23_08_01
Eigen::Matrix3d hat_MoI; //final value of combined MoI Calculation
Eigen::Matrix3d origin_MoI; // original MoI Tensor
Eigen::Matrix3d hat_CoM_x_main; // CoM Cross product matrix :: main
Eigen::Matrix3d hat_CoM_x_sub1; // CoM Cross product matrix :: sub1
Eigen::Matrix3d hat_CoM_x_sub2; // CoM Cross product matrix :: sub2
Eigen::Matrix3d accent_MoI; // J'_i :: each module MoI w.r.t. main coordinates
Eigen::Matrix3d accent_MoI_main; //for J_hat :: main
Eigen::Matrix3d accent_MoI_sub1; //for J_hat :: sub1
Eigen::Matrix3d accent_MoI_sub2; //for J_hat :: sub2

geometry_msgs::Vector3 CoM; // hat CoM Vector for debugging :: system
Eigen::Vector3d X_c_p1; // for force distributor
Eigen::Vector3d X_c_p2; // for case 2345 CoM compensation --> setCM (M_bar matrix) 
Eigen::Vector3d p_c_main;
Eigen::Vector3d p_c_sub1;
Eigen::Vector3d p_c_sub2;

// For Force distribute && alpha
Eigen::VectorXd allocation_factor(2);
Eigen::MatrixXd K_M(4,2);
Eigen::MatrixXd invK_M(2,4);
Eigen::VectorXd Dump(4);

int case_num = 0;
int module_num_count = 1; // number of modules
int shape_num=2;
int toggle_sub1 = 0; // setMoI toggle flag
int toggle_sub2 = 0;

// 

//Timer------------------------------------------------
auto end  =std::chrono::high_resolution_clock::now();
auto start=std::chrono::high_resolution_clock::now();
//-----------------------------------------------------

geometry_msgs::Vector3 prev_angular_Vel;
geometry_msgs::Vector3 angular_Accel;


//-----------------------------------------------------

//-----------------------------------------------------
//Accelerometer LPF------------------------------------
double x_ax_dot = 0;
double x_ay_dot = 0;
double x_az_dot = 0;
double x_ax = 0;
double x_ay = 0;
double x_az = 0;
double accel_cutoff_freq = 1.0;
//-----------------------------------------------------
//Position DOB-----------------------------------------
double pos_dob_cutoff_freq=1.0;

Eigen::MatrixXd MinvQ_A(4,4);
Eigen::MatrixXd MinvQ_B(4,1);
Eigen::MatrixXd MinvQ_C(1,4);
Eigen::MatrixXd Q_A(2,2);
Eigen::MatrixXd Q_B(2,1);
Eigen::MatrixXd Q_C(1,2);
Eigen::MatrixXd MinvQ_X_x(4,1);
Eigen::MatrixXd MinvQ_X_x_dot(4,1);
Eigen::MatrixXd MinvQ_X_y(1,1);
Eigen::MatrixXd Q_X_x(2,1);
Eigen::MatrixXd Q_X_x_dot(2,1);
Eigen::MatrixXd Q_X_y(1,1);
Eigen::MatrixXd MinvQ_Y_x(4,1);
Eigen::MatrixXd MinvQ_Y_x_dot(4,1);
Eigen::MatrixXd MinvQ_Y_y(1,1);
Eigen::MatrixXd Q_Y_x(2,1);
Eigen::MatrixXd Q_Y_x_dot(2,1);
Eigen::MatrixXd Q_Y_y(1,1);

double dhat_X = 0;
double dhat_Y = 0;
double X_tilde_r = 0;
double Y_tilde_r = 0;


static char sSTX() { return static_cast<char>(0x02);}
static char sETX() { return static_cast<char>(0x03);}
static char sEOT() { return static_cast<char>(0x04);}
static char sENQ() { return static_cast<char>(0x05);}
static char sACK() { return static_cast<char>(0x06);}
static char sNAK() { return static_cast<char>(0x15);}
static char sCR() { return static_cast<char>(13);}
static char sLF() { return static_cast<char>(10);}
// std::to_string(sSTX())
// std::to_string(sETX())
std::string buf1 = "abc";
std::string buf2 = "def";
// Function Define ------------------------------ //

std::string data_2_string(){
        std::string buffer = "<" +std::to_string(111.11)+"PE"+std::to_string(222.22)+"TE"+std::to_string(333.33)+"KE"+std::to_string(444.44)+"XE"+std::to_string(555.55)+"YE"+std::to_string(666.66)+"ZE"
        +std::to_string(kill_mode)+">";
        return buffer;
}

void shape_selector(int num){
	// combination rule 
	// 1_ main drone must be located on the far left
	// 2_ main drone must be located at the bottom
	// 3_ sub1 drone must be battery exchanger	
	
	// setMoI value changed in here	
	if(num==1){
		                
                p_c_main << x_c_hat,y_c_hat,z_c_hat;
                p_c_sub1 << 0, 0, 0;
                p_c_sub2 << 0, 0, 0;

		mass_sub1 = 0;
                mass_sub2 = 0;

                toggle_sub1 = 0;
                toggle_sub2 = 0;
                
                X_c_p1 << 0,0,0;
                X_c_p2 << x_c_hat,y_c_hat,z_c_hat;

        

	}
	else if(num==2){
                

                p_c_main << x_c_hat,y_c_hat,z_c_hat;
                p_c_sub1 << x_c_hat,y_c_hat-l_module,z_c_hat;
                p_c_sub2 << 0,0,0;

		mass_sub1=5.6;
                mass_sub2=0;

                toggle_sub1=1;
                toggle_sub2=0;

                X_c_p1 << 0,y_c_hat,z_c_hat;
                X_c_p2 << x_c_hat,0,0;

	}
	else if(num==3){
	
		p_c_main << x_c_hat,y_c_hat,z_c_hat;
                p_c_sub1 << 0,0,0;
                p_c_sub2 << x_c_hat-l_module,y_c_hat,z_c_hat;

		mass_sub1=0;
                mass_sub2=5.6;

                toggle_sub1=0;
                toggle_sub2=1;

                X_c_p1 << 0,y_c_hat,0;
                X_c_p2 << x_c_hat,0,z_c_hat;


	}
	else if(num==4){
                

                p_c_main << x_c_hat,y_c_hat,z_c_hat;
                p_c_sub1 << x_c_hat,y_c_hat-l_module,z_c_hat;
                p_c_sub2 << x_c_hat-2*l_module,y_c_hat,z_c_hat;
             
		mass_sub1=5.6;
                mass_sub2=5.6;

                toggle_sub1=1;
                toggle_sub2=1;

                X_c_p1 << 0,y_c_hat,z_c_hat;
                X_c_p2 << x_c_hat,0,0;

	}
	else if(num==5){
                

                p_c_main << x_c_hat,y_c_hat,z_c_hat;
                p_c_sub1 << x_c_hat-2*l_module,y_c_hat-l_module,z_c_hat;
                p_c_sub2 << x_c_hat-l_module,y_c_hat,z_c_hat;


		mass_sub1=5.6;
                mass_sub2=5.6;

                toggle_sub1=1;
                toggle_sub2=1;

                X_c_p1 << x_c_hat, 0, z_c_hat;
                X_c_p2 << 0, y_c_hat, 0;
	}
	else if(num==6){
                
                p_c_main << x_c_hat,y_c_hat,z_c_hat;
                p_c_sub1 << x_c_hat-l_module,y_c_hat-l_module,z_c_hat;
                p_c_sub2 << x_c_hat,y_c_hat-l_module,z_c_hat;
              
		mass_sub1=5.6;
                mass_sub2=5.6;
                toggle_sub1=1;
                toggle_sub2=1;

                X_c_p1 << x_c_hat, y_c_hat, z_c_hat;
                X_c_p2 << 0, 0, 0;
	}
	else if(num==7){


                p_c_main << x_c_hat,y_c_hat,z_c_hat;
                p_c_sub1 << x_c_hat,y_c_hat-l_module,z_c_hat;
                p_c_sub2 << x_c_hat-l_module,y_c_hat,z_c_hat;

		mass_sub1=5.6;
                mass_sub2=5.6;
                toggle_sub1=1;
                toggle_sub2=1;

                X_c_p1 << x_c_hat, y_c_hat, z_c_hat;
                X_c_p2 << 0, 0, 0;
	}
	//J_hat=sigma(J_accent_i) 
	
	

	// if case167, enter the case selector
	// if case2,3,4,5 must  pass by the case selection 
}

void K_matrix(){

        K_M << F_yd*X_c_p1(2)-F_zd*X_c_p1(1), F_yd*X_c_p1(2)-F_zd*X_c_p1(2),
                -(F_xd*X_c_p1(2)-F_zd*X_c_p1(1)), -(F_xd*X_c_p1(2)-F_zd*(X_c_p1(1)-l_module)),
                F_xd*X_c_p1(1)-F_yd*X_c_p1(0), F_xd*(X_c_p1(1)-l_module)-F_yd*X_c_p1(0),
                1,      1;

        invK_M = K_M.completeOrthogonalDecomposition().pseudoInverse();

        allocation_factor = invK_M*Dump;
        //ROS_INFO("%f %f", allocation_factor(0),allocation_factor(1));
        /*
        ROS_INFO("%f %f \n %f %f \n %f %f \n %f %f",
        invK_M(0,0),invK_M(0,1),invK_M(0,2),invK_M(0,3),invK_M(1,0),invK_M(1,1),invK_M(1,2),invK_M(1,3));*/
}

void setMoI(){
	
	origin_MoI << J_xx, -J_xy, -J_xz,
		      -J_yx, J_yy, -J_yz,
		      -J_zx, -J_zy, J_zz;

	hat_CoM_x_main << 0, -p_c_main(2), p_c_main(1), 
                          p_c_main(2), 0, -p_c_main(0),
                          -p_c_main(1), p_c_main(0), 0;

        hat_CoM_x_sub1 << 
		0, -p_c_sub1(2), p_c_sub1(1),
		p_c_sub1(2), 0, -p_c_sub1(0),
	       -p_c_sub1(1), p_c_sub1(0), 0;

        hat_CoM_x_sub2 << 
		0, -p_c_sub2(2), p_c_sub2(1),
		p_c_sub2(2), 0, -p_c_sub2(0),
	       -p_c_sub2(1), p_c_sub2(0), 0;



	
	hat_MoI = (origin_MoI  + mass*hat_CoM_x_main*hat_CoM_x_main)+
                        toggle_sub1*(origin_MoI  + mass*hat_CoM_x_sub1*hat_CoM_x_sub1)+
                        toggle_sub2*(origin_MoI  + mass*hat_CoM_x_sub2*hat_CoM_x_sub2);
        /*	
	ROS_INFO("%f|%f|%f, %f|%f|%f, %f|%f|%f",
	hat_MoI(0,0),hat_MoI(0,1),hat_MoI(0,2),
	hat_MoI(1,0),hat_MoI(1,1),hat_MoI(1,2),
	hat_MoI(2,0),hat_MoI(2,1),hat_MoI(2,2));*/
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
        double tauhat_r = J_xx*pow(fq_cutoff,3)*x_r1;

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
        double tauhat_p = J_yy*pow(fq_cutoff,3)*x_p1;

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
        double tauhat_y = J_zz*pow(fq_cutoff,3)*x_y1;

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




void get_Rotation_matrix(){
        Rotz << cos(imu_rpy.z), -sin(imu_rpy.z),   0,
                sin(imu_rpy.z),  cos(imu_rpy.z),   0,
                             0,               0, 1.0;

        Roty << cos(imu_rpy.y),   0, sin(imu_rpy.y),
                             0, 1.0,              0,
               -sin(imu_rpy.y),   0, cos(imu_rpy.y);

        Rotx << 1.0,              0,               0,
                  0, cos(imu_rpy.x), -sin(imu_rpy.x),
                  0, sin(imu_rpy.x),  cos(imu_rpy.x);
}


void external_force_estimation(){
        get_Rotation_matrix();

        x_ax_dot=-accel_cutoff_freq*x_ax+imu_lin_acc.x;
        x_ax+=x_ax_dot*delta_t.count();
        x_ay_dot=-accel_cutoff_freq*x_ay+imu_lin_acc.y;
        x_ay+=x_ay_dot*delta_t.count();
        x_az_dot=-accel_cutoff_freq*x_az+imu_lin_acc.z;
        x_az+=x_az_dot*delta_t.count();
        lin_acl.x=accel_cutoff_freq*x_ax;
        lin_acl.y=accel_cutoff_freq*x_ay;
        lin_acl.z=accel_cutoff_freq*x_az;

        double Fx=1.0/4.0*mass*g*(sin(theta1)+sin(theta2)-sin(theta3)-sin(theta4))/r2;
        double Fy=1.0/4.0*mass*g*(sin(theta1)-sin(theta2)-sin(theta3)+sin(theta4))/r2;
        double Fz=-mass*g;

        double body_x_ddot_error=lin_acl.x-Fx/mass;
        double body_y_ddot_error=lin_acl.y-Fy/mass;
        double body_z_ddot_error=lin_acl.z-Fz/mass;

        Eigen::Vector3d Fe;
        Eigen::Vector3d body_accel_error;

        body_accel_error << body_x_ddot_error, body_y_ddot_error, body_z_ddot_error;
        Fe = mass*Rotz*Roty*Rotx*body_accel_error;

        Fe_x_x_dot = -Fe_cutoff_freq*Fe_x_x+Fe(0);
        Fe_x_x+=Fe_x_x_dot*delta_t.count();
        Fe_y_x_dot = -Fe_cutoff_freq*Fe_y_x+Fe(1);
        Fe_y_x+=Fe_y_x_dot*delta_t.count();
        Fe_z_x_dot = -Fe_cutoff_freq*Fe_z_x+Fe(2);
        Fe_z_x+=Fe_z_x_dot*delta_t.count();

        external_force.x=Fe_cutoff_freq*Fe_x_x;
        external_force.y=Fe_cutoff_freq*Fe_y_x;
        external_force.z=Fe_cutoff_freq*Fe_z_x;

        if(fabs(external_force.x)<external_force_deadzone) external_force.x=0;
        if(fabs(external_force.y)<external_force_deadzone) external_force.y=0;
        if(fabs(external_force.z)<external_force_deadzone) external_force.z=0;
}

void admittance_controller(){

                X_e_x1_dot=-D/M*X_e_x1-K/M*X_e_x2+external_force.x;
                X_e_x2_dot=X_e_x1;
                X_e_x1+=X_e_x1_dot*delta_t.count();
                X_e_x2+=X_e_x2_dot*delta_t.count();
                X_e=-1.0/M*X_e_x2;

                Y_e_x1_dot=-D/M*Y_e_x1-K/M*X_e_x2+external_force.y;
                Y_e_x2_dot=Y_e_x1;
                Y_e_x1+=Y_e_x1_dot*delta_t.count();
                Y_e_x2+=Y_e_x2_dot*delta_t.count();
                Y_e=-1.0/M*Y_e_x2;

        X_r=X_d-X_e;
        Y_r=Y_d-Y_e;
        Z_r=Z_d;

        desired_position_change.x=X_r;
        desired_position_change.y=Y_r;

}

void position_dob(){
        MinvQ_X_x_dot=MinvQ_A*MinvQ_X_x+MinvQ_B*pos.x;
        MinvQ_X_x+=MinvQ_X_x_dot*delta_t.count();
        MinvQ_X_y=MinvQ_C*MinvQ_X_x;

        Q_X_x_dot=Q_A*Q_X_x+Q_B*X_tilde_r;
        Q_X_x+=Q_X_x_dot*delta_t.count();
        Q_X_y=Q_C*Q_X_x;
        dhat_X=MinvQ_X_y(0)-Q_X_y(0);

        MinvQ_Y_x_dot=MinvQ_A*MinvQ_Y_x+MinvQ_B*pos.y;
        MinvQ_Y_x+=MinvQ_Y_x_dot*delta_t.count();
        MinvQ_Y_y=MinvQ_C*MinvQ_Y_x;

        Q_Y_x_dot=Q_A*Q_Y_x+Q_B*Y_tilde_r;
        Q_Y_x+=Q_Y_x_dot*delta_t.count();
        Q_Y_y=Q_C*Q_Y_x;
        dhat_Y=MinvQ_Y_y(0)-Q_Y_y(0);

        X_tilde_r=X_r-dhat_X;
        Y_tilde_r=Y_r-dhat_Y;

        reference_position.x=X_tilde_r;
        reference_position.y=Y_tilde_r;
}
