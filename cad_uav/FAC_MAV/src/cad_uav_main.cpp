//2022.05.16 Coaxial-Octorotor version
//2022.06.23 Ground Station Application
//2022.08.XX DOB (Disturbance Observer) Application
//2022.09.05 ESC (Extremum Seeking Control) Application
//2022.09.21 Controller mode selection Application

#include <cad_uav_util.hpp>
#include <vector>


//Function Operator--------------------------------------------------------------------



void rpyT_ctrl();
void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des);
double Force_to_PWM(double F);
void jointstateCallback(const sensor_msgs::JointState& msg);
void imu_Callback(const sensor_msgs::Imu& msg);
sensor_msgs::JointState servo_msg_create(double desired_theta1, double desired_theta2, double desired_theta3, double desired_theta4);
void sbusCallback(const std_msgs::Int16MultiArray::ConstPtr& array);
void batteryCallback(const std_msgs::Int16& msg);
void posCallback(const geometry_msgs::Vector3& msg);
void rotCallback(const geometry_msgs::Quaternion& msg);
void filterCallback(const sensor_msgs::Imu& msg);
void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void setCM();
void setCM_Xc_p2();
void setSA();
void setMoI();//23_08_01
void publisherSet();
int32_t pwmMapping(double pwm);
void pwm_Command(double pwm1, double pwm2, double pwm3, double pwm4);
void pwm_Kill();
void pwm_Max();
void pwm_Arm();
void pwm_Calibration();
void pid_Gain_Setting();
void disturbance_Observer();
void sine_wave_vibration();
void get_Rotation_matrix();
void external_force_estimation();
void admittance_controller();
void position_dob();
//23.08.03---------------------------------------------
void shape_selector(int num);
void force_distributor();
void K_matrix();
//-----------------------------------------------------


int main(int argc, char **argv){
	
    	ros::init(argc, argv,"cad_uav_main_ctrl");

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
		x_c_hat=nh.param<double>("x_center_of_mass",1.0);
		y_c_hat=nh.param<double>("y_center_of_mass",1.0);
		z_c_hat=nh.param<double>("z_center_of_mass",1.0);
		CoM.x = x_c_hat;
		CoM.y = y_c_hat;
		CoM.z = z_c_hat;

		//Conventional Flight Mode Control Gains
			//Roll, Pitch PID gains
			conv_Pa=nh.param<double>("conv_attitude_rp_P_gain",3.5);
			conv_Ia=nh.param<double>("conv_attitude_rp_I_gain",0.4);
			conv_Da=nh.param<double>("conv_attitude_rp_D_gain",0.5);

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
			tilt_Pa=nh.param<double>("tilt_attitude_rp_P_gain",3.5);
			tilt_Ia=nh.param<double>("tilt_attitude_rp_I_gain",0.4);
			tilt_Da=nh.param<double>("tilt_attitude_rp_D_gain",0.5);

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
	
	//set position DOB------------------------------------------
	MinvQ_A << -(tilt_Pp+r2*tilt_Dp*pos_dob_cutoff_freq)/tilt_Dp, -(tilt_Dp*pow(pos_dob_cutoff_freq,2)+r2*tilt_Pp*pos_dob_cutoff_freq+tilt_Ip)/tilt_Dp, -(tilt_Pp*pow(pos_dob_cutoff_freq,2)+r2*tilt_Ip*pos_dob_cutoff_freq)/tilt_Dp, -(tilt_Ip*pow(pos_dob_cutoff_freq,2))/tilt_Dp,
	           1.0,   0,   0, 0,
	             0, 1.0,   0, 0,
	             0,   0, 1.0, 0;
	MinvQ_B << 1.0, 0, 0, 0;
	MinvQ_C << (mass*pos_dob_cutoff_freq)/tilt_Dp, pos_dob_cutoff_freq, (tilt_Pp*pos_dob_cutoff_freq)/tilt_Dp, (tilt_Ip*pos_dob_cutoff_freq)/tilt_Dp;

	Q_A << -r2*pos_dob_cutoff_freq, -pow(pos_dob_cutoff_freq,2),
                                   1.0,                           0;
	Q_B << 1.0, 0;
	Q_C << 0, pow(pos_dob_cutoff_freq,2);
	//----------------------------------------------------------
	
	//Initialization parameter matrix & vector for Control -----
		p_c_main << 0, 0, 0;
		p_c_sub1 << 0, 0, 0;
		p_c_sub2 << 0, 0, 0;
		setMoI();
		//setCM(); //Control matrix
        setCM_Xc_p2(); // Control matrix w.r.t. Xc_p2
		K_matrix();
		F_cmd << 0, 0, 0, 0; //force command
		rpy_ddot_d << 0, 0, 0; // angular_acceleration desired
		tau_cmd << 0, 0, 0; //torque command
		Dump << 0,0,0,1; // for allocation factor calculation
		

	//----------------------------------------------------------


    PWMs = nh.advertise<std_msgs::Int16MultiArray>("PWMs", 1); // PWM 1,2,3,4
    PWM_generator = nh.advertise<std_msgs::Int32MultiArray>("command",1);  // publish to pca9685
    goal_dynamixel_position_  = nh.advertise<sensor_msgs::JointState>("goal_dynamixel_position",100); // desired theta1,2
	euler = nh.advertise<geometry_msgs::Vector3>("angle",1); // roll, pitch, yaw
	desired_angle = nh.advertise<geometry_msgs::Vector3>("desired_angle",100);
	Forces = nh.advertise<std_msgs::Float32MultiArray>("Forces",100); // F 1,2,3,4
	desired_torque = nh.advertise<geometry_msgs::Vector3>("torque_d",100);
	linear_velocity = nh.advertise<geometry_msgs::Vector3>("lin_vel",100);
	angular_velocity = nh.advertise<geometry_msgs::Vector3>("gyro",100);
	desired_position = nh.advertise<geometry_msgs::Vector3>("pos_d",100);
	position = nh.advertise<geometry_msgs::Vector3>("pos",100);
	desired_force = nh.advertise<geometry_msgs::Vector3>("force_d",100);
	battery_voltage = nh.advertise<std_msgs::Float32>("battery_voltage",100);
	delta_time = nh.advertise<std_msgs::Float32>("delta_t",100);
	desired_velocity = nh.advertise<geometry_msgs::Vector3>("lin_vel_d",100);
	Center_of_Mass = nh.advertise<geometry_msgs::Vector3>("Center_of_Mass",100);
	angular_Acceleration = nh.advertise<geometry_msgs::Vector3>("ang_accel",100);
	
	disturbance = nh.advertise<geometry_msgs::Vector3>("dhat",100);
	linear_acceleration = nh.advertise<geometry_msgs::Vector3>("imu_lin_acl",100);
	External_force_data = nh.advertise<geometry_msgs::Vector3>("external_force",100);
	reference_desired_pos_error = nh.advertise<geometry_msgs::Vector3>("pos_e",100);
	reference_pos = nh.advertise<geometry_msgs::Vector3>("pos_r",100);
	//---
	kill_tilt_switch_str_pub = nh.advertise<std_msgs::String>("kill_tilt_switch_str",1);
	//---

    ros::Subscriber dynamixel_state = nh.subscribe("joint_states",100,jointstateCallback,ros::TransportHints().tcpNoDelay());
   	ros::Subscriber att = nh.subscribe("/imu/data",1,imu_Callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber rc_in = nh.subscribe("/sbus",100,sbusCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber battery_checker = nh.subscribe("/battery",100,batteryCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_pos=nh.subscribe("/t265_pos",100,posCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_rot=nh.subscribe("/t265_rot",100,rotCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_odom=nh.subscribe("/rs_t265/odom/sample",100,t265OdomCallback,ros::TransportHints().tcpNoDelay());
	
	ros::Timer timerPublish = nh.createTimer(ros::Duration(1.0/200.0),std::bind(publisherSet));
    ros::spin();
    return 0;
}

void publisherSet(){
	


	//-parameter changer w.r.t. module shape-//
	
	//description//
	//if module combined left or right in 2 modules flight case
	// if
	// case 1 :: 1drone - 
	// case 2 :: 2drone --
	// case 3 :: 2drone : 
	// case 4 :: 3drone --- 
	// case 5 :: 3drone |
	// case 6 :: 3drone r
	// case 7 :: 3drone L
		
	
	//--Timer Update-------------------------------//

	end=std::chrono::high_resolution_clock::now();
	delta_t=end-start; // use for derivation 
	dt.data=delta_t.count();
	start=std::chrono::high_resolution_clock::now();
	//-------------------------------------------//
	
	shape_selector(2);

	setMoI();
	/*
	ROS_INFO("%f|%f|%f, %f|%f|%f, %f|%f|%f",
	hat_MoI(0,0),hat_MoI(0,1),hat_MoI(0,2),
	hat_MoI(1,0),hat_MoI(1,1),hat_MoI(1,2),
	hat_MoI(2,0),hat_MoI(2,1),hat_MoI(2,2));*/
	// Control Allocation Matrix update	
	//setCM();
	setCM_Xc_p2();
	//------------------------------
	
	// Inertia Tensor update 23.08.01
	//------------------------------
	
	//angular_accerlation update-----------------------------------------
	angular_Accel.x = (imu_ang_vel.x-prev_angular_Vel.x)/delta_t.count();
	angular_Accel.y = (imu_ang_vel.y-prev_angular_Vel.y)/delta_t.count();
	angular_Accel.z = (imu_ang_vel.z-prev_angular_Vel.z)/delta_t.count();
	//-------------------------------------------------------------------
	
	
	if(!position_mode){
		X_d_base=pos.x;
		Y_d_base=pos.y;
		X_d = X_d_base;
		Y_d = Y_d_base;
		X_r = X_d;
		Y_r = Y_d;
		e_X_i=0;
		e_Y_i=0;
		if(attitude_mode){
			e_X_dot_i=0;
			e_Y_dot_i=0;	
		}	
	}

	if(false/*kill_mode*/){	
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
		theta1_command=0.0;
		theta2_command=0.0;
		theta3_command=0.0;
		theta4_command=0.0;
		pwm_Kill();
		//ROS_INFO("Kill mode");	
	}
	else{

		rpyT_ctrl();
	//	pwm_Arm();		

		//ROS_INFO("Arm mode");	
	}

//	pwm_Calibration();	
	angle_d.x=r_d;
	angle_d.y=p_d;
	angle_d.z=y_d;
	desired_pos.x = X_d;
	desired_pos.y = Y_d;
	desired_pos.z = Z_d;
	Force.data.resize(4);
	Force.data[0] = F1;
	Force.data[1] = F2;
	Force.data[2] = F3;
	Force.data[3] = F4;
	CoM.x = x_c_hat;
	CoM.y = y_c_hat;
	CoM.z = z_c_hat;
	PWMs.publish(PWMs_cmd);// PWMs_d value
	euler.publish(imu_rpy);//rpy_act value
	desired_angle.publish(angle_d);//rpy_d value
	Forces.publish(Force);// force conclusion
	goal_dynamixel_position_.publish(servo_msg_create(theta1_command,theta2_command, theta3_command, theta4_command)); // desired theta
	desired_torque.publish(torque_d); // torque desired
	linear_velocity.publish(lin_vel); // actual linear velocity 
	PWM_generator.publish(PWMs_val);
	desired_position.publish(desired_pos);//desired position 
	position.publish(pos); // actual position
	desired_force.publish(force_d); // desired force it need only tilt mode 
	battery_voltage.publish(battery_voltage_msg);
	delta_time.publish(dt);
	desired_velocity.publish(desired_lin_vel);
	Center_of_Mass.publish(CoM);
	angular_Acceleration.publish(angular_Accel);
	linear_acceleration.publish(lin_acl);
	disturbance.publish(dhat);

	//main에서 sub으로 보내야할 데이터 alpha factor
	//
	kill_tilt_switch_str.data = data_2_string();
	kill_tilt_switch_str_pub.publish(kill_tilt_switch_str);

	prev_angular_Vel = imu_ang_vel;
        prev_lin_vel = lin_vel;
//-------------------not use publish set -----------------------------------//	
	 
	External_force_data.publish(external_force);
	reference_desired_pos_error.publish(desired_position_change);
	reference_pos.publish(reference_position);

//-------------------------------------------------------------------------//
/*ROS_INFO("%f %f %f %f \n %f %f %f %f", CM(0,0),CM(0,1),CM(0,2),CM(0,3),
CM_Xc_p2(0,0),CM_Xc_p2(0,1),CM_Xc_p2(0,2),CM_Xc_p2(0,3));*/
}


void setCM(){
	//Co-rotating type //2023_07_27 update
	CM << (y_c_hat+r_arm/r2)*cos(theta1)+(-(l_servo-z_c_hat)+xi)*sin(theta1)/r2,  (y_c_hat+r_arm/r2)*cos(theta2)+((l_servo-z_c_hat)-xi)*sin(theta2)/r2,   (y_c_hat-r_arm/r2)*cos(theta3)+((l_servo-z_c_hat)-xi)*sin(theta3)/r2,  (y_c_hat-r_arm/r2)*cos(theta4)+(-(l_servo-z_c_hat)+xi)*sin(theta4)/r2,
	      -(x_c_hat-r_arm/r2)*cos(theta1)+((l_servo-z_c_hat)+xi)*sin(theta1)/r2, -(x_c_hat+r_arm/r2)*cos(theta2)+((l_servo-z_c_hat)+xi)*sin(theta2)/r2, -(x_c_hat+r_arm/r2)*cos(theta3)+(-(l_servo-z_c_hat)-xi)*sin(theta3)/r2, -(x_c_hat-r_arm/r2)*cos(theta4)+(-(l_servo-z_c_hat)-xi)*sin(theta4)/r2,
	      -xi*cos(theta1)+(r_arm-(x_c_hat-y_c_hat)/r2)*sin(theta1) 			   ,   xi*cos(theta2)+(r_arm+(x_c_hat+y_c_hat)/r2)*sin(theta2)			  ,   -xi*cos(theta3)+(r_arm+(x_c_hat-y_c_hat)/r2)*sin(theta3)			  ,   xi*cos(theta4)+(r_arm-(x_c_hat+y_c_hat)/r2)*sin(theta4),
																   -cos(theta1),                                                          -cos(theta2),                                                           -cos(theta3),                                                           -cos(theta4);
    invCM = CM.inverse();
//	      -xi*cos(theta1)+(y_c_hat-x_c_hat+r2*r_arm)*sin(theta1)/r2,  xi*cos(theta2)+(x_c_hat+y_c_hat+r2*r_arm)*sin(theta2)/r2,  -xi*cos(theta3)+(x_c_hat-y_c_hat+r2*r_arm)*sin(theta3)/r2,  xi*cos(theta4)+(-x_c_hat-y_c_hat+r2*r_arm)*sin(theta4)/r2,
}

void setCM_Xc_p2(){

	CM_Xc_p2 << (X_c_p2(1)+r_arm/r2)*cos(theta1)+(-(l_servo-X_c_p2(2))+xi)*sin(theta1)/r2,  (X_c_p2(1)+r_arm/r2)*cos(theta2)+((l_servo-X_c_p2(2))-xi)*sin(theta2)/r2,   (X_c_p2(1)-r_arm/r2)*cos(theta3)+((l_servo-X_c_p2(2))-xi)*sin(theta3)/r2,  (X_c_p2(1)-r_arm/r2)*cos(theta4)+(-(l_servo-X_c_p2(2))+xi)*sin(theta4)/r2,
	      -(X_c_p2(0)-r_arm/r2)*cos(theta1)+((l_servo-X_c_p2(2))+xi)*sin(theta1)/r2, -(X_c_p2(0)+r_arm/r2)*cos(theta2)+((l_servo-X_c_p2(2))+xi)*sin(theta2)/r2, -(X_c_p2(0)+r_arm/r2)*cos(theta3)+(-(l_servo-X_c_p2(2))-xi)*sin(theta3)/r2, -(X_c_p2(0)-r_arm/r2)*cos(theta4)+(-(l_servo-X_c_p2(2))-xi)*sin(theta4)/r2,
	      -xi*cos(theta1)+(r_arm-(X_c_p2(0)-X_c_p2(1))/r2)*sin(theta1) 			   ,   xi*cos(theta2)+(r_arm+(X_c_p2(0)+X_c_p2(1))/r2)*sin(theta2)			  ,   -xi*cos(theta3)+(r_arm+(X_c_p2(0)-X_c_p2(1))/r2)*sin(theta3)			  ,   xi*cos(theta4)+(r_arm-(X_c_p2(0)+X_c_p2(1))/r2)*sin(theta4),
																   -cos(theta1),                                                          -cos(theta2),                                                           -cos(theta3),                                                           -cos(theta4);
    
	invCM_Xc_p2 = CM_Xc_p2.inverse();
}

void setSA(){
	SA << F1/r2,  F2/r2, -F3/r2, -F4/r2,
	      F1/r2, -F2/r2, -F3/r2,  F4/r2;//,
	//	  (y_c_hat-x_c_hat+r2*r_arm)*F1/r2, (x_c_hat+y_c_hat+r2*r_arm)*F2/r2, (x_c_hat-y_c_hat+r2*r_arm)*F3/r2, (-x_c_hat-y_c_hat+r2*r_arm)*F4/r2;

//	std::cout << "SA : \n" << SA <<std::endl;
	pinvSA = SA.completeOrthogonalDecomposition().pseudoInverse();
}
void rpyT_ctrl() {
	
	pid_Gain_Setting();
	y_d_tangent=y_vel_limit*(((double)Sbus[0]-(double)1500)/(double)500);
	if(fabs(y_d_tangent)<y_d_tangent_deadzone || fabs(y_d_tangent)>y_vel_limit) y_d_tangent=0;
	y_d+=y_d_tangent;
	if(altitude_mode){
		if(Sbus[2]>1800){
			Z_d-=0.0005;
		}
		else if(Sbus[2]<1200){
			Z_d+=0.0005;
		}
			
		if(Z_d <-0.7) Z_d = -0.7;
		if(Z_d > 0) Z_d = 0;
		//Z_d = -altitude_limit*(((double)Sbus[2]-(double)1500)/(double)500)-altitude_limit;
	}
	else{
		T_d = -T_limit*(((double)Sbus[2]-(double)1500)/(double)500)-T_limit;
	}
	//ROS_INFO("%lf",T_d);

	double e_r = 0;
	double e_p = 0;
	double e_y = 0;
	double e_X = 0;
	double e_Y = 0;
	double e_Z = 0;
	double e_X_dot = 0;
	double e_Y_dot = 0;
		
	//ROS_INFO("%lf",time_count);


	if(true /*position_mode || velocity_mode*/){
		if(position_mode){
			X_d = X_d_base - XY_limit*(((double)Sbus[1]-(double)1500)/(double)500);
			Y_d = Y_d_base + XY_limit*(((double)Sbus[3]-(double)1500)/(double)500);
		
			e_X = X_r-pos.x;//X_d - pos.x;
			e_Y = Y_r-pos.y;//Y_d - pos.y;
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
		//*/
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
		if(fabs(X_ddot_d) > XYZ_ddot_limit) X_ddot_d = (X_ddot_d/fabs(X_ddot_d))*XYZ_ddot_limit;
		if(fabs(Y_ddot_d) > XYZ_ddot_limit) Y_ddot_d = (Y_ddot_d/fabs(Y_ddot_d))*XYZ_ddot_limit;
		
		if(tilt_mode){
			r_d = 0.0;
			p_d = 0.0;
			F_xd = mass*(X_ddot_d*cos(imu_rpy.z)*cos(imu_rpy.y)+Y_ddot_d*sin(imu_rpy.z)*cos(imu_rpy.y)-(Z_ddot_d)*sin(imu_rpy.y));
			F_yd = mass*(-X_ddot_d*(cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.x)*sin(imu_rpy.y))+Y_ddot_d*(cos(imu_rpy.x)*cos(imu_rpy.z)+sin(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d)*cos(imu_rpy.y)*sin(imu_rpy.x));

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
			F_xd=-mass*XYZ_ddot_limit*(((double)Sbus[1]-(double)1500)/(double)500);
			F_yd=mass*XYZ_ddot_limit*(((double)Sbus[3]-(double)1500)/(double)500);
			//ROS_INFO("Attitude & Tilt !!!");

		}
		else{
			r_d=rp_limit*(((double)Sbus[3]-(double)1500)/(double)500);
			p_d=rp_limit*(((double)Sbus[1]-(double)1500)/(double)500);
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

	rpy_ddot_d(0) = Pa * e_r + Ia * e_r_i + Da * (-imu_ang_vel.x);//- (double)0.48;
	rpy_ddot_d(1) = Pa * e_p + Ia * e_p_i + Da * (-imu_ang_vel.y);//+ (double)0.18; 
	rpy_ddot_d(2) = Py * e_y + Dy * (-imu_ang_vel.z);

	angular_acc_d.x = rpy_ddot_d(0);//r_ddot_d
	angular_acc_d.y = rpy_ddot_d(1);//p_ddot_d
	angular_acc_d.z = rpy_ddot_d(2);//y_ddot_d

	tau_cmd = (hat_MoI*rpy_ddot_d)/module_num_count; //torque distributor
	//ROS_INFO("%f | %f | %f",tau_cmd(0),tau_cmd(1),tau_cmd(2));
	
	
	tau_r_d = tau_cmd(0);
	tau_p_d = tau_cmd(1);
	tau_y_d = tau_cmd(2);



	if(fabs(tau_y_d) > tau_y_limit) tau_y_d = tau_y_d/fabs(tau_y_d)*tau_y_limit;
	
	if(altitude_mode){
		Z_ddot_d = Pz * e_Z + Iz * e_Z_i - Dz * lin_vel.z;
		desired_lin_vel.z = Z_ddot_d; // But this is desired acceleration
		if(!attitude_mode) F_zd = mass*(X_ddot_d*(sin(imu_rpy.x)*sin(imu_rpy.z)+cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y))-Y_ddot_d*(cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_ddot_d)*cos(imu_rpy.x)*cos(imu_rpy.y));
		else F_zd = mass*(Z_ddot_d);
		//ROS_INFO("Altitude");
	}
	else{
		e_Z_i = 0;
		e_Z_dot_i = 0;
		F_zd=T_d;
		//ROS_INFO("Manual Thrust!!");
	}
//	if(F_zd > -0.5*mass*g) F_zd = -0.5*mass*g;
//	if(F_zd <= -1.5*mass*g) F_zd = -1.5*mass*g; //1.5
	
	

	K_matrix(); // For force allocation :: 23.08.17
	
	if(shape_num==2)
	{
	F_xd = allocation_factor(0)*F_xd;
	F_yd = allocation_factor(0)*F_yd;
	F_zd = allocation_factor(0)*F_zd;

	}


	//ROS_INFO("%f %f %f",F_xd,F_yd,F_zd);
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
	//u << tau_r_d, tau_p_d, tau_y_d-tau_y_sin, F_zd;
	//std::cout << "tau_y_sin : " << tau_y_sin << "    ";	
	prev_angular_Vel = imu_ang_vel;
	ud_to_PWMs(tau_r_d, tau_p_d, tau_y_d, Thrust_d); //but not use 22.10.12
	//ud_to_PWMs(tautilde_r_d, tautilde_p_d, tautilde_y_d, Thrust_d);
}

 

void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des) {	
 	
	//Co-rotating coaxial
	//Conventional type
	F_cmd = invCM_Xc_p2*u;//invCM*u;
	
	F1 = F_cmd(0);
	F2 = F_cmd(1);
	F3 = F_cmd(2);
	F4 = F_cmd(3);
	//double tau_y_cos=-F1*xi*cos(theta1)+F2*xi*cos(theta2)-F3*xi*cos(theta3)+F4*xi*cos(theta4);
	//control_by_theta << F_xd, F_yd, tau_y_d-tau_y_cos;
	//std::cout << "tau_y_cos : " << tau_y_cos << std::endl;	
	//std::cout << "pinvSA : \n" << pinvSA <<std::endl;
	//setSA();
	//sine_theta_command=pinvSA*control_by_theta;
	//ROS_INFO("%lf %lf %lf %lf",sine_theta_command(0),sine_theta_command(1),sine_theta_command(2),sine_theta_command(3));
	if(!tilt_mode){
		theta1_command = 0.0;
        theta2_command = 0.0;
		theta3_command = 0.0;
		theta4_command = 0.0;
	}
	//Tilting type
	else {
		theta1_command = asin((F_xd+F_yd)/(F1+F3)/r2);
		theta2_command = asin((F_xd-F_yd)/(F2+F4)/r2);
		theta3_command = -theta1_command;//asin(sine_theta_command(2));
		theta4_command = -theta2_command;//asin(sine_theta_command(3));
 		if(fabs(theta1_command)>hardware_servo_limit) theta1_command = (theta1_command/fabs(theta1_command))*hardware_servo_limit;
		if(fabs(theta2_command)>hardware_servo_limit) theta2_command = (theta2_command/fabs(theta2_command))*hardware_servo_limit;
		if(fabs(theta3_command)>hardware_servo_limit) theta3_command = (theta3_command/fabs(theta3_command))*hardware_servo_limit;
		if(fabs(theta4_command)>hardware_servo_limit) theta4_command = (theta4_command/fabs(theta4_command))*hardware_servo_limit;

		//ROS_INFO("%lf %lf %lf %lf",theta1_command, theta2_command, theta3_command, theta4_command);
	}
	
	//pwm_Kill();
	pwm_Command(Force_to_PWM(F1),Force_to_PWM(F2), Force_to_PWM(F3), Force_to_PWM(F4));
	// ROS_INFO("1:%d, 2:%d, 3:%d, 4:%d",PWMs_cmd.data[0], PWMs_cmd.data[1], PWMs_cmd.data[2], PWMs_cmd.data[3]);
	// Force_to_PWM(-T_d/4.0);
	// ROS_INFO("%f 1:%d, 2:%d, 3:%d, 4:%d",z_d,PWMs_cmd.data[0], PWMs_cmd.data[1], PWMs_cmd.data[2], PWMs_cmd.data[3]);
}

 

double Force_to_PWM(double F) {
	double pwm;
	//double A = -9.8*pow(10.0,-8.0)*pow(voltage,2.0)+3.23*pow(10.0,-6.0)*voltage-1.8*pow(10.0,-5.0);
	//double B = 0.000243*pow(voltage,2.0)-0.00663*voltage+0.03723;
	//double C = -0.11063*pow(voltage,2.0)+2.332691*voltage-10.885;
	double param1 = 710;//-B/(2.0*A);
	double param2 = 0.00016;//1.0/A;
	double param3 = 0.00041888;//(pow(B,2.0)-4*A*C)/(4*pow(A,2.0));
	double param4 = 0.00008;
	//Force=A*pwm^2+B*pwm+C
	// A = 0.00004 / B = -0.0568 / C = 17.546 

	if(param2*F+param3>0){
		pwm = param1 + sqrt(param2 * F + param3)/param4;
		// ROS_INFO("%lf",pwm);
	}
	else pwm = 1100.;
	if (pwm > 1900)	pwm = 1900;
	if(pwm < 1100) pwm = 1100;
	/*if(altitude_mode){//altitude mode
		if(Z_d_base<=0){
			if(Z_d>Z_d_base && !start_flag) {
				pwm=1100;
			}
			else if(Z_d<Z_d_base) start_flag=true;
		}
		else pwm=param1;
	}*/
	return pwm;
}

void jointstateCallback(const sensor_msgs::JointState& msg){
    	rac_servo_value=msg;
	theta1=msg.position[0];
	theta2=msg.position[1];
	theta3=msg.position[2];
	theta4=msg.position[3];
	
    	//ROS_INFO("theta1:%lf   theta2:%lf",theta1, theta2);
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
	base_yaw = cam_att(2);
    	if(base_yaw - yaw_prev < -pi) yaw_rotate_count++;
	else if(base_yaw - yaw_prev > pi) yaw_rotate_count--;
	yaw_now = base_yaw+2*pi*yaw_rotate_count;
	//ROS_INFO("now : %lf / prev : %lf / count : %d",yaw_now, yaw_prev, yaw_rotate_count);
	imu_rpy.z = yaw_now;
	yaw_prev = base_yaw;
	// ROS_INFO("imuCallback time : %f",(((double)ros::Time::now().sec-(double)imuTimer.sec)+((double)ros::Time::now().nsec-(double)imuTimer.nsec)/1000000000.));
	//imuTimer = ros::Time::now();
}

void filterCallback(const sensor_msgs::Imu& msg){
	filtered_angular_rate=msg.angular_velocity;
}

sensor_msgs::JointState servo_msg_create(double desired_theta1, double desired_theta2, double desired_theta3, double desired_theta4){
	sensor_msgs::JointState servo_msg;

	servo_msg.header.stamp=ros::Time::now();

	servo_msg.name.resize(4);
	servo_msg.name[0]="id_1";
	servo_msg.name[1]="id_2";
	servo_msg.name[2]="id_3";
	servo_msg.name[3]="id_4";


	servo_msg.position.resize(4);
	servo_msg.position[0]=desired_theta1;
	servo_msg.position[1]=desired_theta2;
	servo_msg.position[2]=desired_theta3;
	servo_msg.position[3]=desired_theta4;
	//ROS_INFO("rr: %lf, rp: %lf",rr,rp);
	return servo_msg;
}

void sbusCallback(const std_msgs::Int16MultiArray::ConstPtr& array){
	for(int i=0;i<10;i++){
		Sbus[i]=map<int16_t>(array->data[i], 352, 1696, 1000, 2000);
	}
	
	if(Sbus[4]<1500) kill_mode=true;
	else kill_mode=false;
	
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

	if(Sbus[9]>1500) admittance_mode=true;
	else admittance_mode=true;
//	ROS_INFO("%d, %d, %d, %d, %d, %d, %d, %d",Sbus[0],Sbus[1],Sbus[2],Sbus[3],Sbus[4],Sbus[5],Sbus[6],Sbus[7]);
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
	if(voltage>25.2) voltage=25.2;
	if(voltage<20.0) voltage=20.0;
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
	tf::Matrix3x3(quat).getRPY(t265_att.x,t265_att.y,t265_att.z);
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
	
	R_v << 0, 1, 0,
	    	1, 0, 0,
		0, 0, -1;
	/*
	R_v << cos(pi/2.), -sin(pi/2.),  0.,
 	       sin(pi/2.),  cos(pi/2.),  0.,
	                0.,           0.,  1.;
	*/
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

int32_t pwmMapping(double pwm){
	return (int32_t)(65535.*pwm/(1./pwm_freq*1000000.));
}

void pwm_Command(double pwm1, double pwm2, double pwm3, double pwm4){
	PWMs_cmd.data.resize(4);
	PWMs_cmd.data[0] = pwm1;
	PWMs_cmd.data[1] = pwm2;
	PWMs_cmd.data[2] = pwm3;
	PWMs_cmd.data[3] = pwm4;
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(pwm1);
	PWMs_val.data[1] = pwmMapping(pwm2);
	PWMs_val.data[2] = pwmMapping(pwm3);
	PWMs_val.data[3] = pwmMapping(pwm4);
	PWMs_val.data[4] = -1;
	PWMs_val.data[5] = -1;
	PWMs_val.data[6] = -1;
	PWMs_val.data[7] = -1;
	PWMs_val.data[8] = -1;
	PWMs_val.data[9] = -1;
	PWMs_val.data[10] = -1;
	PWMs_val.data[11] = -1;
	PWMs_val.data[12] = -1;
	PWMs_val.data[13] = -1;
	PWMs_val.data[14] = -1;
	PWMs_val.data[15] = -1;

}

void pwm_Max(){
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(2000.);
	PWMs_val.data[1] = pwmMapping(2000.);
	PWMs_val.data[2] = pwmMapping(2000.);
	PWMs_val.data[3] = pwmMapping(2000.);
	PWMs_val.data[4] = -1;
	PWMs_val.data[5] = -1;
	PWMs_val.data[6] = -1;
	PWMs_val.data[7] = -1;
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
	PWMs_cmd.data.resize(4);
	PWMs_cmd.data[0] = 1000;
	PWMs_cmd.data[1] = 1000;
	PWMs_cmd.data[2] = 1000;
	PWMs_cmd.data[3] = 1000;
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(1000.);
	PWMs_val.data[1] = pwmMapping(1000.);
	PWMs_val.data[2] = pwmMapping(1000.);
	PWMs_val.data[3] = pwmMapping(1000.);
	PWMs_val.data[4] = -1;
	PWMs_val.data[5] = -1;
	PWMs_val.data[6] = -1;
	PWMs_val.data[7] = -1;
	PWMs_val.data[8] = -1;
	PWMs_val.data[9] = -1;
	PWMs_val.data[10] = -1;
	PWMs_val.data[11] = -1;
	PWMs_val.data[12] = -1;
	PWMs_val.data[13] = -1;
	PWMs_val.data[14] = -1;
	PWMs_val.data[15] = -1;

}

void pwm_Arm(){
	PWMs_cmd.data.resize(4);
	PWMs_cmd.data[0] = 1500;
	PWMs_cmd.data[1] = 1500;
	PWMs_cmd.data[2] = 1500;
	PWMs_cmd.data[3] = 1500;
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(1500.);
	PWMs_val.data[1] = pwmMapping(1500.);
	PWMs_val.data[2] = pwmMapping(1500.);
	PWMs_val.data[3] = pwmMapping(1500.);
	PWMs_val.data[4] = -1;
	PWMs_val.data[5] = -1;
	PWMs_val.data[6] = -1;
	PWMs_val.data[7] = -1;
	PWMs_val.data[8] = -1;
	PWMs_val.data[9] = -1;
	PWMs_val.data[10] = -1;
	PWMs_val.data[11] = -1;
	PWMs_val.data[12] = -1;
	PWMs_val.data[13] = -1;
	PWMs_val.data[14] = -1;
	PWMs_val.data[15] = -1;

}
void pwm_Calibration(){
	//if(Sbus[4]>1500) pwm_Arm();
	//else pwm_Kill();
        if(Sbus[6]>1700) pwm_Kill();
	if(Sbus[6]<1000) pwm_Arm();
//ROS_INFO("Sbus[6] : %d", Sbus[6]);	
}


void pid_Gain_Setting(){
	if(Sbus[7]<=1500){
		Pa = conv_Pa;
		Ia = conv_Ia;
		Da = conv_Da;

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
		Pa = tilt_Pa;
		Ia = tilt_Ia;
		Da = tilt_Da;

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


