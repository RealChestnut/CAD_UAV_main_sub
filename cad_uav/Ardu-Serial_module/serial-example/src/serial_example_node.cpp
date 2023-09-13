/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */
#include <iostream>
#include <ros/ros.h>
#include <string.h>
#include <vector>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

using namespace std;

serial::Serial ser;
std_msgs::String ToSub;
std_msgs::String push_data;
static char sSTX() { return static_cast<char>(0x02);}
static char sETX() { return static_cast<char>(0x03);}
static char sEOT() { return static_cast<char>(0x04);}
static char sENQ() { return static_cast<char>(0x05);}
static char sACK() { return static_cast<char>(0x06);}
static char sNAK() { return static_cast<char>(0x15);}
static char sCR() { return static_cast<char>(13);}
static char sLF() { return static_cast<char>(10);}
float buffer=0;
string dumi;
/*
void alpha_callback(const std_msgs::Float32& msg);
void write_callback(const std_msgs::String::ConstPtr& msg);
*/
void alpha_callback(const std_msgs::Float32::ConstPtr& msg){
    
    buffer=msg->data;
    //ROS_INFO("%f",buffer);

    ToSub.data = to_string(buffer);
    //ser.write(ToSub.data);
    // ser.wirte는 무조건 std_msgs의 string형이여야함
    //ROS_INFO_STREAM("write data" << ToSub.data);
}

void push_data_callback(const std_msgs::String& msg){
    push_data.data = msg.data;
    //dumi=std::to_string(sSTX())+"," + push_data.data +","+std::to_string(sETX());
    //ROS_INFO_STREAM("string : " << dumi);
}

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port : " << msg->data);
    ser.write(msg->data);
}


int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1, write_callback);
    ros::Subscriber alpha_data = nh.subscribe("force_alpha",1,alpha_callback);
    ros::Subscriber push_data_sub = nh.subscribe("kill_tilt_switch_str",1,push_data_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1);

    

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    
    }else{
        return -1;
    }
    
    ros::Rate loop_rate(500);
    while(ros::ok()){
         
        ros::spinOnce();

        //write data-------------------------------//
        string buffer ="heelo";
        std_msgs::String buffer_ros;
        vector<string> buf;

	string::size_type Fpos = push_data.data.find_first_not_of(',',0);
	string::size_type Lpos = push_data.data.find_first_of(',',Fpos);

	while (string::npos != Fpos || string::npos != Lpos)
	{
		buf.push_back(push_data.data.substr(Fpos,Lpos-Fpos));
		Fpos = push_data.data.find_first_not_of(',',Lpos);
		Lpos = push_data.data.find_first_of(',',Fpos);
	}
	for(int i=0; i<buf.size(); i++)
	{
		ROS_INFO_STREAM(buf[i]);
	        ser.write(buf[i]);	

	}
	
	
	ser.write(push_data.data);

            //ROS_INFO_STREAM("test : " <<dumi.length());
            ROS_INFO_STREAM("subscribe : " <<push_data.data);

        //----------------------------------------//
        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            
            result.data = "dheell";//ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);
        }
        loop_rate.sleep();

    }
}

