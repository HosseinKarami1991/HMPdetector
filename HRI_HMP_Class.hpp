/*
 * HRI_HMP_Class.hpp
 *
 *  Created on: June 14, 2016
 *      Author: Kourosh Darvish
 */

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
//#include <baxter_core_msgs/EndEffectorState.h>
//#include <baxter_core_msgs/EndEffectorCommand.h>
//#include "/opt/ros/indigo/include/sensor_msgs/JointState.h"
#include <string>
#include <iterator>
#include <sstream>


using namespace std;

class HRI_HMP_Class {
	public:
		bool hri_hmp_flag;

		HRI_HMP_Class();
		void Callback_hri_hmp(const std_msgs::String::ConstPtr& msg1);
		void hri_hmp_process(void);
		void publish_hri_hmp(void);
		string parameterHMP[5];
		//float parameterHMP2[4];

	private:
		ros::NodeHandle nh;
		ros::Subscriber sub_hri_hmp;
		ros::Publisher pub_HMP_cmnd;





};

