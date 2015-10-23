/*
 * Copyright (C) 2014 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 *
 * joint_to_cart.cpp
 *
 * Created on : Nov 15, 2014
 * Author     : ashukla
 * Email      : ashwini.shukla@epfl.ch
 * Website    : lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include <tf/tf.h>
#include <signal.h>
#include <Eigen/Core>

#define EE_STATE_POSE_TOPIC "/joint_to_cart/est_ee_pose"
#define EE_CMD_POSE_TOPIC   "/cart_to_joint/des_ee_pose"
#define EE_STATE_FT_TOPIC "/joint_to_cart/est_ee_ft"

#define RADIUS				0.10
#define R_GAIN				0.3
#define X_OFFSET			0.5
#define Y_OFFSET			0.2
#define HEIGHT				0.5
#define OMEGA				30*M_PI/180.0


tf::Pose ee_pose;
volatile bool isOkay, isFTOkay;
Eigen::VectorXd wrench;

void eeStateCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
	const geometry_msgs::PoseStamped* data = msg.get();
	ee_pose.setOrigin(tf::Vector3(data->pose.position.x,data->pose.position.y,data->pose.position.z));
	ee_pose.setRotation(tf::Quaternion(data->pose.orientation.x,data->pose.orientation.y,data->pose.orientation.z,data->pose.orientation.w));
	isOkay = true;
}

void eeFTCallback(const geometry_msgs::WrenchStampedConstPtr& msg) {
	const geometry_msgs::WrenchStamped* data = msg.get();
	wrench[0]= data->wrench.force.x;
	wrench[1]= data->wrench.force.y;
	wrench[2]= data->wrench.force.z;
	isFTOkay = true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "test_traj");
	ros::NodeHandle nh;
	ros::NodeHandle _nh("~");
	wrench.resize(3);
	ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>(EE_STATE_POSE_TOPIC, 1, eeStateCallback);
	ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>(EE_CMD_POSE_TOPIC, 1);
	ros::Subscriber sub_ft = nh.subscribe<geometry_msgs::WrenchStamped>(EE_STATE_FT_TOPIC, 1, eeFTCallback);

	ros::Rate r(10);
	isOkay = false; isFTOkay = false;
	ROS_INFO("Waiting for EE Pose topic");
	while(ros::ok() && (!isOkay||!isFTOkay)) {
		ros::spinOnce();
		r.sleep();
	}

	// Hard coded starting position for pouring. Works for LASA robot
	tf::Vector3 start_pos(-0.65, -0.11, 0.274);
	tf::Matrix3x3 start_orient(tf::Quaternion(-0.006, 0.907, -0.420, -0.114));
	tf::Vector3 curr_pos = start_pos;
	geometry_msgs::PoseStamped msg_pose;

	// Rotations about x,y,z axes
	double rx=0, ry=0, rz=0;

	// Computing world frame orientation for the EE from the euler rotations rx, ry, rz
	tf::Matrix3x3 rot;
	rot.setRPY(rx,ry,rz);
	rot = start_orient*rot;
	tf::Quaternion curr_orient;
	rot.getRotation(curr_orient);

	msg_pose.header.stamp = ros::Time::now();
	msg_pose.pose.position.x = curr_pos.x();
	msg_pose.pose.position.y = curr_pos.y();
	msg_pose.pose.position.z = curr_pos.z();

	msg_pose.pose.orientation.w = curr_orient.w();
	msg_pose.pose.orientation.x = curr_orient.x();
	msg_pose.pose.orientation.y = curr_orient.y();
	msg_pose.pose.orientation.z = curr_orient.z();

	pub.publish(msg_pose);



	// Rotate the EE at rot_speed (deg/sec) until a target_decrease in force is sensed.
	double rate = 100, rot_speed = 3, target_decrease = 0.1;
	ros::Rate tr(rate);
	ROS_INFO_STREAM("Start fz="<<wrench[2]);
	ROS_INFO("Press Enter to start");
	getchar();
	isFTOkay = false;
	ROS_INFO("Waiting for EE FT topic");
	while(ros::ok() && (!isOkay||!isFTOkay)) {
		ros::spinOnce();
		r.sleep();
	}
	double start_fz = wrench[2];
	while(ros::ok()) {
		ros::spinOnce();
		ROS_INFO_STREAM_THROTTLE(0.5, "Decrease="<<start_fz-wrench[2]);

		// If the target_decrease is achieved. Stop motion.
		if(start_fz - wrench[2] > target_decrease) {
			ROS_INFO("Required amount poured!");
			msg_pose.header.stamp = ros::Time::now();
			msg_pose.pose.position.x = ee_pose.getOrigin().x();
			msg_pose.pose.position.y = ee_pose.getOrigin().y();
			msg_pose.pose.position.z = ee_pose.getOrigin().z();

			rx = 0;
			rot.setRPY(rx,ry,rz);
			rot = start_orient*rot;
			rot.getRotation(curr_orient);

			msg_pose.pose.orientation.w = curr_orient.w();
			msg_pose.pose.orientation.x = curr_orient.x();
			msg_pose.pose.orientation.y = curr_orient.y();
			msg_pose.pose.orientation.z = curr_orient.z();

			pub.publish(msg_pose);
			break;
		}

		// Keep rotating if target_decrease not reached
		rx -= rot_speed*M_PI/180.0/rate;
		rot.setRPY(rx,ry,rz);
		rot = start_orient*rot;
		rot.getRotation(curr_orient);

		msg_pose.header.stamp = ros::Time::now();
		msg_pose.pose.position.x = curr_pos.x();
		msg_pose.pose.position.y = curr_pos.y();
		msg_pose.pose.position.z = curr_pos.z();

		msg_pose.pose.orientation.w = curr_orient.w();
		msg_pose.pose.orientation.x = curr_orient.x();
		msg_pose.pose.orientation.y = curr_orient.y();
		msg_pose.pose.orientation.z = curr_orient.z();

		pub.publish(msg_pose);
		tr.sleep();
	}
	return 0;
}
