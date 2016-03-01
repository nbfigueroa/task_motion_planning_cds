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

#define EE_STATE_POSE_TOPIC "/joint_to_cart/est_ee_pose"
#define EE_CMD_POSE_TOPIC   "/cart_to_joint/des_ee_pose"
#define RADIUS				0.10
#define R_GAIN				0.3
#define X_OFFSET			0.5
#define Y_OFFSET			0.2
#define HEIGHT				0.5
#define OMEGA				30*M_PI/180.0


tf::Pose ee_pose;
volatile bool isOkay;

void eeStateCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
	const geometry_msgs::PoseStamped* data = msg.get();
	ee_pose.setOrigin(tf::Vector3(data->pose.position.x,data->pose.position.y,data->pose.position.z));
	ee_pose.setRotation(tf::Quaternion(data->pose.orientation.x,data->pose.orientation.y,data->pose.orientation.z,data->pose.orientation.w));
	isOkay = true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "test_traj");
	ros::NodeHandle nh;
	ros::NodeHandle _nh("~");

	ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>(EE_STATE_POSE_TOPIC, 1, eeStateCallback);
	ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>(EE_CMD_POSE_TOPIC, 1);
	ros::Rate r(10);
	isOkay = false;
	ROS_INFO("Waiting for EE Pose topic");
	while(ros::ok() && !isOkay) {
		ros::spinOnce();
		r.sleep();
	}

	tf::Vector3 curr_pos(ee_pose.getOrigin());
	tf::Matrix3x3 orient(ee_pose.getRotation());

	geometry_msgs::PoseStamped msg_pose;

	// Parameters (commandline/launch file) to set how much deltas to move in position and orientation
	// Positions in meter Orientation in Degrees (Euler Angles)
	double dx = 0, dy=0, dz=0;
	double rx=0, ry=0, rz=0;
	double total_time = 2;

	_nh.getParam("dx", dx);
	_nh.getParam("dy", dy);
	_nh.getParam("dz", dz);

	_nh.getParam("rx", rx);
	_nh.getParam("ry", ry);
	_nh.getParam("rz", rz);

	// Time in which to complete the motion
	_nh.getParam("time", total_time);
	rx *= M_PI/180.0;ry *= M_PI/180.0;rz *= M_PI/180.0;


	ROS_INFO_STREAM("Dx: "<<dx);
	ROS_INFO_STREAM("Dy: "<<dy);
	ROS_INFO_STREAM("Dz: "<<dz);
	ROS_INFO_STREAM("Rx: "<<rx);
	ROS_INFO_STREAM("Ry: "<<ry);
	ROS_INFO_STREAM("Rz: "<<rz);

	tf::Matrix3x3 rot;
	rot.setRPY(rx,ry,rz);

	rot = orient*rot;

	tf::Quaternion final_orient;
	rot.getRotation(final_orient);
	tf::Quaternion start_orient = ee_pose.getRotation();
	tf::Quaternion curr_orient;
	tf::Vector3 final_pos = curr_pos + tf::Vector3(dx, dy, dz);
	tf::Vector3 start_pos = curr_pos;

	double rate = 100, t=0;
	ros::Rate tr(rate);
	while(ros::ok() && t <= total_time) {



		// Linear interpolation between start and final position
		curr_pos = (1-t/total_time)*start_pos + t/total_time*final_pos;

		// Quaternion slerp interpolation between start and final orientation
		curr_orient = start_orient.slerp(final_orient, t/total_time);

		t+= 1.0/rate;
		msg_pose.header.stamp = ros::Time::now();
		msg_pose.pose.position.x = curr_pos.x();
		msg_pose.pose.position.y = curr_pos.y();
		msg_pose.pose.position.z = curr_pos.z();

		msg_pose.pose.orientation.w = curr_orient.w();
		msg_pose.pose.orientation.x = curr_orient.x();
		msg_pose.pose.orientation.y = curr_orient.y();
		msg_pose.pose.orientation.z = curr_orient.z();
		ROS_INFO_STREAM("t "<<t );

		pub.publish(msg_pose);
		tr.sleep();
	}
	r.sleep();

	return 0;
}
