/*
 * Copyright (C) 2014 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 *
 * test_rolling.cpp
 *
 * Created on : Nov 18, 2014
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
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>
#include <signal.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define EE_CMD_FT_TOPIC   	"/cart_to_joint/des_ee_ft"
#define EE_STATE_POSE_TOPIC "/joint_to_cart/est_ee_pose"
#define EE_STATE_FT_TOPIC "/joint_to_cart/est_ee_ft"
#define EE_CMD_POSE_TOPIC   "/cart_to_joint/des_ee_pose"
#define DEFAULT_HEIGHT		0
#define DEFAULT_FORCE		0
#define DEFAULT_THR			5.0
geometry_msgs::WrenchStamped wr;
geometry_msgs::PoseStamped msg_pose;
tf::Pose ee_pose;
Eigen::VectorXd wrench;
volatile bool isStOkay, isFtOkay;
ros::Publisher pub_ft, pub_pose;
bool shut;
void handler(int signal) {
	wr.wrench.force.x = 0;
	wr.wrench.force.y = 0;
	wr.wrench.force.z = 0;

	wr.wrench.torque.x = 0;
	wr.wrench.torque.y = 0;
	wr.wrench.torque.z = 0;
	pub_ft.publish(wr);
	shut = true;

}

void eeStateCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
	const geometry_msgs::PoseStamped* data = msg.get();
	ee_pose.setOrigin(tf::Vector3(data->pose.position.x,data->pose.position.y,data->pose.position.z));
	ee_pose.setRotation(tf::Quaternion(data->pose.orientation.x,data->pose.orientation.y,data->pose.orientation.z,data->pose.orientation.w));
	isStOkay = true;
}

void eeFTCallback(const geometry_msgs::WrenchStampedConstPtr& msg) {
	const geometry_msgs::WrenchStamped* data = msg.get();
	wrench[0]= data->wrench.force.x;
	wrench[1]= data->wrench.force.y;
	wrench[2]= data->wrench.force.z;
	isFtOkay = true;
}

int main(int argc, char** argv) {
	shut = false;
	wrench.resize(3);
	signal(SIGINT, handler);
	ros::init(argc, argv, "test_traj", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;
	ros::NodeHandle _nh("~");

	double force, thr;

	// Detect the table using this threshold.
	// Then roll on the table with this force.
	if(!_nh.getParam("fz", force)) {
		force = DEFAULT_FORCE;
	}
	if(!_nh.getParam("threshold", thr)) {
		thr = DEFAULT_THR;
	}

	thr = fabs(thr);
	force = fabs(force);
	double rate = 200;
	ros::Rate thread_rate(rate);


	ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>(EE_STATE_POSE_TOPIC, 1, eeStateCallback);
	ros::Subscriber sub_ft = nh.subscribe<geometry_msgs::WrenchStamped>(EE_STATE_FT_TOPIC, 1, eeFTCallback);
	ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>(EE_CMD_POSE_TOPIC, 1);
	pub_ft = nh.advertise<geometry_msgs::WrenchStamped>(EE_CMD_FT_TOPIC, 1);
	ros::Rate r(1);
	isStOkay = isFtOkay = false;
	ROS_INFO("Waiting for EE Pose topic");
	while(!shut && (!isStOkay || !isFtOkay)) {
		ros::spinOnce();
		r.sleep();
	}

	tf::Vector3 start_p(ee_pose.getOrigin());
	tf::Quaternion start_o(ee_pose.getRotation());


	// Hard coded vertical orientation good for rolling. Works for both BOXY and LASA robots
	Eigen::Matrix3d mat;
	mat << Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 0, -1);

	Eigen::Quaterniond q(mat);
	start_o.setX(q.x());
	start_o.setY(q.y());
	start_o.setZ(q.z());
	start_o.setW(q.w());
	ROS_INFO_STREAM("Detection threshold: "<<thr);
	ROS_INFO_STREAM("Vertical Force: "<<force);
	ROS_INFO("Node started");


	msg_pose.pose.position.x = start_p.x();
	msg_pose.pose.position.y = start_p.y();
	msg_pose.pose.position.z = start_p.z();
	msg_pose.pose.orientation.w = start_o.w();
	msg_pose.pose.orientation.x = start_o.x();
	msg_pose.pose.orientation.y = start_o.y();
	msg_pose.pose.orientation.z = start_o.z();
	pub_pose.publish(msg_pose);
	wr.wrench.force.x = 0;
	wr.wrench.force.y = 0;
	wr.wrench.force.z = 0;
	wr.wrench.torque.x = 0;
	wr.wrench.torque.y = 0;
	wr.wrench.torque.z = 0;
	pub_ft.publish(wr);


	ROS_INFO("Going to home");
	while(!shut) {
		ros::spinOnce();
		double oerr =(ee_pose.getRotation() - start_o).length() ;
		double perr = (ee_pose.getOrigin() - start_p).length();
		ROS_INFO_STREAM_THROTTLE(1.0, "Reaching error "<<perr<<", "<<oerr);
		if(perr< 0.01 && oerr < 0.01) {
			break;
		}
		thread_rate.sleep();
	}

	ROS_INFO("Finding Table...");
	r.sleep();
	while(!shut) {
		ros::spinOnce();
		pub_pose.publish(msg_pose);
		msg_pose.pose.position.z = msg_pose.pose.position.z - 0.03/rate;

		// Keep going down untill sensed Z force is less than specified threshold
		if(fabs(wrench[2]) > thr) {
			break;
		}
		thread_rate.sleep();
		ROS_WARN_STREAM_THROTTLE(0.1, "Current normal force: "<<wrench[2]);
	}
	if(shut) {
		return 1;
	}
	ros::spinOnce();
	tf::Vector3 table(ee_pose.getOrigin());
	ROS_INFO_STREAM("Table found at height "<<table[2]);
	msg_pose.pose.position.z = table[2];


	r.sleep();
	wr.wrench.force.z = -force;
	pub_pose.publish(msg_pose);
	pub_ft.publish(wr);

	ROS_INFO_STREAM("Rolling with force "<<force);
	r.sleep();

	// Roll forward with this speed (m/s)
	double rolling_speed = 0.04, center = msg_pose.pose.position.y;
	double range = 0.1;
	while(!shut) {
		ros::spinOnce();
		//TODO: Fix the end effector orientation on LASA robot to be the same rolling direction as boxy
//		msg_pose.pose.position.x = msg_pose.pose.position.x + rolling_speed/rate;
		msg_pose.pose.position.y = msg_pose.pose.position.y + rolling_speed/rate;
		ROS_INFO_STREAM_THROTTLE(0.5, "XPos: "<<msg_pose.pose.position.y);
//		if( fabs(msg_pose.pose.position.x - center) >  range) {
		if( fabs(msg_pose.pose.position.y - center) >  range) {
			ROS_INFO("Change direction");
			rolling_speed *= -1;
		}
		pub_pose.publish(msg_pose);
		thread_rate.sleep();
	}

	wr.wrench.force.x = 0;
	wr.wrench.force.y = 0;
	wr.wrench.force.z = 0;

	wr.wrench.torque.x = 0;
	wr.wrench.torque.y = 0;
	wr.wrench.torque.z = 0;
	pub_ft.publish(wr);
	ROS_INFO("Sent zero force");
	r.sleep();

	// Go up once Ctrl-C is pressed
	msg_pose.pose.position.z = table[2] + 0.15;
	msg_pose.pose.position.y = table[1];
	pub_pose.publish(msg_pose);

	// Sleep a little to ensure message is sent.
	r.sleep();
	return 0;
}
