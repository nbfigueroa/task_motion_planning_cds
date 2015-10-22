/*
 * Copyright (C) 2014 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 *
 * joint_to_cart.cpp
 *
 * Created on : Oct 30, 2014
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


/**
 * Robot cartesian state estimator. Estimates end effector pose and force/torques from the joint angles/torques
 */
#include "RTKRobotArm.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/JointState.h"
#include <tf/tf.h>
#include <Eigen/Core>
#include <Eigen/LU>


// Default tolerance for adding tol*I for the matrix inversion to be stable
#define DEFAULT_SINGULAR_TOL 0.1


std::string input_joint_topic;
std::string output_cart_pose;
std::string output_cart_ft;
char buf[255];
int nEndEffectorId, numdof;
bool negate_torque;

Eigen::VectorXd read_torque, read_jpos;
Eigen::VectorXd ee_ft; tf::Pose ee_pose;

RobotArm* mRobot;

ros::Publisher pub_ft, pub_pose;
geometry_msgs::WrenchStamped msg_ft;
geometry_msgs::PoseStamped msg_pose;
double singular_tol;

bool parseParams(const ros::NodeHandle& n) {


	bool ret = true;
	if(!n.getParam("input_joint_topic", input_joint_topic)) {
		ROS_ERROR("Must provide joint state topic!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Input topic: "<<input_joint_topic);
	}

	if(!n.getParam("output_cart_pose", output_cart_pose)) {
		ROS_ERROR("Must provide output cartesian position topic name!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Output pose topic: "<<output_cart_pose);
	}

	if(!n.getParam("output_cart_ft", output_cart_ft)) {
		ROS_ERROR("Must provide output cartesian force/torque topic name!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Output FT topic: "<<output_cart_ft);
	}


	if(!n.getParam("negate_torque", negate_torque)) {
		negate_torque = false;
	} else {
		ROS_INFO_STREAM("Negate Torque "<<negate_torque);
	}

	if(!n.getParam("singular_tol", singular_tol)) {
		singular_tol = DEFAULT_SINGULAR_TOL;
	} else {
		ROS_INFO_STREAM("Singular Tolerance: "<<singular_tol);
	}
	return ret;
}


// Convert joint torques to end effector force/torque
void computeFT(Eigen::VectorXd& ee_ft) {
	Eigen::MatrixXd jac;
	mRobot->getJacobian(jac);
	Eigen::MatrixXd trans = jac*jac.transpose() + singular_tol*Eigen::MatrixXd::Identity(jac.rows(), jac.rows());
	// f = inv(J*J')*J*tau
	ee_ft = (trans.inverse()*jac)*read_torque;
}

void sendFT(Eigen::VectorXd& ee_FT) {
	msg_ft.header.stamp = ros::Time::now();
	msg_ft.wrench.force.x = ee_FT[0];
	msg_ft.wrench.force.y = ee_FT[1];
	msg_ft.wrench.force.z = ee_FT[2];

	msg_ft.wrench.torque.x = ee_FT[3];
	msg_ft.wrench.torque.y = ee_FT[4];
	msg_ft.wrench.torque.z = ee_FT[5];

	pub_ft.publish(msg_ft);
}

void sendPose(tf::Pose& pose) {

	msg_pose.header.stamp = ros::Time::now();

	tf::Vector3 tmp = pose.getOrigin();
	msg_pose.pose.position.x = tmp.x();
	msg_pose.pose.position.y = tmp.y();
	msg_pose.pose.position.z = tmp.z();

	tf::Quaternion quat = pose.getRotation();
	msg_pose.pose.orientation.w = quat.w();
	msg_pose.pose.orientation.x = quat.x();
	msg_pose.pose.orientation.y = quat.y();
	msg_pose.pose.orientation.z = quat.z();

	pub_pose.publish(msg_pose);
}

// Main callback controlling the output rate
void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg) {

	const sensor_msgs::JointState* data = msg.get();
	int es = data->effort.size();
	int ps = data->position.size();
	if(es != ps) {
		ROS_WARN_STREAM_THROTTLE(1, "Effort and position sized unequal! EffortSize: "<<es<<" PosSize: "<<numdof);
		return;
	}

	// Collect the right joint angles by name
	int counter=0;
	for(unsigned int i=0;i<es; ++i) {
		sprintf(buf, "right_arm_%d_joint", counter);
		if(!strcmp(buf, data->name[i].c_str())) {
			// DLR gravity is negative!!
			if(negate_torque) {
				read_torque[counter] = - data->effort[i];
			} else {
				read_torque[counter] = data->effort[i];
			}
			read_jpos[counter] = data->position[i];
			++counter;
		}
		// If cannot find all the joints
		if(counter == numdof) { break; }
	}
	// If found more or less joints than expected!
	if(counter != numdof) {
		ROS_WARN_STREAM_THROTTLE(1, "Only found "<<counter<<" DOF in message. Expected "<<numdof);
	} else {
		// All OK. Compute and send.
			mRobot->setJoints(read_jpos);
			mRobot->getEEPose(ee_pose);
			computeFT(ee_ft);

			sendFT(ee_ft);
			sendPose(ee_pose);
	}

}


int main(int argc, char** argv) {

	ros::init(argc, argv, "joint_to_cart");
	ros::NodeHandle nh;
	ros::NodeHandle _nh("~");

	mRobot = new RTKRobotArm(true);
	if(!mRobot->initialize(_nh)) {
		ROS_ERROR("Error while loading robot");
		return 1;
	}

	if(!parseParams(_nh)) {
		ROS_ERROR("Errors while parsing arguments.");
		return 1;
	}

	numdof = mRobot->numdof;
	read_torque.resize(numdof);
	read_jpos.resize(numdof);
	ee_ft.resize(6);

	pub_pose = nh.advertise<geometry_msgs::PoseStamped>(output_cart_pose, 1);
	pub_ft = nh.advertise<geometry_msgs::WrenchStamped>(output_cart_ft, 1);
	ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>(input_joint_topic, 1, jointStateCallback);


	ROS_INFO("Node started");
	ros::spin();

	return 0;
}



