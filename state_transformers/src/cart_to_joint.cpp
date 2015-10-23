/*
 * Copyright (C) 2014 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 *
 * joint_to_cart.cpp
 *
 * Created on : Nov 7, 2014
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
 * Controller for converting cartesian commands to joint velocities.
 */

#include "RTKRobotArm.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/JointState.h"
#include "iai_control_msgs/MultiJointVelocityImpedanceCommand.h"
#include "tf/LinearMath/Quaternion.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <signal.h>
#include "std_msgs/String.h"

#define DEFAULT_JSTIFF 		200
#define DEFAULT_JDAMP		0.7
#define DEFAULT_FT_TOL 		0.05
#define DEFAULT_REACH_TOL 	0.05
#define DEFAULT_TRAJ_TRACK_GAIN	 0.05
#define DEFAULT_FORCE_TRACK_GAIN 0.03
#define DEFAULT_MAX_EE_FT	45
#define DEFAULT_RATE		100

std::string input_pose_topic, input_ft_topic, input_stiff_topic, input_joint_topic, input_estimate_ft_topic, output_joints_topic, world_frame;
int nEndEffectorId, numdof, rate;
char buf[255];
volatile bool isJointOkay, isFTOkay, isAllOkay, shut;
bool bOrientCtrl, bUseForce, bUseIAI, simulation;
double reach_tol, ft_tol, force_tracking_gain, traj_tracking_gain, max_ee_ft_norm;
static ros::Time t_old;


Eigen::VectorXd  joint_vel, joint_stiff, joint_damp;

Eigen::VectorXd des_ee_stiff, des_ee_ft, est_ee_ft, shift_ee_ft;
tf::Pose des_ee_pose,  curr_ee_pose;
Eigen::VectorXd read_jpos;

Eigen::VectorXd joint_lims;
RobotArm* mRobot;

ros::Publisher pub_joints;

sensor_msgs::JointState msg_vel_stiff_jstate;
iai_control_msgs::MultiJointVelocityImpedanceCommand msg_vel_stiff_iai;

// Boolean for detecting Ctrl-C
void handler(int sig) {
	shut = true;
}

bool parseParams(const ros::NodeHandle& n) {

	bool ret = true;

	if(!n.getParam("input_joint_topic", input_joint_topic)) {
		ROS_ERROR("Must provide input joint position topic!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Input Joint position topic: "<<input_joint_topic);
	}

	if(!n.getParam("input_pose_topic", input_pose_topic)) {
		ROS_ERROR("Must provide desired Cartesian position topic!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Input Cartesian position topic: "<<input_pose_topic);
	}

	if(!n.getParam("input_ft_topic", input_ft_topic)) {
		ROS_ERROR("Must provide output desired Cartesian force/torque topic!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Input F/T topic: "<<input_ft_topic);
	}

	if(!n.getParam("input_stiff_topic", input_stiff_topic)) {
		ROS_ERROR("Must provide input Cartesian stiffness topic!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Input Cartesian stiffness topic: "<<input_stiff_topic);
	}

	if(!n.getParam("input_estimate_ft_topic", input_estimate_ft_topic)) {
		ROS_ERROR("Must provide estimated Cartesian stiffness topic!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Estimated Cartesian stiffness topic: "<<input_estimate_ft_topic);
	}

	if(!n.getParam("output_joints_topic", output_joints_topic)) {
		ROS_ERROR("Must provide output joints topic!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Output joints topic: "<<output_joints_topic);
	}

	if(!n.getParam("orientation_ctrl", bOrientCtrl)) {
		bOrientCtrl = false;
	}
	ROS_INFO_STREAM("Orientation control: "<<bOrientCtrl);

	if(!n.getParam("reach_tol", reach_tol)) {
		reach_tol = DEFAULT_REACH_TOL;
	}
	ROS_INFO_STREAM("Reaching tolerance: "<<reach_tol);

	if(!n.getParam("ft_tol", ft_tol)) {
		ft_tol = DEFAULT_FT_TOL;
	}
	ROS_INFO_STREAM("Force/Torque tolerance: "<<ft_tol);

	if(!n.getParam("force_tracking_gain", force_tracking_gain)) {
		force_tracking_gain = DEFAULT_FORCE_TRACK_GAIN;
	}
	ROS_INFO_STREAM("Force Tracking Gain: "<<force_tracking_gain);

	if(!n.getParam("traj_tracking_gain", traj_tracking_gain)) {
		traj_tracking_gain = DEFAULT_TRAJ_TRACK_GAIN;
	}
	ROS_INFO_STREAM("Trajectory Tracking Gain: "<<traj_tracking_gain);

	if(!n.getParam("rate", rate)) {
		rate = DEFAULT_RATE;
	}

	if(!n.getParam("max_ee_ft_norm", max_ee_ft_norm)) {
		max_ee_ft_norm = DEFAULT_MAX_EE_FT;
	}
	ROS_INFO_STREAM("Max EE FT Norm: "<<max_ee_ft_norm);


	if(!n.getParam("use_ft", bUseForce)) {
		bUseForce = false;
	}
	ROS_INFO_STREAM("Use Force/Torque: "<<bUseForce);

	if(!n.getParam("use_boxy", bUseIAI)) {
		bUseIAI = false;
	}
	ROS_INFO_STREAM("Use Boxy: "<<bUseIAI);

	if(!n.getParam("world_frame", world_frame)) {
			bUseIAI = false;
		}
		ROS_INFO_STREAM("world_frame: "<<world_frame);

	if(!n.getParam("simulation", simulation)) {
	} else {
		ROS_INFO_STREAM("Simulation Mode: "<< simulation);
	}

	return ret;
}

// Send out joint velocity, stiffness and damping
void sendJointMessage(Eigen::VectorXd& vel, Eigen::VectorXd& stiff, Eigen::VectorXd& damp) {

	if(bUseIAI) { // Using Boxy messages
		msg_vel_stiff_iai.header.stamp = ros::Time::now();
		for(int i=0; i<numdof; ++i) {
			if(i<3){
				msg_vel_stiff_iai.stiffness[i] = stiff[i]*1;
			}
			else
				msg_vel_stiff_iai.stiffness[i] = stiff[i];
			msg_vel_stiff_iai.damping[i] = damp[i];
			msg_vel_stiff_iai.velocity[i] = vel[i];			
		}
		msg_vel_stiff_iai.add_torque.clear();

		pub_joints.publish(msg_vel_stiff_iai);
	} else { // Using LASA messages
		for(int i=0; i<numdof; ++i) {
			msg_vel_stiff_jstate.velocity[i] = vel[i];
		}
		pub_joints.publish(msg_vel_stiff_jstate);
	}

}

//TODO: Compute the joint impedance here. Conversion from cart_stiffness to joint_stiffness not implemented yet.
void computeJointImpedance(Eigen::VectorXd& joint_stiff, Eigen::VectorXd& joint_damp) {
	if(joint_stiff.size() != numdof) {
		joint_stiff.resize(numdof);
	}
	if(joint_damp.size() != numdof) {
		joint_damp.resize(numdof);
	}
	for(int i=0; i<numdof; ++i) {
		joint_stiff[i] = DEFAULT_JSTIFF;
		joint_damp[i] = DEFAULT_JDAMP;
	}
}

// Convert difference in desired and sensed forces to cartesian velocity.
// Implemented as constant step (equal to force_tracking_gain) in the direction of force error.
// This works with the integration (bug) in the low level controller. It relies on the low level
// controller integrating the constant error until it reaches the desired force
void forceErrorToVelocity(const Eigen::VectorXd& force_err, Eigen::VectorXd& cart_vel) {

	double force_err_nrm;

	if(bOrientCtrl) {
		force_err_nrm = force_err.norm();
	} else {
		double force_error = force_err[0]*force_err[0]+force_err[1]*force_err[1]+force_err[2]*force_err[2];
		force_err_nrm = sqrt(force_error);
	}

	if(bOrientCtrl) {
		for(int i=0; i<6; ++i) {
			cart_vel[i] = force_err[i]/force_err_nrm*force_tracking_gain;
		}
	} else {
		for(int i=0; i<3; ++i) {
			cart_vel[i] = force_err[i]/force_err_nrm*force_tracking_gain;
		}
	}
}

// Convert current cartesian commands to joint velocity
void computeJointVelocity(Eigen::VectorXd& jvel) {
	if(jvel.size() != numdof) {
		jvel.resize(numdof);
	}

	if(est_ee_ft.norm() > max_ee_ft_norm) {
		for(int i=0; i<jvel.size(); ++i) {
			jvel[i]=0.0;
		}
		ROS_WARN_THROTTLE(0.1, "Too much force! Sending zero velocity");
		return;
	}

//	tf::Pose curr_ee_pose;
	mRobot->setJoints(read_jpos);
	mRobot->getEEPose(curr_ee_pose);

	// Two kinds of cartesian velocities - due to position error and force error
	Eigen::VectorXd vel_due_to_pos, vel_due_to_force;
	if(bOrientCtrl) {
		vel_due_to_pos.resize(6);
		vel_due_to_force.resize(6);
	} else {
		vel_due_to_pos.resize(3);
		vel_due_to_force.resize(3);
	}
	for(int i=0; i<vel_due_to_force.size(); ++i) {
		vel_due_to_force[i] = 0;
		vel_due_to_pos[i] = 0;
	}

	ROS_INFO_STREAM_THROTTLE(0.5, "Publishing EE TF");
	static tf::TransformBroadcaster br;
	if(ros::Time::now() - t_old > ros::Duration(0.1)){
		br.sendTransform(tf::StampedTransform(des_ee_pose, ros::Time::now(), world_frame, "/des_ee_tf"));
		br.sendTransform(tf::StampedTransform(curr_ee_pose, ros::Time::now(), world_frame, "/curr_ee_tf"));
		t_old = ros::Time::now();
	}
		
	

  // Cartesian velocity due to position/orientation error
	tf::Vector3 linvel = des_ee_pose.getOrigin() - curr_ee_pose.getOrigin();
	vel_due_to_pos(0) = linvel.getX();
	vel_due_to_pos(1) = linvel.getY();
	vel_due_to_pos(2) = linvel.getZ();
	double pos_err =  linvel.length();
	ROS_INFO_STREAM_THROTTLE(0.5, "Position Err:\t"<< pos_err);

// Nadia's way...
	// If Orientation is activated from launch file
	double qdiff = acos(abs(des_ee_pose.getRotation().dot(curr_ee_pose.getRotation())));
	if(bOrientCtrl) {
		// Computing angular velocity using quaternion difference
		tf::Quaternion tmp = curr_ee_pose.getRotation();
		tf::Quaternion angvel = (des_ee_pose.getRotation() - tmp)*tmp.inverse();
		vel_due_to_pos(3) = angvel.getX();
		vel_due_to_pos(4) = angvel.getY();
		vel_due_to_pos(5) = angvel.getZ();
		ROS_INFO_STREAM_THROTTLE(0.5, "Orient. Err:\t"<<qdiff);
	}

//	if (pos_err < 0.001 && qdiff < 0.01){ // LASA 0.5deg
      if (pos_err < 0.015 && qdiff < 0.05){ // Boxy 1.7 deg
		vel_due_to_pos(3) = 0;
		vel_due_to_pos(4) = 0;
		vel_due_to_pos(5) = 0;
	}


/*
// Aswhini's way...
	// If Orientation is activated from launch file
	if(bOrientCtrl) {
		// Computing angular velocity using quaternion difference
		tf::Quaternion tmp = curr_ee_pose.getRotation();
		tf::Quaternion angvel = (des_ee_pose.getRotation() - tmp)*tmp.inverse();
		vel_due_to_pos(3) = angvel.getX();
		vel_due_to_pos(4) = angvel.getY();
		vel_due_to_pos(5) = angvel.getZ();
        tf::Quaternion t = angvel;
//		ROS_INFO_STREAM_THROTTLE(0.5, "Orient. Err:\t"<<angvel.length());
	}
*/

	// Compute errors in position
	double err = vel_due_to_pos.norm();

	// Increase tracking performance by tuning traj_traking_gain
	vel_due_to_pos *= traj_tracking_gain;

	// If force is activated from launch file
	if(bUseForce) {

		// Compute force error
		Eigen::VectorXd ft_err = des_ee_ft - est_ee_ft;
		double ft_err_nrm;
		if(bOrientCtrl) {
			ft_err_nrm = ft_err.norm();
		} else {
			double force_error = ft_err[0]*ft_err[0] +ft_err[1]*ft_err[1]+ft_err[2]*ft_err[2];
			ft_err_nrm = sqrt(force_error);
		}

		// Remove force if too small. This is necessary due to residual errors in EE force estimation.
		// Dont apply external forces until desired position is almost reached within reach_tol
		if(ft_err_nrm > ft_tol && err < reach_tol) {
			forceErrorToVelocity(ft_err, vel_due_to_force);
		}
		ROS_INFO_STREAM_THROTTLE(0.5, "Force Err:\t"<<ft_err_nrm);
	}

	// Add the cartesian velocities due to position and force errors and compute the joint_velocity
	mRobot->getIKJointVelocity(vel_due_to_force+vel_due_to_pos, jvel);

}

// Main callback controlling the output rate.
void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg) {
	const sensor_msgs::JointState* data = msg.get();
	int ps = data->position.size();
	int name = data->name.size();

        //15 is not fixed (la,ra,torso)
	int joint_state_size; 
	if (simulation)	
		joint_state_size = 21;
	else
		joint_state_size = 14;
		
	//Check joint state message size
	if(name != joint_state_size)
	  return;

	// Collect the right joints by name
	int counter=0;
	for(unsigned int i=0;i<ps; ++i) {
		sprintf(buf, "right_arm_%d_joint", counter);
		if(!strcmp(buf, data->name[i].c_str())) {
			read_jpos[counter++] = data->position[i];
		}
		if(counter == numdof) { break; }
	}

	// If number of joints not as expected!
	if(counter != numdof) {
		ROS_WARN_STREAM_THROTTLE(1, "Only found "<<counter<<" DOF in message. Expected "<<numdof);
		isJointOkay = false;
	} else {
		isJointOkay = true;

		if(isAllOkay) {
			// All OK. Compute and send joint velocity		        
			computeJointImpedance(joint_stiff, joint_damp);
			computeJointVelocity(joint_vel);
			sendJointMessage(joint_vel, joint_stiff, joint_damp);
		}
		else
			ROS_WARN_STREAM_THROTTLE(1, "Not computing Joint Velocities");
	}
}

// Callback for the desired cartesian pose
void cartCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
	const geometry_msgs::PoseStamped* data = msg.get();
	des_ee_pose.setOrigin(tf::Vector3(data->pose.position.x,data->pose.position.y,data->pose.position.z));
	des_ee_pose.setRotation(tf::Quaternion(data->pose.orientation.x,data->pose.orientation.y,data->pose.orientation.z,data->pose.orientation.w));

	ROS_INFO_STREAM_THROTTLE(1, "Received Position: "<<des_ee_pose.getOrigin().x()<<","<<des_ee_pose.getOrigin().y()<<","<<des_ee_pose.getOrigin().z());
	if(bOrientCtrl) {
		tf::Quaternion q = des_ee_pose.getRotation();
		ROS_INFO_STREAM_THROTTLE(1, "Received Orientation: "<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w());
	}
}


// Callback for the desired end effector force/torque
void ftCallback(const geometry_msgs::WrenchStampedConstPtr& msg) {
	const geometry_msgs::WrenchStamped* data = msg.get();
	des_ee_ft[0] = data->wrench.force.x;
	des_ee_ft[1] = data->wrench.force.y;
	des_ee_ft[2] = data->wrench.force.z;

	des_ee_ft[3] = data->wrench.torque.x;
	des_ee_ft[4] = data->wrench.torque.y;
	des_ee_ft[5] = data->wrench.torque.z;

	ROS_INFO_STREAM_THROTTLE(1, "Received FT: "<<des_ee_ft[0]<<","<<des_ee_ft[1]<<","<<des_ee_ft[2]<<","
			<<des_ee_ft[3]<<","<<des_ee_ft[4]<<","<<des_ee_ft[5]);
}

// Estimated FT on the end-effector coming from the state estimator
void estFTCallback(const geometry_msgs::WrenchStampedConstPtr& msg) {
	const geometry_msgs::WrenchStamped* data = msg.get();
	est_ee_ft[0] = data->wrench.force.x;
	est_ee_ft[1] = data->wrench.force.y;
	est_ee_ft[2] = data->wrench.force.z;

	est_ee_ft[3] = data->wrench.torque.x;
	est_ee_ft[4] = data->wrench.torque.y;
	est_ee_ft[5] = data->wrench.torque.z;

	est_ee_ft -= shift_ee_ft;
	//	est_ee_ft[5] = -1.0;
	isFTOkay = true;
	ROS_INFO_STREAM_THROTTLE(1, "Received FT: "<<des_ee_ft[0]<<","<<des_ee_ft[1]<<","<<des_ee_ft[2]<<","
			<<des_ee_ft[3]<<","<<des_ee_ft[4]<<","<<des_ee_ft[5]);
}

// TODO: cartesian stiffness not used at the moment.
void stiffnessCallback(const geometry_msgs::WrenchStampedConstPtr& msg) {
	const geometry_msgs::WrenchStamped* data = msg.get();
	des_ee_stiff[0] = data->wrench.force.x;
	des_ee_stiff[1] = data->wrench.force.y;
	des_ee_stiff[2] = data->wrench.force.z;

	des_ee_stiff[3] = data->wrench.torque.x;
	des_ee_stiff[4] = data->wrench.torque.y;
	des_ee_stiff[5] = data->wrench.torque.z;
}


void actionStateCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_WARN("I heard action: [%s]. Doing SHIFT...", msg->data.c_str());
	if (msg->data.c_str()=="reach"){
		shift_ee_ft = est_ee_ft;
		ROS_WARN("Shiftiing!!");
		ROS_WARN_STREAM("Shift: "<<shift_ee_ft.transpose());
	} 
}


int main(int argc, char** argv) {

//	ros::init(argc, argv, "cart_to_joint", ros::init_options::NoSigintHandler);
	ros::init(argc, argv, "cart_to_joint");
	ros::NodeHandle nh;
	ros::NodeHandle _nh("~");
//	shut = false;
//	signal(SIGINT, handler);


	t_old = ros::Time::now();

	if(!parseParams(_nh)) {
		ROS_ERROR("Errors while parsing arguments.");
		return 1;
	}

	// Initialize robot with/without orientation (variable from launch file)
	mRobot = new RTKRobotArm(bOrientCtrl);
	if(!mRobot->initialize(_nh)) {
		ROS_ERROR("Error while loading robot");
		return 1;
	}

	numdof = mRobot->numdof;
	des_ee_ft.resize(6);
	est_ee_ft.resize(6);
	des_ee_stiff.resize(6);
	shift_ee_ft.resize(6);
	for(int i=0; i<6; ++i) {
		des_ee_ft(i)=0;
		est_ee_ft(i)=0;
		shift_ee_ft(i)=0;
	}

	read_jpos.resize(numdof);
	joint_lims.resize(numdof);

	// TODO: need to find a better way to do this from launch files
	/********** Specific for RTKRobotArm (KUKA) ****************/
/*	joint_lims[0] = 150;joint_lims[1] = 107;joint_lims[2] = 150; joint_lims[3] = 107;
	joint_lims[4] = 150;joint_lims[5] = 115;joint_lims[6] = 150;*/

	/********** Specific for RTKRobotArm (BOXY) ****************/
	joint_lims[0] = 150;joint_lims[1] = 107;joint_lims[2] = 150; joint_lims[3] = 107;
	joint_lims[4] = 150;joint_lims[5] = 115;joint_lims[6] = 150;

	joint_lims = joint_lims*M_PI/180.0;
	mRobot->setJointLimits(joint_lims);
	Eigen::VectorXd np; np.resize(numdof);
	np[0] = -0.62; np[1] = 0.76; np[2] = 0.61; np[3] = -1.0;
	np[4] = -0.40; np[5] = 1.4; np[6] = 1.4;
	// TODO: not implemented properly yet
//	mRobot->setNullPosture(np);
	/*********************************************************/

	if(bUseIAI) {
		msg_vel_stiff_iai.stiffness.resize(numdof);
		msg_vel_stiff_iai.damping.resize(numdof);
		msg_vel_stiff_iai.add_torque.resize(numdof);
		msg_vel_stiff_iai.velocity.resize(numdof);
	} else {
		msg_vel_stiff_jstate.velocity.resize(numdof);
	}


	if(bUseIAI) {
		pub_joints = nh.advertise<iai_control_msgs::MultiJointVelocityImpedanceCommand>(output_joints_topic, 3);
	} else {
		pub_joints = nh.advertise<sensor_msgs::JointState>(output_joints_topic, 3);
	}

	ros::Subscriber sub_pos = nh.subscribe<geometry_msgs::PoseStamped>(input_pose_topic, 1, cartCallback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber sub_ft = nh.subscribe<geometry_msgs::WrenchStamped>(input_ft_topic, 1, ftCallback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber sub_stiff = nh.subscribe<geometry_msgs::WrenchStamped>(input_stiff_topic, 1, stiffnessCallback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber sub_est_ft = nh.subscribe<geometry_msgs::WrenchStamped>(input_estimate_ft_topic, 1, estFTCallback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber sub_jstate = nh.subscribe<sensor_msgs::JointState>(input_joint_topic, 1, jointStateCallback, ros::TransportHints().tcpNoDelay());
        ros::Subscriber sub = nh.subscribe("Action_State", 1, actionStateCallback, ros::TransportHints().tcpNoDelay());


	joint_vel.resize(numdof); joint_stiff.resize(numdof), joint_damp.resize(numdof);
	for(int i=0; i<numdof; ++i) {
		joint_vel[i] = 0.;
		joint_stiff[i] = DEFAULT_JSTIFF;
		joint_damp[i] = DEFAULT_JDAMP;
	}

	ROS_INFO("Waiting for robot joint state and FT estimate topic...");
	ros::Rate r(1000);
	isJointOkay = false; isFTOkay = false; isAllOkay = false;
	while(ros::ok() && (!isJointOkay || (bUseForce && !isFTOkay)) ) {
//	while(ros::ok()){ 
		ros::spinOnce();
		r.sleep();
	}

	tf::Pose tmp;
	mRobot->setJoints(read_jpos);
	mRobot->getEEPose(tmp);

	des_ee_pose = tmp;

	// Remove the residual force/torque from the future estimates. To compensate for error in ee force estimation or tool calibration
	shift_ee_ft = est_ee_ft;

	ROS_INFO_STREAM("Shift: "<<shift_ee_ft.transpose());
	if(shift_ee_ft.norm() > ft_tol) {
		ROS_WARN("Shift in EE FT estimate more than the required tolerance!!");
		ROS_WARN("Either increase the tolerance or calibrate the tool better");
		ros::Duration warn(2);
		warn.sleep();
	}
	ROS_INFO("Node started");

	isAllOkay = true;

	// This is to ensure that we send atleast one zero velocity message before dying!
	// For safety.
	ros::Rate r_(1000);
	while(ros::ok()) {
		ros::spinOnce();
		r_.sleep();
	}

	for(int i=0; i<numdof; ++i) {
		joint_vel[i] = 0.0;
	}

	sendJointMessage(joint_vel, joint_stiff, joint_damp);
	ros::Duration(1.0).sleep();
	nh.shutdown();
	return 0;
}



