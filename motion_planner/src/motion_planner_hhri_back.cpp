/*
 * Copyright (C) 2014 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 *
 * motion_planner.cpp
 *
 * Created on : Nov 13, 2014
 * Author     : nbfigueroa
 * Email      : nadia.figueroafernandez@epfl.ch
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
#include <signal.h>

//-- TF Stuff --//
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

//-- Custom ActionLib Stuff --//
#include "actionlib/server/simple_action_server.h"
#include <lasa_action_planners/PLAN2CTRLAction.h>

//-- Message Types --//
#include <robohow_common_msgs/MotionPhase.h>
#include <robohow_common_msgs/MotionModel.h>
#include <robohow_common_msgs/GaussianMixtureModel.h>
#include <robohow_common_msgs/GaussianDistribution.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"

//-- CDS Stuff --//
#include "CDSExecution.h"

//-- Eigen Stuff --//
#include <Eigen/Core>
#include <Eigen/Geometry>


//#define EE_STATE_POSE_TOPIC "/joint_to_cart/est_ee_pose"
//#define EE_STATE_FT_TOPIC "/joint_to_cart/est_ee_ft"

//USING FRI RTK_MIRROR
#define EE_STATE_POSE_TOPIC "/KUKA/Pose"
#define EE_STATE_FT_TOPIC "/KUKA/FT"
//#define EE_STATE_FT_TOPIC "/joint_to_cart/est_ee_ft"
#define EE_STATE_STIFF_TOPIC "/KUKA/Stiff"

#define EE_CMD_POSE_TOPIC   "/cart_to_joint/des_ee_pose"
//#define EE_CMD_FT_TOPIC   "/cart_to_joint/des_ee_ft"
//#define EE_CMD_STIFF_TOPIC   "/cart_to_joint/des_ee_stiff"

//#define EE_CMD_POSE_TOPIC   "/KUKA/des_ee_pose"
#define EE_CMD_POSE_TOPIC2   "/KUKA/des_ee_pose"
#define EE_CMD_FT_TOPIC   "/KUKA/des_ee_ft"
#define EE_CMD_STIFF_TOPIC   "/KUKA/des_ee_stiff"



#define BASE_LINK			"/base_link"
#define MAX_ROLLING_FORCE	30
#define FORCE_WAIT_TOL		9

tf::Pose ee_pose;
Eigen::VectorXd ee_ft;
Eigen::VectorXd ee_stiff;
volatile bool isOkay, isFTOkay, shut;
int mState;
string base_path, path_matlab_plot;
bool use_boxy_tool=false, homing=false, mytruncate=false;
int plot_dyn = 0, plot_published = 0;
std_msgs::Float32 gmr_msg;
double FORCE_SCALING=1.5, init_err_f = 0.0;
std_msgs::Int32 plot_dyn_msg;
geometry_msgs::WrenchStamped msg_ft;
geometry_msgs::TwistStamped msg_stiff;
double table_height = 0.0;

// Boolean for detecting Ctrl-C
void handler(int sig) {
	shut = true;
}

/*void ppCallback(const std_msgs::Int32::ConstPtr& msg){
	plot_published =  msg->data;
	if (plot_published==1)
		ROS_WARN_STREAM("Plot Succesfully Published");
}*/


void toPose(const MathLib::Matrix4& mat4, tf::Pose& pose) {
	MathLib::Matrix3 m1 = mat4.GetOrientation();
	MathLib::Vector3 v1 = m1.GetRotationAxis();
	tf::Vector3 ax(v1(0), v1(1), v1(2));
	pose.setRotation(tf::Quaternion(ax, m1.GetRotationAngle()));
	v1.Set(mat4.GetTranslation());
	pose.setOrigin(tf::Vector3(v1(0),v1(1),v1(2)));
}

MathLib::Matrix4 toMatrix4(const tf::Pose& pose) {
	MathLib::Matrix4 mat;
	mat.Identity();
	tf::Matrix3x3 mat33(pose.getRotation());

	mat.SetTranslation(MathLib::Vector3(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()));
	mat.SetOrientation(MathLib::Matrix3(mat33[0][0], mat33[0][1], mat33[0][2],
			mat33[1][0], mat33[1][1], mat33[1][2],
			mat33[2][0], mat33[2][1], mat33[2][2]));
	return mat;
}


void eeStateCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
	const geometry_msgs::PoseStamped* data = msg.get();
	ee_pose.setOrigin(tf::Vector3(data->pose.position.x,data->pose.position.y,data->pose.position.z));
	ee_pose.setRotation(tf::Quaternion(data->pose.orientation.x,data->pose.orientation.y,data->pose.orientation.z,data->pose.orientation.w));

	//Add Roller
	static tf::TransformBroadcaster br;
//	tf::Transform  roller;
//	roller.setOrigin(tf::Vector3(0,0,0.28));
//	roller.setRotation(tf::Quaternion (0,0,0,1));
//
	tf::Transform  flange;
	flange.setOrigin(tf::Vector3 (ee_pose.getOrigin().x(),ee_pose.getOrigin().y(),ee_pose.getOrigin().z()));
	flange.setRotation(tf::Quaternion (ee_pose.getRotation().x(),ee_pose.getRotation().y(),ee_pose.getRotation().z(),ee_pose.getRotation().w()));
//
//	tf::Transform kuka_pose;
//	kuka_pose.mult(flange,roller);
	br.sendTransform(tf::StampedTransform(flange, ros::Time::now(), "/world_frame", "/kuka_pose"));
//
//	ee_pose.setOrigin(tf::Vector3 (kuka_pose.getOrigin()));
//	tf::Matrix3x3 rot = kuka_pose.getBasis();
//	tf::Quaternion q;
//	rot.getRotation(q);
//	ee_pose.setRotation(q);
	isOkay = true;
}

void eeFTCallback(const geometry_msgs::WrenchStampedConstPtr& msg) {
	const geometry_msgs::WrenchStamped* data = msg.get();
	ee_ft[0]= data->wrench.force.x;
	ee_ft[1]= data->wrench.force.y;
	ee_ft[2]= data->wrench.force.z;

	ee_ft[3]= data->wrench.torque.x;
	ee_ft[4]= data->wrench.torque.y;
	ee_ft[5]= data->wrench.torque.z;
	isFTOkay = true;
}

void eeStiffCallback(const geometry_msgs::TwistStampedConstPtr& msg) {
	const geometry_msgs::TwistStamped* data = msg.get();
	ee_stiff[0]= data->twist.linear.x;
	ee_stiff[1]= data->twist.linear.y;
	ee_stiff[2]= data->twist.linear.z;

	ee_stiff[3]= data->twist.angular.x;
	ee_stiff[4]= data->twist.angular.y;
	ee_stiff[5]= data->twist.angular.z;
}

class PLAN2CTRLAction
{
protected:

	// Rolling phases s
	enum DoughTaskPhase {
		PHASEREACH=1,
		PHASEROLL,
		PHASEBACK,
		PHASEHOME,
		PHASEREACH_HUMAN
	};

	// Rolling modes. FIXED corresponds to numbers that are good for BOXY and LASA robots.
	// VISION will use the attractors provided in the action message
	enum ActionMode {
		ACTION_LASA_FIXED = 1,
		ACTION_BOXY_FIXED,
		ACTION_VISION
	};

	ros::NodeHandle nh_;
	ros::Subscriber sub_, sub_ft_, sub_stiff_, sub_plot_pub_;
	ros::Publisher pub_, pub2_, pub_ft_, pub_stiff_, pub_action_state_, pub_gmr_out_, pub_action_name_, pub_model_fname_, pub_plot_dyn_, pub_ee_pos_att_, pub_ee_ft_att_;
	geometry_msgs::PoseStamped msg_pose, msg_pose2;
	geometry_msgs::WrenchStamped msg_ft;
	unsigned int action_mode;
	bool bWaitForForces;
	string world_frame;


	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
	actionlib::SimpleActionServer<lasa_action_planners::PLAN2CTRLAction> as_;
	std::string action_name_;
	// create messages that are used to published feedback/result
	lasa_action_planners::PLAN2CTRLFeedback feedback_;
	lasa_action_planners::PLAN2CTRLResult result_;


	// Load models from base_path depending on the PHASEROLL.
	// Model files should be stored in paths that match this pattern
	GMR* getNewGMRMappingModel(std::string base_path, std::string force_gmm_id) {
		char buf[1025];
		sprintf(buf, "%s/Phase%d/forceGMM_%s.txt", base_path.c_str(), PHASEROLL, force_gmm_id.c_str());
		GMR* gmr = new GMR(buf);
		std::vector<int> in(1), out(1);
		in[0] = 0; out[0] = 1;
		gmr->initGMR(in, out);

		if (strncmp(force_gmm_id.c_str(),"last",4)==0){
			FORCE_SCALING=2;
			ROS_WARN_STREAM("Applying scaling to GMR Output of: "<< FORCE_SCALING);
		}
		

		if(gmr->isInit) {
			return gmr;
		} else {
			delete gmr;
			return NULL;
		}
	}

	void getGMRResult(GMR* gmr, const std::vector<double>& input, std::vector<double>& output) {
		gmr->getGMROutput(input.data(), output.data());
	}


	// The famous sendPose!
	void sendPose(const tf::Pose& pose_) {
		geometry_msgs::PoseStamped msg;
		msg.pose.position.x = pose_.getOrigin().x();
		msg.pose.position.y = pose_.getOrigin().y();
		msg.pose.position.z = pose_.getOrigin().z();

		msg.pose.orientation.x = pose_.getRotation().x();
		msg.pose.orientation.y = pose_.getRotation().y();
		msg.pose.orientation.z = pose_.getRotation().z();
		msg.pose.orientation.w = pose_.getRotation().w();

		pub_.publish(msg);


//		// Remove Roller to send to rtk_mirror
		tf::Pose ee_pose2;
		ee_pose2.setOrigin(tf::Vector3(pose_.getOrigin().x(),pose_.getOrigin().y(),pose_.getOrigin().z()));
		ee_pose2.setRotation(tf::Quaternion(pose_.getRotation().x(),pose_.getRotation().y(),pose_.getRotation().z(),pose_.getRotation().w()));
//
//		//Remove Roller
//		tf::Transform  roller;
//		roller.setOrigin(tf::Vector3(0,0,-0.28));
//		roller.setRotation(tf::Quaternion (0,0,0,1));
//
		tf::Transform  roller_frame;
		roller_frame.setOrigin(tf::Vector3 (ee_pose2.getOrigin().x(),ee_pose2.getOrigin().y(),ee_pose2.getOrigin().z()));
		roller_frame.setRotation(tf::Quaternion (ee_pose2.getRotation().x(),ee_pose2.getRotation().y(),ee_pose2.getRotation().z(),ee_pose.getRotation().w()));
//
//		tf::Transform kuka_pose;
//		kuka_pose.mult(roller_frame,roller);
//
		static tf::TransformBroadcaster br;
		br.sendTransform(tf::StampedTransform(roller_frame, ros::Time::now(), "/world_frame", "/kuka_pose_rtk"));
//
//		ee_pose2.setOrigin(tf::Vector3 (kuka_pose.getOrigin()));
//		tf::Matrix3x3 rot = kuka_pose.getBasis();
//		tf::Quaternion q;
//		rot.getRotation(q);
//		ee_pose2.setRotation(q);
//
//		geometry_msgs::PoseStamped msg2;
//		msg2.pose.position.x = ee_pose2.getOrigin().x();
//		msg2.pose.position.y = ee_pose2.getOrigin().y();
//		msg2.pose.position.z = ee_pose2.getOrigin().z();
//
//		msg2.pose.orientation.x = ee_pose2.getRotation().x();
//		msg2.pose.orientation.y = ee_pose2.getRotation().y();
//		msg2.pose.orientation.z = ee_pose2.getRotation().z();
//		msg2.pose.orientation.w = ee_pose2.getRotation().w();
		pub2_.publish(msg);
	}

	void sendNormalForce(double fz) {
		ROS_INFO_STREAM("Sending normal force command:"<< fz);
		msg_ft.wrench.force.x = 0;
		msg_ft.wrench.force.y = 0;
		msg_ft.wrench.force.z = fz;

		msg_ft.wrench.torque.x = 0;
		msg_ft.wrench.torque.y = 0;
		msg_ft.wrench.torque.z = 0;

		pub_ft_.publish(msg_ft);
	}

	// This will block until the desired force is achieved!
	void sendAndWaitForNormalForce(double fz) {
		if(bWaitForForces) {
			ROS_INFO_STREAM("Waiting for force "<<fz<<" N.");
			sendPose(ee_pose);
			ros::Rate wait(1000);
			while(ros::ok()) {
				sendNormalForce(fz);
				ROS_INFO_STREAM("Z Force sensed on ee: "<< ee_ft[2] << " desired force: " << fz);
				ROS_INFO_STREAM("Sending Normal force: " << fz << " Fz diff: " << fabs(ee_ft[2]-fz));
				
				if(fabs(ee_ft[2]-fz) < FORCE_WAIT_TOL) {
					break;
				}
				ros::spinOnce();
				wait.sleep();
			}
		}
	}


	void sendStiff(const Eigen::VectorXd& stiff) {
		geometry_msgs::TwistStamped msg;
		msg_stiff.twist.linear.x = stiff[0];
		msg_stiff.twist.linear.y = stiff[1];
		msg_stiff.twist.linear.z = stiff[2];

		msg_stiff.twist.angular.x = stiff[3];
		msg_stiff.twist.angular.y = stiff[4];
		msg_stiff.twist.angular.z = stiff[5];

//		msg_stiff.twist.linear.x = 1200;
//		msg_stiff.twist.linear.y = 1200;
//		msg_stiff.twist.linear.z = 1200;
//
//		msg_stiff.twist.angular.x = 700;
//		msg_stiff.twist.angular.y = 700;
//		msg_stiff.twist.angular.z = 700;

		pub_stiff_.publish(msg_stiff);
	}

	// Go to this pose
	bool go_home(tf::Pose& pose_) {


		tf::Vector3 start_p(pose_.getOrigin());
		tf::Quaternion start_o(pose_.getRotation());

		msg_pose.pose.position.x = start_p.x();
		msg_pose.pose.position.y = start_p.y();
		msg_pose.pose.position.z = start_p.z();
		msg_pose.pose.orientation.w = start_o.w();
		msg_pose.pose.orientation.x = start_o.x();
		msg_pose.pose.orientation.y = start_o.y();
		msg_pose.pose.orientation.z = start_o.z();
		pub_.publish(msg_pose);
		sendNormalForce(0);

		ros::Rate thread_rate(2);
		ROS_INFO("Homing...");
		while(ros::ok()) {
			double oerr =(ee_pose.getRotation() - start_o).length() ;
			double perr = (ee_pose.getOrigin() - start_p).length();
			feedback_.progress = 0.5*(perr+oerr);
			as_.publishFeedback(feedback_);
			ROS_INFO_STREAM("Error: "<<perr<<", "<<oerr);
			if(perr< 0.02 && oerr < 0.02) {
				break;
			}
			if (as_.isPreemptRequested() || !ros::ok() )
			{
				sendNormalForce(0);
				sendPose(ee_pose);
				ROS_INFO("%s: Preempted", action_name_.c_str());
				// set the action state to preempted
				as_.setPreempted();
				return false;
			}
			thread_rate.sleep();
		}
		return ros::ok();

	}

	// Go down until hit the table. For safety min_height is specified. If no table found until this height, returns false.
	// vertical_speed with which to move downwards
	// thr_force - normal force threshold at which table is assumed to be detected
	bool find_table_for_rolling(double min_height, double vertical_speed, double thr_force) {
		double rate = 500;
		thr_force = fabs(thr_force);
		ros::Rate thread_rate(rate);

		double startz = ee_pose.getOrigin().z();

		msg_pose.pose.position.x = ee_pose.getOrigin().x();
		msg_pose.pose.position.y = ee_pose.getOrigin().y();
		msg_pose.pose.position.z = startz;
		msg_pose.pose.orientation.x = ee_pose.getRotation().x();
		msg_pose.pose.orientation.y = ee_pose.getRotation().y();
		msg_pose.pose.orientation.z = ee_pose.getRotation().z();
		msg_pose.pose.orientation.w = ee_pose.getRotation().w();

		// Publish attractors if running in simulation or with fixed values
		if ((action_mode == ACTION_LASA_FIXED) || (action_mode == ACTION_BOXY_FIXED)) {
		    static tf::TransformBroadcaster br;
			tf::Transform  table;
		    table.setOrigin(tf::Vector3 (ee_pose.getOrigin().x(),ee_pose.getOrigin().y(),ee_pose.getOrigin().z() - min_height));
		    table.setRotation(tf::Quaternion (ee_pose.getRotation().x(),ee_pose.getRotation().y(),ee_pose.getRotation().z(),ee_pose.getRotation().w()));
		    br.sendTransform(tf::StampedTransform(table, ros::Time::now(), world_frame, "/attractor"));
		}

		ROS_INFO_STREAM("Finding table up to max dist. "<<min_height<<" with vertical speed "<<vertical_speed<<" and threshold force "<<thr_force<<"N.");
		while(ros::ok()) {
			msg_pose.pose.position.z = msg_pose.pose.position.z - vertical_speed/rate;
			pub_.publish(msg_pose);
			pub2_.publish(msg_pose);

			// Go down until force reaches the threshold
			if(fabs(ee_ft[2]) > thr_force) {
				break;
			}
			if(fabs(ee_pose.getOrigin().z()-startz) > min_height) {
				ROS_INFO("Max distance reached");
				return false;
			}
			thread_rate.sleep();
			feedback_.progress = ee_ft[2];
			as_.publishFeedback(feedback_);
		}
		if(!ros::ok()) {
			return false;
		}
		tf::Vector3 table(ee_pose.getOrigin());
		ROS_INFO_STREAM("Table found at height "<<table[2]);
		msg_pose.pose.position.z = table[2];

		table_height = table[2] - 0.015;

		pub_.publish(msg_pose);
		pub2_.publish(msg_pose);
		sendAndWaitForNormalForce(0);


		return true;
	}

	// Roll with "force" and horizontal "speed" until the length "range"
	bool rolling(double force, double speed, double range) {

		ROS_INFO_STREAM("Rolling with force "<<force<<", speed "<<speed<<", range "<<range);
		force = fabs(force);

		sendNormalForce(force);
		msg_pose.pose.position.x  = ee_pose.getOrigin().x();
		msg_pose.pose.position.y  = ee_pose.getOrigin().y();
		msg_pose.pose.position.z  = ee_pose.getOrigin().z();

		tf::Quaternion q = ee_pose.getRotation();
		msg_pose.pose.orientation.x = q.x();
		msg_pose.pose.orientation.y = q.y();
		msg_pose.pose.orientation.z = q.z();
		msg_pose.pose.orientation.w = q.w();

		double center = ee_pose.getOrigin().y();
		double rate = 200;
		ros::Rate thread_rate(rate);
		int count=0;
		while(ros::ok()) {
			msg_pose.pose.position.y = msg_pose.pose.position.y + speed/rate;
			feedback_.progress = msg_pose.pose.position.y;
			as_.publishFeedback(feedback_);
			if( fabs(msg_pose.pose.position.y - center) >  range) {
				ROS_INFO("Change direction");
				speed *= -1;
				if(++count > 5) {
					break;
				}
			}
			pub_.publish(msg_pose);
			thread_rate.sleep();
		}

		msg_pose.pose.position.z = ee_pose.getOrigin().z() + 0.15;
		pub_.publish(msg_pose);
		sendNormalForce(0);

		return true;
	}

	// Do stuff with learned models
	// Phase - Reach, Roll or Back?
	// Dynamics type - to select the type of master/slave dynamics (linear/model etc.)
	// reachingThreshold - you know
	// model_dt - you know
	bool learned_model_execution(DoughTaskPhase phase, CDSController::DynamicsType masterType, CDSController::DynamicsType slaveType, double reachingThreshold, double model_dt, tf::Transform trans_obj, tf::Transform trans_att, std::string model_base_path, std::string force_gmm_id) {

		ROS_INFO_STREAM(" Model Path "<<model_base_path);	
		ROS_INFO_STREAM(" Learned model execution with phase "<<phase);
		ROS_INFO_STREAM(" Reaching threshold "<<reachingThreshold);
		ROS_INFO_STREAM(" Model DT "<<model_dt);
		if (force_gmm_id!="")
                 ROS_INFO_STREAM(" Force GMM ID: "<< force_gmm_id);


		ros::Rate wait(1);
		tf::Transform  trans_final_target, ee_pos_att;

		// To Visualize EE Frames		    
		static tf::TransformBroadcaster br;
		br.sendTransform(tf::StampedTransform(trans_final_target, ros::Time::now(), world_frame, "/attractor"));
		br.sendTransform(tf::StampedTransform(trans_obj, ros::Time::now(), world_frame, "/object_frame"));


		// convert attractor information to world frame
		trans_final_target.mult(trans_obj, trans_att);

		ROS_INFO_STREAM("Final target origin "<<trans_final_target.getOrigin().getX()<<","<<trans_final_target.getOrigin().getY()<<","<<trans_final_target.getOrigin().getZ());
		ROS_INFO_STREAM("Final target orient "<<trans_final_target.getRotation().getX()<<","<<trans_final_target.getRotation().getY()<<","<<trans_final_target.getRotation().getZ()<<","<<trans_final_target.getRotation().getW());

		// Publish attractors if running in simulation or with fixed values
		if ((action_mode == ACTION_LASA_FIXED) || (action_mode == ACTION_BOXY_FIXED)) {
		    br.sendTransform(tf::StampedTransform(trans_final_target, ros::Time::now(), world_frame, "/attractor"));
		}

		// Initialize CDS
		CDSExecution *cdsRun = new CDSExecution;
		cdsRun->initSimple(model_base_path, phase, force_gmm_id);
		cdsRun->setObjectFrame(toMatrix4(trans_obj));
		cdsRun->setAttractorFrame(toMatrix4(trans_att));
		cdsRun->setCurrentEEPose(toMatrix4(ee_pose));
		cdsRun->setDT(model_dt);


		if (phase==PHASEBACK || phase==PHASEROLL)
			 masterType = CDSController::LINEAR_DYNAMICS;

		// Roll slow but move fast for reaching and back phases.
		// If models have proper speed, this whole block can go!
		if(phase == PHASEROLL) {
			//cdsRun->setMotionParameters(1,1,0.5,reachingThreshold, masterType, slaveType);
			cdsRun->setMotionParameters(1,1,2,reachingThreshold, masterType, slaveType);
			// large threshold to avoid blocking forever
			// TODO: should rely on preempt in action client.
//			reachingThreshold = 0.02;
//			reachingThreshold = 0.025;
		} else {
			cdsRun->setMotionParameters(1,1,2,reachingThreshold, masterType, slaveType);
		}


		cdsRun->postInit();

		// If phase is rolling, need force model as well
		GMR* gmr_perr_force = NULL;
		if(phase == PHASEROLL) {
			gmr_perr_force = getNewGMRMappingModel(model_base_path, force_gmm_id);
			if(!gmr_perr_force) {
				ROS_ERROR("Cannot initialize GMR model");
				return false;
			}
		}

		ros::Duration loop_rate(model_dt);
		tf::Pose mNextRobotEEPose = ee_pose;
		tf::Quaternion q_curr, q_target;
		std::vector<double> gmr_in, gmr_out;
		gmr_in.resize(1);gmr_out.resize(1);
		double prog_curr, full_err, pos_err, ori_err, new_err, gmr_err;
		tf::Vector3 att_t, curr_t;

		ROS_INFO("Execution started");
		tf::Pose p;
		bool bfirst = true;

		std_msgs::String string_msg, action_name_msg, model_fname_msg;
		std::stringstream ss, ss_model;

		if(phase ==  PHASEREACH) {
			ss << "reach ";
		}
		else if (phase == PHASEROLL){
			ss << "roll";
		}	
		else if (phase == PHASEBACK){
			ss << "back";
		}		

		if (!homing){
			string_msg.data = ss.str();
			pub_action_state_.publish(string_msg);


//			ss_model << path_matlab_plot << "/Phase" <<  phase << "/masterGMM.fig";

			if (force_gmm_id=="")
				ss_model << path_matlab_plot << "/Phase" <<  phase << "/masterGMM.fig";
			else
				ss_model << path_matlab_plot << "/Phase" <<  phase << "/ForceProfile_" << force_gmm_id << ".fig";
			//ss_model << "/Phase" <<  phase << "/masterGMM.fig";
			
			model_fname_msg.data = ss_model.str();		
			pub_model_fname_.publish(model_fname_msg);

			action_name_msg.data = ss.str();
			pub_action_name_.publish(action_name_msg);

			plot_dyn = 1;			
			plot_dyn_msg.data = plot_dyn;
			pub_plot_dyn_.publish(plot_dyn_msg);
			
		}
		
		while(ros::ok()) {
			
			if (!homing)
				plot_dyn = 1;
			else
				plot_dyn = 0;

			plot_dyn_msg.data = plot_dyn;
			pub_plot_dyn_.publish(plot_dyn_msg);

			// Nadia's stuff
			// Current progress variable (position/orientation error).
			// TODO: send this back to action client as current progress
			pos_err = (trans_final_target.getOrigin() - ee_pose.getOrigin()).length();
			//Real Orientation Error qdiff = acos(dot(q1_norm,q2_norm))*180/pi
			ori_err = acos(abs(trans_final_target.getRotation().dot(ee_pose.getRotation())));


			ROS_INFO_STREAM_THROTTLE(0.5,"Position Threshold : " << reachingThreshold << " ... Current Error: "<<pos_err);
			ROS_INFO_STREAM_THROTTLE(0.5,"Orientation Threshold : " << 0.01 << " ... Current Error: "<<ori_err);

			/*double att_pos_err = (trans_final_target.getOrigin() - p.getOrigin()).length();
			double att_ori_err = acos(abs(trans_final_target.getRotation().dot(p.getRotation())));

			ROS_INFO_STREAM_THROTTLE(0.5,"Des-Att Position Error: " << att_pos_err);
			ROS_INFO_STREAM_THROTTLE(0.5,"Des-Att Orientation Error: " << att_ori_err);*/

			switch (phase) {
			// Home, reach and back are the same control-wise
			case PHASEREACH:

			case PHASEBACK:
				// Current progress variable (position/orientation error).
				// TODO: send this back to action client as current progress
				prog_curr = 0.5*((trans_final_target.getOrigin() - ee_pose.getOrigin()).length() + (trans_final_target.getRotation()-ee_pose.getRotation()).length());

				// Compute Next Desired EE Pose
				cdsRun->setCurrentEEPose(toMatrix4(mNextRobotEEPose));
				toPose(cdsRun->getNextEEPose(), mNextRobotEEPose);
				p = mNextRobotEEPose;

				// Aswhini's Hack! Dont rely on model's orientation interpolation. Set it equal to target orientation to avoid
				// going the wrong way around
				q_target = ee_pose.getRotation().slerp(trans_final_target.getRotation(),0.15);
				p.setRotation(q_target);


				//Publish desired force	
				gmr_msg.data = 0.0;
				pub_gmr_out_.publish(gmr_msg);

				ee_stiff[0] = 700;ee_stiff[1] = 700;ee_stiff[2] = 700;
				ee_stiff[3] = 200;ee_stiff[4] = 200;ee_stiff[5] = 200;

				break;

			// Home, reach and back are the same control-wise
			case PHASEREACH_HUMAN:
				p = ee_pose;
				ee_stiff[0] = 100;ee_stiff[1] = 100;ee_stiff[2] = 100;
				ee_stiff[3] = 200;ee_stiff[4] = 200;ee_stiff[5] = 200;
				break;

			case PHASEROLL:

				// Current progress in rolling phase is simply the position error	
				prog_curr = (trans_final_target.getOrigin() - ee_pose.getOrigin()).length();

				// New position error being fed to GMR Force Model	
				att_t = tf::Vector3(trans_final_target.getOrigin().getX(),trans_final_target.getOrigin().getY(),0.0);
				curr_t = tf::Vector3(ee_pose.getOrigin().getX(),ee_pose.getOrigin().getY(),0.0);
				new_err = (att_t - curr_t).length();

 				gmr_err = new_err;
				//Hack! Truncate errors to corresponding models				
				if ((strncmp(force_gmm_id.c_str(),"first",5)==0) && (new_err > 0.03)){
					gmr_err = 0.03;
 				}
				
				if ((strncmp(force_gmm_id.c_str(),"mid",3)==0) && (new_err > 0.07)){
					gmr_err = 0.07;
 				}
				if ((strncmp(force_gmm_id.c_str(),"last",4)==0) && (new_err > 0.13)){
					gmr_err = 0.13;
				}
				//pos_err = prog_curr;
				ori_err = 0;
				gmr_err = -gmr_err;

				gmr_in[0] = gmr_err; // distance between EE and attractor

				// Query the model for desired force
				getGMRResult(gmr_perr_force, gmr_in, gmr_out);

/*				double fz_plot;
				getGMRResult(gmr_perr_force, -gmr_in, fz_plot);*/
				//-> INSTEAD OF SENDING GMR OUTPUT / SEND EST_EE_FT(Z)
				// Send fz and distance to attractor for plotting		
				msg_ft.wrench.force.x = gmr_err;
				msg_ft.wrench.force.y = gmr_out[0]; //ee_ft[2]
				msg_ft.wrench.force.z = 0;
				msg_ft.wrench.torque.x = 0;
				msg_ft.wrench.torque.y = 0;
				msg_ft.wrench.torque.z = 0;
				pub_ee_ft_att_.publish(msg_ft);

				// Hack! Scale the force to be in reasonable values
				gmr_out[0] = FORCE_SCALING*fabs(gmr_out[0]);

				ROS_INFO_STREAM_THROTTLE(0.5,"Distance to Attractor: " << new_err << " GMR output (N): " << gmr_out[0]);

				// Hack! Safety first!
				if(gmr_out[0] > MAX_ROLLING_FORCE) {
					gmr_out[0] = MAX_ROLLING_FORCE;
				}

				gmr_msg.data = gmr_out[0];
				pub_gmr_out_.publish(gmr_msg);


				// Give some time for the force to catch up the first time. Then roll with constant force thereafter.
				if(bfirst) {
//					sendAndWaitForNormalForce(gmr_out[0]);
					sendAndWaitForNormalForce(20);
					bfirst = false;
				} else {
//					sendNormalForce(gmr_out[0]);
					sendNormalForce(20);
				}
				ROS_INFO_STREAM_THROTTLE(0.5, "Force Sent to Controller: "<<gmr_out[0]);



				cdsRun->setCurrentEEPose(toMatrix4(mNextRobotEEPose));
				toPose(cdsRun->getNextEEPose(), mNextRobotEEPose);

				p = mNextRobotEEPose;

				// Hack! Dont rely on model orientation. Use target orientation instead.
				p.setRotation(trans_final_target.getRotation());

				// Hack! Dont rely on the Z component of the model. It might go below the table!
				p.getOrigin().setZ(trans_final_target.getOrigin().getZ());
				p.getOrigin().setZ(table_height);

				ee_stiff[0] = 700;ee_stiff[1] = 700;ee_stiff[2] = 100;
				ee_stiff[3] = 200;ee_stiff[4] = 200;ee_stiff[5] = 200;

				homing=false;
				break;
			default:
				ROS_ERROR_STREAM("No such phase defined "<<phase);
				return false;
			}

			// Send the computed pose from one of the above phases
			sendPose(p);

			// Send the computed stiffness from one of the above phases
			sendStiff(ee_stiff);


			// convert and send ee pose to attractor frame to plots
			ee_pos_att.mult(trans_final_target.inverse(), p);
			geometry_msgs::PoseStamped msg;
			msg.pose.position.x = ee_pos_att.getOrigin().x();
			msg.pose.position.y = ee_pos_att.getOrigin().y();
			msg.pose.position.z = ee_pos_att.getOrigin().z();
			msg.pose.orientation.x = ee_pos_att.getRotation().x();
			msg.pose.orientation.y = ee_pos_att.getRotation().y();
			msg.pose.orientation.z = ee_pos_att.getRotation().z();
			msg.pose.orientation.w = ee_pos_att.getRotation().w();
			pub_ee_pos_att_.publish(msg);
			

			//ROS_INFO_STREAM_THROTTLE(0.5,"Error "<<prog_curr);

			// check that preempt has not been requested by the client
			if (as_.isPreemptRequested() || !ros::ok())
			{
				sendPose(ee_pose);
				sendNormalForce(0);
				ROS_INFO("%s: Preempted", action_name_.c_str());
				// set the action state to preempted
				as_.setPreempted();
				return false;
			}
			feedback_.progress = prog_curr;
			as_.publishFeedback(feedback_);
			//Orientation error	0.05
			if(pos_err < reachingThreshold && (ori_err < 0.05 || isnan(ori_err))) {
				break;
			}


			loop_rate.sleep();
		}
		delete cdsRun;

		if(phase ==  PHASEREACH) {
			// Hack! If phase is "reach", find the table right after reaching
			if (bWaitForForces && !homing)	{		
				bool x = find_table_for_rolling(0.35, 0.1, 5);
				//-> Send command to dynamics plotter to stop logging				 
				return x;
			}
			if (!homing){
				//-> Send command to dynamics plotter to stop logging				 
			}
		} else if (phase == PHASEROLL){
			// Hack! wait for zero force before getting ready to recieve further commands.
			// This is to avoid dragging the dough.
			sendAndWaitForNormalForce(0);
			//-> Send command to dynamics plotter to start plotting
			return ros::ok();
		} else {
			//->Send command to dynamics plotter to start plotting
			return ros::ok();
		}
	}

public:

	PLAN2CTRLAction(std::string name) :
		as_(nh_, name, boost::bind(&PLAN2CTRLAction::executeCB, this, _1), false),
		action_name_(name)
{
		ee_ft.resize(6);
		ee_stiff.resize(6);
		// ROS TOPICS for controllers
		sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(EE_STATE_POSE_TOPIC, 1, eeStateCallback);
		sub_ft_ = nh_.subscribe<geometry_msgs::WrenchStamped>(EE_STATE_FT_TOPIC, 1, eeFTCallback);
		sub_stiff_ = nh_.subscribe<geometry_msgs::TwistStamped>(EE_STATE_STIFF_TOPIC, 1, eeStiffCallback);

		pub_ = nh_.advertise<geometry_msgs::PoseStamped>(EE_CMD_POSE_TOPIC, 1);
		pub2_ = nh_.advertise<geometry_msgs::PoseStamped>(EE_CMD_POSE_TOPIC2, 1);
		pub_ft_ = nh_.advertise<geometry_msgs::WrenchStamped>(EE_CMD_FT_TOPIC, 1);
		pub_stiff_ = nh_.advertise<geometry_msgs::TwistStamped>(EE_CMD_STIFF_TOPIC, 1);

		pub_gmr_out_ = nh_.advertise<std_msgs::Float32>("GMR_OUT", 1);
		pub_action_state_ = nh_.advertise<std_msgs::String>("Action_State", 1000);
		pub_action_name_ = nh_.advertise<std_msgs::String>("/motion_planner/action_name", 1000);
		pub_model_fname_ = nh_.advertise<std_msgs::String>("/motion_planner/model_file_name", 1000);
		pub_plot_dyn_ = nh_.advertise<std_msgs::Int32>("/motion_planner/plot_dynamics", 1000);
		pub_ee_pos_att_ = nh_.advertise<geometry_msgs::PoseStamped>("/motion_planner/ee_pos_att", 1000);
		pub_ee_ft_att_ = nh_.advertise<geometry_msgs::WrenchStamped>("/motion_planner/ee_ft_att", 1000);
		//sub_plot_pub_ = nh_.subscribe<std_msgs::Int32>("/motion_planner/plot_published", 10, ppCallback);
		as_.start();
}


	~PLAN2CTRLAction(void)
	{
	}

	// Select which mode to use from launch file. Fixed numbers for LASA/BOXY robot or the Vision tracking.
	void initialize() {
		std::string ad;
		ros::NodeHandle _nh("~");
		_nh.getParam("action_mode", ad);
		if(ad == "lasa_fixed") {
			action_mode = ACTION_LASA_FIXED;
		} else if(ad == "boxy_fixed") {
			action_mode = ACTION_BOXY_FIXED;
		} else if(ad == "vision") {
			action_mode = ACTION_VISION;
		} else {
			ROS_ERROR_STREAM("Unrecognized action type. Must be one of 'lasa_fixed', 'boxy_fixed', 'vision'. Found '"<<ad<<"'");
			throw;
		}

		_nh.getParam("world_frame", world_frame);
		_nh.getParam("model_base_path", base_path);
		_nh.getParam("use_boxy_tool", use_boxy_tool);
		_nh.getParam("path_matlab_plot", path_matlab_plot);
		
		if (use_boxy_tool)
		   ROS_INFO("Using Boxy Tool adding extra Rotation");

		ROS_INFO_STREAM("Selected Action Mode: " << ad);

		// Useful for running this in simulation. WaitForForces() would never return if ran with simulator.
		if(!_nh.getParam("wait_for_force", bWaitForForces)) {
			ROS_INFO_STREAM("Set the Waiting for forces flag");
			bWaitForForces = true;
		}
	}
	void executeCB(const lasa_action_planners::PLAN2CTRLGoalConstPtr &goal)
	{

		std::string desired_action = goal->action_type;
		ROS_INFO_STREAM( "Desired Action is " << desired_action);

		isOkay = false, isFTOkay = false;
		ros::Rate r(10);
		ROS_INFO("Waiting for EE pose/ft topic...");
		while(ros::ok() && (!isOkay || !isFTOkay)) {
			r.sleep();
		}
		if(!ros::ok()) {
			result_.success = 0;
			ROS_INFO("%s: Failed", action_name_.c_str());
			as_.setAborted(result_);
			return;
		}

		// initialize action progress as null
		feedback_.progress = 0;
		bool success = false;
		///////////////////////////////////////////////
		/////----- EXECUTE REQUESTED ACTION ------/////
		///////////////////////////////////////////////

		if (goal->action_type=="HOME"){
			// TODO: do something meaningful here
			tf::Pose pose(ee_pose);
			success = go_home(pose);
		} else if(goal->action_type == "HOME_POUR") {
			// Home for pouring with lasa robot
			tf::Pose pose(ee_pose);
			pose.setOrigin(tf::Vector3(-0.65, -0.11, 0.474));
			pose.setRotation(tf::Quaternion(-0.006, 0.907, -0.420, -0.114));
			success = go_home(pose);
		}
		if(goal->action_type=="FIND_TABLE"){
			// Go down until table found
			tf::Pose pose(ee_pose);
			success = go_home(ee_pose);
			if(success) {
				success = find_table_for_rolling(goal->height, 0.03, goal->threshold);
			}
		}
		if(goal->action_type=="ROLL_TEST"){

			/**** For now use this instead **/
			tf::Pose p(ee_pose);
			/*********************************/

			success = go_home(p);
			if(success) {
				success = find_table_for_rolling(0.15, 0.03, 5);
				if(success) {
					success = rolling(goal->force, goal->speed, 0.1);
				}
			}
		}
		// Use learned models to do shit
		if(goal->action_type=="LEARNED_MODEL"){
			DoughTaskPhase phase;
			if(goal->action_name == "roll") {
				phase = PHASEROLL;
			} else if(goal->action_name == "reach") {
				phase = PHASEREACH;
			} else if(goal->action_name == "back") {
				phase = PHASEBACK;
			} else if(goal->action_name == "home") {
				phase = PHASEHOME;
			} else if(goal->action_name == "reach_human") {
				phase = PHASEREACH_HUMAN;
			} else {
				ROS_ERROR_STREAM("Unidentified action name "<<goal->action_name.c_str());
				result_.success = 0;
				as_.setAborted(result_);
				return;
			}

			//TODO: update these from action
			double reachingThreshold = 0.02;
			double model_dt = 0.001; //model_dt = 0.01;
			//CDSController::DynamicsType masterType = CDSController::LINEAR_DYNAMICS;
			CDSController::DynamicsType masterType = CDSController::MODEL_DYNAMICS;
//			CDSController::DynamicsType slaveType = CDSController::LINEAR_DYNAMICS;
			CDSController::DynamicsType slaveType = CDSController::UTHETA;
			tf::Transform trans_obj, trans_att;

			//Little hack for using reaching phase for HOMING				
			if (phase==PHASEHOME){
				     phase=PHASEREACH;
				     homing = true;
			}

			switch (action_mode) {
			case ACTION_BOXY_FIXED:
				
				trans_obj.setIdentity();
				trans_obj.setOrigin(tf::Vector3(0.8, -0.43, 0.64)); 

				if(phase == PHASEREACH) {
				        trans_att.setOrigin(tf::Vector3(-0.05, 0,0)); // TODO: remember to add 0.15 to tf values as well
					trans_att.setRotation(tf::Quaternion(-0.01, 0.99, 0.005, -0.009));
				} else if(phase == PHASEROLL) {
					trans_att.setOrigin(tf::Vector3(0.05, 0, 0));
					trans_att.setRotation(tf::Quaternion(-0.01, 0.99, 0.005, -0.009));
				} else if(phase == PHASEBACK) {
					trans_att.setOrigin(tf::Vector3(-0.15, -0.2, 0.15));
					trans_att.setRotation(tf::Quaternion(-0.01, 0.99, 0.005, -0.009));
				}
				break;
			case ACTION_LASA_FIXED:
				if(phase == PHASEREACH) {
					trans_obj.setOrigin(tf::Vector3(-0.55, -0.10, 0.3)); // TODO: remember to add 0.15 to tf values as well (z was 0.15)
					trans_obj.setRotation(tf::Quaternion(0.0, 1.0, 0.0, 0.0));
				} else if(phase == PHASEROLL) {
					trans_obj.setOrigin(tf::Vector3(-0.55, -0.25, ee_pose.getOrigin().z()));
					trans_obj.setRotation(tf::Quaternion(0.0, 1.0, 0.0, 0.0));
				} else if(phase == PHASEBACK) {
					trans_obj.setOrigin(tf::Vector3(-0.55, -0.25, 0.3)); //(z was 0.15)
					trans_obj.setRotation(tf::Quaternion(0.0, 1.0, 0.0, 0.0));
				}
				trans_att.setIdentity();
				break;
			case ACTION_VISION:
				trans_obj.setRotation(tf::Quaternion(goal->object_frame.rotation.x,goal->object_frame.rotation.y,
						goal->object_frame.rotation.z,goal->object_frame.rotation.w));
				trans_obj.setOrigin(tf::Vector3(goal->object_frame.translation.x, goal->object_frame.translation.y,
						goal->object_frame.translation.z+0.28));

				trans_att.setRotation(tf::Quaternion(goal->attractor_frame.rotation.x,goal->attractor_frame.rotation.y,
						goal->attractor_frame.rotation.z,goal->attractor_frame.rotation.w));
				trans_att.setOrigin(tf::Vector3(goal->attractor_frame.translation.x, goal->attractor_frame.translation.y,
						goal->attractor_frame.translation.z));


				if (bWaitForForces){ //HACK FOR BOXY
					// Hack! For setting heights for reach and add offset of roller 
					if(phase == PHASEREACH) {
						trans_att.setOrigin(tf::Vector3(goal->attractor_frame.translation.x, goal->attractor_frame.translation.y,goal->attractor_frame.translation.z + 0.1));
					}

					// Hack! safety for rolling
					if(phase == PHASEROLL) {
						trans_obj.setOrigin(tf::Vector3(goal->object_frame.translation.x, goal->object_frame.translation.y,ee_pose.getOrigin().getZ()));
						trans_att.setOrigin(tf::Vector3(goal->attractor_frame.translation.x, goal->attractor_frame.translation.y, 0.0));				
					}
				}

				/*if (bWaitForForces){ //HACK FOR LASA
					// Hack! For setting heights for reach and back
					if(phase == PHASEREACH || phase == PHASEBACK) {
						trans_obj.getOrigin().setZ(0.15);
						trans_att.getOrigin().setZ(0.0);
					}

					// Hack! safety for rolling
					if(phase == PHASEROLL) {
						trans_obj.getOrigin().setZ(ee_pose.getOrigin().getZ());
						trans_att.getOrigin().setZ(0.0);
					}
				}*/

				break;
			default:
				break;
			}


			std::string force_gmm_id = goal->force_gmm_id;
			success = learned_model_execution(phase, masterType, slaveType, reachingThreshold,
					model_dt, trans_obj, trans_att, base_path, force_gmm_id);


		}

		result_.success = success;

                homing=false;
		if(success)
		{

			if (!homing){
					ROS_WARN_STREAM("STORE PLOT");
					plot_dyn = 2;
					plot_dyn_msg.data = plot_dyn;	
					pub_plot_dyn_.publish(plot_dyn_msg);
					ros::Rate r(1);
					r.sleep();
			}

			//Wait for message of "plot target_link_libraries(motion_planner   ${catkin_LIBRARIES} )"
			/*ros::Rate wait(1000);
			while(ros::ok() && (plot_published!=1)) {
				ros::spinOnce();
				wait.sleep();
			}*/						
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			as_.setSucceeded(result_);
				

		} else {
			ROS_INFO("%s: Failed", action_name_.c_str());
			as_.setAborted(result_);
		}




	}

};


int main(int argc, char** argv) {

	ros::init(argc, argv, "plan2ctrl");

	ROS_INFO("Initializing Server");
	ROS_INFO("doing motion planner lasa");


	PLAN2CTRLAction action_execution(ros::this_node::getName());
	action_execution.initialize();

	ros::Rate wait(1000);
	while(ros::ok()) {
		ros::spinOnce();
		wait.sleep();
	}

	return 0;
}
