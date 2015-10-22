/*
 * Copyright (C) 2014 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 *
 * motion_planner_pouring.cpp
 *
 * Created on : Feb 6, 2015
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
//-- TF Stuff --//
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
//-- Custom ActionLib Stuff --//
#include "actionlib/server/simple_action_server.h"
#include <lasa_action_planners/PLAN2CTRLAction.h>
//-- Message Types --//
#include <robohow_common_msgs/MotionPhase.h>
#include <robohow_common_msgs/MotionModel.h>
#include <robohow_common_msgs/GaussianMixtureModel.h>
#include <robohow_common_msgs/GaussianDistribution.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
//-- CDS Stuff --//
#include "CDSExecution.h"
//-- Eigen Stuff --//
#include <Eigen/Core>
#include <Eigen/Geometry>

#define EE_STATE_POSE_TOPIC "/joint_to_cart/est_ee_pose"
#define EE_STATE_FT_TOPIC "/joint_to_cart/est_ee_ft"
#define EE_CMD_POSE_TOPIC   "/cart_to_joint/des_ee_pose"
#define EE_CMD_FT_TOPIC   "/cart_to_joint/des_ee_ft"
#define BASE_LINK			"/base_link"
#define FORCE_SCALING		3.0
#define MAX_ROLLING_FORCE	30
#define FORCE_WAIT_TOL		5


tf::Pose ee_pose, curr_ee_pose, des_ee_pose;
Eigen::VectorXd ee_ft;
volatile bool isOkay, isFTOkay;
int mState;
string base_path;
bool initial_config = true, simulation;
int tf_count(0);
double reachingThreshold (0.01), orientationThreshold (0.02), model_dt (0.001); //Defaults: [m],[rad],[s]
double k = 1;

void eeStateCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
	const geometry_msgs::PoseStamped* data = msg.get();
	ee_pose.setOrigin(tf::Vector3(data->pose.position.x,data->pose.position.y,data->pose.position.z));
	ee_pose.setRotation(tf::Quaternion(data->pose.orientation.x,data->pose.orientation.y,data->pose.orientation.z,data->pose.orientation.w));
	isOkay = true;
}


class PLAN2CTRLAction
{
protected:

	// Pouring phases
	enum PouringPhase {
		PHASEHOME=1,
		PHASEPOUR=1,
		PHASEBACK=2
	};

	// Pouring modes. FIXED corresponds to numbers that are good LASA or ROMEO robots.
	// VISION will use the attractors provided in the action message
	enum ActionMode {
		ACTION_LASA_FIXED = 1,
		ACTION_ROMEO_FIXED,
		ACTION_VISION
	};

	ros::NodeHandle nh_;
	ros::Subscriber sub_, sub_ft_;
	ros::Publisher pub_, pub_ft_;
	geometry_msgs::PoseStamped msg_pose;
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

	void toPose(const MathLib::Matrix4& mat4, tf::Pose& pose) {
		MathLib::Matrix3 m1 = mat4.GetOrientation();
		MathLib::Vector3 v1 = m1.GetRotationAxis();
		tf::Vector3 ax(v1(0), v1(1), v1(2));
		pose.setRotation(tf::Quaternion(ax, m1.GetRotationAngle()));
		v1.Set(mat4.GetTranslation());
		pose.setOrigin(tf::Vector3(v1(0),v1(1),v1(2)));
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
	}

	// Do stuff with learned models
	// Phase - Reach, Roll or Back?
	// Dynamics type - to select the type of master/slave dynamics (linear/model etc.)
	// reachingThreshold - you know
	// model_dt - you know
	bool learned_model_execution(PouringPhase phase, CDSController::DynamicsType masterType, CDSController::DynamicsType slaveType,
			double reachingThreshold, double orientationThreshold, double model_dt,
			tf::Transform trans_obj, tf::Transform trans_att, std::string model_base_path) {

		ROS_INFO_STREAM(" Model Path "<<model_base_path);
		ROS_INFO_STREAM("Learned model execution with phase "<<phase);
		ROS_INFO_STREAM(" Reaching threshold "<<reachingThreshold);
		ROS_INFO_STREAM(" Orientation threshold "<<orientationThreshold);
		ROS_INFO_STREAM(" Model DT "<<model_dt);

		ros::Rate wait(1);
		tf::Transform  trans_final_target;

		// convert attractor information to world frame
		trans_final_target.mult(trans_obj, trans_att);

		ROS_INFO_STREAM("Final target origin "<<trans_final_target.getOrigin().getX()<<","<<trans_final_target.getOrigin().getY()<<","<<trans_final_target.getOrigin().getZ());
		ROS_INFO_STREAM("Final target orient "<<trans_final_target.getRotation().getX()<<","<<trans_final_target.getRotation().getY()<<","<<trans_final_target.getRotation().getZ()<<","<<trans_final_target.getRotation().getW());

		if (initial_config == true)
			curr_ee_pose = ee_pose;
		else
			curr_ee_pose = des_ee_pose;

		// Initialize CDS
		CDSExecution *cdsRun = new CDSExecution;
		cdsRun->initSimple(model_base_path, phase);
		cdsRun->setObjectFrame(toMatrix4(trans_obj));
		cdsRun->setAttractorFrame(toMatrix4(trans_att));
		cdsRun->setCurrentEEPose(toMatrix4(curr_ee_pose));
		cdsRun->setDT(model_dt);
		cdsRun->setMotionParameters(0.5,1,1,reachingThreshold, masterType, slaveType);
		cdsRun->postInit();

		ros::Duration loop_rate(model_dt);
		tf::Pose mNextRobotEEPose = curr_ee_pose;
		tf::Transform trans_ee;
		std::vector<double> gmr_in, gmr_out;
		gmr_in.resize(1);gmr_out.resize(1);
		double pos_err, ori_err, prog_curr, full_err;

		ROS_INFO("Execution started");
		tf::Pose p;
		bool bfirst = true;

		static tf::TransformBroadcaster br;
		while(ros::ok()) {
			if (initial_config == true)
				curr_ee_pose = ee_pose;
			else
				curr_ee_pose = des_ee_pose;

			// Publish attractors if running in simulation or with fixed values
			trans_ee.setRotation(tf::Quaternion(curr_ee_pose.getRotation()));
			trans_ee.setOrigin(tf::Vector3(curr_ee_pose.getOrigin()));

			// To Visualize EE Frames
			if (simulation==true){
				int frame_viz = int(model_dt*1000);
				if (tf_count==0 || tf_count%frame_viz==0){
					stringstream ss;
					ss <<  "/ee_tf_" << tf_count;
					br.sendTransform(tf::StampedTransform(trans_ee, ros::Time::now(), world_frame, ss.str()));
				}
				tf_count++;
			}
			else{
				br.sendTransform(tf::StampedTransform(trans_ee, ros::Time::now(), world_frame, "/ee_tf"));
			}


			br.sendTransform(tf::StampedTransform(trans_final_target, ros::Time::now(), world_frame, "/attractor"));
			br.sendTransform(tf::StampedTransform(trans_obj, ros::Time::now(), world_frame, "/object_frame"));

			// Current progress variable (position/orientation error).
			// TODO: send this back to action client as current progress
			pos_err = (trans_final_target.getOrigin() - curr_ee_pose.getOrigin()).length();
			//Real Orientation Error qdiff = acos(dot(q1_norm,q2_norm))*180/pi
			ori_err = acos(abs(trans_final_target.getRotation().dot(curr_ee_pose.getRotation())));
			ROS_INFO_STREAM_THROTTLE(0.5,"Position Threshold : " << reachingThreshold << " ... Current Error: "<<pos_err);
			ROS_INFO_STREAM_THROTTLE(0.5,"Orientation Threshold : " << orientationThreshold << " ... Current Error: "<<ori_err);

			double att_pos_err = (trans_final_target.getOrigin() - des_ee_pose.getOrigin()).length();
			double att_ori_err = acos(abs(trans_final_target.getRotation().dot(des_ee_pose.getRotation())));

			ROS_INFO_STREAM_THROTTLE(0.5,"Des-Att Position Error: " << att_pos_err);
			ROS_INFO_STREAM_THROTTLE(0.5,"Des-Att Orientation Error: " << att_ori_err);

			// Compute Next Desired EE Pose
			cdsRun->setCurrentEEPose(toMatrix4(mNextRobotEEPose));
			toPose(cdsRun->getNextEEPose(), mNextRobotEEPose);
			des_ee_pose = mNextRobotEEPose;

			// Make next pose the current pose for open-loop simulation
			if (simulation==true)
				initial_config=false;

			// If orientation error is VERY low or nan because of qdiff take target orientation
			if (att_ori_err < 0.005 || isnan(att_ori_err)) //[rad] and [m]//
//			if (isnan(att_ori_err)) //[rad] and [m]
				des_ee_pose.setRotation(tf::Quaternion(trans_final_target.getRotation()));

			// Send the computed pose from one of the above phases
			if (simulation==false)
				sendPose(des_ee_pose);

			// Broadcast/view Desired EE Pose
			br.sendTransform(tf::StampedTransform(des_ee_pose, ros::Time::now(), world_frame, "/des_ee_mp"));
			ROS_INFO_STREAM_THROTTLE(1, "Sent Position: "<<des_ee_pose.getOrigin().x()<<","<<des_ee_pose.getOrigin().y()<<","<<des_ee_pose.getOrigin().z());
			tf::Quaternion q = des_ee_pose.getRotation();
			ROS_INFO_STREAM_THROTTLE(1, "Sent Orientation: "<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w());

			feedback_.progress = prog_curr;
			as_.publishFeedback(feedback_);

			if(pos_err < reachingThreshold && (ori_err < orientationThreshold || isnan(ori_err))) {
				break;
			}
			loop_rate.sleep();
		}
		delete cdsRun;
		return ros::ok();
	}

public:

	PLAN2CTRLAction(std::string name) :
		as_(nh_, name, boost::bind(&PLAN2CTRLAction::executeCB, this, _1), false),
		action_name_(name)
{
		ee_ft.resize(6);
		// ROS TOPICS for controllers
		sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(EE_STATE_POSE_TOPIC, 1, eeStateCallback);
		pub_ = nh_.advertise<geometry_msgs::PoseStamped>(EE_CMD_POSE_TOPIC, 1);
		pub_ft_ = nh_.advertise<geometry_msgs::WrenchStamped>(EE_CMD_FT_TOPIC, 1);
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
		} else if(ad == "romeo_fixed") {
			action_mode = ACTION_ROMEO_FIXED;
		} else if(ad == "vision") {
			action_mode = ACTION_VISION;
		} else {
			ROS_ERROR_STREAM("Unrecognized action type. Must be one of 'lasa_fixed', 'romeo_fixed', 'vision'. Found '"<<ad<<"'");
			throw;
		}

		_nh.getParam("world_frame", world_frame);
		_nh.getParam("model_base_path", base_path);
		_nh.getParam("simulation", simulation);
		_nh.getParam("model_dt", model_dt);
		_nh.getParam("reachingThreshold", reachingThreshold);
		_nh.getParam("orientationThreshold", orientationThreshold);

		ROS_INFO_STREAM("Selected Action Mode: " << ad);
	}
	void executeCB(const lasa_action_planners::PLAN2CTRLGoalConstPtr &goal)
	{

		std::string desired_action = goal->action_type;
		ROS_INFO_STREAM( "Desired Action is " << desired_action);

		isOkay = false;
		ros::Rate r(10);
		ROS_INFO("Waiting for EE pose/ft topic...");
		while(ros::ok() && (!isOkay)) {
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


		// Use learned models to do shit
		if(goal->action_type=="LEARNED_MODEL"){
			PouringPhase phase;
			if(goal->action_name == "home") {
				phase = PHASEHOME;
			} else if(goal->action_name == "pour") {
				phase = PHASEPOUR;
			} else if(goal->action_name == "back") {
				phase = PHASEBACK;
			} else {
				ROS_ERROR_STREAM("Unidentified action name "<<goal->action_name.c_str());
				result_.success = 0;
				as_.setAborted(result_);
				return;
			}

			//TODO: update these from action
//			double reachingThreshold = 0.01, orientationThreshold = 0.02, model_dt = 0.001; //[m] [rad]
			CDSController::DynamicsType masterType = CDSController::MODEL_DYNAMICS;
			CDSController::DynamicsType slaveType = CDSController::UTHETA;
			tf::Transform trans_obj, trans_att;

			switch (action_mode) {
			case ACTION_ROMEO_FIXED:
				break;
			case ACTION_LASA_FIXED:

			case ACTION_VISION:
				 trans_obj.setRotation(tf::Quaternion(goal->object_frame.rotation.x,goal->object_frame.rotation.y,
					goal->object_frame.rotation.z,goal->object_frame.rotation.w));
			     trans_obj.setOrigin(tf::Vector3(goal->object_frame.translation.x, goal->object_frame.translation.y,
					goal->object_frame.translation.z));
				 trans_att.setRotation(tf::Quaternion(goal->attractor_frame.rotation.x,goal->attractor_frame.rotation.y,
							goal->attractor_frame.rotation.z,goal->attractor_frame.rotation.w));
				 trans_att.setOrigin(tf::Vector3(goal->attractor_frame.translation.x, goal->attractor_frame.translation.y,
							goal->attractor_frame.translation.z));
				break;
			default:
				break;
			}

			success = learned_model_execution(phase, masterType, slaveType, reachingThreshold, orientationThreshold,
					model_dt, trans_obj, trans_att, base_path);
		}

		result_.success = success;
		if(success)
		{
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
	PLAN2CTRLAction action_execution(ros::this_node::getName());
	action_execution.initialize();
	ros::spin();
    return 0;
}
