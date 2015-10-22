/*
 * RTKRobotArm.h
 *
 *  Created on: Nov 7, 2014
 *      Author: ashukla
 */

#ifndef _RTKRobotArm_H_
#define _RTKRobotArm_H_

#include "RobotArm.h"
#include "Robot.h"
#include "IKGroupSolver.h"
#include "KinematicChain.h"
#include "Various.h"
#include "MathLib.h"
#include "ros/ros.h"
#include <Eigen/Core>

#define DEFAULT_MAX_JVEL_DEG 		30

/**
 * Derived class for RobotToolKit (MathLib) robot.
 */
class RTKRobotArm : public RobotArm {
protected:
	Robot* mRobot;
	KinematicChain mKinematicChain;
	RevoluteJointSensorGroup    mSensorsGroup;
	MathLib::Vector full_robot_state, llim, ulim, null_posture;
	MathLib::IndicesVector mapping;
	IKGroupSolver mIKSolver;
	MathLib::Vector mJointVelLimits[2];

	bool bOrientCtrl, bUseNull;
	int nEndEffectorId;
	double max_jvel;
public:
	std::string rob_name;
	std::string data_path;
	RTKRobotArm();

	// Additional constructor for with or without orientation control
	RTKRobotArm(bool borient);
	virtual ~RTKRobotArm();
	// Node handle to read robot parameters from launch file. Robot name, end_eff, joint limits etc.
	virtual bool initialize(const ros::NodeHandle& nh);

	virtual void setJoints(const Eigen::VectorXd& joints);
	virtual void getJacobian(Eigen::MatrixXd& jacobian) ;
	virtual void getIKJointVelocity(const Eigen::VectorXd& cart_vel, Eigen::VectorXd& joint_vel);
	virtual void getEEPose(tf::Pose& pose) ;
	virtual void setJointLimits(const Eigen::VectorXd& lims);
	virtual void setNullPosture(const Eigen::VectorXd& n);

private:

	//private stuff to copy between eigen and mathlib
	static void copy(const Eigen::MatrixXd& from, MathLib::Matrix& to);
	static void copy(const MathLib::Matrix& from, Eigen::MatrixXd& to);
	static void copy(const MathLib::Vector& from, Eigen::VectorXd& to) ;
	static void copy(const Eigen::VectorXd& from, MathLib::Vector& to) ;

	bool _initialize(const ros::NodeHandle& n);
};



#endif
