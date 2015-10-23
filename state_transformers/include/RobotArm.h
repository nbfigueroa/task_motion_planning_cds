/*
 * RobotArm.h
 *
 *  Created on: Nov 17, 2014
 *      Author: ashukla
 */

#ifndef _ROBOTARM_H_
#define _ROBOTARM_H_

#include <tf/tf.h>
#include <Eigen/Core>

/***
 * Interface class for providing basic robot functionality. Derived classes must implement these methods
 *
 */
class RobotArm {
protected:
//	RobotArm(): numdof(0){}

public:
	std::string base_link;
	std::string ee_link;
	int numdof;

	virtual ~RobotArm(){}
	virtual bool initialize(const ros::NodeHandle& nh)=0;
	virtual void getIKJointVelocity(const Eigen::VectorXd& cart_vel, Eigen::VectorXd& joint_vel)=0;
	virtual void setJoints(const Eigen::VectorXd& joints)=0;
	virtual void getJacobian(Eigen::MatrixXd& jacobian)=0;
	virtual void getEEPose(tf::Pose& pose) = 0;
	virtual void setJointLimits(const Eigen::VectorXd& lims) =0;
	virtual void setNullPosture(const Eigen::VectorXd& n)=0;

};


#endif /* _ROBOTARM_H_ */
