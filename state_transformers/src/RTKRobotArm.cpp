/*
 * RTKRobotArm.cpp
 *
 *  Created on: Nov 7, 2014
 *      Author: ashukla
 */

#include "RTKRobotArm.h"

RTKRobotArm::RTKRobotArm() : mRobot(NULL), bOrientCtrl(false), bUseNull(false), max_jvel(DEFAULT_MAX_JVEL_DEG), nEndEffectorId(-1){

}

RTKRobotArm::RTKRobotArm(bool borient) : mRobot(NULL), bOrientCtrl(borient), bUseNull(false), max_jvel(DEFAULT_MAX_JVEL_DEG), nEndEffectorId(-1){

}


RTKRobotArm::~RTKRobotArm() {
	delete mRobot;
	// TODO Auto-generated destructor stub
}

bool RTKRobotArm::_initialize(const ros::NodeHandle& n) {
	bool ret=true;

	// path to RTK format data folder. Keep robots here!
	if(!n.getParam("data_path", data_path)) {
		ROS_ERROR("Must provide data_path!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Data Folder: "<<data_path);
	}

	// RTK format robot name
	if(!n.getParam("robot_name", rob_name)) {
		ROS_ERROR("Must provide robot_name!");
		ret = false;
	} else {
		ROS_INFO_STREAM("Robot: "<<rob_name);
	}

	// EE link name in the structure.xml
	if(!n.getParam("ee_link", ee_link)) {
		ROS_ERROR("Must provide ee_link!");
		ret = false;
	} else {
		ROS_INFO_STREAM("EELink: "<<ee_link);
	}

	if(!n.getParam("max_jvel_deg", max_jvel)) {
		max_jvel = DEFAULT_MAX_JVEL_DEG;
	}
	ROS_INFO_STREAM("Max. Joint Vel. (deg): "<<max_jvel);

	return ret;
}

bool RTKRobotArm::initialize(const ros::NodeHandle& nh) {

	if(!mRobot) { // If robot is already initialize, dont make another one
		if(_initialize(nh)) {
			mRobot = new Robot(true);
			FileFinder::AddBasePath(data_path);
			if( ! mRobot->Load(rob_name, "") ) {
				ROS_ERROR_STREAM("Cannot load robot with name "<<rob_name);
				return false;
			}
			numdof = mRobot-> GetDOFCount();

			llim.Resize(numdof); ulim.Resize(numdof);
			null_posture.Resize(numdof);
			bUseNull = false;
			llim = -DEG2RAD(180); ulim = DEG2RAD(180);
			nEndEffectorId = mRobot->GetLinkIndex(ee_link);
			if(nEndEffectorId < 0) {
				ROS_ERROR("Cannot find end effector link!");
				return 1;
			}
			mKinematicChain.SetRobot(mRobot);
			mKinematicChain.Create(0,0,nEndEffectorId);
			mapping = mKinematicChain.GetJointMapping();

			mSensorsGroup.SetSensorsList(mRobot->GetSensors());
			mSensorsGroup.ReadSensors();
			full_robot_state = mSensorsGroup.GetJointPositions();

			mIKSolver.SetSizes(numdof) ;
			if(bOrientCtrl) {
			mIKSolver.AddSolverItem(6);
			} else {
				mIKSolver.AddSolverItem(3);
			}
			mIKSolver.SetVerbose(false);                // No comments
			mIKSolver.SetThresholds(0.0001,0.0001);    // Singularities thresholds
			mIKSolver.Enable(true,0);                   // Enable first solver
			mIKSolver.SetDofsIndices(mapping,0); // Joint maps for first solver
			mJointVelLimits[0].Resize(numdof);
			mJointVelLimits[1].Resize(numdof);
			mJointVelLimits[0] = DEG2RAD(-max_jvel);
			mJointVelLimits[1] = DEG2RAD( max_jvel);
			mIKSolver.SetLimits(mJointVelLimits[0],mJointVelLimits[1]);


			ROS_INFO_STREAM("Robot loaded with "<<mRobot->GetDOFCount()<<" DOF");
		} else {
			ROS_ERROR("Cannot find parameters to load RTK robot");
			return false;
		}
	}
	return true;
}
void RTKRobotArm::setJointLimits(const Eigen::VectorXd& lims) {
	if(lims.size() != numdof) {
		ROS_ERROR("Cannot set limits!");
		return;
	}
	for(int i=0; i<numdof; ++i) {
		llim[i] = -lims[i];
		ulim[i] = lims[i];
	}
}

//TODO: Dont use this!!! Needs fixing
void RTKRobotArm::setNullPosture(const Eigen::VectorXd& n) {
	if(n.size() != numdof) {
		ROS_ERROR("Cannot set null posture!");
		return;
	}
	for(int i=0; i<numdof; ++i) {
		null_posture(i) = n(i);
	}
	// Being super safe until this is fixed
//	bUseNull = true;
}

void RTKRobotArm::setJoints(const Eigen::VectorXd& joints) {
	if(joints.size() != numdof) {
		ROS_WARN_THROTTLE(0.1, "Size of joints not same as DOF");
		return;
	}
	// Update RTK robot
	for(int i=0; i<numdof; ++i) {
			full_robot_state[mapping[i]] = joints[i];
	}
	mSensorsGroup.SetJointPositions(full_robot_state);
	mSensorsGroup.WriteSensors();
	mRobot->UpdateLinks();
	mKinematicChain.Update();
}
void RTKRobotArm::getJacobian(Eigen::MatrixXd& jacobian) {
	if(mRobot) {
		// Collect jacobian from RTK
		Matrix jac = mKinematicChain.GetJacobian();
		if(jacobian.rows() != jac.RowSize() || jacobian.cols() != jac.ColumnSize()) {
			jacobian.resize(jac.RowSize(), jac.ColumnSize());
		}
		// Copy to Eigen format
		copy(jac, jacobian);
	} else {
		ROS_WARN("RTK Robot not initialized");
	}
}
void RTKRobotArm::getEEPose(tf::Pose& pose) {
	if(mRobot) {
		//Collect ee pose from RTK
		MathLib::Vector eeq(4);
		Matrix3 tmp = mRobot->GetReferenceFrame(nEndEffectorId).GetOrient();
		tmp.GetQuaternionRepresentation(eeq);
		MathLib::Vector3 eep = mRobot->GetReferenceFrame(nEndEffectorId).GetOrigin();

		// Convert to tf
		pose.setRotation(tf::Quaternion(eeq[1], eeq[2], eeq[3], eeq[0]));
		pose.setOrigin(tf::Vector3(eep[0], eep[1], eep[2]));
	} else{
		ROS_WARN("RTK Robot not initialized");
	}
}

// Desired cart_vel used to compute the corresponding IK joint_vel
void RTKRobotArm::getIKJointVelocity(const Eigen::VectorXd& cart_vel, Eigen::VectorXd& joint_vel) {
	MathLib::Vector cerr;
	if(bOrientCtrl) {
		cerr.Resize(6);
	} else {
		cerr.Resize(3);
	}
	copy(cart_vel, cerr);

	if(joint_vel.size() != numdof) {
		joint_vel.resize(numdof);
	}
	mIKSolver.SetTarget(cerr);
	mIKSolver.SetJacobian(mKinematicChain.GetJacobian());

	// Try first with all joint weights equal
	MathLib::Vector wts(numdof);
	wts.One();
	mSensorsGroup.ReadSensors();
	MathLib::Vector curr = mSensorsGroup.GetJointPositions();

	if(bUseNull) {
		MathLib::Vector nul(numdof);
		for(int i=0; i<numdof; ++i) {
			nul[i] = null_posture(i) - curr[mapping[i]];
		}
		mIKSolver.SetNullTarget(nul);
	}
	mIKSolver.SetDofsWeights(wts);
	mIKSolver.Solve();

	MathLib::Vector ikout = mIKSolver.GetOutput();
	for(int i=0; i<numdof; ++i) {
		joint_vel[i] = ikout[mapping[i]];
	}

	// Check if any joint is near limit
	bool redo = false;
	for(int i=0; i<numdof; ++i) {
		if( (curr(i)> ulim(i) - DEG2RAD(5) && joint_vel[i] > 0) ||
				(curr(i) < llim(i) + DEG2RAD(5) && joint_vel[i] < 0) ) {
			ROS_WARN_STREAM_THROTTLE(0.5, "Reducing DOF "<<i<<" IK weight");
			redo = true;
			// Set the problematic joint weights to a small value
			wts(i) = 0.1;
		} else {
			wts(i) = 1.0;
		}
	}

	// If any joint is in problem, resolve with new joint weights
	if(redo) {
		mIKSolver.SetDofsWeights(wts);
		mIKSolver.Solve();
		MathLib::Vector ikout = mIKSolver.GetOutput();
		for(int i=0; i<numdof; ++i) {
			joint_vel[i] = ikout[mapping[i]];
		}
	}

}
void RTKRobotArm::copy(const Eigen::VectorXd& from, MathLib::Vector& to) {
	for(int i=0; i<from.size(); ++i) {
        to(i) = from(i);
	}
}
void RTKRobotArm::copy(const MathLib::Vector& from, Eigen::VectorXd& to) {
	for(int i=0; i<from.Size(); ++i) {
        to(i) = from.At(i);
	}
}
void RTKRobotArm::copy(const MathLib::Matrix& from, Eigen::MatrixXd& to) {
	int r = from.RowSize();
	int c = from.ColumnSize();
	for(int i=0; i<r; ++i) {
		for(int j=0; j<c; ++j) {
			to(i,j) = from.At(i,j);
		}
	}
}
void RTKRobotArm::copy(const Eigen::MatrixXd& from, MathLib::Matrix& to) {
	int r = from.rows();
	int c = from.cols();
	for(int i=0; i<r; ++i) {
		for(int j=0; j<c; ++j) {
            to(i,j) = from(i,j);
		}
	}
}
