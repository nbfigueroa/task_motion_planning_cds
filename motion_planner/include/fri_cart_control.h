/*
 * Copyright (C) 2014 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author:  Lucia Pais
 * email:   lucia.pais@epf.ch
 * website: lasa.epfl.ch
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

#ifndef fri_cart_control_H_
#define fri_cart_control_H_

#include "RobotLib/RobotInterface.h"
#include "KUKARobotModel/LWRRobot.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>


// Translational Stiffness values
#define TRANS_STIFF_X	1200
#define TRANS_STIFF_Y	1200
#define TRANS_STIFF_Z	1200

// Rotational Stiffness values
#define ROT_STIFF_X	300
#define ROT_STIFF_Y	300
#define ROT_STIFF_Z	300


class fri_cart_control : public RobotInterface
{
protected:
    LWRRobot* mLWRRobot;
    Robot::ControlMode ctrlmode;

    Vector3		vDesiredStiffness;	// Translational stiffness
    Vector3		vStiffnessModulation;	// Stiffness Modulation (lambda factor)
    Vector 		nDesiredForce;

	void		initializeRobot();	// initializes Robot and Control Mode
	void		initializeTaskVariables();

// ===================================================
// ========  Robot Position Variables ================
// ===================================================
    int nEndEffectorId;
	void 		getCrtEEPose(Vector3& position, Matrix3& orientation);		// Returns current robot Cartesian coordinates
	Matrix4		getCrtEEFullFrame();

	bool    	bSync;				// True if the current robot joint state should be sinked in CDDynamics
	bool		bSyncCart;			// True if the current robot cartesian state should be sinked in CDDynamics

	void 		synchronizeJointMotion();
	void		synchronizeCartMotion();

// ===================================================
// ==============    Various Utilities  ==============
// ===================================================

	// Various Things
	void			checkCtrlMode();
	// Drawing trajectory
	double 			timeIDX;

public:
            fri_cart_control();
    virtual ~fri_cart_control();

    virtual Status              RobotInit();
    virtual Status              RobotFree();

    virtual Status              RobotStart();
    virtual Status              RobotStop();

    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();
};



#endif
