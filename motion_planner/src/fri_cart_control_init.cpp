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

#include "fri_cart_control.h"

void fri_cart_control::initializeRobot(){

	// ===================================================
	// ========  Initialize Robot and CTRL mode ==========
	// ===================================================
	mLWRRobot = (LWRRobot*) mRobot;
	ctrlmode = Robot::CTRLMODE_POSITION;
	mSensorsGroup.SetSensorsList(mRobot->GetSensors());
	mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());

	nEndEffectorId = mRobot->GetLinkIndex("TOOL");
	cout << "The end effector id is " << nEndEffectorId << endl;
	if (nEndEffectorId == -1)
		ROS_ERROR_STREAM("ERROR: End effector not found");
}


void fri_cart_control::initializeTaskVariables(){

	// ===================================================
	// ========  Initialize Stuff for grating  ===========
	// ===================================================

//	nDesiredForce.Resize(1); nDesiredForce.Zero();

	bSync 			= false;
	bSyncCart 		= false;

	// Variables used for drawing the trajectory in the simulator
	timeIDX = 0;
/*	mRobTraj.Resize(12000, 3);
	Vector3 tpos; Matrix3 tmat; getCrtEEPose(tpos, tmat);
	mRobTraj(0,0) = tpos(0); mRobTraj(0,1) = tpos(1);  mRobTraj(0,2) = tpos(2);

*/
}


}
