/*
 * Copyright (C) 2014 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 *
 * test_force.cpp
 *
 * Created on : Nov 15, 2014
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
#include <signal.h>
#define EE_CMD_FT_TOPIC   "/cart_to_joint/des_ee_ft"
#define DEFAULT_FX        0
#define DEFAULT_FY        0
#define DEFAULT_FZ        -5

geometry_msgs::WrenchStamped wr;
ros::Publisher pub;
bool shut;
void handler(int signal) {
	wr.wrench.force.x = 0;
	wr.wrench.force.y = 0;
	wr.wrench.force.z = 0;

	wr.wrench.torque.x = 0;
	wr.wrench.torque.y = 0;
	wr.wrench.torque.z = 0;

	pub.publish(wr);
	shut = true;

}

int main(int argc, char** argv) {
	shut = false;
	signal(SIGINT, handler);
	ros::init(argc, argv, "test_traj", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;
	ros::NodeHandle _nh("~");

	// Parameters to set how much force/torque to apply. May come from commandline or launch files
	double fx=0, fy=0, fz=0, tx=0, ty=0, tz=0;
	bool sinusoid = false;
	double freq=10*M_PI/180.0;

	_nh.getParam("fx", fx);
	_nh.getParam("fy", fy);
	_nh.getParam("fz", fz);

	_nh.getParam("tx", tx);
	_nh.getParam("ty", ty);
	_nh.getParam("tz", tz);

	// Generate a sinusoidal force/torque by using the above numbers as amplitudes
	_nh.getParam("sine", sinusoid);
	// Frequency for the sinusoidal force
	_nh.getParam("freq", freq);

	double rate = 100;
	ros::Rate thread_rate(rate);
	pub = nh.advertise<geometry_msgs::WrenchStamped>(EE_CMD_FT_TOPIC, 1);
	ros::Rate r(1); r.sleep();
	ROS_INFO_STREAM("Force: "<<fx<<","<<fy<<","<<fz);
	ROS_INFO_STREAM("Torque: "<<tx<<","<<ty<<","<<tz);

	wr.wrench.force.x = fx;
	wr.wrench.force.y = fy;
	wr.wrench.force.z = fz;

	wr.wrench.torque.x = tx;
	wr.wrench.torque.y = ty;
	wr.wrench.torque.z = tz;

	pub.publish(wr);
	ROS_INFO("Forces sent");
	ROS_INFO("Press Ctrl-C to send zero forces and quit");

	double t=0;
	// Keep publishing forces if sinusoid is switched on.
	while(true) {

		thread_rate.sleep();
		if(shut) {
			nh.shutdown();
			return 0;
		}
		if(sinusoid) {
			double _fx = fx*(1+sin(freq*t))/2.0;
			double _fy = fy*(1+sin(freq*t))/2.0;
			double _fz = fz*(1+sin(freq*t))/2.0;
			t += 1.0/rate;
			wr.wrench.force.x = _fx;
			wr.wrench.force.y = _fy;
			wr.wrench.force.z = _fz;
			if(!shut) {
				pub.publish(wr);
			}
		}

	}

	return 0;
}
