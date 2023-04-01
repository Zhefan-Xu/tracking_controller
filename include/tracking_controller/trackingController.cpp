/*
	FILE: trackingController.cpp
	-------------------------------
	function implementation of px4 tracking controller
*/

#include <tracking_controller/trackingController.h>

namespace trackingController{
	trackingController::trackingController(const ros::NodeHandle& nh) : nh_(nh){
		this->registerPub();
		this->registerCallback();
	}

	void trackingController::registerPub(){
		// command publisher
		this->cmdPub_ = this->nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
	}


	void trackingController::registerCallback(){
		// odom subscriber
		this->odomSub_ = this->nh_.subscribe("/mavros/local_position/odom", 1, &trackingController::odomCB, this);
	
		// target setpoint subscriber
		this->targetSub_ = this->nh_.subscribe("/autonomous_flight/target_state", 1, &trackingController::targetCB, this);
	
		// controller publisher timer
		this->cmdTimer_ = this->nh_.createTimer(ros::Duration(0.01), &trackingController::cmdCB, this);
	}


	void trackingController::odomCB(const nav_msgs::OdometryConstPtr& odom){
		this->odom_ = *odom;
	}

	void trackingController::targetCB(const tracking_controller::TargetConstPtr& target){
		this->target_ = *target;
	}

	void trackingController::cmdCB(const ros::TimerEvent&){
		Eigen::Vector4d cmd;
		this->publishCommand(cmd);
	}

	Eigen::Vector3d trackingController::computeDesiredAcc(const Eigen::Vector3d& targetPos, const Eigen::Vector3d& targetVel, const Eigen::Vector3d& targetAcc){
		
	}

	void trackingController::publishCommand(const Eigen::Vector4d& cmd){
		mavros_msgs::AttitudeTarget cmdMsg;
		cmdMsg.header.stamp = ros::Time::now();
		cmdMsg.header.frame_id = "map";
		cmdMsg.body_rate.x = cmd(0);
		cmdMsg.body_rate.y = cmd(1);
		cmdMsg.body_rate.z = cmd(2);
		cmdMsg.thrust = cmd(3);
		cmdMsg.type_mask = cmdMsg.IGNORE_ATTITUDE; 
		this->cmdPub_.publish(cmdMsg);
	}
}