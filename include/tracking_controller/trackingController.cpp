/*
	FILE: trackingController.cpp
	-------------------------------
	function implementation of px4 tracking controller
*/

#include <tracking_controller/trackingController.h>

namespace controller{
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
		this->odomReceived_ = true;
	}

	void trackingController::targetCB(const tracking_controller::TargetConstPtr& target){
		this->target_ = *target;
		this->targetReceived_ = true;
	}

	void trackingController::cmdCB(const ros::TimerEvent&){
		if (not this->odomReceived_ or not this->targetReceived_){return;}
		Eigen::Vector4d cmd;

		// 1. Find target reference attitude from the desired acceleration
		Eigen::Vector4d attitudeRefQuat = this->computeAttitudeRef();


		// 2. Compute the body rate from the reference attitude


		this->publishCommand(cmd);
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


	Eigen::Vector4d trackingController::computeAttitudeRef(){
		// Find the reference acceleration for motors, then convert the acceleration into attitude

		/* There are four components of reference acceleration:
			1. target acceleration (from setpoint)
			2. acceleration from feedback of position and velocity (P control)
			3. air drag (not consider this now)
			4. gravity
		*/

		// 1. target acceleration
		Eigen::Vector3d accTarget (this->target_.acceleration.x, this->target_.acceleration.y, this->target_.acceleration.z);


		// 2. position & velocity feedback control
		Eigen::Vector3d currPos (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d currVel (this->odom_.twist.twist.linear.x, this->odom_.twist.twist.linear.y, this->odom_.twist.twist.linear.z);
		Eigen::Vector3d targetPos (this->target_.position.x, this->target_.position.y, this->target_.position.z);
		Eigen::Vector3d targetVel (this->target_.velocity.x, this->target_.velocity.y, this->target_.velocity.z);
		Eigen::Vector3d accFeedback = this->pPos_ * (targetPos - currPos) + this->pVel_ * (targetVel - currVel);


		// 3. air drag
		Eigen::Vector3d accAirdrag (0.0, 0.0, 0.0);

		// 4. gravity
		Eigen::Vector3d gravity {0.0, 0.0, -9.8};

		// Final reference acceleration for motors
		Eigen::Vector3d accRef = accTarget + accFeedback - accAirdrag - gravity;


		// Convert the reference acceleration into the reference attitude
		double currYaw = controller::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		Eigen::Vector3d currDirection (cos(currYaw), sin(currYaw), 0.0);
		Eigen::Vector3d zDirection = accRef/accRef.norm();
		Eigen::Vector3d yDirection = zDirection.cross(currDirection)/(zDirection.cross(currDirection)).norm();
		Eigen::Vector3d xDirection = zDirection.cross(yDirection)/(zDirection.cross(yDirection)).norm();

		// with three axis vector, we can construct the rotation matrix
		Eigen::Matrix3d attitudeRot;
		attitudeRot << xDirection(0), yDirection(0), zDirection(0),
					   xDirection(1), yDirection(1), zDirection(1),
					   xDirection(2), yDirection(2), zDirection(2);
		Eigen::Vector4d attitudeQuat = controller::rot2Quaternion(attitudeRot);
		return attitudeQuat;
	}
}