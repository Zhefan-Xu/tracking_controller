/*
	FILE: trackingController.h
	-------------------------------
	function definition of px4 tracking controller
*/
#ifndef TRACKING_CONTROLLER_H
#define TRACKING_CONTROLLER_H
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <tracking_controller/Target.h>
#include <tracking_controller/utils.h>

using std::cout; using std::endl;
namespace controller{
	class trackingController{
		private:
			ros::NodeHandle nh_;
			ros::Subscriber odomSub_; // Subscribe to odometry
			ros::Subscriber targetSub_; // subscriber for the tracking target states
			ros::Publisher cmdPub_; // command publisher
			ros::Timer cmdTimer_; // command timer

			// parameters
			Eigen::Vector3d pPos_;
			Eigen::Vector3d pVel_;
			double attitudeControlTau_;
			double hoverThrottle_;

			// controller data
			bool odomReceived_ = false;
			bool targetReceived_ = false;
			nav_msgs::Odometry odom_;
			tracking_controller::Target target_;


		public:
			trackingController(const ros::NodeHandle& nh);
			void initParam();
			void registerPub();
			void registerCallback();

			// callback functions
			void odomCB(const nav_msgs::OdometryConstPtr& odom);
			void targetCB(const tracking_controller::TargetConstPtr& target);
			void cmdCB(const ros::TimerEvent&);

			void publishCommand(const Eigen::Vector4d &cmd);
			void computeAttitudeAndAccRef(Eigen::Vector4d& attitudeRefQuat, Eigen::Vector3d& accRef);
			void computeBodyRate(const Eigen::Vector4d& attitudeRefQuat, const Eigen::Vector3d& accRef, Eigen::Vector4d& cmd);


	};
}

#endif