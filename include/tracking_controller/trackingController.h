/*
	FILE: trackingController.h
	-------------------------------
	function definition of px4 tracking controller
*/
#ifndef TRACKING_CONTROLLER_H
#define TRACKING_CONTROLLER_H
#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>

namespace trackingController{
	class trackingController{
		private:
			ros::NodeHandle nh_;
			ros::Subscriber targetSub_; // subscriber for the tracking target states
			ros::Publisher cmdPub_; // command publisher

		public:
			trackingController(const ros::NodeHandle& nh);
			void registerPub();
			void registerCallback();
	};
}

#endif