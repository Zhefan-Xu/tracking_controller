/*
	FILE: trackingController.h
	-------------------------------
	function definition of px4 tracking controller
*/
#ifndef TRACKING_CONTROLLER_H
#define TRACKING_CONTROLLER_H
#include <ros/ros.h>

namespace trackingController{
	class trackingController{
		private:
			ros::NodeHandle nh_;

		public:
			trackingController(const ros::NodeHandle& nh);
	};
}

#endif