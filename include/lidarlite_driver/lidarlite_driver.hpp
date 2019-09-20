/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * MIT License                                                             *
 * @author      Mithun Diddi <diddi.m@husky.neu.edu>                       *
 * @maintainer  Mithun Diddi <diddi.m@husky.neu.edu>                       *
 * @website    https://web.northeastern.edu/fieldrobotics                  *
 * @copyright (c) 2019, Northeastern University Field Robotics Lab(NEUFRL),*
 *             All rights reserved.                                        *
 *                                                                         *
 * Permission is hereby granted, free of charge, to any person obtaining   *
 * copy of this software and associated documentation files                *
 * (the "Software"), to deal in the Software without restriction,          *
 * including *without limitation the rights to use, copy, modify, merge,   *
 * publish, distribute, sublicense, and/or sell copies of the Software,    *
 * and to permit persons to whom the Software is furnished to do so,       *
 * subject to the following conditions:                                    *
 *                                                                         *
 * The above copyright notice and this permission notice shall be included *
 * in all copies or substantial portions of the Software.                  *
 *                                                                         *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS *
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF              *
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  *
 *                                                                         *
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR        *
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF          *
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH *
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.              *
 * This code is wrapped using JHLidarLite github repository Licensed Under *
 * MIT License, located at include/JHLidarLite/ locally. the original copy *
 * of license can be obtained \                                            *
 * from https://github.com/jetsonhacks/JHLidarLite.git                     *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
 
#ifndef LIDARLITE_DRIVER_H  
#define LIDARLITE_DRIVER_H 

// System includes
#include "unistd.h"
#include <iostream>
#include <string>
// boost includes
#include <boost/thread.hpp>
// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
// nodelet includes
#include "nodelet/nodelet.h"
#include <nodelet/loader.h>
#include "pluginlib/class_list_macros.h"
//#include <tf/tf.h>
#include <sensor_msgs/Range.h>

//#include <std_srvs/Trigger.h>

#include "JHLidarLite_V2/src/lidarlite.h"

namespace lldriver_ns
{
	class Lidarlite_driver: public nodelet::Nodelet
	{
		public:
			Lidarlite_driver();
			virtual ~Lidarlite_driver();

			void measurementloop();
		private:
			virtual void onInit();
			ros::NodeHandle nodeHandle_,nodeHandlePvt_;
		   	ros::Publisher rangePub_;
			//ros::ServiceServer zeroingRangeSrv_;
			std::string robot_ns_, tf_prefix_, sensor_location_;
			double lidar_rate_;
			bool readparams();
			void ros_reg_topics();
    	    LidarLite *lidarLite_ = new LidarLite();
        	std::unique_ptr<boost::thread> MeasurementThread_;
    };	//Lidarlite_driver_class
} // lldriver_ns
#endif // LIDARLITE_DRIVER_H
