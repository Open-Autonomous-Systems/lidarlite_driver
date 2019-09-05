/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * MIT License                                                             *
 * @author      Mithun Diddi <diddi.m@husky.neu.edu>                       *
 * @maintainer  Mithun Diddi <diddi.m@husky.neu.edu>                       *
 * @website    https://web.northeastern.edu/fieldrobotics                  *
 * @copyright (c) 2018, Northeastern University Field Robotics Lab(NEUFRL),*
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
 
#include "lidarlite_driver/lidarlite_driver.hpp"

lldriver_ns::Lidarlite_driver::Lidarlite_driver(ros::NodeHandle& nodeHandle) :nodeHandle_(nodeHandle)
{
    ROS_ASSERT(readparams());
    ros_reg_topics();
    ROS_DEBUG("Lidarlite_driver constructor success");
}                 
lldriver_ns::Lidarlite_driver::~Lidarlite_driver()
{

}
void lldriver_ns::Lidarlite_driver::runloop()
{
    try
    {
        LidarLite *lidarLite = new LidarLite() ;
        int err = lidarLite->openLidarLite();
        if (err < 0)
        {
            printf("Error: %d", lidarLite->error);
        }
        else
        {
            int hardwareVersion = lidarLite->getHardwareVersion() ;
            int softwareVersion = lidarLite->getSoftwareVersion() ;
            printf("Hardware Version: %d\n",hardwareVersion) ;
            printf("Software Version: %d\n",softwareVersion) ;
        }
    }
    catch(...)
    {
        std::cout<<"error in"<<std::endl;
    }
    
    ros::Rate loop_rate(lidar_rate_);
    while(ros::ok())
    {
        loop_rate.sleep();
        
		sensor_msgs::Range RangeMsg;
        RangeMsg.header.stamp = ros::Time::now();
        RangeMsg.range = current_range_;
    
        ros::spinOnce();
    }
}
void lldriver_ns::Lidarlite_driver::ros_reg_topics()
{
    //publishers
    rangePub_ = nodeHandle_.advertise<sensor_msgs::Range>(robot_ns_+"/lidarlite/range", 10);
    //service server
    //zeroingRangeSrv_ = nodeHandle_.advertiseService(robot_ns_+"/lidarlite/setzero_position",
    //    &Lidarlite_driver::zeroing_cb, this);
}

bool lldriver_ns::Lidarlite_driver::readparams()
{
    //vars
    if(!nodeHandle_.getParam("robot_ns", robot_ns_)) return false;
    if(!nodeHandle_.getParam("lidar_rate", lidar_rate_)) return false;
    return true;
}
