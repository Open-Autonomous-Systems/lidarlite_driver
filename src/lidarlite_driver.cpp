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
 
#include "lidarlite_driver/lidarlite_driver.hpp"

PLUGINLIB_EXPORT_CLASS(lldriver_ns::Lidarlite_driver, nodelet::Nodelet)

lldriver_ns::Lidarlite_driver::Lidarlite_driver()
{
    // nodelet set me free!!
}

lldriver_ns::Lidarlite_driver::~Lidarlite_driver()
{
    // Interrupt Measurment thread if not interrupted already by sigint
    // wait for it to join
    if (MeasurementThread_)
    {
        MeasurementThread_->interrupt();
        MeasurementThread_->join();
    }
    MeasurementThread_.reset();
    // close i2c port and delete obj
    if(lidarLite_->kI2CFileDescriptor > 0) lidarLite_->closeLidarLite();
    delete lidarLite_;
}

void lldriver_ns::Lidarlite_driver::onInit()
{
    nodeHandle_ = getNodeHandle();
    nodeHandlePvt_ = getPrivateNodeHandle();
    ROS_ASSERT(readparams());
    ros_reg_topics();
    LidarLite *lidarLite_;
    lidarLite_ = new LidarLite(i2cBus_);
    MeasurementThread_.reset(new boost::thread(boost::bind(&lldriver_ns::Lidarlite_driver::measurementloop, this)));
    NODELET_INFO("Lidarlite_driver onInit success");
}

void lldriver_ns::Lidarlite_driver::measurementloop()
{
    try
    {
        int err = lidarLite_->openLidarLite();
        if (err < 0)
        {
            NODELET_FATAL("Error: %d", lidarLite_->error);
        }
        else
        {
            int hardwareVersion = lidarLite_->getHardwareVersion() ;
            int softwareVersion = lidarLite_->getSoftwareVersion() ;
            NODELET_INFO("Hardware Version: %d\n",hardwareVersion) ;
            NODELET_INFO("Software Version: %d\n",softwareVersion) ;
        }

        ros::Rate loop_rate(lidar_rate_);
        
        std::string sensorFrameId;
        if(sensor_location_.compare("")!=0)
            sensorFrameId = tf_prefix_ + "/lidarlite" + "_" + sensor_location_;
        else
            sensorFrameId = tf_prefix_ + "/lidarlite";

        while(ros::ok() && lidarLite_->error >= 0)
        {
            loop_rate.sleep();
            int distance = lidarLite_->getDistance();

            if (distance < 0)
            {
                int llError ;
                llError = lidarLite_->getError();
                NODELET_ERROR_THROTTLE(1,"Lidar-Lite error: %d\n",llError);
                continue;
            }

            sensor_msgs::RangePtr rangeMsgPtr;
            rangeMsgPtr.reset(new sensor_msgs::Range());
            rangeMsgPtr->header.stamp = ros::Time::now();
            rangeMsgPtr->header.frame_id = sensorFrameId;
            rangeMsgPtr->radiation_type = sensor_msgs::Range::INFRARED;
            rangeMsgPtr->field_of_view = 0.008; //8 milliRadians
            rangeMsgPtr->min_range = 0.010; //approx 1 cm
            rangeMsgPtr->max_range = 40.0; //max distance of 40.0 M
            rangeMsgPtr->range = float(distance)/100.0; //converting cm to M
            rangePub_.publish(rangeMsgPtr);
            ros::spinOnce();
        }
    }
    catch(...)
    {
        NODELET_ERROR("ran into exception in measurementloop() ");
        lidarLite_->closeLidarLite();
    }
}
void lldriver_ns::Lidarlite_driver::ros_reg_topics()
{
    //publishers
    std::string sensor_suffix;
    if(sensor_location_.compare("")!=0) sensor_suffix= "/" + sensor_location_;

    rangePub_ = nodeHandle_.advertise<sensor_msgs::Range>("lidarlite/range" +sensor_suffix, 5);
    //service server
    //zeroingRangeSrv_ = nodeHandle_.advertiseService(robot_ns_+"/lidarlite/setzero_position",
    //    &Lidarlite_driver::zeroing_cb, this);
}

bool lldriver_ns::Lidarlite_driver::readparams()
{
    //vars
    nodeHandlePvt_.param<std::string>("robot_ns", robot_ns_,"");
    nodeHandlePvt_.param<double>("lidar_rate", lidar_rate_, 50.0);
    nodeHandlePvt_.param<int>("i2cBus", i2cBus_, 0);
    nodeHandlePvt_.param<std::string>("tf_prefix", tf_prefix_, "");
    nodeHandlePvt_.param<std::string>("sensor_location", sensor_location_, "");

    return true;
}
