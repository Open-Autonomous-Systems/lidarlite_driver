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
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. *
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
    // close i2c port and delete obj
    if(lidarLite_->kI2CFileDescriptor > 0) lidarLite_->closeLidarLite();
    lidarLite_.reset();
    lidarLiteFIR_.reset();
}

void lldriver_ns::Lidarlite_driver::onInit()
{
    nodeHandle_ = getNodeHandle();
    nodeHandlePvt_ = getPrivateNodeHandle();
    ROS_ASSERT(readParams());
    register2Ros();
    lidarLite_ = std::make_shared<LidarLite>(i2cBus_);
    lidarLiteFIR_ =std::make_shared<FirFilter>(filterCoefficientVec_);
    //
    if(!initializeSensor()) ros::requestShutdown();
    //
    double measurementPeriod = 1.0/lidarRate;
    timer_ = nodeHandle_.createTimer( \
        ros::Duration(measurementPeriod),\
        boost::bind(&lldriver_ns::Lidarlite_driver::getMeasurement,\
            this,_1));
    NODELET_INFO("LidarLite_driver:: onInit success");
}

void lldriver_ns::Lidarlite_driver::getMeasurement(const ros::TimerEvent& e)
{
    try
    {
        if (!lidarLite_) throw std::invalid_argument("lidarlite_ is null_Ptr");
        if (lidarLite_->error >= 0)
        {
            int distance = lidarLite_->getDistance();
            ros::Time msgTime = ros::Time::now();
            if (distance < 0)
            {
                int llError ;
                llError = lidarLite_->getError();
                std::string errStr = "distance_error "+ std::to_string(llError);
                throw std::invalid_argument(errStr);
            }
            float rawData = float(distance)/100.0; //converting cm to M
            float filteredData;
            filteredData = lidarLiteFIR_->UpdateFilterAndGetOutput(rawData);

            // publish
            sensor_msgs::RangePtr rangeMsgPtr;
            rangeMsgPtr.reset(new sensor_msgs::Range());
            rangeMsgPtr->header.stamp = msgTime;
            rangeMsgPtr->header.frame_id = sensorFrameId_;
            rangeMsgPtr->radiation_type = sensor_msgs::Range::INFRARED;
            rangeMsgPtr->field_of_view = 0.008; //8 milliRadians
            rangeMsgPtr->min_range = 0.010; //approx 1 cm
            rangeMsgPtr->max_range = 40.0; //max distance of 40.0 M
            rangeMsgPtr->range = rawData;
            rangePub_.publish(rangeMsgPtr);
            // publish filtered data
            sensor_msgs::RangePtr filteredRangeMsgPtr;
            filteredRangeMsgPtr.reset(new sensor_msgs::Range());
            filteredRangeMsgPtr->header.stamp = msgTime;
            filteredRangeMsgPtr->header.frame_id = sensorFrameId_;
            filteredRangeMsgPtr->radiation_type = sensor_msgs::Range::INFRARED;
            filteredRangeMsgPtr->field_of_view = 0.008; //8 milliRadians
            filteredRangeMsgPtr->min_range = 0.010; //approx 1 cm
            filteredRangeMsgPtr->max_range = 40.0; //max distance of 40.0 M
            filteredRangeMsgPtr->range = filteredData;
            filteredRangePub_.publish(filteredRangeMsgPtr);
        }
    }
    catch(std::invalid_argument ex)
    {
        NODELET_ERROR_THROTTLE(1.0,"lidarLite::getMeasurement:: exception "
                                   "%s", ex.what());
        //lidarLite_->closeLidarLite();
        //initializeSensor();
    }
    catch(...)
    {
        NODELET_ERROR_THROTTLE(1.0,"lidarLite::getMeasurement:: exception ");
    }
}

void lldriver_ns::Lidarlite_driver::register2Ros()
{
    //publishers
    std::string sensor_suffix;
    if(sensor_location_.compare("")!=0) sensor_suffix= "/" + sensor_location_;
    //
    rangePub_ = nodeHandle_.advertise<sensor_msgs::Range>("lidarlite/range"
                                                          +sensor_suffix + "_raw", 5);
    filteredRangePub_ = nodeHandle_.advertise<sensor_msgs::Range>
            ("lidarlite/range" +sensor_suffix+ "_filtered", 5);
    //service server
    //zeroingRangeSrv_ = nodeHandle_.advertiseService(robot_ns_+"/lidarlite/setzero_position",
    //    &Lidarlite_driver::zeroing_cb, this);
}

bool lldriver_ns::Lidarlite_driver::readParams() {
    //vars
    nodeHandlePvt_.param<std::string>("robot_ns", robot_ns_, "");
    nodeHandlePvt_.param<std::string>("tf_prefix", tf_prefix_, "");
    nodeHandlePvt_.param<std::string>("sensor_location", sensor_location_, "");
    if(sensor_location_.compare("")!=0)
        sensorFrameId_ = tf_prefix_ + "/lidarlite" + "_" + sensor_location_;
    else
        sensorFrameId_ = tf_prefix_ + "/lidarlite";
    //
    nodeHandlePvt_.param<double>("lidar_rate", lidarRate, 50.0);
    nodeHandlePvt_.param<int>("i2cBus", i2cBus_, 0);
    nodeHandlePvt_.getParam("filter_coefficients", filterCoefficientVec_);
    return true;
}

bool lldriver_ns::Lidarlite_driver::initializeSensor() {
    bool i2cStatus = false;
    try {
        // check
        if(!lidarLite_) throw std::invalid_argument("lidarlite_ is null_Ptr");
        //
        i2cStatus = lidarLite_->openLidarLite();
        if(i2cStatus) {
            int hardwareVersion = lidarLite_->getHardwareVersion();
            int softwareVersion = lidarLite_->getSoftwareVersion();
            NODELET_INFO("Hardware Version: %d\n", hardwareVersion);
            NODELET_INFO("Software Version: %d\n", softwareVersion);
        }
        else{
            std::string errString = "errorCode "+ std::to_string                                                                                              (lidarLite_->error);
            throw std::invalid_argument(errString);
        }
    }
    catch (std::invalid_argument ex)
    {
        lidarLite_->closeLidarLite();
        NODELET_ERROR("LidarLite_driver::onInit:: error opening i2c address, "
                      "exception %s", ex.what());
    }
    return i2cStatus;
}
