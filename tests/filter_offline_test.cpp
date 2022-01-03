// System includes
#include "unistd.h"
#include <iostream>
#include <string>
// boost includes
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
//
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
//
#include <sensor_msgs/Range.h>
//
#include "fir_filter/fir_filter.h"
#include "median_filter/median_filter.h"

class Filter_Test
{
public:
    Filter_Test(ros::NodeHandle& nodeHandle, std::string& inBagPath, std::string& outBagName, std::vector<float>& filterCoefficientVec)
    {
        nodeHandle_ = nodeHandle;
        std::cout<<"writing out bag at "<<outBagName<<std::endl;
        outBag_.open(outBagName, rosbag::bagmode::Write);
        // input bag
        inBag_.open(inBagPath, rosbag::bagmode::Read);
        //
        filterCoefficientVec_ = filterCoefficientVec;
        lidarLiteFIR_ = std::make_shared<FirFilter>(filterCoefficientVec_);
        lidarliteMedian_ = std::make_shared<MedianFilter<float, 5>>();
        lidarLiteMedianFIR_ =std::make_shared<FirFilter>(filterCoefficientVec_);
        //
        lastMeasurement_ = 0.0;
    };
    ~Filter_Test()
    {
        inBag_.close();
        outBag_.close();
        lidarLiteFIR_.reset();
        lidarLiteMedianFIR_.reset();
        lidarliteMedian_.reset();
    };
    void loadBag()
    {
        std::vector<std::string> topics;
        std::string rangeTopic = "/uas4/lidarlite/range";
        topics.push_back(std::string(rangeTopic));
        rosbag::View view(inBag_, rosbag::TopicQuery(topics));
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            if (m.getTopic() == rangeTopic || ("/" + m.getTopic() == rangeTopic))
            {
                sensor_msgs::Range::ConstPtr rangeMsgPtr = m.instantiate<sensor_msgs::Range>();
                if (rangeMsgPtr != NULL)
                {
                    float rawData = rangeMsgPtr->range;
                    //ROS_INFO("data is %f", rawData);
                    // perform data validity check
                    rawData = (std::fabs(rawData) > 1e-1) ? rawData : 0.0;
                    //
                    float medianData = lidarliteMedian_->Insert(rawData);
                    sensor_msgs::Range medianFilteredMsg;
                    medianFilteredMsg.header = rangeMsgPtr->header;
                    medianFilteredMsg.range =medianData;
                    outBag_.write("/lidarlite/range/down_median_preprocessed", rangeMsgPtr->header.stamp, medianFilteredMsg);
                    //
                    rawData = (std::fabs(rawData - lastMeasurement_) <= 2.0) ?
                              rawData : lastMeasurement_;
                    lastMeasurement_ = rawData;
                    sensor_msgs::Range preFilteredMsg;
                    preFilteredMsg.header = rangeMsgPtr->header;
                    preFilteredMsg.range = rawData;
                    outBag_.write("/lidarlite/range/down_vel_preprocessed", rangeMsgPtr->header.stamp, preFilteredMsg);
                    //
                    float filteredData;
                    filteredData = lidarLiteFIR_->UpdateFilterAndGetOutput(rawData);
                    //ROS_INFO("filtered data is %f", filteredData);
                    sensor_msgs::Range filteredRangeMsg;
                    filteredRangeMsg.header = rangeMsgPtr->header;
                    filteredRangeMsg.range = filteredData;
                    outBag_.write("/lidarlite/range/down_vel_fir_filtered", rangeMsgPtr->header.stamp, filteredRangeMsg);
                    //
                    float medianFirFilteredData;
                    medianFirFilteredData = lidarLiteMedianFIR_->UpdateFilterAndGetOutput(medianData);
                    sensor_msgs::Range medianFirRangeMsg;
                    medianFirRangeMsg.header = rangeMsgPtr->header;
                    medianFirRangeMsg.range = medianFirFilteredData;
                    outBag_.write("/lidarlite/range/down_median_fir_filtered", rangeMsgPtr->header.stamp, medianFirRangeMsg);
                }
            }
        };

    };
private:
    ros::NodeHandle nodeHandle_,nodeHandlePvt_;
    std::shared_ptr<FirFilter> lidarLiteFIR_;
    std::shared_ptr<FirFilter> lidarLiteMedianFIR_;
    std::shared_ptr<MedianFilter<float, 5>> lidarliteMedian_;
    std::vector<float> filterCoefficientVec_;
    float lastMeasurement_;
    rosbag::Bag inBag_, outBag_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv,"filter_test_node");
    ros::NodeHandle nodeHandle("~");
    try
    {
        if (argc <  2)
        {
            std::cout<<"Usage 'rosrun lidarlite_driver filter_test_node <path to"
                       "inputrosbag> <optional_arg output_bag_full_path>'"<<std::endl;
            ros::shutdown();
        }
        // inBag
        std::string inBagPath = argv[1];
        // outBag get name and open it in write mode
        std::string outBagName;
        if (argc ==3)
        {
            outBagName = argv[2];
            // verify if this fullPathName has .bag extension
            // and this program has permissions to write in that location
        }
        else
        {
            std::size_t found = inBagPath.find(".bag");
            outBagName = inBagPath.substr(0,found)+"_filtered.bag";
        }
        const float coefficientArr[] = {
                -0.0001125561527973,-0.0003194338924405,-0.0006701416261524,-0.001176989637092,
                -0.001822088511133,-0.002546941038084,-0.003245508570341,-0.003762789930982,
                -0.003900457672626,-0.003430261024855,-0.002114819279475,0.0002657602896762,
                0.003884119193443, 0.008836268133424,  0.01511654306001,  0.02260073570253,
                0.03104055429452,  0.04007137065902,  0.04923363166479,  0.05800656177229,
                0.06585110316987,  0.07225767727155,  0.07679351626891,  0.07914414585594,
                0.07914414585594,  0.07679351626891,  0.07225767727155,  0.06585110316987,
                0.05800656177229,  0.04923363166479,  0.04007137065902,  0.03104055429452,
                0.02260073570253,  0.01511654306001, 0.008836268133424, 0.003884119193443,
                0.0002657602896762,-0.002114819279475,-0.003430261024855,-0.003900457672626,
                -0.003762789930982,-0.003245508570341,-0.002546941038084,-0.001822088511133,
                -0.001176989637092,-0.0006701416261524,-0.0003194338924405,-0.0001125561527973
        };
        std::vector<float> filterCoefficientVec(std::begin(coefficientArr), std::end(coefficientArr));
        Filter_Test filterObj(nodeHandle, inBagPath, outBagName, filterCoefficientVec);
        filterObj.loadBag();
        // close bags
        std::cout<<"Finished filtering"<<std::endl;
    }
    catch(const ros::Exception& re)
    {
        std::cout<<re.what()<<std::endl;
    }
    catch(...)
    {
        std::cout<<"error \n";
    }
}