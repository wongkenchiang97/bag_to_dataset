#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <ros/ros.h>
#include "boost/filesystem.hpp"
#include <rosbag/bag.h>
#include <boost/foreach.hpp>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#define foreach BOOST_FOREACH

using namespace boost::filesystem;

int main(int argc,char** argv){
    ros::init(argc,argv,"pointcloud_to_pcd_node");

    /*Get parameters from ROS*/
    ros::NodeHandle nh,pnh("~");
    std::string dataset_dir,bag_dir,topic;
    double dt;
    pnh.getParam("dataset_dir",dataset_dir);
    pnh.getParam("bag_dir",bag_dir);
    pnh.getParam("topic",topic);
    pnh.getParam("dt",dt);

    /*Get data from bag*/
    std::vector<std::string> topics;
    topics.push_back(topic);
    rosbag::Bag bag;
    bag.open(bag_dir, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    ros::Time latest_stamp;
    bool train = true;
    int64_t seq_train = 0;
    int64_t seq_valid = 0;
    ROS_INFO("saving pcd...");
    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::PointCloud2ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (msg != NULL)
        {
            /*Save train data*/
            if(train && msg->header.stamp-latest_stamp>=ros::Duration(dt))
            {
                latest_stamp = msg->header.stamp;
                train = false;
                pcl::PointCloud<pcl::PointXYZ> cloud;
                pcl::fromROSMsg(*msg,cloud);
                pcl::Indices idx;
                pcl::removeNaNFromPointCloud(cloud,cloud,idx);
                pcl::PointCloud<pcl::PointXYZ> cloud_dense(cloud,idx);
                if(!cloud_dense.empty())
                {
                    auto ret = pcl::io::savePCDFileASCII ((dataset_dir+"/train/data/"+std::to_string(seq_train)+".pcd"), cloud_dense);
                    if(ret<0){
                        ROS_WARN("failed to save train data.");
                    }else{
                        ROS_INFO("saved train data to %s.",(dataset_dir+"/train/data/"+std::to_string(seq_train)+".pcd").c_str());
                        seq_train++;
                    }
                }else{
                    ROS_WARN("train data[%ld]: is empty.",seq_train);
                }
            }
            
            /*Save valid data*/
            if(!train && msg->header.stamp-latest_stamp>=ros::Duration(0.5*dt))
            {
                train = true;
                pcl::PointCloud<pcl::PointXYZ> cloud;
                pcl::fromROSMsg(*msg,cloud);
                pcl::Indices idx;
                pcl::removeNaNFromPointCloud(cloud,cloud,idx);
                pcl::PointCloud<pcl::PointXYZ> cloud_dense(cloud,idx);
                if(!cloud_dense.empty())
                {
                    auto ret = pcl::io::savePCDFileASCII ((dataset_dir+"/valid/data/"+std::to_string(seq_valid)+".pcd"), cloud_dense);
                    if(ret<0){
                        ROS_WARN("failed to save valid data.");
                    }else{
                        ROS_INFO("saved valid data to %s.",(dataset_dir+"/valid/data/"+std::to_string(seq_valid)+".pcd").c_str());
                        seq_valid++;
                    }
                }else{
                    ROS_WARN("valid data[%ld]: is empty.",seq_valid);
                }
            }
        }
    }

    bag.close();
    ROS_INFO("bag to pcd done.");

    std::cout<<"dataset_dir: "<<dataset_dir<<std::endl;
    std::cout<<"bag_dir: "<<bag_dir<<std::endl;
    std::cout<<"topic: "<<topic<<std::endl;

    return 0;
}