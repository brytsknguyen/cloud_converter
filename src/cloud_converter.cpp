/**
* This file is part of pointcloud converter.
* 
* Copyright (C) 2020 Thien-Minh Nguyen <thienminh.nguyen at ntu dot edu dot sg>,
* School of EEE
* Nanyang Technological Univertsity, Singapore
* 
* For more information please see <https://britsknguyen.github.io>.
* or <https://github.com/britsknguyen/pointcloud_converter>.
* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* pointcloud_converter is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* pointcloud_converter is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with pointcloud_converter.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Created by Thien-Minh Nguyen on 15/12/20.
//

#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// #include "utility.h"

#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define RESET "\033[0m"

struct PointOuster
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t  ring;
    // uint16_t ambient; // Available in NTU VIRAL and multicampus datasets
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointOuster,
                                 (float, x, x) (float, y, y) (float, z, z)
                                 (float, intensity, intensity)
                                 (uint32_t, t, t)
                                 (uint16_t, reflectivity, reflectivity)
                                 (uint8_t,  ring, ring)
                                //  (uint16_t, ambient, ambient)
                                 (uint32_t, range, range))

struct PointVelodyne
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointVelodyne,
                                  (float, x, x) (float, y, y) (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (float, time, time))

struct PointHesai
{
    PCL_ADD_POINT4D
    float intensity;
    double timestamp;
    uint16_t ring;                   ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointHesai,
                                 (float, x, x) (float, y, y) (float, z, z)
                                 (float, intensity, intensity)
                                 (double, timestamp, timestamp)
                                 (uint16_t, ring, ring))



typedef pcl::PointCloud<PointOuster> CloudOuster;
typedef pcl::PointCloud<PointOuster>::Ptr CloudOusterPtr;

typedef pcl::PointCloud<PointVelodyne> CloudVelodyne;
typedef pcl::PointCloud<PointVelodyne>::Ptr CloudVelodynePtr;

typedef pcl::PointCloud<PointHesai> CloudHesai;
typedef pcl::PointCloud<PointHesai>::Ptr CloudHesaiPtr;

using namespace std;
using namespace Eigen;

class CloudConverter
{

private:
        
    // Node handler
    ros::NodeHandlePtr nh_ptr;

    // Subscribers
    ros::Subscriber lidar_sub;
    string input_type = "velodyne";

    // Subscribers
    ros::Publisher lidar_pub;
    string output_type = "ouster";

    // Maximum threads for parallelirazation
    int MAX_THREAD = std::thread::hardware_concurrency();
    
public:
    
    // Destructor
    ~CloudConverter() {}

    // Constructor
    CloudConverter(ros::NodeHandlePtr &nh_ptr_) : nh_ptr(nh_ptr_)
    {
        // Initialize the variables and subsribe/advertise topics here
        Initialize();
    }

    void Initialize()
    {

        /* #region Lidar --------------------------------------------------------------------------------------------*/
        
        // Read the lidar topic
        string input_topic = "/velodyne/points";
        nh_ptr->getParam("input_topic", input_topic);

        // Input type
        nh_ptr->getParam("input_type", input_type);

        // Output topic
        string output_topic = "/ouster/points";
        nh_ptr->getParam("output_topic", output_topic);

        // Output type
        nh_ptr->getParam("output_type", output_type);

        if (input_type == output_type)
        {
            printf("Input and Output types are the same, exitting\n");
            exit(-1);
        }

        // Subscribe to the input
        lidar_sub = nh_ptr->subscribe<sensor_msgs::PointCloud2>(input_topic, 100, &CloudConverter::PcHandler, this);

        // Advertise the output
        lidar_pub = nh_ptr->advertise<sensor_msgs::PointCloud2>(output_topic, 100);
    }

    void PcHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        // Use ouster as the standard type
        CloudOusterPtr cloudOuster(new CloudOuster());

        // Read the data into ouster type
        if (input_type == "ouster")
            pcl::fromROSMsg(*msg, *cloudOuster);
        else if (input_type == "velodyne")
        {
            CloudVelodynePtr cloudVelodyne(new CloudVelodyne);
            pcl::fromROSMsg(*msg, *cloudVelodyne);

            int pointsTotal = cloudVelodyne->size();
            cloudOuster->resize(pointsTotal);

            #pragma omp parallel for num_threads(MAX_THREAD)
            for(int i = 0; i < pointsTotal; i++)
            {
                auto &src = cloudVelodyne->points[i];
                auto &dst = cloudOuster->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.t = src.time * 1e9f;
                dst.range = sqrt(src.x * src.x + src.y * src.y + src.z * src.z);
            }
        }
        else if (input_type == "hesai")
        {
            double hsToOusterIntensity = 1500/255.0;

            CloudHesaiPtr cloudHesai(new CloudHesai());
            pcl::fromROSMsg(*msg, *cloudHesai);

            int pointsTotal = cloudHesai->size();
            cloudOuster->resize(pointsTotal);

            #pragma omp parallel for num_threads(MAX_THREAD)
            for(int i = 0; i < pointsTotal; i++)
            {
                auto &src = cloudHesai->points[i];
                auto &dst = cloudOuster->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity * hsToOusterIntensity;
                dst.ring = src.ring;
                dst.t = (int)((src.timestamp - msg->header.stamp.toSec()) * 1e9f);
                dst.range = sqrt(src.x*src.x + src.y*src.y + src.z*src.z);
            }
        }

        // Load the data to output type
        if (output_type == "ouster")
            publishCloud(lidar_pub, *cloudOuster, msg->header.stamp, msg->header.frame_id);
        else if (output_type == "velodyne")
        {
            int pointsTotal = cloudOuster->size();

            CloudVelodynePtr cloudVelodyne(new CloudVelodyne);
            cloudVelodyne->resize(pointsTotal);

            #pragma omp parallel for num_threads(MAX_THREAD)
            for(int i = 0; i < pointsTotal; i++)
            {
                auto &src = cloudOuster->points[i];
                auto &dst = cloudVelodyne->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t / 1.0e9f;
            }

            publishCloud(lidar_pub, *cloudVelodyne, msg->header.stamp, msg->header.frame_id);
        }
        else if (output_type == "hesai")
        {

            double hsToOusterIntensity = 1500/255.0;

            int pointsTotal = cloudOuster->size();

            CloudHesaiPtr cloudHesai(new CloudHesai);
            cloudHesai->resize(pointsTotal);

            #pragma omp parallel for num_threads(MAX_THREAD)
            for(int i = 0; i < pointsTotal; i++)
            {
                auto &src = cloudOuster->points[i];
                auto &dst = cloudHesai->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = (int)(min(src.intensity, (float)1500.0) / hsToOusterIntensity);
                dst.ring = src.ring;
                dst.timestamp = (int)(src.t/1.0e9 + msg->header.stamp.toSec());
            }

            publishCloud(lidar_pub, *cloudHesai, msg->header.stamp, msg->header.frame_id);
        }

    }

    template <typename PointType>
    void publishCloud(ros::Publisher &pub, pcl::PointCloud<PointType> &cloud, ros::Time stamp, std::string frame)
    {
        sensor_msgs::PointCloud2 cloud_;
        pcl::toROSMsg(cloud, cloud_);
        cloud_.header.stamp = stamp;
        cloud_.header.frame_id = frame;
        pub.publish(cloud_);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_converter");
    ros::NodeHandle nh("~");
    ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(nh);

    ROS_INFO(KGRN "----> Pointcloud converter Lidar Started." RESET);

    CloudConverter cloud_converter(nh_ptr);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();
    
    return 0;
}
