#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

ros::Publisher pub_velo;
ros::Publisher pub_velo_wo_speed;
pcl::PointCloud<pcl::PointXYZ> cloud_fuse;
pcl::PointCloud<pcl::PointXYZ> cloud_fuse_wo_speed;
pcl::PointCloud<pcl::PointXYZ> obstacle_cloud_;  
pcl::PointCloud<pcl::PointXYZ> obstacle_cloud_wo_speed;   	

class Velo_pointcloud {
public:
	void callback_lidar_sync(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_1, 
	const sensor_msgs::PointCloud2ConstPtr& cloud_msg_2);

	void transform_pointcloud();

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZ> transformed_cloud;           
	Eigen::Matrix4f transform;

};

Velo_pointcloud velo_1;
Velo_pointcloud velo_2;
Velo_pointcloud velo_3;


void vlpCallback(const ros::TimerEvent& event);
void constructFullClouds(pcl::PointCloud<pcl::PointXYZ>* scan,
                                    unsigned npoints, size_t &obs_count, pcl::PointCloud<pcl::PointXYZ>* obstacle_cloud);


#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))
ros::Publisher pub_velo1;
ros::Publisher pub_velo2;
ros::Publisher pub_velo3;