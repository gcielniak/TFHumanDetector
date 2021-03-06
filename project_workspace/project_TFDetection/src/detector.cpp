#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/eigen.h>
#include "pcl/filters/voxel_grid.h"

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg (*input, cloud);

  // Downsample using a voxel grid for faster performance
    pcl::VoxelGrid<pcl::PointXYZRGB> downsample;
    downsample.setInputCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>(cloud)));
    downsample.setLeafSize(0.01, 0.01, 0.01);
    pcl::PointCloud<pcl::PointXYZRGB> cloud_downsampled;
    downsample.filter(cloud_downsampled);
    ROS_INFO_STREAM("Voxel Downsampled Cloud");

  // Create the passthrough filtering object on the z axis
  pcl::PassThrough<pcl::PointXYZRGB> z_pass;
  z_pass.setInputCloud (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>(cloud_downsampled)));
  z_pass.setFilterFieldName ("z");
  z_pass.setFilterLimits (2.0, 4.0);
  //pass.setFilterLimitsNegative (true);
  pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered_z;
  z_pass.filter (cloud_filtered_z);

  // Create the passthrough filtering object on the y axis
  pcl::PassThrough<pcl::PointXYZRGB> y_pass;
  y_pass.setInputCloud (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>(cloud_filtered_z)));
  y_pass.setFilterFieldName ("y");
  y_pass.setFilterLimits (-5.0, 1.05);
  //pass.setFilterLimitsNegative (true);
  pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered_y;
  y_pass.filter (cloud_filtered_y);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg (cloud_filtered_y, output);
  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "project_TFDetection");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("detector_output", 1);

  // Spin
  ros::spin ();
}
