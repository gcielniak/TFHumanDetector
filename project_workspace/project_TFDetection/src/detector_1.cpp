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
  y_pass.setFilterFieldName ("z");
  y_pass.setFilterLimits (0.0, 2.0);
  //pass.setFilterLimitsNegative (true);
  pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered_y;
  y_pass.filter (cloud_filtered_y);

  // Cluster the objects based on euclidean distance
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_cluster;
    KdTreePtr clusters_tree =
        boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
    pcl_cluster.setClusterTolerance(0.025);
    pcl_cluster.setMinClusterSize(100);
    pcl_cluster.setSearchMethod(clusters_tree);
    pcl_cluster.setInputCloud(
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_filtered_y));
    pcl_cluster.extract (clusters);
    ROS_INFO_STREAM("Number of clusters found matching the given constraints: "
                    << clusters.size());

//Copied from http://www.pcl-users.org/clustering-and-a-bounding-box-td3905570.html
visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, std::string ns ,int id, float r, float g, float b)
{
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;
 
  pcl::compute3DCentroid (*cloud_cluster, centroid);
  pcl::getMinMax3D (*cloud_cluster, min, max);
 
  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;
  marker.header.frame_id = cloud_cluster->header.frame_id;
  marker.header.stamp = ros::Time::now();
 
  marker.ns = ns;
  marker.id = id;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
 
  marker.pose.position.x = centroid[0];
  marker.pose.position.y = centroid[1];
  marker.pose.position.z = centroid[2];
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
 
  marker.scale.x = (max[0]-min[0]);
  marker.scale.y = (max[1]-min[1]);
  marker.scale.z = (max[2]-min[2]);
 
  if (marker.scale.x ==0)
      marker.scale.x=0.1;

  if (marker.scale.y ==0)
    marker.scale.y=0.1;

  if (marker.scale.z ==0)
    marker.scale.z=0.1;
   
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.5;

  marker.lifetime = ros::Duration();
//   marker.lifetime = ros::Duration(0.5);
  return marker;
} 

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg (cloud_filtered_y, output);
  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output_passthrough", 1);

  // Spin
  ros::spin ();
}
