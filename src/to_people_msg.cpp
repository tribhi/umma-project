#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>
#include <people_msgs/Person.h>
#include <people_msgs/People.h>

#include <vector>
#include <cstring>
#include <typeinfo>
#include <iostream>
#include <cmath>

ros::Publisher pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud2 cloud;
  projector_.projectLaser(*scan_in, cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pclcloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud, *pclcloud);

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*pclcloud, centroid);

  people_msgs::People people;
  people_msgs::Person person;
  person.name = "people";
  person.position.x = centroid.x();
  person.position.y = centroid.y();
  person.position.z = centroid.z();
  
  people.header = scan_in->header;
  people.people.push_back(person);

  pub.publish(people);
}

int main( int argc, char **argv ) {

  ros::init( argc, argv, "laser_projection" );

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("people_scan", 1, scanCallback);
  pub = n.advertise<people_msgs::People>("people", 1);
  
  ros::spin();
  

  return 0;
}
