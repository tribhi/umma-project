#include <ros/ros.h>
#include <ros/console.h>
//#include <image_transport/image_transport.h>
//#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
//#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>

#include <vector>
#include <cstring>
#include <typeinfo>
#include <iostream>
#include <cmath>


ros::Publisher pub;
ros::Publisher pub_laserscan;

class boundingbox {
public:
  int labelSize;
  int index[];
  sensor_msgs::PointCloud pub_cloud;
  //peopleNum;
  void boundingboxCallback(darknet_ros_msgs::BoundingBoxes boundingBoxes);
  void ptCallback(const sensor_msgs::PointCloud2ConstPtr& ptcloud);
  void downsampleCallback(const sensor_msgs::PointCloud2ConstPtr& ptcloud);

};


void boundingbox::boundingboxCallback(darknet_ros_msgs::BoundingBoxes boundingBoxes)
{ 
  //std::cout<<"aaaaaaaaaaaaaaaaaaa"<<test.size()<<std::endl;
  labelSize = boundingBoxes.bounding_boxes.size();
  index[0] = -1;
  int peopleNum = 0;
  //ROS_INFO("Number of Boundingboxes: %d\n", labelSize);
  for (int j=0; j<labelSize; j++) {
    if (boundingBoxes.bounding_boxes[j].Class == "person") {
      //ROS_INFO_STREAM(boundingBoxes.bounding_boxes[j].Class);
      index[4*peopleNum] = boundingBoxes.bounding_boxes[j].xmin;
      index[4*peopleNum+1] = boundingBoxes.bounding_boxes[j].xmax;
      index[4*peopleNum+2] = boundingBoxes.bounding_boxes[j].ymin;
      index[4*peopleNum+3] = boundingBoxes.bounding_boxes[j].ymax;
      peopleNum = peopleNum + 1;
      //ROS_INFO("???: %d\n", peopleNum);
    }
  } 
  labelSize = peopleNum;
  //ROS_INFO("!!!!!!!!!!!!!: %d\n", labelSize);

}


void boundingbox::ptCallback(const sensor_msgs::PointCloud2ConstPtr& ptcloud)
{
  //std::cout<< "aaaaaaaaaaaaaa" << index[0] <<std::endl;
  //std::cout<< "bbbbbbbbbbbbbb" << labelSize <<std::endl;
  int height = ptcloud->height;
  int width = ptcloud->width;
  /* 
  int ptStep = ptcloud->point_step;
  int rowStep = ptcloud->row_step;
  int x_offset = ptcloud->fields[0].offset;
  int y_offset = ptcloud->fields[1].offset;
  int z_offset = ptcloud->fields[2].offset;


  float x = 0.0;
  float y = 0.0;
  float z = 0.0;

  for (int i=0; i<height*width; i++) {
 
    memcpy(&x, &(ptcloud->data[ptStep*i+x_offset]), sizeof(float));
    memcpy(&y, &(ptcloud->data[ptStep*i+y_offset]), sizeof(float));
    memcpy(&z, &(ptcloud->data[ptStep*i+z_offset]), sizeof(float));
  }
  */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*ptcloud, *cloud);

    //ROS_INFO_STREAM(cloud->points[0]);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    bool flag;
    // index[]=[xmin,xmax,ymin,ymax];
    for (int i=0; i<cloud->points.size(); i++) {
      flag = false;
      for (int j=0; j<labelSize; j++) {
        if ((i/width > index[j*4+2] && i/width < index[j*4+3]) && (i%width > index[4*j] && i%width < index[4*j+1])) {
          if (!isnan(cloud->points[i].x))
            flag = true;
        }
      }
      if (flag)
        inliers->indices.push_back(i);
    }
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud);

    ROS_INFO_STREAM(cloud->points.size());

    pub.publish(cloud);
}


void boundingbox::downsampleCallback(const sensor_msgs::PointCloud2ConstPtr& ptcloud)
{
  int height = ptcloud->height;
  int width = ptcloud->width;
  //ROS_INFO_STREAM(ptcloud->header);
  //tf2_ros::Buffer tfBuffer;
  //tf2_ros::TransformListener tfListener(tfBuffer);
  //geometry_msgs::TransformStamped transformStamped;
  //transformStamped = tfBuffer.lookupTransform("base_link", ptcloud->header.frame_id, ros::Time(0));
  /* 
  int ptStep = ptcloud->point_step;
  int rowStep = ptcloud->row_step;
  int x_offset = ptcloud->fields[0].offset;
  int y_offset = ptcloud->fields[1].offset;
  int z_offset = ptcloud->fields[2].offset;
  

  float x = 0.0;
  float y = 0.0;
  float z = 0.0;

  for (int i=0; i<height*width; i++) {
 
    memcpy(&x, &(ptcloud->data[ptStep*i+x_offset]), sizeof(float));
    memcpy(&y, &(ptcloud->data[ptStep*i+y_offset]), sizeof(float));
    memcpy(&z, &(ptcloud->data[ptStep*i+z_offset]), sizeof(float));
  }
  */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*ptcloud, *cloud);
    //cloud->header.frame_id = "base_link";

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    bool flag;
    // index[]=[xmin,xmax,ymin,ymax];
    for (int i=0; i<cloud->points.size(); i++) {
      flag = false;
      for (int j=0; j<labelSize; j++) {
        if ((i/width > index[j*4+2]/4 && i/width < index[j*4+3]/4) && (i%width > index[4*j]/4 && i%width < index[4*j+1]/4)) {
          if (!isnan(cloud->points[i].x))
            if (sqrt(pow(cloud->points[i].x,2)+pow(cloud->points[i].z,2))+pow(cloud->points[i].y,2)<=5)
              flag = true;
        }
      }
      if (flag)
        inliers->indices.push_back(i);
    }
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud);

    ROS_INFO_STREAM(cloud->points.size());


    pub.publish(cloud);
}





int main( int argc, char **argv )
{
  ros::init( argc, argv, "people_ptCloud" );

  ros::NodeHandle n;

  boundingbox bbx;

  ros::Subscriber sub = n.subscribe<darknet_ros_msgs::BoundingBoxes>("darknet_ros/bounding_boxes", 1, &boundingbox::boundingboxCallback, &bbx);
  //depth/points
  //ros::Subscriber sub2 = n.subscribe<sensor_msgs::PointCloud2>("/head_camera/depth/points", 1, &boundingbox::ptCallback, &bbx);
  //downsampled points
  ros::Subscriber sub3 = n.subscribe<sensor_msgs::PointCloud2>("/head_camera/depth_downsample/points", 1, &boundingbox::downsampleCallback, &bbx);

  pub = n.advertise<sensor_msgs::PointCloud2>("people_point", 1);
  
  ros::spin();
  

  return 0;
}
