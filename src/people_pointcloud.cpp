#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <people_msgs/Person.h>
#include <people_msgs/People.h>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef darknet_ros_msgs::BoundingBoxes BoundingBoxes;
const int DOWNSAMPLE_FACTOR = 4;

//global publishers and listener for access in the callback
ros::Publisher pc_pub;
ros::Publisher people_pub;
ros::Publisher people_markers_pub;
tf::TransformListener *listener;

std::string fixed_frame = "base_link";

int get_ordered_pc_index(const int &x,const int &y,const int &width)
{
    return y * width + x;
}

void callback(const BoundingBoxes::ConstPtr& bb, const PointCloud::ConstPtr& pc, float reliability_threshold, float filter_limit_lower, float filter_limit_upper)
{
    int people_detected = 0;
    int pc_width(pc->width);
    int pc_height(pc->height);
    people_msgs::People people;
    visualization_msgs::Marker people_markers;
    ros::Time t = pcl_conversions::fromPCL(pc->header.stamp);
    people_markers.header.frame_id = fixed_frame;
    people_markers.header.stamp = t;
    people_markers.ns = "sphere_list";
    people_markers.action = visualization_msgs::Marker::ADD;
    people_markers.pose.orientation.w = 1.0;
    people_markers.id = 0;
    people_markers.type = visualization_msgs::Marker::SPHERE_LIST;
    people_markers.scale.x = 0.1;
    people_markers.scale.y = 0.1;
    people_markers.scale.z = 0.1;

    // Spheres are green
    people_markers.color.g = 1.0f;
    people_markers.color.a = 1.0;

    people.header.frame_id = fixed_frame;
    people.header.stamp = t;

    PointCloud::Ptr fullCloud(new PointCloud);
    fullCloud->header.frame_id = fixed_frame;
    fullCloud->header.stamp = pc->header.stamp;
    //PointCloud::Ptr tempfullCloud(new PointCloud);

    BOOST_FOREACH (const darknet_ros_msgs::BoundingBox& box,bb->bounding_boxes)
    {
        std::string classification(box.Class);
        float threshold = reliability_threshold;//.6f;

        if (classification == "person" && box.probability > threshold)
        {
            people_detected++;
            int xmin(box.xmin/DOWNSAMPLE_FACTOR);
            int xmax(box.xmax/DOWNSAMPLE_FACTOR);
            int ymin(box.ymin/DOWNSAMPLE_FACTOR);
            int ymax(box.ymax/DOWNSAMPLE_FACTOR);

            pcl::PointIndices indices;
            for (int x = xmin; x < xmax; x++)
            {
                for (int y = ymin; y < ymax; y++)
                {
                    // int index = (pc_height - y) * pc_width + x;
                    int index = get_ordered_pc_index(x,y,pc_width);
                    indices.indices.push_back(index);
                }
            }
            pcl::ExtractIndices<pcl::PointXYZ> eifilter (true); // Initializing with true will allow us to extract the removed indices
            eifilter.setInputCloud (pc);
            eifilter.setIndices (boost::make_shared<const pcl::PointIndices> (indices));
            PointCloud::Ptr tempCloud(new PointCloud);
            eifilter.filter (*tempCloud);
            // tempCloud->height = ymax - ymin;
            // tempCloud->width = xmax - xmin;

            // Calculate centroid of bounding box point cloud
            pcl::PointXYZ centroid;
            pcl::computeCentroid(*tempCloud, centroid);

            // Filtere points that are delta_z behind centroid to remove background
            float delta_z = .125;
            pcl::PassThrough<pcl::PointXYZ> ptfilter; 
            PointCloud::Ptr z_filtered_cloud(new PointCloud);
            ptfilter.setInputCloud (tempCloud);
            ptfilter.setFilterFieldName ("z");
            ptfilter.setFilterLimits (0.0, centroid.z + delta_z);
            ptfilter.filter (*z_filtered_cloud);

            // Transform point cloud to base_link to filter floor
            PointCloud::Ptr z_filtered_cloud_tf(new PointCloud);
            tf::StampedTransform transform;

            try{
                listener->waitForTransform(fixed_frame,pc->header.frame_id,t, ros::Duration(3.0));
                listener->lookupTransform (fixed_frame,pc->header.frame_id,t, transform);
                pcl_ros::transformPointCloud(fixed_frame,*z_filtered_cloud,*z_filtered_cloud_tf,*listener);
            }
            catch (tf::TransformException &ex)
            {
                ROS_INFO_STREAM("Transform failure: " << ex.what());
                return;
            }

            // Filter points that are likely to be the floor
            PointCloud::Ptr z_filtered_cloud_2(new PointCloud);
            ptfilter.setInputCloud (z_filtered_cloud_tf);
            ptfilter.setFilterFieldName ("z");
            ptfilter.setFilterLimits (filter_limit_lower, filter_limit_upper);
            ptfilter.filter (*z_filtered_cloud_2);

            // Calculate centroid of twice filtered and transformed point cloud
            pcl::PointXYZ filtered_centroid;
            pcl::computeCentroid(*z_filtered_cloud_tf, filtered_centroid);

            geometry_msgs::Point personPoint;
            personPoint.x = filtered_centroid.x;
            personPoint.y = filtered_centroid.y;
            personPoint.z = filtered_centroid.z;
            people_markers.points.push_back(personPoint);

            people_msgs::Person person;
            person.name = "Evil Human";
            person.position = personPoint;
            person.reliability = box.probability;
            people.people.push_back(person);

            // Publish point cloud
           // pc_pub.publish(z_filtered_cloud_2);

            *fullCloud += *z_filtered_cloud_2;
            //tempfullCloud = fullCloud;
           // ROS_INFO_STREAM("subcloud size= " << z_filtered_cloud_2->size());
        }

    }
    ROS_INFO_STREAM(people_detected << " person(s) were extracted in this image");
    people_pub.publish(people);
    people_markers_pub.publish(people_markers);
    pc_pub.publish(*fullCloud);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "person_bb");
    ros::NodeHandle nh;
    pc_pub = nh.advertise<PointCloud>("people_point", 1);
    people_pub = nh.advertise<people_msgs::People>("people", 1);
    people_markers_pub = nh.advertise<visualization_msgs::Marker>("people_markers", 10);
    
    float reliability_threshold;
    float filter_limit_lower;
    float filter_limit_upper;
    nh.param<float>("reliability_threshold", reliability_threshold, 0.6);
    nh.param<float>("filter_limit_lower", filter_limit_lower, 0.3);
    nh.param<float>("filter_limit_upper", filter_limit_upper, 3.0);

    ROS_INFO_STREAM("reliability_threshold: " << reliability_threshold);
    ROS_INFO_STREAM("filter_limit_lower: " << filter_limit_lower);
    ROS_INFO_STREAM("filter_limit_upper: " << filter_limit_upper);

    listener = new tf::TransformListener();

    message_filters::Subscriber<PointCloud> pc_sub(nh, "/head_camera/depth_downsample/points", 1);
    message_filters::Subscriber<BoundingBoxes> bb_sub(nh, "/darknet_ros/bounding_boxes", 1);

    typedef message_filters::sync_policies::ApproximateTime<BoundingBoxes, PointCloud> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), bb_sub, pc_sub);

    sync.registerCallback(boost::bind(&callback, _1, _2, reliability_threshold, filter_limit_lower, filter_limit_upper));

    ros::spin();

    return 0;
}
