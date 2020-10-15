/*
 * Author: 
 * Version: 0.1
 *
 * ROS Node that subscribes to a point cloud and UWB range and finds
 * 
 */

#include <ctime>
#include <vector>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>




class DroneFinder
{
    /*
    * DroneFinder Class - ROS NODE
    * 
    * Subscribes to:
    *   - UAV height topic  (sensor_msgs/Range)
    *   - UAV UWB range     (sensor_msgs/Range)
    *   - Lidar point cloud (sensor_msgs/ TODO )
    */

    private:

        ros::NodeHandle n;                  // ROS Node Handle

        ros::Publisher drone_cloud_pub;     // Publish extracted drone point cloud
        ros::Publisher aux_cloud_pub;       // Publish 
        ros::Publisher meanpoint_pub;       // Publish 

        ros::Subscriber cloud_sub;          // Subscription to LSLidar point cloud
        ros::Subscriber uwb_pos_sub;        // Subscription to UWB pose topic
        ros::Subscriber tfmini_sub;         // Subscription to TFmini range topic

        float lidar_z = 0.6;                // Lidar height over TFmini ref (TODO use argument)
        float drone_z = -1.0;               // UAV Height
        float uwb_range = 0;                // UWB received range
        bool received_uwb = false;          // UWB received at least once

        geometry_msgs::Pose drone_pose;     // Output drone POSE
        geometry_msgs::Point meanPoint;     // Estimated UAV position from UWB+Lidar search
 

    public:

        // Constructor
        Drone_Finder();

        // Cloud to be published
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

        // Callbacks and other methods
        void lidar_pointcloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
        void uwb_pos_cb( geometry_msgs::Pose pose);
        void tfmini_cb(sensor_msgs::Range msg);
        void kdtree_search();
};

Drone_Finder::Drone_Finder(void) {
    /*
    * Constructor
    * 
    * */
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_sub = n.subscribe("/lslidar_point_cloud", 1000, &PCL_conv::cloud_cb, this);
    uwb_drone_pos_cb = n.subscribe("/dwm1001/tag/drone/position", 1000, &PCL_conv::deca_pos_cb, this);
    tfmini_sub = n.subscribe("/tfmini_ros_node/TFmini", 1000, &PCL_conv::tfmini_cb, this);

    utuTIERS_drone_cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("utuTIERS_drone_points", 1000);
    deca_drone_cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("deca_drone_points", 1000);
    aux_cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("aux_drone_points", 1000);
    meanpoint_pub = n.advertise<geometry_msgs::Point> ("lidar_mean_point", 1000);
}


void PCL_conv::uwb_drone_pos_cb(geometry_msgs::Pose pose)
{
    /*
    * Store UWB position 
    */
    this->drone_pose = pose;
    this->drone_pose.position.z = this->z_pos_drone - 0.8; //substract the z position of the reference (3d lidar) to the lidar1D measurement 
    this-> = true;
}



void PCL_conv::tfmini_cb(sensor_msgs::Range msg)
{
    /*
    * Store TFmini range in internal attribute
    */
    this->drone_z = msg.range;
}


/*CALLBACK TO CONVERT LIDAR DATA TO POINT CLOUD*/
void PCL_conv::lidar_pointcloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){ //convert msgs pointcloud2 to pcl cloud

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

    kdtree_search();
}

 
void DroneFinder::kdtree_search()
{
    /*
    * Initializes a kdTree for searching the point cloud near the UWB location
    * 
    * Searches at fixed intervals and 
    * 
    * */

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud (cloud);

    pcl::PointXYZ searchPoint;
  
  meanPoint.x = 0.0;
  meanPoint.y = 0.0;
  meanPoint.z = 0.0;

  searchPoint.x = drone_pose.position.x;//1.23f;
  searchPoint.y = drone_pose.position.y;//5.85f;
  searchPoint.z = drone_pose.position.z;//-0.52f;

  // K nearest neighbor search

  int K = 1;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;

  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      std::cout << "    "  <<   (*cloud)[ pointIdxNKNSearch[i] ].x 
                << " " << (*cloud)[ pointIdxNKNSearch[i] ].y 
                << " " << (*cloud)[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
  }

 searchPoint.x = (*cloud)[ pointIdxNKNSearch[0] ].x  ;
 searchPoint.y = (*cloud)[ pointIdxNKNSearch[0] ].y  ;
 searchPoint.z = (*cloud)[ pointIdxNKNSearch[0] ].z  ;
  // Neighbors within radius search

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = 0.3f;

  
  // create point cloud object
  pcl::PointCloud<pcl::PointXYZ>::Ptr droneCloud (new pcl::PointCloud<pcl::PointXYZ>);
  droneCloud->header.frame_id = "laser_link";


  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  {
    for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
    {
     
      pcl::PointXYZ newPoint;
      newPoint.x = (*cloud)[ pointIdxRadiusSearch[i] ].x ;
      newPoint.y = (*cloud)[ pointIdxRadiusSearch[i] ].y ;
      newPoint.z = (*cloud)[ pointIdxRadiusSearch[i] ].z ;
      droneCloud->points.push_back(newPoint);

      meanPoint.x += newPoint.x;
      meanPoint.y += newPoint.y;
      meanPoint.z += newPoint.z;

        
    }

    //calculate mean point to take it as the lidar's estimation
    meanPoint.x = meanPoint.x/pointIdxRadiusSearch.size();
    meanPoint.y = meanPoint.y/pointIdxRadiusSearch.size();
    meanPoint.z = meanPoint.z/pointIdxRadiusSearch.size();

    
    
    // publish point cloud
    if(droneCloud->size() > 0 ) {
      ros::Time time_st = ros::Time::now ();

      droneCloud->header.stamp= time_st.toNSec()/1e3;

      
      if(received_utuTIERS){
        utuTIERS_drone_cloud_pub.publish (droneCloud);
        received_utuTIERS = false;
      }
        
      else if(received_deca){
        deca_drone_cloud_pub.publish (droneCloud);
        received_deca = false;
      }
      meanpoint_pub.publish(meanPoint);
    }
  
  }

}

int main(int argc, char **argv)
{
    // *
    // * The ros::init() function needs to see argc and argv so that it can perform
    // * any ROS arguments and name remapping that were provided at the command line.
    // * For programmatic remappings you can use a different version of init() which takes
    // * remappings directly, but for most command-line programs, passing argc and argv is
    // * the easiest way to do it.  The third argument to init() is the name of the node.
    // *
    // * You must call one of the versions of ros::init() before using any other
    // * part of the ROS system.
    

    ros::init(argc, argv, "pcl_conver");

    PCL_conv pcl_conver;
    
    ros::spin();

    return 0;
}