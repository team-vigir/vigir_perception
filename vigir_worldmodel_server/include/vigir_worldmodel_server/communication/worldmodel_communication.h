//=================================================================================================
// Copyright (c) 2013, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef WORLDMODEL_COMMUNICATION_H__
#define WORLDMODEL_COMMUNICATION_H__

#include <ros/ros.h>


#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
//#include <octomap_ros/OctomapROS.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs//GetOctomap.h>

#include <vigir_perception_msgs/EnvironmentRegionRequest.h>
#include <vigir_perception_msgs/PointCloudRegionRequest.h>
#include <vigir_perception_msgs/OctomapRegionRequest.h>
#include <vigir_perception_msgs/RaycastRequest.h>
#include <vigir_perception_msgs/PointCloudTypeRegionRequest.h>
#include <vigir_perception_msgs/GetLocomotionTargetPoseAction.h>

#include <actionlib/server/simple_action_server.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/OccupancyGrid.h>

#include <std_srvs/Empty.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


#include <vigir_worldmodel_server/octomap/worldmodel_octomap.h>
#include <vigir_worldmodel_server/point_cloud/point_cloud_aggregator.h>

#include <vigir_worldmodel_server/core/worldmodel_cloud_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vigir_utilities/file_utils.h>

namespace vigir_worldmodel{

  class WorldmodelCommunication{
  public:

    //typedef pcl::PointCloud<ScanPointT> PCLPointCloud;
    //typedef octomap_msgs::BoundingBoxQuery BBXSrv;

    WorldmodelCommunication(ros::NodeHandle& m_nh,
                            boost::shared_ptr<WorldmodelOctomap>& octomap,
                            boost::shared_ptr<PointCloudAggregator<ScanPointT> >& scan_cloud_aggregator,
                            boost::shared_ptr<PointCloudAggregator<ScanPointT> >& unfiltered_scan_cloud_aggregator,
                            boost::shared_ptr<PointCloudAggregator<StereoPointT> >& stereo_head_cloud_aggregator,
                            boost::shared_ptr<tf::TransformListener>& tf_listener)
    : octomap_(octomap)
    , scan_cloud_aggregator_(scan_cloud_aggregator)
    , unfiltered_scan_cloud_aggregator_(unfiltered_scan_cloud_aggregator)
    , stereo_head_cloud_aggregator_(stereo_head_cloud_aggregator)
    , tf_listener_(tf_listener)
    {

      // Main 3D geometry providers
      octomap_binary_srv_server_ = m_nh.advertiseService("/flor/worldmodel/octomap_full", &WorldmodelCommunication::octomapBinarySrv, this);
      octomap_binary_roi_srv_server_ = m_nh.advertiseService("/flor/worldmodel/octomap_roi", &WorldmodelCommunication::octomapBinaryRoiSrv, this);

      pointcloud_srv_server_ =  m_nh.advertiseService("/flor/worldmodel/pointcloud_roi", &WorldmodelCommunication::pointcloudSrv, this);


      octomap_full_pub_ = m_nh.advertise<octomap_msgs::Octomap>("/flor/worldmodel/ocs_octomap", 1, false);


      // OCS communication
      ocs_crop_pointcloud_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/flor/worldmodel/ocs/cloud_result", 1, false);
      ocs_crop_pointcloud_stereo_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/flor/worldmodel/ocs/stereo_cloud_result", 1, false);
      ocs_crop_pointcloud_sub_ = m_nh.subscribe("/flor/worldmodel/ocs/cloud_request", 1, &WorldmodelCommunication::ocsCloudRequestCallback, this);

      ocs_crop_octomap_pub_= m_nh.advertise<octomap_msgs::Octomap>("/flor/worldmodel/ocs/octomap_result", 1, false);
      ocs_crop_octomap_sub_ = m_nh.subscribe("/flor/worldmodel/ocs/octomap_request", 1, &WorldmodelCommunication::ocsOctomapRequestCallback, this);

      ocs_dist_query_cloud_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/flor/worldmodel/ocs/dist_query_pointcloud_result", 1, false);
      ocs_dist_query_cloud_frame_sub_ = m_nh.subscribe("/flor/worldmodel/ocs/dist_query_pointcloud_request_frame", 1, &WorldmodelCommunication::ocsDistQueryCloudRequestFrameCallback, this);
      ocs_dist_query_cloud_world_sub_ = m_nh.subscribe("/flor/worldmodel/ocs/dist_query_pointcloud_request_world", 1, &WorldmodelCommunication::ocsDistQueryCloudRequestWorldCallback, this);

      ocs_dist_query_dist_pub_ = m_nh.advertise<std_msgs::Float64>("/flor/worldmodel/ocs/dist_query_distance_result", 1, false);
      ocs_dist_query_dist_sub_ = m_nh.subscribe("/flor/worldmodel/ocs/dist_query_distance_request_world", 1, &WorldmodelCommunication::ocsDistQueryDistanceRequestFrameCallback, this);


      ocs_crop_gridmap_pub_= m_nh.advertise<nav_msgs::OccupancyGrid>("/flor/worldmodel/ocs/gridmap_result", 1, false);
      ocs_crop_gridmap_sub_ = m_nh.subscribe("/flor/worldmodel/ocs/gridmap_request", 1, &WorldmodelCommunication::ocsGridmapRequestCallback, this);

      //Images
      //left_camera_lidar_depth_image_pub_ = m_nh.advertise<sensor_msgs::Image>("/multisense_sl/camera/left/lidar_depth_image",1, false);

      // Grid maps
      occupancy_grid_pub_ = m_nh.advertise<nav_msgs::OccupancyGrid>("/flor/worldmodel/grid_map", 1, false);
      leg_level_occupancy_grid_pub_ = m_nh.advertise<nav_msgs::OccupancyGrid>("/flor/worldmodel/grid_map_leg_level", 1, false);
      upper_body_level_occupancy_grid_pub_ = m_nh.advertise<nav_msgs::OccupancyGrid>("/flor/worldmodel/grid_map_upper_body_level", 1, false);
      //ground_lvl_occupancy_grid_pub_ = m_nh.advertise<nav_msgs::OccupancyGrid>("/flor/worldmodel/grid_map_ground_lvl", 1, false);
      occupancy_grid_near_robot_pub_ = m_nh.advertise<nav_msgs::OccupancyGrid>("/flor/worldmodel/grid_map_near_robot", 1, false);

      // Reset and others
      sys_command_sub_ = m_nh.subscribe("/syscommand", 1, &WorldmodelCommunication::sysCommandCallback, this);

      pub_timer_ = m_nh.createTimer(ros::Duration(5.0), &WorldmodelCommunication::pubTimerCallback, this, false);

      ros::NodeHandle pnh("~");
      pnh.param("octomap_save_folder", p_octomap_save_folder_ ,std::string(""));

      
      target_pose_action_server_.reset(new actionlib::SimpleActionServer<vigir_perception_msgs::GetLocomotionTargetPoseAction>(
                                       pnh,
                                       "get_locomotion_target_pose",                                    
                                       false));
      target_pose_action_server_->registerGoalCallback( boost::bind(&WorldmodelCommunication::execute_locomotion_target_provider, this, boost::ref(target_pose_action_server_)));

      debug_target_pose_cloud_pub_ = m_nh.advertise<sensor_msgs::PointCloud2>("/flor/worldmodel/debug_action_cloud", 1, false);
      debug_target_pose_pose_pub_ = m_nh.advertise<geometry_msgs::PoseStamped>("/flor/worldmodel/debug_action_pose", 1, false);

      target_pose_action_server_->start();

    }


    ~WorldmodelCommunication()
    {}

    void execute_locomotion_target_provider(boost::shared_ptr<actionlib::SimpleActionServer<vigir_perception_msgs::GetLocomotionTargetPoseAction> >& as)
    {
      // Do lots of awesome groundbreaking robot stuff here
      //as->setSucceeded();

      const vigir_perception_msgs::GetLocomotionTargetPoseGoalConstPtr& goal (as->acceptNewGoal());

      double lowest_foot_height = this->getLowestFootHeight();

      geometry_msgs::Pose action_result_pose;

      tf::StampedTransform transform;
      try{
        tf_listener_->lookupTransform("/world", "/pelvis", ros::Time(0), transform);
      }catch(tf::TransformException e){
        ROS_ERROR("Transform lookup failed in get locomotion pose action server goal callback: %s",e.what());
        as->setAborted();
        return;
      }

      tf::Vector3 pose_in_front_of_robot = transform * tf::Vector3(3.0, 0.0, 0.0);


      pcl::PointCloud<ScanPointT>::Ptr cloud(new pcl::PointCloud<ScanPointT>());

      uint32_t aggregation_size = 200;

      //ROS_INFO("Pointcloud data service request called with aggregation size request %d and used aggregation size %d", (int)req.aggregation_size,  (int)aggregation_size);

      geometry_msgs::Point min;
      min.x = pose_in_front_of_robot.x() - 2.0;
      min.y = pose_in_front_of_robot.y() - 2.0;
      min.z = lowest_foot_height + 0.3;


      geometry_msgs::Point max;
      max.x = pose_in_front_of_robot.x() + 2.0;
      max.y = pose_in_front_of_robot.y() + 2.0;
      max.z = lowest_foot_height + 1.0;

      ROS_DEBUG("Min: %f %f %f Max: %f %f %f", min.x, min.y, min.z, max.x, max.y, max.z);

      if (!scan_cloud_aggregator_->getAggregateCloudBbxFiltered(cloud, "/world", min, max, 0.03, aggregation_size)){
        as->setAborted();
        return;
      }


      if (debug_target_pose_cloud_pub_.getNumSubscribers() > 0){
          sensor_msgs::PointCloud2 cloud_ros;
          pcl::toROSMsg(*cloud, cloud_ros);
          cloud_ros.header.frame_id = "/world";
          cloud_ros.header.stamp = ros::Time::now();
          debug_target_pose_cloud_pub_.publish(cloud_ros);
      }

      std::vector<pcl::PointIndices> cluster_indices;

      pcl::search::KdTree<ScanPointT>::Ptr tree (new pcl::search::KdTree<ScanPointT>);
      tree->setInputCloud (cloud);

      pcl::EuclideanClusterExtraction<ScanPointT> ec;
      ec.setClusterTolerance (0.05);
      ec.setMinClusterSize (20);
      ec.setMaxClusterSize (3000);
      ec.setSearchMethod (tree);
      ec.setInputCloud (cloud);
      ec.extract (cluster_indices);
      ROS_DEBUG("Extraction done: %ld", cluster_indices.size());

      size_t num_clusters = cluster_indices.size();



      tf::StampedTransform robot_world_transform;

      try{
          tf_listener_->lookupTransform("/pelvis", "/world", ros::Time(0), robot_world_transform);
      }catch(tf::TransformException& ex){
          ROS_ERROR_STREAM( "Transform failed " << ex.what());
          as->setAborted();
          return;
      }

      double sqr_min_dist = std::numeric_limits<double>::max();

      int max_clusters = std::min(5, (int)cluster_indices.size());

      const tf::Vector3& robot_pos = robot_world_transform.getOrigin();

      ScanPointT* closest_point = 0;

      for (int i = 0; i < max_clusters; ++i){

          //cloud_cluster->clear();

          pcl::PointIndices& indices = cluster_indices[i];

          size_t size = indices.indices.size();

          ROS_DEBUG("Indices size: %ld", size);



          for (size_t i = 0; i < size; ++i){
              //cloud_cluster->points.push_back(cloud->points[indices.indices[i]]);
              const ScanPointT& curr_point = cloud->points[indices.indices[i]];

              double sqr_dist =  ((curr_point.x - robot_pos.x()) * (curr_point.x - robot_pos.x()) +
                                  (curr_point.y - robot_pos.y()) * (curr_point.y - robot_pos.y()));

              if (sqr_min_dist > sqr_dist){
                  sqr_min_dist = sqr_dist;
                  closest_point = &cloud->points[indices.indices[i]];
              }
          }
      }

      if (closest_point){
        Eigen::Vector2d closest_point_vec (closest_point->x, closest_point->y);
        Eigen::Vector2d dir = closest_point_vec - Eigen::Vector2d(robot_pos.x(), robot_pos.y());
        Eigen::Vector2d dir_normalized (dir.normalized());

        Eigen::Vector2d target_pos (closest_point_vec - (dir_normalized*0.5));

        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = "/world";
        target_pose.header.stamp = ros::Time::now();
        target_pose.pose.position.x = target_pos.x();
        target_pose.pose.position.y = target_pos.y();

        double yaw = std::atan2(dir.y(), dir.x());

        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), target_pose.pose.orientation);

        debug_target_pose_pose_pub_.publish(target_pose);

        vigir_perception_msgs::GetLocomotionTargetPoseActionResult result;
        result.result.target_pose = target_pose;
        as->setSucceeded(result.result);
      }else{
          ROS_ERROR_STREAM( "No closest point found");
          as->setAborted();

      }

    }

    bool octomapBinarySrv(octomap_msgs::GetOctomap::Request  &req,
    octomap_msgs::GetOctomap::Response &res)
    {
      ROS_INFO("Sending binary map data on service request");

      std::string frame_name = "/pelvis";

      WorldmodelOctomap octo(frame_name);

      size_t aggregator_size = scan_cloud_aggregator_->size();

      size_t target_size = std::min(aggregator_size, (size_t)200);

      if (target_size > 1){
        int index = target_size-1;

        //pcl::PointCloud<ScanPointT> pc;

        boost::shared_ptr<pcl::PointCloud<ScanPointT> > pc (new pcl::PointCloud<ScanPointT>());
        tf::Point origin(0,0,0);

        for (;index >= 0; --index){
          if (scan_cloud_aggregator_->getSingleCloud(pc, frame_name, &origin, index)){
            octo.insertCloud(origin, *pc);
          }
        }
      }

      if (!octomap_msgs::binaryMapToMsg(*octo.getCurrentMap().getOcTree(), res.map))
        return false;

      /*
      const OctomapContainer& map = octomap_->getCurrentMap();
      res.map.header.frame_id = map.getFrameId();
      res.map.header.stamp = ros::Time::now();
      if (!octomap_msgs::binaryMapToMsg(*map.getOcTree(), res.map))
        return false;
      */

      return true;
    }

    bool octomapBinaryRoiSrv(vigir_perception_msgs::OctomapRegionRequest::Request  &req,
                        vigir_perception_msgs::OctomapRegionRequest::Response &res)
    {
      std::string frame_name = req.region_req.header.frame_id;

      //If equal to root frame and want default resolution, we can just cut out a bbx. @TODO FIXME: Use parameter for frame
      if (frame_name == "/world" && req.region_req.resolution <= 0.0){
        ROS_INFO("Sending region of interest binary map data on service request, cut-out of existing map");

        boost::shared_ptr<octomap::OcTree> octree(new octomap::OcTree(0.05));

        octomap_->getBbxFilteredOctomap(octree, frame_name, req.region_req.bounding_box_min, req.region_req.bounding_box_max);

        if (!octomap_msgs::binaryMapToMsg(*octree, res.map))
          return false;

        return true;

      }else{        

        ROS_INFO("Sending region of interest binary map data on service request, regenerating from pointcloud");


        double resolution = octomap_->getCurrentMap().getOcTree()->getResolution();

        // Use default resolution unless one > 0.0 has been set in request
        if (req.region_req.resolution > 0.0){
          resolution = req.region_req.resolution;
        }

        // Construct map from scan if a different frame than "/world" is requested
        WorldmodelOctomap octo(frame_name, resolution);

        size_t aggregator_size = scan_cloud_aggregator_->size();

        size_t target_size = std::min(aggregator_size, (size_t)400);

        if (target_size > 1){
          int index = target_size-1;

          boost::shared_ptr<pcl::PointCloud<ScanPointT> > pc (new pcl::PointCloud<ScanPointT>());
          tf::Point origin(0,0,0);

          for (;index >= 0; --index){
            if (scan_cloud_aggregator_->getSingleCloudBbxFiltered(pc, frame_name, req.region_req.bounding_box_min, req.region_req.bounding_box_max, &origin, index)){
              octo.insertCloud(origin, *pc);
            }
          }
        }

        if (!octomap_msgs::binaryMapToMsg(*octo.getCurrentMap().getOcTree(), res.map))
          return false;

        return true;
      }
    }

    bool pointcloudSrv(vigir_perception_msgs::PointCloudRegionRequest::Request& req,
    vigir_perception_msgs::PointCloudRegionRequest::Response& res)
    {
      pcl::PointCloud<ScanPointT>::Ptr cloud(new pcl::PointCloud<ScanPointT>());

      uint32_t aggregation_size = 500;

      if (req.aggregation_size != 0){
        aggregation_size = req.aggregation_size;
      }

      ROS_INFO("Pointcloud data service request called with aggregation size request %d and used aggregation size %d", (int)req.aggregation_size,  (int)aggregation_size);

      if (scan_cloud_aggregator_->getAggregateCloudBbxFiltered(cloud, req.region_req.header.frame_id, req.region_req.bounding_box_min, req.region_req.bounding_box_max, req.region_req.resolution, aggregation_size)){
        pcl::toROSMsg(*cloud, res.cloud);
        return true;
      }else{
        return false;
      }
    }

    void ocsCloudRequestCallback(const vigir_perception_msgs::PointCloudTypeRegionRequest::ConstPtr& msg)
    {
      const vigir_perception_msgs::EnvironmentRegionRequest& env_req = msg->environment_region_request;

      geometry_msgs::Point min_point, max_point;
      checkMinMax(env_req.bounding_box_min, env_req.bounding_box_max, min_point, max_point);

      uint32_t aggregation_size = 500;

      if (msg->aggregation_size != 0){
        aggregation_size = msg->aggregation_size;
      }

      if (msg->data_source == vigir_perception_msgs::PointCloudTypeRegionRequest::LIDAR_FILTERED){
        pcl::PointCloud<ScanPointT>::Ptr cloud(new pcl::PointCloud<ScanPointT>());
        scan_cloud_aggregator_->getAggregateCloudBbxFiltered(cloud, "/world", min_point, max_point, msg->environment_region_request.resolution, aggregation_size);
        sensor_msgs::PointCloud2 pc_ros;
        pcl::toROSMsg(*cloud, pc_ros);
        ocs_crop_pointcloud_pub_.publish(pc_ros);
      }else if(msg->data_source == vigir_perception_msgs::PointCloudTypeRegionRequest::LIDAR_UNFILTERED){
        pcl::PointCloud<ScanPointT>::Ptr cloud(new pcl::PointCloud<ScanPointT>());
        unfiltered_scan_cloud_aggregator_->getAggregateCloudBbxFiltered(cloud, "/world", min_point, max_point, msg->environment_region_request.resolution, aggregation_size);
        sensor_msgs::PointCloud2 pc_ros;
        pcl::toROSMsg(*cloud, pc_ros);
        ocs_crop_pointcloud_pub_.publish(pc_ros);
      }else if(msg->data_source == vigir_perception_msgs::PointCloudTypeRegionRequest::STEREO){
        pcl::PointCloud<StereoPointT>::Ptr cloud(new pcl::PointCloud<StereoPointT>());
        stereo_head_cloud_aggregator_->getAggregateCloudBbxFiltered(cloud, "/world", min_point, max_point, msg->environment_region_request.resolution, 1);
        sensor_msgs::PointCloud2 pc_ros;
        pcl::toROSMsg(*cloud, pc_ros);
        ocs_crop_pointcloud_stereo_pub_.publish(pc_ros);
      }
    }

    void ocsOctomapRequestCallback(const vigir_perception_msgs::EnvironmentRegionRequest::ConstPtr& msg)
    {
      boost::shared_ptr<octomap::OcTree> octree(new octomap::OcTree(0.05));

      geometry_msgs::Point min_point, max_point;
      checkMinMax(msg->bounding_box_min, msg->bounding_box_max, min_point, max_point);

      octomap_->getBbxFilteredOctomap(octree, "/world", min_point, max_point);
      octree->toMaxLikelihood();
      octree->prune();

      octomap_msgs::Octomap octomap;
      octomap_msgs::binaryMapToMsg(*octree, octomap);
      octomap.header.frame_id = octomap_->getCurrentMap().getFrameId();
      octomap.header.stamp = ros::Time::now();

      ocs_crop_octomap_pub_.publish(octomap);
    }

    void ocsGridmapRequestCallback(const vigir_perception_msgs::EnvironmentRegionRequest::ConstPtr& msg)
    {
      geometry_msgs::Point min_point, max_point;
      checkMinMax(msg->bounding_box_min, msg->bounding_box_max, min_point, max_point);

      Eigen::Vector2d min_vec (min_point.x, min_point.y);
      Eigen::Vector2d max_vec (max_point.x, max_point.y);

      double resolution = msg->resolution;
      //resolution = 0.25;

      if (resolution <= 0.06){
        //Write out slice of onboard map if resolution high

        nav_msgs::OccupancyGrid grid_map;
        octomap_->getOccupancyGridmap(octomap_->getCurrentMap().getOcTree(), grid_map, min_point.z, max_point.z, 16, &min_vec, &max_vec);
        ocs_crop_gridmap_pub_.publish(grid_map);
      }else{
        //Special treatment for request with different resolution

        ROS_INFO("Sending grid map region of interest data on ocs request");

        // Construct map from scan @TODO FIXME VRC Hack: Set range to 25.0 if requesting lower res
        WorldmodelOctomap octo("/world", resolution, 25.0);

        size_t aggregator_size = scan_cloud_aggregator_->size();

        size_t target_size = std::min(aggregator_size, (size_t)400);

        if (target_size > 1){
          int index = target_size-1;

          boost::shared_ptr<pcl::PointCloud<ScanPointT> > pc (new pcl::PointCloud<ScanPointT>());
          tf::Point origin(0,0,0);

          for (;index >= 0; --index){
            if (scan_cloud_aggregator_->getSingleCloudBbxFiltered(pc, "/world", min_point, max_point, &origin, index)){
              octo.insertCloud(origin, *pc);
            }
          }
        }
        nav_msgs::OccupancyGrid grid_map;
        octo.getOccupancyGridmap(octo.getCurrentMap().getOcTree(), grid_map, min_point.z, max_point.z, 16, &min_vec, &max_vec);
        ocs_crop_gridmap_pub_.publish(grid_map);

      }
    }


    void pubTimerCallback(const ros::TimerEvent& event)
    {
      double lowest_foot_height = getLowestFootHeight();

      if (occupancy_grid_pub_.getNumSubscribers() > 0){
        nav_msgs::OccupancyGrid map;

        if (octomap_->getOccupancyGridmap(map, lowest_foot_height + 0.15, lowest_foot_height + 1.5)){
          occupancy_grid_pub_.publish (map);
        }
      }

      if (upper_body_level_occupancy_grid_pub_.getNumSubscribers() > 0){
        nav_msgs::OccupancyGrid map;

        if (octomap_->getOccupancyGridmap(map, lowest_foot_height + 0.7, lowest_foot_height + 1.5)){
          upper_body_level_occupancy_grid_pub_.publish (map);
        }
      }

      if (leg_level_occupancy_grid_pub_.getNumSubscribers() > 0){
        nav_msgs::OccupancyGrid map;

        if (octomap_->getOccupancyGridmap(map, lowest_foot_height + 0.15, lowest_foot_height + 0.7)){
          leg_level_occupancy_grid_pub_.publish (map);
        }
      }

      // @ToDo Proof of concept, make available properly later
      /*
      if (left_camera_lidar_depth_image_pub_.getNumSubscribers() > 0){
        ros::Time start = ros::Time::now();

        sensor_msgs::CameraInfo cam_info;
        cam_info.header.stamp = ros::Time::now();
        cam_info.header.frame_id = "left_camera_optical_frame";
        cam_info.width = 800;
        cam_info.height = 800;
        cam_info.K[0] = 476.7030836014194;
        cam_info.K[4] = 476.7030836014194;

        pcl::RangeImagePlanar::Ptr range_image(new pcl::RangeImagePlanar());

        //pcl::PointCloud<ScanPointT>::Ptr cloud(new pcl::PointCloud<ScanPointT>());

        scan_cloud_aggregator_->getRangeImagePlanar(range_image, cam_info, 2000);

        sensor_msgs::Image::Ptr depth_image (new sensor_msgs::Image());
        depth_image->header = cam_info.header;
        depth_image->width = cam_info.width;
        depth_image->height = cam_info.height;
        depth_image->step = sizeof(float)*depth_image->width;
        depth_image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;

        size_t num_pixels = depth_image->width * depth_image->height;

        depth_image->data.resize(num_pixels * sizeof(float));

        std::vector<uint8_t>& data = depth_image->data;

        float nan = std::numeric_limits<float>::quiet_NaN();

        for (size_t i = 0; i < num_pixels; ++i){
          if (range_image->getPoint(i).range > 0.0 && range_image->getPoint(i).range < 4.0){
            memcpy (&data[i*sizeof(float)], &range_image->getPoint(i).range, sizeof (float));
          }else{
            memcpy (&data[i*sizeof(float)], &nan, sizeof (float));//data[i*sizeof(float)]
          }
        }

        left_camera_lidar_depth_image_pub_.publish(depth_image);
        ROS_INFO("Generating depth image took %f seconds", (ros::Time::now() - start).toSec());
      }
      */

      /*
      if (ground_lvl_occupancy_grid_pub_.getNumSubscribers() > 0){
        nav_msgs::OccupancyGrid map;

        if (octomap_->getOccupancyGridmap(map, -0.2, 0.2)){

          size_t size = map.info.width * map.info.height;

          for (size_t i = 0; i < size; ++i){
            unsigned char data = map.data[i];

            if (data == 100){
              map.data[i] = 0;
            }else if (data == 0){
              map.data[i] = 100;
            }
          }

          ground_lvl_occupancy_grid_pub_.publish (map);
        }
      }
      */

      if (octomap_full_pub_.getNumSubscribers() > 0){
        octomap_msgs::Octomap octomap;
        octomap_msgs::binaryMapToMsg(*octomap_->getCurrentMap().getOcTree(), octomap);
        octomap.header.frame_id = octomap_->getCurrentMap().getFrameId();
        octomap.header.stamp = ros::Time::now();

        octomap_full_pub_.publish(octomap);
      }

      if (occupancy_grid_near_robot_pub_.getNumSubscribers() > 0){


        tf::StampedTransform transform;

        bool tf_success = false;

        try{
          tf_listener_->lookupTransform("/world", "/pelvis", ros::Time(0), transform);
          tf_success = true;
        }catch(tf::TransformException& ex){
          ROS_ERROR_STREAM( "Transform when retrieving robot pose for local map: " << ex.what());
        }

        if (tf_success){
          double size = 3.0;
          double min_height = lowest_foot_height + 0.2;
          double max_height = lowest_foot_height + 2.0;

          geometry_msgs::Point min;
          min.x = transform.getOrigin().x() - size;
          min.y = transform.getOrigin().y() - size;
          min.z = min_height;
          geometry_msgs::Point max;
          max.x = transform.getOrigin().x() + size;
          max.y = transform.getOrigin().y() + size;
          max.z = max_height;

          //boost::shared_ptr<octomap::OcTree> octree;
          //octree.reset(new octomap::OcTree(0.05));
          //WorldmodelOctomap octo(frame_name);

          //octomap_->getBbxFilteredOctomap(octree, "/world", min, max);

          nav_msgs::OccupancyGrid grid_map;

          Eigen::Vector2d min_vec (min.x, min.y);
          Eigen::Vector2d max_vec (max.x, max.y);
          octomap_->getOccupancyGridmap(octomap_->getCurrentMap().getOcTree(), grid_map, min_height, max_height, 16, &min_vec, &max_vec);
          occupancy_grid_near_robot_pub_.publish(grid_map);
        }
      }
    }


  void ocsDistQueryCloudRequestFrameCallback(const::geometry_msgs::PointStamped::ConstPtr& msg)
  {
    tf::StampedTransform camera_transform;

    try{
      tf_listener_->waitForTransform("/world", msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
      tf_listener_->lookupTransform("/world", msg->header.frame_id, msg->header.stamp, camera_transform);
    }catch(tf::TransformException e){
      ROS_ERROR("Transform lookup failed in dist query cloud callback: %s",e.what());
      return;
    }

    octomap::point3d origin (octomap::pointTfToOctomap(camera_transform.getOrigin()));

    //tf::Quaternion rotation=stamped_pose.getRotation();
    tf::Point end_point = camera_transform * tf::Point(msg->point.x, msg->point.y, msg->point.z);

    tf::Vector3 direction_tf = end_point - camera_transform.getOrigin();

    octomap::point3d direction (octomap::pointTfToOctomap(direction_tf));

    octomap::point3d hit_point;

    // If we get a hit, retrieve a point cloud at point hit
    if (octomap_->castRay(origin, direction, hit_point)){

      pcl::PointCloud<ScanPointT>::Ptr cloud(new pcl::PointCloud<ScanPointT>());

      double dist_threshold = 0.15;

      geometry_msgs::Point min, max;

      min.x = hit_point.x() - dist_threshold;
      min.y = hit_point.y() - dist_threshold;
      min.z = hit_point.z() - dist_threshold;

      max.x = hit_point.x() + dist_threshold;
      max.y = hit_point.y() + dist_threshold;
      max.z = hit_point.z() + dist_threshold;

      scan_cloud_aggregator_->getAggregateCloudBbxFiltered(cloud, "/world", min, max, 0.0);

      pcl::PointCloud<ScanPointT>::Ptr filtered_cloud(new pcl::PointCloud<ScanPointT>());

      const float voxel_grid_size = 0.01f;
      pcl::VoxelGrid<ScanPointT> vox_grid;
      vox_grid.setInputCloud (cloud);
      vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
      //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
      vox_grid.filter (*filtered_cloud);

      sensor_msgs::PointCloud2 pc_ros;
      pcl::toROSMsg(*cloud, pc_ros);
      ocs_dist_query_cloud_pub_.publish(pc_ros);

    }else{
      ROS_ERROR("Cast ray returned false, no occupied cell hit during raycast.");
    }
  }

  void ocsDistQueryCloudRequestWorldCallback(const::vigir_perception_msgs::RaycastRequest::ConstPtr& msg)
  {

    octomap::point3d origin (msg->origin.x, msg->origin.y, msg->origin.z);

    octomap::point3d direction (msg->direction.x, msg->direction.y, msg->direction.z);

    octomap::point3d hit_point;

    // If we get a hit, retrieve a point cloud at point hit
    if (octomap_->castRay(origin, direction, hit_point)){

      pcl::PointCloud<ScanPointT>::Ptr cloud(new pcl::PointCloud<ScanPointT>());

      double dist_threshold = 0.15;

      geometry_msgs::Point min, max;

      min.x = hit_point.x() - dist_threshold;
      min.y = hit_point.y() - dist_threshold;
      min.z = hit_point.z() - dist_threshold;

      max.x = hit_point.x() + dist_threshold;
      max.y = hit_point.y() + dist_threshold;
      max.z = hit_point.z() + dist_threshold;

      scan_cloud_aggregator_->getAggregateCloudBbxFiltered(cloud, "/world", min, max, 0.0);

      pcl::PointCloud<ScanPointT>::Ptr filtered_cloud(new pcl::PointCloud<ScanPointT>());

      const float voxel_grid_size = 0.01f;
      pcl::VoxelGrid<ScanPointT> vox_grid;
      vox_grid.setInputCloud (cloud);
      vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
      //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
      vox_grid.filter (*filtered_cloud);

      sensor_msgs::PointCloud2 pc_ros;
      pcl::toROSMsg(*cloud, pc_ros);
      ocs_dist_query_cloud_pub_.publish(pc_ros);

    }else{
      ROS_ERROR("Cast ray returned false, no occupied cell hit during raycast.");
    }
  }

  void ocsDistQueryDistanceRequestFrameCallback(const::vigir_perception_msgs::RaycastRequest::ConstPtr& msg)
  {

    octomap::point3d origin (msg->origin.x, msg->origin.y, msg->origin.z);

    octomap::point3d direction (msg->direction.x, msg->direction.y, msg->direction.z);

    octomap::point3d hit_point;

    double distance = -1.0;

    if (octomap_->castRay(origin, direction, hit_point)){
      distance = origin.distance(hit_point);
    }else{
      ROS_ERROR("Cast ray returned false, no occupied cell hit during raycast.");
    }

    std_msgs::Float64 float_msg;
    float_msg.data = distance;
    ocs_dist_query_dist_pub_.publish(float_msg);
  }

    void sysCommandCallback(const std_msgs::String::ConstPtr& msg)
    {      
      if (msg->data == "reset"){
        ROS_INFO("Resetting worldmodel internal state");
        scan_cloud_aggregator_->reset();
        octomap_->reset();

        if (octomap_->isUpdatedFromExternal())
        {
          std_srvs::Empty srv_empty;

          ros::service::call("clear_octomap", srv_empty);
        }

      }else if (msg->data == "save_octomap"){
        std::string file_name;
        if (FileUtils::getTimeBasedUniqueFilename(p_octomap_save_folder_, "octo", ".bt", file_name)){
          ROS_INFO("Writing octomap to %s", file_name.c_str());
          octomap_->writeOctomapToFile(file_name);
        }else{
          ROS_WARN("No octomap save folder specified, not saving octomap");
        }
      }else if (msg->data == "save_pointcloud"){
        std::string file_name;
        if (FileUtils::getTimeBasedUniqueFilename(p_octomap_save_folder_, "pointcloud", ".pcd", file_name)){
          ROS_INFO("Writing pointcloud to %s", file_name.c_str());
          scan_cloud_aggregator_->writeCloudToFile(file_name);
        }else{
          ROS_WARN("No pointcloud save folder specified, not saving pointcloud");
        }
      }
    }

    void checkMinMax(const geometry_msgs::Point& min_orig,
                     const geometry_msgs::Point& max_orig,
                     geometry_msgs::Point& min_new,
                     geometry_msgs::Point& max_new)
    {
      if (min_orig.x > max_orig.x){
        max_new.x = min_orig.x;
        min_new.x = max_orig.x;
      }else{
        max_new.x = max_orig.x;
        min_new.x = min_orig.x;
      }

      if (min_orig.y > max_orig.y){
        max_new.y = min_orig.y;
        min_new.y = max_orig.y;
      }else{
        max_new.y = max_orig.y;
        min_new.y = min_orig.y;
      }

      if (min_orig.z > max_orig.z){
        max_new.z = min_orig.z;
        min_new.z = max_orig.z;
      }else{
        max_new.z = max_orig.z;
        min_new.z = min_orig.z;
      }
    }

    double getLowestFootHeight() const
    {
      return 0.15;
      tf::StampedTransform left_foot_transform;
      tf::StampedTransform right_foot_transform;

      try{
        tf_listener_->lookupTransform("/world", "/l_foot", ros::Time(0), left_foot_transform);
        tf_listener_->lookupTransform("/world", "/r_foot", ros::Time(0), right_foot_transform);
      }catch(tf::TransformException& ex){
        ROS_ERROR_STREAM( "Transform failed when retrieving robot feet poses for local maps: " << ex.what() << " Assuming default offset 0 instead.");
        return 0.0;
      }

      if (left_foot_transform.getOrigin().z() < right_foot_transform.getOrigin().z()){
        return left_foot_transform.getOrigin().z();
      }else{
        return right_foot_transform.getOrigin().z();
      }

    }


  protected:
    boost::shared_ptr<WorldmodelOctomap> octomap_;
    boost::shared_ptr<PointCloudAggregator<ScanPointT> > scan_cloud_aggregator_;
    boost::shared_ptr<PointCloudAggregator<ScanPointT> > unfiltered_scan_cloud_aggregator_;
    boost::shared_ptr<PointCloudAggregator<StereoPointT> > stereo_head_cloud_aggregator_;
    boost::shared_ptr<tf::TransformListener> tf_listener_;

    ros::ServiceServer octomap_binary_srv_server_;
    ros::ServiceServer octomap_binary_roi_srv_server_;

    ros::Publisher octomap_full_pub_;

    ros::ServiceServer pointcloud_srv_server_;

    //ros::ServiceServer m_octomapFullService;

    ros::Publisher occupancy_grid_pub_;
    ros::Publisher leg_level_occupancy_grid_pub_;
    ros::Publisher upper_body_level_occupancy_grid_pub_;
    ros::Publisher occupancy_grid_near_robot_pub_;

    //ros::Publisher left_camera_lidar_depth_image_pub_;

    ros::Publisher ocs_crop_pointcloud_pub_;
    ros::Publisher ocs_crop_pointcloud_stereo_pub_;
    ros::Subscriber ocs_crop_pointcloud_sub_;

    ros::Publisher ocs_crop_octomap_pub_;
    ros::Subscriber ocs_crop_octomap_sub_;

    ros::Publisher ocs_dist_query_cloud_pub_;
    ros::Subscriber ocs_dist_query_cloud_frame_sub_;
    ros::Subscriber ocs_dist_query_cloud_world_sub_;

    ros::Publisher ocs_dist_query_dist_pub_;
    ros::Subscriber ocs_dist_query_dist_sub_;

    ros::Publisher ocs_crop_gridmap_pub_;
    ros::Subscriber ocs_crop_gridmap_sub_;

    ros::Subscriber write_pointcloud_sub_;

    ros::Subscriber sys_command_sub_;

    ros::Timer pub_timer_;

    std::string p_octomap_save_folder_;

    boost::shared_ptr<actionlib::SimpleActionServer<vigir_perception_msgs::GetLocomotionTargetPoseAction> > target_pose_action_server_;
    ros::Publisher debug_target_pose_cloud_pub_;
    ros::Publisher debug_target_pose_pose_pub_;
  };

}

#endif
