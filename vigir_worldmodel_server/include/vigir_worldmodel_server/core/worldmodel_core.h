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

#ifndef WORLDMODEL_CORE__
#define WORLDMODEL_CORE__

#include <ros/ros.h>

#include <vigir_worldmodel_server/octomap/worldmodel_octomap.h>
#include <vigir_worldmodel_server/octomap/octomap_visualization.h>

#include <vigir_worldmodel_server/communication/worldmodel_communication.h>

#include <vigir_worldmodel_server/point_cloud/point_cloud_aggregator.h>
#include <vigir_worldmodel_server/point_cloud/point_cloud_visualization.h>

#include <vigir_worldmodel_server/sensors/point_cloud_subscription_adapter.h>

#include <vigir_worldmodel_server/state/transform_service_provider.h>
#include <vigir_worldmodel_server/state/state_provider.h>
#include <vigir_worldmodel_server/state/tf_pose_republisher.h>
#include <vigir_worldmodel_server/state/stab_republisher.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
//#include <flor_utilities/timing.h>

#include <message_filters/subscriber.h>

#include <vigir_worldmodel_server/core/worldmodel_cloud_types.h>

namespace vigir_worldmodel
{
/**
 * Main world modelling class. Subscribes to point cloud data and makes environment data available through ROS
 * interface.
 */
class WorldmodelCore
{
public:
  /**
   * Constructor, node handles as argument make it easy to use this either in a node or nodelet.
   */
  WorldmodelCore(ros::NodeHandle& nh_in, ros::NodeHandle& pnh_in)
    : nh_(nh_in)
    , pnh_(pnh_in)
    , octo_marker_vis_(pnh_in)
    , point_cloud_vis_(pnh_in)
    , unfiltered_point_cloud_vis_(pnh_in, "unfiltered_cloud_vis")
  {
    // Ignore PCL warning spam
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    tf_listener_ = boost::make_shared<tf::TransformListener>();

    bool local_mapping;

    pnh_.param("root_frame", p_root_frame_, std::string("/world"));
    pnh_.param("use_external_octomap", p_use_external_octomap_, false);
    pnh_.param("octomap_max_range", p_octomap_max_range_, 5.0);
    pnh_.param("local_mapping", local_mapping, false);

    // waitForTf(pnh_);

    octomap_ = boost::make_shared<WorldmodelOctomap>(p_root_frame_, 0.05, p_octomap_max_range_, local_mapping);

    scan_cloud_aggregator_ = boost::make_shared<PointCloudAggregator<ScanPointT> >(tf_listener_, 8000);
    scan_cloud_updater_ =
        boost::make_shared<PointCloudSubscriptionAdapter<ScanPointT> >(scan_cloud_aggregator_, "/scan_cloud_filtered");

    unfiltered_scan_cloud_aggregator_ = boost::make_shared<PointCloudAggregator<ScanPointT> >(tf_listener_, 4000);
    unfiltered_scan_cloud_updater_ =
        boost::make_shared<PointCloudSubscriptionAdapter<ScanPointT> >(unfiltered_scan_cloud_aggregator_, "/scan_"
                                                                                                          "cloud");

    // stereo_cloud_aggregator_.reset(new PointCloudAggregator<StereoPointT>(tf_listener_, 10));
    // stereo_cloud_updater_.reset(new PointCloudSubscriptionAdapter<StereoPointT>(stereo_cloud_aggregator_,
    // "/multisense_sl/points2_low_rate"));

    vis_timer_ = pnh_in.createTimer(ros::Duration(2.0), &WorldmodelCore::visTimerCallback, this, false);

    if (!p_use_external_octomap_)
    {
      octo_update_timer_ =
          pnh_in.createTimer(ros::Duration(0.1), &WorldmodelCore::octoUpdateTimerCallback, this, false);
    }
    else
    {
      octomap_->setUpdatedFromExternal(true);
      octo_external_update_sub_ =
          pnh_in.subscribe("octomap_external_update", 1, &WorldmodelCore::octomapExternalUpdateCallback, this);
    }

    double periodic_octomap_save_period = pnh_in.param("periodic_octomap_save_period", 0.0);
    octomap_->startPeriodicMapSaving(pnh_in.param("periodic_octomap_save_folder", std::string("")),
                                     ros::Duration(periodic_octomap_save_period));

    communication_.reset(new WorldmodelCommunication(pnh_in, p_root_frame_, octomap_, scan_cloud_aggregator_,
                                                     unfiltered_scan_cloud_aggregator_, stereo_cloud_aggregator_,
                                                     tf_listener_));

    // State provider publishes pose data based on tf. Moved into worldmodel to prevent many dedicated tf subscriber
    // nodes consuming both CPU and bandwidth. Runs it´s own thread

    pnh_in.param("publish_frames_as_poses", p_publish_frames_as_poses_, true);

    if (p_publish_frames_as_poses_)
    {
      state_provider_.reset(new StateProvider());

      state_provider_->addStateRepublisher(boost::shared_ptr<StateRepublisherInterface>(
          new TfPoseRepublisher(tf_listener_, "/robot_pose_odom", p_root_frame_, "base_link")));

      state_provider_->addStateRepublisher(boost::shared_ptr<StateRepublisherInterface>(
          new StabRepublisher(tf_listener_, "base_link", "base_stabilized")));
      /*
       state_provider_->addStateRepublisher(boost::shared_ptr<StateRepublisherInterface>(new TfPoseRepublisher(
                                                                                           tf_listener_,
                                                                                           "/flor/r_arm_current_pose",
                                                                                           p_root_frame_,
                                                                                           "/r_hand"
                                                                                           )));

       state_provider_->addStateRepublisher(boost::shared_ptr<StateRepublisherInterface>(new TfPoseRepublisher(
                                                                                           tf_listener_,
                                                                                           "/flor/l_arm_current_pose",
                                                                                           p_root_frame_,
                                                                                           "/l_hand"
                                                                                         )));
                                                                                         */
      pnh_in.param("publish_frames_rate", p_publish_frames_rate_, 30.0);
      state_provider_->start(p_publish_frames_rate_);
    }

    pnh_in.param("publish_map_pose", p_publish_map_pose_, false);

    if (p_publish_map_pose_)
    {
      map_state_provider_.reset(new StateProvider());

      map_state_provider_->addStateRepublisher(boost::shared_ptr<StateRepublisherInterface>(
          new TfPoseRepublisher(tf_listener_, "/robot_pose", "map", "base_link")));

      map_state_provider_->start(10.0);
    }

    transform_service_provider_.reset(new TransformServiceProvider(tf_listener_));
  }

  ~WorldmodelCore()
  {
  }

  /**
   * This callback optionally publishes visualizations based on a timer
   * Will only consume significant cycles if number of subscribers is larger 0.
   */
  void visTimerCallback(const ros::TimerEvent& event)
  {
    if (octo_marker_vis_.hasSubscribers())
    {
      octo_marker_vis_.publishVis(octomap_->getCurrentMap());
    }

    if (point_cloud_vis_.hasSubscribers())
    {
      ros::WallTime start = ros::WallTime::now();

      pcl::PointCloud<ScanPointT>::Ptr cloud;
      cloud.reset(new pcl::PointCloud<ScanPointT>());
      if (scan_cloud_aggregator_->getAggregateCloud(cloud, p_root_frame_, p_root_frame_, 2000))
      {
        point_cloud_vis_.publishVis(*cloud);
      }

      ROS_DEBUG("Generating and publishing filtered cloud took %f seconds.", (ros::WallTime::now() - start).toSec());
    }

    if (unfiltered_point_cloud_vis_.hasSubscribers())
    {
      ros::WallTime start = ros::WallTime::now();

      pcl::PointCloud<ScanPointT>::Ptr cloud;
      cloud.reset(new pcl::PointCloud<ScanPointT>());
      if (unfiltered_scan_cloud_aggregator_->getAggregateCloud(cloud, p_root_frame_, "", 2000))
      {
        unfiltered_point_cloud_vis_.publishVis(*cloud);
      }

      ROS_DEBUG("Generating and publishing unfiltered cloud took %f seconds.", (ros::WallTime::now() - start).toSec());
    }
  }

  void octoUpdateTimerCallback(const ros::TimerEvent& event)
  {
    // Timing octo_timer;

    pcl::PointCloud<ScanPointT> pc;
    tf::Point origin(0, 0, 0);

    while (scan_cloud_aggregator_->getCloudUpperBound(pc, p_root_frame_, octomap_->getCurrentMap().getLastUpdateStamp(),
                                                      &origin))
    {
      octomap_->insertCloud(origin, pc);
    }
  }

  void octomapExternalUpdateCallback(const octomap_msgs::OctomapConstPtr& msg)
  {
    // if (!octomap_->updateOctomap(*msg))
    //  ROS_WARN("External octomap update failed!");

    octomap_->updateOctomap(*msg);
  }

  void waitForTf(ros::NodeHandle& pnh)
  {
    ros::WallTime start = ros::WallTime::now();
    ROS_INFO("Waiting for tf to become available");

    pnh.param("required_frames", p_required_frames_list_, std::string(""));

    if (p_required_frames_list_.empty())
    {
      ROS_WARN("No list of tf frames to wait for specified! Could lead to transform errors during startup.");
      return;
    }

    boost::algorithm::split(required_frames_list_, p_required_frames_list_, boost::is_any_of("\t "));

    bool transforms_successful = false;

    while (!transforms_successful)
    {
      bool success = true;

      for (size_t i = 0; i < required_frames_list_.size(); ++i)
      {
        success = success && tf_listener_->waitForTransform(p_root_frame_, required_frames_list_[i], ros::Time(0),
                                                            ros::Duration(10.0));
        if (!success)
          ROS_WARN("Worldmodel server waiting for (%s) tf...", required_frames_list_[i].c_str());
      }

      transforms_successful = success;
    }
    ros::WallTime end = ros::WallTime::now();
    ROS_INFO("Finished waiting for tf, waited %f seconds", (end - start).toSec());
  }

protected:
  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;

  boost::shared_ptr<tf::TransformListener> tf_listener_;

  boost::shared_ptr<WorldmodelOctomap> octomap_;

  boost::shared_ptr<PointCloudAggregator<ScanPointT> > scan_cloud_aggregator_;
  boost::shared_ptr<PointCloudSubscriptionAdapter<ScanPointT> > scan_cloud_updater_;

  boost::shared_ptr<PointCloudAggregator<ScanPointT> > unfiltered_scan_cloud_aggregator_;
  boost::shared_ptr<PointCloudSubscriptionAdapter<ScanPointT> > unfiltered_scan_cloud_updater_;

  boost::shared_ptr<PointCloudAggregator<StereoPointT> > stereo_cloud_aggregator_;
  boost::shared_ptr<PointCloudSubscriptionAdapter<StereoPointT> > stereo_cloud_updater_;

  boost::shared_ptr<WorldmodelCommunication> communication_;

  boost::shared_ptr<StateProvider> state_provider_;
  boost::shared_ptr<StateProvider> map_state_provider_;

  boost::shared_ptr<TransformServiceProvider> transform_service_provider_;

  // Visualizers (for debugging during development)
  OctomapVisualization octo_marker_vis_;
  PointCloudVisualization point_cloud_vis_;
  PointCloudVisualization unfiltered_point_cloud_vis_;

  std::string p_root_frame_;
  std::string p_required_frames_list_;

  bool p_publish_frames_as_poses_;
  bool p_publish_map_pose_;
  double p_publish_frames_rate_;

  std::vector<std::string> required_frames_list_;
  bool p_use_external_octomap_;
  double p_octomap_max_range_;

  ros::Timer vis_timer_;
  ros::Timer octo_update_timer_;

  ros::Subscriber octo_external_update_sub_;
};

}  // namespace vigir_worldmodel

#endif
