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

#ifndef TF_STAB_REPUBLISHER_H__
#define TF_STAB_REPUBLISHER_H__

#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"

#include <vigir_worldmodel_server/state/state_republisher_interface.h>


#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"


#include <tf/tf.h>

#include <algorithm>

using namespace std;

//bool comparePoseStampedStamps (const geometry_msgs::PoseStamped& t1, const geometry_msgs::PoseStamped& t2) { return (t1.header.stamp < t2.header.stamp); }

class StabRepublisher : public StateRepublisherInterface
{
public:
  StabRepublisher(const boost::shared_ptr<tf::TransformListener>& tf_listener,
                    const std::string& base_frame_name,
                    const std::string& stabilized_frame_name)
    : tf_listener_(tf_listener)
    , p_base_frame_name_(base_frame_name)
    , p_stabilized_frame_name_(stabilized_frame_name)
  {
    ros::NodeHandle nh;


    //pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(pose_pub_topic, 1, false);

    //pose_source_.header.frame_id = p_source_frame_name_;
    //pose_source_.pose.orientation.w = 1.0;
  }


  void execute(const ros::Time& time)
  {

    // Always get latest transform
    pose_source_.header.stamp = time;

    geometry_msgs::PoseStamped pose_out;
    try {
        //tf_listener_->transformPose(p_target_frame_name_, pose_source_, pose_out);
        tf::Transform base_transform;


        tf::StampedTransform robot_world_transform;

        try{
            tf_listener_->lookupTransform(p_base_frame_name_, "/world", ros::Time(0), robot_world_transform);
        }catch(tf::TransformException& ex){
            ROS_ERROR_STREAM( "Transform failed " << ex.what());
            return;
        }

        double roll, pitch, yaw;
        robot_world_transform.getBasis().getRPY(roll, pitch, yaw);

        tf::StampedTransform stab_world_transform = robot_world_transform;

        tf::Quaternion stab_quat;
        stab_quat.setRPY(0, 0, yaw);
        stab_world_transform.setRotation(stab_quat);

        tf::StampedTransform stab_base_link_transform;

        stab_base_link_transform.setData(stab_world_transform.inverse() * robot_world_transform);




        //stab_base_link_transform = robot_world_transform;
        stab_base_link_transform.stamp_ = robot_world_transform.stamp_;

        stab_base_link_transform.child_frame_id_ = p_stabilized_frame_name_;
        stab_base_link_transform.frame_id_ = p_base_frame_name_;

        //tf::Quaternion rot;

        //rot.setRPY(roll, pitch, 0);

        //stab_base_link_transform.setRotation(rot);

        //stab_base_link_transform = transform_diff;

        tfb_.sendTransform(stab_base_link_transform);


        stab_world_transform.child_frame_id_ = "stab_world";
        stab_world_transform.frame_id_ = "world";
        stab_world_transform.stamp_ = robot_world_transform.stamp_;

        tfb_.sendTransform(stab_world_transform);





        //tfb_.sendTransform();
        
        //tf_listener_->transformPose(p_target_frame_name_, pose_source_, pose_out);
        //ROS_INFO(" source pose (%s): %f, %f, %f ---> target pose (%s): %f, %f, %f",
        //         pose_source_.header.frame_id.c_str(), pose_source_.pose.position.x, pose_source_.pose.position.y, pose_source_.pose.position.z,
        //         pose_out.header.frame_id.c_str(),pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z );
        pose_pub_.publish(pose_out);
    }
    catch (tf::TransformException ex){
         ROS_ERROR("Pose republisher tf failure::  %s",ex.what());
    }
  }

  boost::shared_ptr<tf::TransformListener> tf_listener_;

  //parameters
  std::string p_base_frame_name_;
  std::string p_stabilized_frame_name_;

  // Helper pose centered at the origin that gets transformed to source frame
  geometry_msgs::PoseStamped pose_source_;

  ros::Publisher pose_pub_;
  tf::TransformBroadcaster tfb_;

};

#endif

