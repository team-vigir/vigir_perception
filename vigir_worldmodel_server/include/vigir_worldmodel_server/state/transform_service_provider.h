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

#ifndef TRANSFORM_SERVICE_PROVIDER_H__
#define TRANSFORM_SERVICE_PROVIDER_H__

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <vigir_perception_msgs/GetPoseInFrame.h>



namespace vigir_worldmodel{

  class TransformServiceProvider
  {
  public:
    TransformServiceProvider(const boost::shared_ptr<tf::TransformListener>& tf_listener)
      : tf_listener_(tf_listener)
    {
      nh_.reset(new ros::NodeHandle("~"));
      nh_->setCallbackQueue(&callback_queue_);
      this->loop_thread_.reset(new boost::thread( boost::bind( &TransformServiceProvider::serviceCallbackQueue,this ) ));
      
      transform_lookup_service_ = nh_->advertiseService("get_pose_in_frame_service", &TransformServiceProvider::getPoseInFrameServiceCb, this);

    }

    ~TransformServiceProvider()
    {}

    void serviceCallbackQueue()
    {
      static const double timeout = 0.003;

      while (this->nh_->ok()){
        this->callback_queue_.callAvailable(ros::WallDuration(timeout));
      }
    }

    bool getPoseInFrameServiceCb(vigir_perception_msgs::GetPoseInFrame::Request  &req,
                                 vigir_perception_msgs::GetPoseInFrame::Response &res )
    {

      tf::Stamped<tf::Pose> tf_pose_req;

      tf::poseStampedMsgToTF(req.pose, tf_pose_req);

      if (tf_listener_->waitForTransform(req.target_frame,
                                         req.pose.header.frame_id,
                                         req.pose.header.stamp,
                                         ros::Duration(0.5)))
      {
        tf::Stamped<tf::Pose> tf_pose_target;
        tf_listener_->transformPose(req.target_frame, tf_pose_req, tf_pose_target);

        tf::poseStampedTFToMsg(tf_pose_target, res.transformed_pose);

        return true;

      }else{
        ROS_WARN("Timed out waiting for transform from %s to %s",
                 req.pose.header.frame_id.c_str(),
                 req.target_frame.c_str());
        return false;
      }
    }

  protected:
    boost::shared_ptr<ros::NodeHandle> nh_;
    boost::shared_ptr<boost::thread> loop_thread_;
    ros::CallbackQueue callback_queue_;

    ros::ServiceServer transform_lookup_service_;

    boost::shared_ptr<tf::TransformListener> tf_listener_;
  };

}

#endif
