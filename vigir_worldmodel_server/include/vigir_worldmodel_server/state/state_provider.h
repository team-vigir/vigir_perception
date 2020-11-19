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

#ifndef STATE_PROVIDER_H__
#define STATE_PROVIDER_H__

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <vigir_worldmodel_server/state/state_republisher_interface.h>

namespace vigir_worldmodel{

  class StateProvider
  {
  public:
    StateProvider()
    {}

    ~StateProvider()
    {}

    void addStateRepublisher(boost::shared_ptr<StateRepublisherInterface> republisher)
    {
      republishers_.push_back(republisher);
    }

    void start(double loop_rate = 30.0)
    {
      loop_thread_.reset(new boost::thread(boost::bind(&StateProvider::loopFunction, this, loop_rate)));
    }

    void loopFunction(double loop_rate)
    {
      ros::Rate r(loop_rate);

      while(ros::ok())
      {
        size_t size = republishers_.size();
        ros::Time time = ros::Time::now();

        for (size_t i = 0; i < size; ++i){
          republishers_[i]->execute(time);
        }
     
        r.sleep();
      }
    }

  protected:

    boost::shared_ptr<boost::thread> loop_thread_;

    std::vector<boost::shared_ptr<StateRepublisherInterface> > republishers_;
  };

}

#endif
