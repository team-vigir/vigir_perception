//=================================================================================================
// Copyright (c) 2024, Stefan Kohlbrecher, Energy Robotics GmbH
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, Energy Robotics GmbH nor the names of its contributors may be used to
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


#include <vigir_worldmodel_server/core/worldmodel_core.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace vigir_worldmodel {

class WorldmodelNodelet : public nodelet::Nodelet
{
private:
   vigir_worldmodel::WorldmodelCore *impl;

public:
  WorldmodelNodelet() : impl(0) {}
  virtual ~WorldmodelNodelet() { delete impl; }

private:
  void onInit() {
    impl = new vigir_worldmodel::WorldmodelCore(getNodeHandle(), getPrivateNodeHandle());
  }
};

} // namespace hector_qrcode_detection

PLUGINLIB_EXPORT_CLASS(vigir_worldmodel::WorldmodelNodelet, nodelet::Nodelet)
