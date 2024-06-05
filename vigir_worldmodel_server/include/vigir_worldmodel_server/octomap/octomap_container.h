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

#ifndef OCTOMAP_CONTAINER_H__
#define OCTOMAP_CONTAINER_H__

#include <octomap/octomap.h>

namespace vigir_worldmodel
{
/**
 * Contains a octomap plus relevant properties (like bounding box, resolution, hit/miss probabilities etc)
 */
class OctomapContainer
{
public:
  OctomapContainer(const std::string& frame_id, double res = 0.05, double max_range = 10.0, bool local_mapping = false)
    : m_res(res)
    , m_maxRange(max_range)
    , m_maxRange_sq(max_range * max_range)
    , m_local_mapping(local_mapping)
    , m_frame_id(frame_id)
  {
    /*
    m_probHit = 0.7;
    m_probMiss = 0.4;
    m_thresMin = 0.12;
    m_thresMax = 0.97;
    */

    m_octree.reset(new octomap::OcTree(m_res));

    m_treeDepth = m_octree->getTreeDepth();

    /*
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    m_octree->getMetricMin(minX, minY, minZ);
    m_octree->getMetricMax(maxX, maxY, maxZ);

    m_updateBBXMin[0] =  m_octree->coordToKey(minX);
    m_updateBBXMin[1] =  m_octree->coordToKey(minY);
    m_updateBBXMin[2] =  m_octree->coordToKey(minZ);

    m_updateBBXMax[0] =  m_octree->coordToKey(maxX);
    m_updateBBXMax[1] =  m_octree->coordToKey(maxY);
    m_updateBBXMax[2] =  m_octree->coordToKey(maxZ);
    */
  }

  void reset()
  {
    m_octree->clear();
    last_update_timestamp = ros::Time::now();
  }

  void update(octomap::AbstractOcTree* incoming_octomap)
  {
    m_octree.reset(static_cast<octomap::OcTree*>(incoming_octomap));
  }

  /*
  inline void updateMinKey(const octomap::OcTreeKey& in){
    for (unsigned i=0; i<3; ++i)
      m_updateBBXMin[i] = std::min(in[i], m_updateBBXMin[i]);
  };

  inline void updateMaxKey(const octomap::OcTreeKey& in){
    for (unsigned i=0; i<3; ++i)
      m_updateBBXMax[i] = std::max(in[i], m_updateBBXMax[i]);
  };
  */

  octomap::OcTree* getOcTree()
  {
    return m_octree.get();
  };

  const octomap::OcTree* getOcTree() const
  {
    return m_octree.get();
  };

  double getMaxRange() const
  {
    return m_maxRange;
  };

  double getMaxRangeSq() const
  {
    return m_maxRange_sq;
  };

  unsigned getMaxTreeDepth() const
  {
    return m_treeDepth;
  };

  bool isLocalMapping() const
  {
    return m_local_mapping;
  };

  const std::string getFrameId() const
  {
    return m_frame_id;
  };

  const ros::Time& getLastUpdateStamp() const
  {
    return last_update_timestamp;
  };

  void setLastUpdateStamp(const ros::Time& stamp)
  {
    last_update_timestamp = stamp;
  };

  // octomap::OcTreeKey m_updateBBXMin;
  // octomap::OcTreeKey m_updateBBXMax;

protected:
  boost::shared_ptr<octomap::OcTree> m_octree;

  // boost::shared_ptr<octomap::OcTree> om(static_cast<octomap::OcTree*>(octomap_msgs::msgToMap(map)));

  double m_res;
  /*
  double m_probHit;
  double m_probMiss;
  double m_thresMin;
  double m_thresMax;
  */
  unsigned m_treeDepth;

  double m_maxRange;
  double m_maxRange_sq;

  bool m_local_mapping;

  ros::Time last_update_timestamp;
  std::string m_frame_id;
};

}  // namespace vigir_worldmodel

#endif
