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

#ifndef OCTOMAP_VISUALIZATION_H__
#define OCTOMAP_VISUALIZATION_H__

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <pcl/conversions.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <vigir_worldmodel_server/octomap/octomap_container.h>

namespace vigir_worldmodel
{
class OctomapVisualization
{
public:
  OctomapVisualization(ros::NodeHandle& nh)
  {
    m_markerPub = nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, false);

    m_colorFactor = 0.8;

    m_color.r = 0;
    m_color.g = 0;
    m_color.b = 1;
    m_color.a = 1;
  }

  ~OctomapVisualization()
  {
  }

  std_msgs::ColorRGBA heightMapColor(double h)
  {
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    // blend over HSV-values (more colors)

    double s = 1.0;
    double v = 1.0;

    h -= floor(h);
    h *= 6;
    int i;
    double m, n, f;

    i = floor(h);
    f = h - i;
    if (!(i & 1))
      f = 1 - f;  // if i is even
    m = v * (1 - s);
    n = v * (1 - s * f);

    switch (i)
    {
      case 6:
      case 0:
        color.r = v;
        color.g = n;
        color.b = m;
        break;
      case 1:
        color.r = n;
        color.g = v;
        color.b = m;
        break;
      case 2:
        color.r = m;
        color.g = v;
        color.b = n;
        break;
      case 3:
        color.r = m;
        color.g = n;
        color.b = v;
        break;
      case 4:
        color.r = n;
        color.g = m;
        color.b = v;
        break;
      case 5:
        color.r = v;
        color.g = m;
        color.b = n;
        break;
      default:
        color.r = 1;
        color.g = 0.5;
        color.b = 0.5;
        break;
    }

    return color;
  }

  void publishVis(const OctomapContainer& map)
  {
    // Nothing to do if no one's interested in visualization
    if (m_markerPub.getNumSubscribers() == 0)
    {
      return;
    }

    const octomap::OcTree* m_octree = map.getOcTree();

    // init markers:
    visualization_msgs::MarkerArray occupiedNodesVis;
    // each array stores all cubes of a different size, one for each depth level:
    occupiedNodesVis.markers.resize(map.getMaxTreeDepth() + 1);

    unsigned m_maxTreeDepth = map.getMaxTreeDepth();

    double minX, minY, minZ, maxX, maxY, maxZ;
    m_octree->getMetricMin(minX, minY, minZ);
    m_octree->getMetricMax(maxX, maxY, maxZ);

    // now, traverse all leafs in the tree:
    for (octomap::OcTree::iterator it = m_octree->begin(m_maxTreeDepth), end = m_octree->end(); it != end; ++it)
    {
      // bool inUpdateBBX = isInUpdateBBX(it.getKey());

      // call general hook:
      // handleNode(it);
      // if (inUpdateBBX)
      //  handleNodeInBBX(it);

      if (m_octree->isNodeOccupied(*it))
      {
        double z = it.getZ();
        // if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
        {
          // double size = it.getSize();
          double x = it.getX();
          double y = it.getY();

          // Ignore speckles in the map:
          /*
          if (m_filterSpeckles && (it.getDepth() == m_treeDepth +1) && isSpeckleNode(it.getKey())){
            ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
            continue;
          } // else: current octree node is no speckle, send it out
          */
          // handleOccupiedNode(it);
          // if (inUpdateBBX)
          //  handleOccupiedNodeInBBX(it);

          // create marker:
          // if (publishMarkerArray){
          {
            unsigned idx = it.getDepth();
            assert(idx < occupiedNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
            // if (m_useHeightMap){
            {
              double h = (1.0 - std::min(std::max((cubeCenter.z - minZ) / (maxZ - minZ), 0.0), 1.0)) * m_colorFactor;
              occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
            }
          }
        }
      }
    }

    for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i)
    {
      double size = m_octree->getNodeSize(i);

      occupiedNodesVis.markers[i].header.frame_id = "world";
      occupiedNodesVis.markers[i].header.stamp = ros::Time::now();
      occupiedNodesVis.markers[i].ns = "map";
      occupiedNodesVis.markers[i].id = i;
      occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      occupiedNodesVis.markers[i].scale.x = size;
      occupiedNodesVis.markers[i].scale.y = size;
      occupiedNodesVis.markers[i].scale.z = size;
      occupiedNodesVis.markers[i].color = m_color;
      occupiedNodesVis.markers[i].frame_locked = true;

      if (occupiedNodesVis.markers[i].points.size() > 0)
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    m_markerPub.publish(occupiedNodesVis);
  }

  bool hasSubscribers() const
  {
    return (m_markerPub.getNumSubscribers() > 0);
  };

protected:
  ros::Publisher m_markerPub;

  double m_colorFactor;
  std_msgs::ColorRGBA m_color;

  // OctomapContainer map;
};
}  // namespace vigir_worldmodel

#endif
