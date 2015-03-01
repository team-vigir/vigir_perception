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

#ifndef WORLDMODEL_OCTOMAP__
#define WORLDMODEL_OCTOMAP__

#include <ros/ros.h>
#include <tf/tf.h>
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <pcl/ros/conversions.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <vigir_worldmodel_main/core/worldmodel_cloud_types.h>
#include <vigir_worldmodel_main/octomap/octomap_container.h>

#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>

namespace vigir_worldmodel{

  class WorldmodelOctomap{
  public:
    //typedef octomap_msgs::BoundingBoxQuery BBXSrv;

    WorldmodelOctomap(const std::string& frame_id, double res = 0.05, double max_range = 10.0)
    : map(frame_id, res, max_range)
    {
      map.setLastUpdateStamp(ros::Time(0));
    }

    ~WorldmodelOctomap()
    {}

    void reset()
    {
      map.reset();
    }

    bool updateOctomap(const octomap_msgs::Octomap& msg){
      octomap::AbstractOcTree* incoming_octomap = octomap_msgs::msgToMap(msg);

      if (!incoming_octomap)
        return false;

      map.update(incoming_octomap);
      map.setLastUpdateStamp(msg.header.stamp);

      return true;
    }

    void insertCloud(const tf::Point& sensorOriginTf, const pcl::PointCloud<ScanPointT>& cloud)
    {

      if (cloud.header.frame_id != map.getFrameId()){
        ROS_ERROR("Wrong frame_id, can't insert point cloud into octomap!");
        return;
      }
      octomap::KeySet free_cells, occupied_cells;

      octomap::point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

      octomap::OcTree* m_octree = map.getOcTree();

      /*
      // Don't need bbx check at this point
      if (!m_octree->coordToKeyChecked(sensorOrigin, map.m_updateBBXMin)
      || !m_octree->coordToKeyChecked(sensorOrigin, map.m_updateBBXMax))
      {
        ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
      }
      */

      octomap::OcTreeKey test_key;
      if (!m_octree->coordToKeyChecked(sensorOrigin, test_key))
      {
        ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
        return;
      }

      //Reset tmp storage
      m_keyRay.reset();

      ros::Time time;
      time.fromNSec(cloud.header.stamp*1e3);
      map.setLastUpdateStamp(time);

      for (pcl::PointCloud<ScanPointT>::const_iterator it = cloud.begin(); it != cloud.end(); ++it){
        octomap::point3d point(it->x, it->y, it->z);
        // maxrange check

        double maxRange = map.getMaxRange();

        if (((point - sensorOrigin).norm() <= maxRange) ) {

          // free cells
          if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
            free_cells.insert(m_keyRay.begin(), m_keyRay.end());
          }
          // occupied endpoint
          octomap::OcTreeKey key;
          if (m_octree->coordToKeyChecked(point, key)){
            occupied_cells.insert(key);

            // Again, don't need bbx update at this point
            //map.updateMinKey(key);
            //map.updateMaxKey(key);
          }
        }
        /*
      // Not sure we need longer than maxrange rays at the moment.
      } else {// ray longer than maxrange:;
        point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
        if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
          free_cells.insert(m_keyRay.begin(), m_keyRay.end());

          octomap::OcTreeKey endKey;
          if (m_octree->coordToKeyChecked(new_end, endKey)){
            updateMinKey(endKey, m_updateBBXMin);
            updateMaxKey(endKey, m_updateBBXMax);
          } else{
            ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
          }


        }
      }
      */
      }
      // mark free cells only if not seen occupied in this cloud
      for(octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
        if (occupied_cells.find(*it) == occupied_cells.end()){
          m_octree->updateNode(*it, false);
        }
      }

      // now mark all occupied cells:
      for (octomap::KeySet::iterator it = occupied_cells.begin(), end=free_cells.end(); it!= end; it++) {
        m_octree->updateNode(*it, true);
      }
    }

    /**double res = 0.05
     * Implemented as suggested here:
     * https://groups.google.com/forum/#!topic/octomap/QLW43yC9BX4
     */
    void getBbxFilteredOctomap(boost::shared_ptr<octomap::OcTree>& octree, const std::string& frame_id, const geometry_msgs::Point& bbx_min, const geometry_msgs::Point& bbx_max)
    {
      octomap::OcTree* m_octree = map.getOcTree();

      if (m_octree->size() < 100){
        ROS_WARN("Octree has only %d nodes, not generating occupancy grid map for it.", static_cast<int>(m_octree->size()));
        return;
      }

      octomap::point3d min = octomap::pointMsgToOctomap(bbx_min);
      octomap::point3d max = octomap::pointMsgToOctomap(bbx_max);

      for( octomap::OcTree::leaf_bbx_iterator it = m_octree->begin_leafs_bbx(min,max),
      end=m_octree->end_leafs_bbx(); it!= end; ++it)
      {
        octree->updateNode(it.getKey(), it->getValue(), true);
      }

      octree->updateInnerOccupancy();
    }

    const OctomapContainer& getCurrentMap() const { return map; };
    //void setCurrentMap() const { return map; };
    //OctomapContainer& getCurrentMap() { return map; };


    bool getOccupancyGridmap(const octomap::OcTree* m_octree,
                             nav_msgs::OccupancyGrid& grid_map,
                             double min_height,
                             double max_height,
                             unsigned int tree_depth = 16,
                             Eigen::Vector2d* min_trans = 0,
                             Eigen::Vector2d* max_trans = 0)
    {
      if (m_octree->size() < 100){
        ROS_WARN("Octree has only %d nodes, not generating occupancy grid map for it.", static_cast<int>(m_octree->size()));
        return false;
      }

      unsigned int m_treeDepth = tree_depth;
      unsigned int m_maxTreeDepth = m_octree->getTreeDepth();

      grid_map.header.frame_id = map.getFrameId();
      grid_map.header.stamp = ros::Time::now();
      //nav_msgs::MapMetaData oldMapInfo = m_gridmap.info;

      // TODO: move most of this stuff into c'tor and init map only once (adjust if size changes)
      double minX, minY, minZ, maxX, maxY, maxZ;
      m_octree->getMetricMin(minX, minY, minZ);
      m_octree->getMetricMax(maxX, maxY, maxZ);

      // Our min/max must be within octomap min/max. Check that here.
      if ( (min_trans != 0) && (max_trans != 0) ){

        if ((max_trans->x() <= minX) || (min_trans->x() >= maxX) ||
            (max_trans->y() <= minY) || (min_trans->y() >= maxY) ){
          ROS_WARN("Requested occupancy grid map region is outside map bounds, not returning any map");
          return false;
        }


        //X
        if ( (min_trans->x() > minX) && (min_trans->x() < maxX)){
          minX = min_trans->x();
        }

        if ( (max_trans->x() > minX) && (max_trans->x() < maxX)){
          maxX = max_trans->x();
        }
\
        //Y
        if ( (min_trans->y() > minY) && (min_trans->y() < maxY)){
          minY = min_trans->y();
        }

        if ( (max_trans->y() > minY) && (max_trans->y() < maxY)){
          maxY = max_trans->y();
        }
      }

      octomap::point3d minPt(minX, minY, minZ);
      octomap::point3d maxPt(maxX, maxY, maxZ);
      octomap::OcTreeKey minKeyNoPad = m_octree->coordToKey(minPt, tree_depth);
      octomap::OcTreeKey maxKeyNoPad = m_octree->coordToKey(maxPt, tree_depth);


      ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKeyNoPad[0], minKeyNoPad[1], minKeyNoPad[2], maxKeyNoPad[0], maxKeyNoPad[1], maxKeyNoPad[2]);

      octomap::OcTreeKey paddedMaxKey;

      m_paddedMinKey = minKeyNoPad;
      paddedMaxKey = maxKeyNoPad;

      /*
      if (!m_octree->coordToKeyChecked(minPt, m_octree->getTreeDepth(), m_paddedMinKey)){
        ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
        return false;
      }
      if (!m_octree->coordToKeyChecked(maxPt, m_octree->getTreeDepth(), paddedMaxKey)){
        ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
        return false;
      }
      */

      ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", m_paddedMinKey[0], m_paddedMinKey[1], m_paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
      assert(paddedMaxKey[0] >= maxKeyNoPad[0] && paddedMaxKey[1] >= maxKeyNoPad[1]);

      //Want to have same grid map size as portion of octomap we're looking at
      m_multires2DScale = 1; //<< (m_treeDepth - m_maxTreeDepth);
      grid_map.info.width = (paddedMaxKey[0] - m_paddedMinKey[0])/m_multires2DScale +1;
      grid_map.info.height = (paddedMaxKey[1] - m_paddedMinKey[1])/m_multires2DScale +1;

      int mapOriginX = minKeyNoPad[0] - m_paddedMinKey[0];
      int mapOriginY = minKeyNoPad[1] - m_paddedMinKey[1];
      assert(mapOriginX >= 0 && mapOriginY >= 0);

      // might not exactly be min / max of octree:
      octomap::point3d origin = m_octree->keyToCoord(m_paddedMinKey, tree_depth);
      double gridRes = m_octree->getNodeSize(tree_depth);
      //m_projectCompleteMap = (!m_incrementalUpdate || (std::abs(gridRes-grid_map.info.resolution) > 1e-6));
      grid_map.info.resolution = gridRes;
      grid_map.info.origin.position.x = origin.x() - gridRes*0.5;
      grid_map.info.origin.position.y = origin.y() - gridRes*0.5;
      if (m_maxTreeDepth != m_treeDepth){
        grid_map.info.origin.position.x -= m_octree->getResolution()/2.0;
        grid_map.info.origin.position.y -= m_octree->getResolution()/2.0;
      }

      // This is used in the mapIdx call below, so have to set it here.
      grid_map_width_ = grid_map.info.width;

      grid_map.data.clear();
      grid_map.data.resize(grid_map.info.width * grid_map.info.height, -1);

      // Need to use bbx leaf iterator here, as otherwise keyindex will be outside grid map if we use ROI
      for( octomap::OcTree::leaf_bbx_iterator it = m_octree->begin_leafs_bbx(m_paddedMinKey, paddedMaxKey),
           end=m_octree->end_leafs_bbx(); it!= end; ++it)
      {
        double z_val = it.getZ();

        if ((z_val > min_height) && (z_val < max_height)){

          bool occupied = m_octree->isNodeOccupied(*it);

          if (it.getDepth() == m_octree->getTreeDepth()){
            unsigned idx = mapIdx(it.getKey());
            if (occupied)
              grid_map.data[idx] = 100;
            else if (grid_map.data[idx] == -1){
              grid_map.data[idx] = 0;
            }

          } else{

            int intSize = 1 << (m_octree->getTreeDepth() - it.getDepth());
            octomap::OcTreeKey minKey=it.getIndexKey();
            for(int dx=0; dx < intSize; dx++){
              int i = (minKey[0]+dx - m_paddedMinKey[0])/m_multires2DScale;
              if (i>= 0){
                for(int dy=0; dy < intSize; dy++){
                  int j = (minKey[1]+dy - m_paddedMinKey[1]);

                  //Not checking for this suddenly gave errors due to wrap around of negative
                  //values to giant unsigned idx. @TODO: Find out cause
                  if (j >=0){
                    unsigned idx = mapIdx(i, j/m_multires2DScale);
                    if (occupied)
                      grid_map.data[idx] = 100;
                    else if (grid_map.data[idx] == -1){
                      grid_map.data[idx] = 0;
                    }
                  }
                }
              }
            }

          }
        }

      }

      return true;

    }

    bool castRay(const octomap::point3d& origin, const octomap::point3d& direction, octomap::point3d& end_point)
    {
      octomap::OcTree* m_octree = map.getOcTree();

      return m_octree->castRay(origin, direction, end_point,true, 600.0);
    }

    bool getOccupancyGridmap(nav_msgs::OccupancyGrid& grid_map, double min_height, double max_height)
    {
      return this->getOccupancyGridmap(map.getOcTree(), grid_map, min_height, max_height);

    }

    inline unsigned mapIdx(int i, int j) const{
      return grid_map_width_*j + i;
    }

    inline unsigned mapIdx(const octomap::OcTreeKey& key) const{
      return mapIdx((key[0] - m_paddedMinKey[0])/m_multires2DScale,
      (key[1] - m_paddedMinKey[1])/m_multires2DScale);

    }

    void writeOctomapToFile(const std::string& file_name) const
    {
      ROS_INFO("Writing binary octree to file %s", file_name.c_str());

      const octomap::OcTree* m_octree = map.getOcTree();
      m_octree->writeBinaryConst(file_name);
    }

  protected:

    octomap::KeyRay m_keyRay;  // temp storage for ray casting

    OctomapContainer map;

    // Below is used for writing to Occupancy Grid
    unsigned m_multires2DScale;
    octomap::OcTreeKey m_paddedMinKey;    
    unsigned int grid_map_width_;
  };

}

#endif
