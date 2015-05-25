//=================================================================================================
// Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt
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

#ifndef VIGIR_DEPTH_IMAGE_TO_MESH_H_
#define VIGIR_DEPTH_IMAGE_TO_MESH_H_



//#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>

#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/common/time.h>

#include <pcl/common/transforms.h>

#include <pcl/features/normal_3d.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/ply_io.h>

namespace vigir_point_cloud_proc
{

/**
 * @brief The FilteredLocalizedScanConversion class provides
 * conversion functionality for converting a
 * FilteredLocalizedLaserScan messages to point cloud
 * representations.
 */
template <typename PointT>
class DepthImageToMesh
{
public:

  DepthImageToMesh()
  {    
    org_fast_mesh_.reset(new pcl::OrganizedFastMesh<PointT>());
    org_fast_mesh_->setTriangulationType(pcl::OrganizedFastMesh<PointT>::TRIANGLE_ADAPTIVE_CUT);
  }

  bool setInput( const boost::shared_ptr<pcl::PointCloud<PointT> > pc_in){
    pc_ = pc_in;
  }

  bool computeMesh()
  {

    stop_watch_.reset();

    if (!pc_.get())
      return false;

    if (!(pc_->size() > 200))
      return false;

    org_fast_mesh_->setInputCloud(pc_);

    org_fast_mesh_->reconstruct(mesh_);

    //std::cout << "Total Reconstruction finished in " << stop_watch_.getTimeSeconds() << " s.";

    return true;
  }

  const pcl::PolygonMesh& getMesh() const { return mesh_; };

  const pcl::PolygonMeshConstPtr getMeshTransformed(const Eigen::Affine3d& transform) const
  {
    pcl::PolygonMeshPtr transformed_mesh(new pcl::PolygonMesh());

    //Polys can be copied directly
    transformed_mesh->polygons = mesh_.polygons;

    //Transform vertices represented by point cloud
    pcl::PointCloud<PointT> cloud;
    pcl::fromPCLPointCloud2(mesh_.cloud, cloud);

    pcl::PointCloud<PointT> cloud_transformed;
    pcl::transformPointCloud(cloud, cloud_transformed, transform);

    pcl::toPCLPointCloud2(cloud_transformed, transformed_mesh->cloud);

    return transformed_mesh;
  }

private:

  boost::shared_ptr<pcl::PointCloud<PointT> > pc_;

  boost::shared_ptr<pcl::OrganizedFastMesh<PointT> > org_fast_mesh_;

  pcl::PolygonMesh mesh_;

  pcl::StopWatch stop_watch_;

};

}
#endif
