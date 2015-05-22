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

#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/common/time.h>

#include <pcl/filters/voxel_grid.h>



#include <pcl/features/normal_3d.h>

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
    //surface_reconstruction_.reset(new pcl::MarchingCubesRBF<PointTNormal>());

    org_fast_mesh_.reset(new pcl::OrganizedFastMesh<PointT>());
    org_fast_mesh_->setTriangulationType(pcl::OrganizedFastMesh<PointT>::TRIANGLE_ADAPTIVE_CUT);

    voxel_size_ = 0.05;
  }

  bool setInput( const boost::shared_ptr<pcl::PointCloud<PointT> > pc_in){
    pc_ = pc_in;
  }

  void setVoxelFilterSize(double size){
    voxel_size_ = size;
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




    std::cout << "Total Reconstruction finished in " << stop_watch_.getTimeSeconds() << " s.";

    return true;

    //pcl::io::savePLYFile("/home/kohlbrecher/poly.ply", mesh_);


  }

  const pcl::PolygonMesh& getMesh() const { return mesh_; };

private:
  //sensor_msgs::LaserScan scan_;
  //laser_geometry::LaserProjection laser_proj_;
  //tf::Transformer tf_transformer_;
  boost::shared_ptr<pcl::PointCloud<PointT> > pc_;

  //boost::shared_ptr<pcl::SurfaceReconstruction<pcl::PointNormal> > surface_reconstruction_;
  boost::shared_ptr<pcl::OrganizedFastMesh<PointT> > org_fast_mesh_;

  pcl::PolygonMesh mesh_;

  pcl::StopWatch stop_watch_;

  double voxel_size_;



};

}
#endif
