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

#ifndef VIGIR_CLOUD_TO_MESH_H_
#define VIGIR_CLOUD_TO_MESH_H_



//#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>

#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/marching_cubes_hoppe.h>
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
template <typename PointT, typename PointTNormal>
class CloudToMesh
{
public:

  CloudToMesh()
  {
    surface_reconstruction_.reset(new pcl::MarchingCubesRBF<PointTNormal>());

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

    boost::shared_ptr<pcl::PointCloud<PointTNormal> > point_cloud_normal_voxelized (new pcl::PointCloud<PointTNormal> ());

    /*
    pcl::NormalEstimation<PointT, PointTNormal> norm_est;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    boost::shared_ptr<pcl::search::KdTree<PointT> > tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (pc_);
    norm_est.setInputCloud (pc_);
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (100);
    norm_est.compute (*point_cloud_normal_voxelized);
    */

    boost::shared_ptr<pcl::PointCloud<PointT> > pc_voxelized (new pcl::PointCloud<PointT> ());
    pcl::VoxelGrid<PointT> pre_filter;
    pre_filter.setInputCloud(pc_);
    pre_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    pre_filter.filter(*pc_voxelized);

    double time_prefilter = stop_watch_.getTimeSeconds();
    //std::cout << "Prefilter finished in " << time_prefilter << " s.\n";


    pcl::MovingLeastSquares<PointT, PointTNormal> mls;

    //mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointTNormal>::VOXEL_GRID_DILATION);
    //mls.setDilationVoxelSize(0.025);
    //mls.setDilationIterations(1);

    mls.setSearchRadius(0.05);
    mls.setPolynomialOrder(1);
    mls.setComputeNormals(true);
    mls.setInputCloud(pc_voxelized);

    boost::shared_ptr<pcl::PointCloud<PointTNormal> > point_cloud_mls_normal;
    point_cloud_mls_normal.reset(new pcl::PointCloud<PointTNormal>);
    //std::cout << "start mls\n";
    mls.process(*point_cloud_mls_normal);

    double time_mls = stop_watch_.getTimeSeconds();
    //std::cout << "MLS finished in " << time_mls - time_prefilter << " s.\n";


    //std::cout << "start voxel\n";
    pcl::VoxelGrid<PointTNormal> filter;
    filter.setInputCloud(point_cloud_mls_normal);
    filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    filter.filter(*point_cloud_normal_voxelized);

    double time_postfilter = stop_watch_.getTimeSeconds();
    //std::cout << "Postfilter finished in " << time_postfilter - time_mls << " s.\n";



    //pcl::copyPointCloud (*pc_, *point_cloud_normal_voxelized);
    //pointcloud.reset();

    // Create the search method
    //std::cout << "start kdtree\n";
    boost::shared_ptr<pcl::search::KdTree<PointTNormal> > tree2 (new pcl::search::KdTree<PointTNormal>);
    tree2->setInputCloud (point_cloud_normal_voxelized);

    double time_kdtree = stop_watch_.getTimeSeconds();
    //std::cout << "KdTree finished in " << time_kdtree - time_postfilter << " s.\n";
    // Initialize objects


    /*
    //pcl::MarchingCubesGreedy<PointTNormal> mc;
    //pcl::MarchingCubesRBF<PointTNormal> mc;
    pcl::MarchingCubesHoppe<PointTNormal> mc;
    double leafSize = 0.01;
    float isoLevel = 0.5;
    // Set parameters
    //mc.setLeafSize(leafSize);
    //mc.setGridResolution(512,512,512);
    mc.setIsoLevel(isoLevel);   //ISO: must be between 0 and 1.0
    mc.setSearchMethod(tree2);
    mc.setInputCloud(point_cloud_normal_voxelized);
    // Reconstruct
    mc.reconstruct (mesh);
    */

    /*
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(point_cloud_normal_voxelized);
    poisson.setConfidence();
    poisson.reconstruct(mesh);
    */

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> greedy;
    greedy.setSearchRadius(0.1);
    greedy.setMu (2.5);
    greedy.setMaximumNearestNeighbors(50);
    greedy.setMinimumAngle(M_PI/18); // 10 degrees
    greedy.setMaximumAngle(2*M_PI/3); // 120 degrees
    greedy.setNormalConsistency(true);
    greedy.setConsistentVertexOrdering(true);

    greedy.setSearchMethod(tree2);
    greedy.setInputCloud(point_cloud_normal_voxelized);

    //std::cout << "start reconstruct\n";
    greedy.reconstruct(mesh_);

    double time_greedy = stop_watch_.getTimeSeconds();
    //std::cout << "Greedy finished in " << time_greedy - time_kdtree << " s.\n";

    //Saving to disk in VTK format:
    //pcl::io::saveVTKFile ("mesh.vtk", mesh);

    //surface_reconstruction_->setSearchMethod(tree2);
    //surface_reconstruction_->setInputCloud(point_cloud_normal_voxelized);

    //std::cout << "Total Reconstruction finished in " << stop_watch_.getTimeSeconds() << " s.";

    return true;

    //pcl::io::savePLYFile("/home/kohlbrecher/poly.ply", mesh_);


  }

  const pcl::PolygonMesh& getMesh() const { return mesh_; };

private:
  //sensor_msgs::LaserScan scan_;
  //laser_geometry::LaserProjection laser_proj_;
  //tf::Transformer tf_transformer_;
  boost::shared_ptr<pcl::PointCloud<PointT> > pc_;

  boost::shared_ptr<pcl::SurfaceReconstruction<pcl::PointNormal> > surface_reconstruction_;

  pcl::PolygonMesh mesh_;

  pcl::StopWatch stop_watch_;

  double voxel_size_;



};

}
#endif
