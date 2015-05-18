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

#ifndef VIGIR_MESH_CONVERSIONS_H_
#define VIGIR_MESH_CONVERSIONS_H_

#include <visualization_msgs/Marker.h>
#include <shape_msgs/Mesh.h>
#include <pcl_msgs/PolygonMesh.h>

#include <pcl/PolygonMesh.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>



namespace vigir_point_cloud_proc
{
static bool meshToShapeMsg(const pcl::PolygonMesh& in, shape_msgs::Mesh& mesh)
{
  pcl_msgs::PolygonMesh pcl_msg_mesh;

  pcl_conversions::fromPCL(in, pcl_msg_mesh);


  sensor_msgs::PointCloud2Modifier pcd_modifier(pcl_msg_mesh.cloud);

  size_t size = pcd_modifier.size();

  mesh.vertices.resize(size);

  std::cout << "polys: " << pcl_msg_mesh.polygons.size() << " vertices: " << pcd_modifier.size() << "\n";

  sensor_msgs::PointCloud2ConstIterator<float> pt_iter(pcl_msg_mesh.cloud, "x");

  for(size_t i = 0; i < size ; i++, ++pt_iter){
    mesh.vertices[i].x = pt_iter[0];
    mesh.vertices[i].y = pt_iter[1];
    mesh.vertices[i].z = pt_iter[2];
  }

  //ROS_INFO("Found %ld polygons", triangles.size());

  std::cout << "Updated vertices" << "\n";

  //BOOST_FOREACH(const Vertices polygon, triangles)

  mesh.triangles.resize(in.polygons.size());

  for (size_t i = 0; i < in.polygons.size(); ++i)
  {
    if(in.polygons[i].vertices.size() < 3)
    {
      ROS_WARN("Not enough points in polygon. Ignoring it.");
      continue;
    }

    //shape_msgs::MeshTriangle triangle = shape_msgs::MeshTriangle();
    //boost::array<uint32_t, 3> xyz = {{in.polygons[i].vertices[0], in.polygons[i].vertices[1], in.polygons[i].vertices[2]}};
    //triangle.vertex_indices = xyz;

    //mesh.triangles.push_back(shape_msgs::MeshTriangle());
    //mesh.triangles[i].vertex_indices.resize(3);

    for (int j = 0; j < 3; ++j)
      mesh.triangles[i].vertex_indices[j] = in.polygons[i].vertices[j];
  }
  return true;
}

static bool meshToMarkerMsg(const pcl::PolygonMesh& in, visualization_msgs::Marker& marker)
{
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.header.frame_id = in.cloud.header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.id = 1;
  marker.action = visualization_msgs::Marker::ADD;


  shape_msgs::Mesh shape_msg_mesh;

  meshToShapeMsg(in, shape_msg_mesh);

  size_t size_triangles = shape_msg_mesh.triangles.size();

  marker.points.resize(size_triangles*3);

  std::cout << "polys: " << size_triangles << " vertices: " << shape_msg_mesh.vertices.size() << "\n";

  size_t i = 0;

  for (size_t tri_index = 0; tri_index < size_triangles; ++tri_index){

    /*
    std::cout << shape_msg_mesh.triangles[tri_index].vertex_indices[0] <<  " " <<
                 shape_msg_mesh.triangles[tri_index].vertex_indices[1] <<  " " <<
                 shape_msg_mesh.triangles[tri_index].vertex_indices[2] << "\n";
    */

    marker.points[i]   = shape_msg_mesh.vertices[shape_msg_mesh.triangles[tri_index].vertex_indices[0]];
    marker.points[i+1] = shape_msg_mesh.vertices[shape_msg_mesh.triangles[tri_index].vertex_indices[1]];
    marker.points[i+2] = shape_msg_mesh.vertices[shape_msg_mesh.triangles[tri_index].vertex_indices[2]];
    i = i + 3;

  }

}
}
#endif
