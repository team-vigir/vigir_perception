This package provides classes and nodes for generating meshes from 3D data (point clouds and depth images):

* [cloud_to_mesh_node](https://github.com/team-vigir/vigir_perception/blob/master/vigir_point_cloud_proc/src/cloud_to_mesh_node.cpp) subscribes to PointCloud2 data and generates a mesh from it
* [depth_image_to_mesh_node](https://github.com/team-vigir/vigir_perception/blob/master/vigir_point_cloud_proc/src/depth_image_to_mesh_node.cpp) subscribes to DepthImage data and generates a mesh from it

Both nodes publish the mesh as a shape_mgs/Mesh.

See the [launch folder](https://github.com/team-vigir/vigir_perception/tree/master/vigir_point_cloud_proc/launch) for launch file examples.