This package provides classes and nodes for manipulating compressed LIDAR scans:

* [scan_split_node](https://github.com/team-vigir/vigir_perception/blob/master/vigir_filtered_localized_scan_utils/src/scan_split_node.cpp) splits incoming LIDAR scans into smaller parts.
* [scan_to_clouds_pub_node](https://github.com/team-vigir/vigir_perception/blob/master/vigir_filtered_localized_scan_utils/src/scan_to_clouds_pub_node.cpp) publishes point clouds based on incoming scan data. Both a self-filtered and a non self-filtered cloud are published.