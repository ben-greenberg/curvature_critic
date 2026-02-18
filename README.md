# curvature_critic
This is the code needed to add a curvature-limiting critic to the DWB controller in the ROS2 Nav2 system. 

To use this new critic you need to build navigation2 from source. You can either clone my branch of the repository, or start from the latest version and make the minimal changes additions of the files in this repository.

In the `dwb_critics` package within `navigation2/nav2_dwb_controller`, replace `CMakeLists.txt` and `default_critics.xml` with the files in this directory. Additionally, add the `curvature.cpp` and `curvature.hpp` files to the corresponding directories in this package.

Finally, add a `Curvature.scale` field to your `nav2_params.yaml` in your `controller_server/FollowPath` plugin parameters, next to all of the other controller parameters. Values around 1.5-2.0 have been found to be effective, but this is just a reference point depends on the scaling of your others critics. 
