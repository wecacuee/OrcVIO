- follow [this blog](https://towardsdatascience.com/installing-nvidia-drivers-cuda-10-cudnn-for-tensorflow-2-1-on-ubuntu-18-04-lts-f1db8bff9ea), install cudnn for cuda 10.1

- download libtorch 1.4 from `wget wget https://download.pytorch.org/libtorch/cu101/libtorch-cxx11-abi-shared-with-deps-1.4.0.zip`, then change the line 148 in caffe cmake as in this patch https://github.com/moshanATucsd/orcvio_cpp/blob/master/ros_wrapper/install-deps/libtorch-1.4.0-bug-cuda.cmake.patch
```
 if(EXISTS ${CUDNN_INCLUDE_PATH}/cudnn_version.h) 
   file(READ ${CUDNN_INCLUDE_PATH}/cudnn_version.h CUDNN_HEADER_CONTENTS) 
 else() 
   file(READ ${CUDNN_INCLUDE_PATH}/cudnn.h CUDNN_HEADER_CONTENTS) 
 endif() 
```

- in cmakelist of starmap, can set the path of libtorch like this 
```
## TODO FIXME do not use the absolute path 
set(Torch_DIR /home/erl/Workspace/libtorch/share/cmake/Torch)
find_package(Torch 1.4 REQUIRED)
```
or set the `CMAKE_PREFIX_PATH`

- change the `bagfile_basename`, `outdata_dir` in `darknet-sort_ros-starmap.launch` for starmap. 

- If input is the compressed image, uncomment `<node name="republish" type="republish" pkg="image_transport" output="screen" 	args="compressed in:=/front_camera/image_raw raw out:=/camera/image_raw" />` in darknet ros launch file. 

- use `roslaunch starmap darknet-sort_ros-starmap.launch` to launch the frontend  

- note that the input of darknet ros should be color images instead of grayscale ones, eg in `darknet-sort_ros-starmap.launch`, the rosbag to `bagfile_basename` should contain color images, otherwise there's an error of [Rviz OGRE Exception](https://answers.ros.org/question/227881/rviz-ogre-exception2invalidparametersexception/)