- The extrinsic in the original LARVIO config is different from OpenVINS, and the `T_cam_imu` is the inverse of `T_C0toI` in OpenVINS launch file. For example, if in OpenVINS we have 
```
        <rosparam param="T_C0toI">
            [
            0.0,  0.0, 1.0, 0.1,
            -1.0,  0.0, 0.0, 0.1,
            0.0, -1.0, 0.0, 0.0,
            0.0,  0.0, 0.0, 1.0
            ]
```
Then in LARVIO it should be 
```
T_cam_imu: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data:
    [0.0,  -1.0, 0.0, 0.1,
      0.0,  0.0, -1.0, 0,
      1.0, 0.0, 0.0, -0.1,
      0.0,  0.0, 0.0, 1.0]
```
- In [Kalibr](https://github.com/ethz-asl/kalibr/wiki/yaml-formats), the output calibration format is 
```
T_cn_cnm1
camera extrinsic transformation, always with respect to the last camera in the chain
(e.g. cam1: T_cn_cnm1 = T_c1_c0, takes cam0 to cam1 coordinates)
T_cam_imu
IMU extrinsics: transformation from IMU to camera coordinates (T_c_i)
```
So in LARVIO, `T_cam_imu` is used in config file, which means `from IMU to camera`, but in [code](https://github.com/PetWorm/LARVIO/blob/9ca8b25df098e978fb7be323691219661a6207e5/src/larvio.cpp#L188) the variable naming notation is changed 
```
  // Transformation offsets between the frames involved.
  cv::Mat T_imu_cam;
  fsSettings["T_cam_imu"] >> T_imu_cam;
  cv::Matx33d R_imu_cam(T_imu_cam(cv::Rect(0,0,3,3)));
  cv::Vec3d t_imu_cam = T_imu_cam(cv::Rect(3,0,1,3));
```
where `R_imu_cam` means from IMU to camera, also refer to [here](https://github.com/PetWorm/LARVIO/blob/9ca8b25df098e978fb7be323691219661a6207e5/src/larvio.cpp#L722)
```
  const Matrix3d& R_b2c =
      state_server.imu_state.R_imu_cam0;
```