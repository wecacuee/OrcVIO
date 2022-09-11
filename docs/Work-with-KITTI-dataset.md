## download dataset and arrange it
```
~/datasets/Kitti/
├── odometry
│   ├── dataset
│   │   ├── poses
│   │   │   ├── 00.txt
│   │   │   ├── ...
│   │   ├── sequences
|   |   |   ├── 00
|   |   |   |   ├── calib.txt
|   |   |   |   ├── times.txt 
|   |   |   |   ├── image_0 
|   |   |   |   ├── image_1
|   |   |   |   ├── image_2
|   |   |   |   ├── image_3
|   |   |   ├── ... 
|   ├── devkit
├── raw_data
|   ├── 2011_09_30
|   |   ├── 2011_09_30_0020_sync
|   |   |   ├── image_00
|   |   |   ├── image_01
|   |   |   ├── image_02
|   |   |   ├── image_03
|   |   |   ├── oxt
|   |   |   ├── velodyne_points
|   |   ├── calib_cam_to_cam.txt
|   |   ├── calib_imu_to_velo.txt
|   |   |── calib_velo_to_cam.txt

```
## Convert KITTI poses into Hamiltonian quaternion

Use this script:
https://github.com/wecacuee/OrcVIO/blob/wip-kitti-mono-sim-imu/python_scripts/trajectory_eval/kitti_odom_gt_converter.py

```
python kitti_odom_gt_converter.py --sequence 7 --kitti-dataset-dir ~/data/orcvio_ws/kitti --output_path {odometry_data}/poses/{sequence!02d}_quat.txt
```

## generate simulated IMU data
- use this repo https://github.com/moshanATucsd/OrcVIO_IMU_simulation
```
Generate the simulated IMU data from ground-truth provided by the odometry dataset, as well as the MSCKF algorithm implemented in OpenVINs
```
- prepare IMU-sim, Camera-sim
```
Check out kitti branch on the OrcVIO_IMU_simulation repo;
Convert transformation matrices into quaternion from ground-truth:
Change dataset path downloaded from Kitti in OrcVIO_IMU_simulation/ov_msckf/src/python/odometry_gt_converter.py
In catkin_imu workspace of the OrcVIO_IMU_simulation repo, run:
catkin build
rosrun ov_msckf odometry_gt_converter.py or python src/OrcVIO_IMU_simulation/ov_msckf/src/python/odometry_gt_converter.py
The converted ground-truth file will be saved at the same directory of the original pose file
Change the basedir param in the ov_msckf/launch/kitti_sim_odom.launch to the odometry basedir; 
Run Simulation:
roslaunch ov_msckf kitti_sim_odom.launch
Evaluate the predicted trajectory:
chmod +x ov_msckf/src/python/evaluate_rte.py 
For evaluating rte: rosrun ov_msckf evaluate_rte.py [converted_gt_path] [estimated_imu_path] --fixed_delta
Example: rosrun ov_msckf evaluate_rte.py /media/erl/disk2/kitti/Kitti_all_data/odometry/dataset/poses/06_quat.txt /home/erl/Workspace/catkin_imu/src/OrcVIO_IMU_simulation/ov_data/sim/imupose.csv --plot kitti_odometry_06_rte --fixed_delta
```
- sample output for `odometry_gt_converter.py` 
```
Shape of original ground-truth pose: 1101
Shape of the ground-truth poses: 1101

Shape of timestamps: 1101
=======================================

Example of original ground-truth poses: [[  1.00000000e+00   1.19762500e-11   1.70463800e-10   5.55111500e-17]
 [  1.19762500e-11   1.00000000e+00   3.56250300e-10   0.00000000e+00]
 [  1.70463800e-10   3.56250300e-10   1.00000000e+00   2.22044600e-16]
 [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]

Example of transformed ground-truth pose: [[  9.99999996e-01   5.32507602e-10  -2.30585038e-10   2.11273665e-10]
 [  5.32507603e-10   9.99999946e-01  -1.78640625e-09   1.87733051e-10]
 [ -2.30585038e-10  -1.78640625e-09   1.00000014e+00   4.06168321e-10]
 [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]

Example of timestamps: [datetime.timedelta(0), datetime.timedelta(0, 0, 103875), datetime.timedelta(0, 0, 207894), datetime.timedelta(0, 0, 311750), datetime.timedelta(0, 0, 415622), datetime.timedelta(0, 0, 519499), datetime.timedelta(0, 0, 623377), datetime.timedelta(0, 0, 727122), datetime.timedelta(0, 0, 830874), datetime.timedelta(0, 0, 934622)]
=======================================

Shape of converted timestamps: (1101, 1)
=======================================

Example of extracted rotation: [[  9.99999996e-01   5.32507602e-10  -2.30585038e-10]
 [  5.32507603e-10   9.99999946e-01  -1.78640625e-09]
 [ -2.30585038e-10  -1.78640625e-09   1.00000014e+00]]

Example of extracted translation: [  2.11273665e-10   1.87733051e-10   4.06168321e-10]
```
- note, if we are using odometry 06, then the groundtruth file will be `/media/erl/disk2/kitti/Kitti_all_data/odometry/dataset/poses/06_quat.txt`
- moreover, `06_quat.txt` can be copied to the orcvio/cache folder and be renamed to `stamped_groundtruth.txt` for trajectory evaluation 
- we can also use `kitti_sim_raw.launch` for raw sequences 
- the mapping between odometry and raw data is 
```
Nr.     Sequence name     Start   End
---------------------------------------
00: 2011_10_03_drive_0027 000000 004540
01: 2011_10_03_drive_0042 000000 001100
02: 2011_10_03_drive_0034 000000 004660
03: 2011_09_26_drive_0067 000000 000800
04: 2011_09_30_drive_0016 000000 000270
05: 2011_09_30_drive_0018 000000 002760
06: 2011_09_30_drive_0020 000000 001100
07: 2011_09_30_drive_0027 000000 001100
08: 2011_09_30_drive_0028 001100 005170
09: 2011_09_30_drive_0033 000000 001590
10: 2011_09_30_drive_0034 000000 001200
```

## combine simulated IMU data and images into ROS bag 

- use this repo https://github.com/moshanATucsd/OrcVIO_kitti2bag
- follow these 
```
Check out kitti branch on the OrcVIO_IMU_simulation repo;
Install the forked kitti2bag repository and build
Follow the same step in previous section to convert the ground-truth into quaternion;
Run simulation, this will generate a simulated imu measurement file ov_data/imumeas.txt
roslaunch ov_msckf kitti_sim_odom.launch
Pack the sim measurements and camera images into a rosbag
For color scale images: 
kitti2bag odom [path_to_odometry_root]  --sim_path [simulated_imu_meas_path] -c color -s 06
For gray scale images:
kitti2bag odom [path_to_odometry_root]  --sim_path [simulated_imu_meas_path] -c gray -s 06
The rosbag will be saved in the directory that the command is run, note down this path ros_bag_path
Example: kitti2bag odom ~/datasets/Kitti/odometry/dataset --sim_path ~/workspace/catkin_imu/src/OrcVIO_IMU_simulation/ov_data/sim/imumeas.csv -c color -s 06
```
- note, for the `ros_bag_path`, eg if we run `kitti2bag odom /media/erl/disk2/kitti/Kitti_all_data/odometry/dataset --sim_path /home/erl/Workspace/catkin_imu/src/OrcVIO_IMU_simulation/ov_data/sim/imumeas.csv -c gray -s 06` in `/media/erl/disk1/orcvio/kitti_rosbags`, then the rosbag named `kitti_odometry_sequence_06.bag` will be saved there, with the following output 
```
Odometry dataset sequence 06 has ground truth information (poses).
Exporting static transformations
Exporting time dependent transformations
Loading from simulated IMU measurements. /home/erl/Workspace/catkin_imu/src/OrcVIO_IMU_simulation/ov_data/sim/imumeas.csv
Example IMU measurements record:
[  4.11807818e-01   4.97449107e-03  -2.75887538e-02   9.80997145e+00
  -6.82903864e-03  -1.20574640e-03   6.63838837e-03]
Exporting simulated IMU data.
Exporting camera 0
100% (1101 of 1101) |#################################################################################################################################################| Elapsed Time: 0:00:05 Time:  0:00:05
Exporting camera 1
100% (1101 of 1101) |#################################################################################################################################################| Elapsed Time: 0:00:05 Time:  0:00:05
## OVERVIEW ##
path:        kitti_odometry_sequence_06.bag
version:     2.0
duration:    1:54s (114s)
start:       Sep 21 2020 15:27:02.62 (1600727222.62)
end:         Sep 21 2020 15:28:56.92 (1600727336.92)
size:        964.9 MB
messages:    34998
compression: none [1115/1115 chunks]
types:       sensor_msgs/CameraInfo [c9a58c1b0b154e0e6da7578cb991d214]
             sensor_msgs/Image      [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/Imu        [6a62c6daae103f4ff57a132d6f95cec2]
             tf2_msgs/TFMessage     [94810edda583a504dfda3829e70d7eec]
topics:      /kitti/camera_gray_left/camera_info     1101 msgs    : sensor_msgs/CameraInfo
             /kitti/camera_gray_left/image           1101 msgs    : sensor_msgs/Image     
             /kitti/camera_gray_right/camera_info    1100 msgs    : sensor_msgs/CameraInfo
             /kitti/camera_gray_right/image          1101 msgs    : sensor_msgs/Image     
             /kitti/oxts/imu                        28393 msgs    : sensor_msgs/Imu       
             /tf                                     1101 msgs    : tf2_msgs/TFMessage    
             /tf_static                              1101 msgs    : tf2_msgs/TFMessage
```
- we can also make a rosbag from raw data, by using `kitti2bag raw_sync /media/erl/disk2/kitti/Kitti_all_data/raw_data --sim_path /home/erl/Workspace/catkin_imu/src/OrcVIO_IMU_simulation/ov_data/sim/imumeas.csv -c gray -t 2011_09_26 -r 0022`
