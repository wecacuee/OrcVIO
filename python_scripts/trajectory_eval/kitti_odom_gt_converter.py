#!/usr/bin/env python
from collections import namedtuple
from numpy.linalg import inv
import numpy as np
import os
import pandas as pd
import pykitti
import typer

import utils 

RawSeq = namedtuple("RawSeq", ("date", "drive", "start", "stop"))

class Odom2Raw:
    # Taken from 
    # http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    # then Download odometry development kit (1 MB)
    # then devkit/readme.txt
    odom2raw_text = """
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
    """

    def __init__(self):
        self._odom2rawseq = [""]*11
        self._parse()

    def _parse(self):
        for line in self.odom2raw_text.split("\n"):
            if line.strip() == "":
                continue
            k, v = [p.strip() for p in line.split(":")]
            date_drive, start, stop = v.split()
            date, drive = date_drive.split("_drive_")
            self._odom2rawseq[int(k)] = RawSeq(
                date = date, # keep them in string format
                drive = drive, # keep them in string format
                start = int(start),
                stop = int(stop))

    def date_drive(self, odomseq):
        return self._odom2rawseq[int(odomseq)][:2]

def main(sequence: int = 7,
         # Change this to the directory where you store KITTI data
         kitti_dataset: str = os.getenv("KITTI_DATASET", os.path.expanduser('~/data/orcvio_ws/kitti')),
         # where to save the generated file
         output_path : str = "{odometry_data}/poses/{sequence!02d}_stamped_groundtruth.txt"
         ):
    """
    Change dataset path downloaded from Kitti
    The converted ground-truth file will be saved at the same directory 
    of the original pose file

    """

    # sequence = "07"
    odom2raw = Odom2Raw()
    date, drive = odom2raw.date_drive(sequence)

    odometry_path = os.path.join(kitti_dataset, 'odometry/dataset')
    raw_data_path = os.path.join(kitti_dataset, 'raw_data')

    # Read both odometry and raw datasets;
    odometry_data = pykitti.odometry(odometry_path , "%02d" % sequence)
    raw_data = pykitti.raw(raw_data_path, date, drive)

    # output_path = os.path.join(odometry_path, "poses/" + sequence + "_quat.txt")
    output_path = output_path.format(odometry_data=odometry_data,
                                     sequence=sequence)

    T_cam0_imu = raw_data.calib.T_cam0_imu
    T_imu_cam0 = inv(T_cam0_imu)
    poses_w_imu = [T_imu_cam0.dot(T.dot(T_cam0_imu)) for T in odometry_data.poses]


    # dataset.calib:      Calibration data are accessible as a named tuple
    # dataset.timestamps: Timestamps are parsed into a list of timedelta objects
    # dataset.poses:      List of ground truth poses T_w_cam0
    # dataset.camN:       Generator to load individual images from camera N
    # dataset.gray:       Generator to load monochrome stereo pairs (cam0, cam1)
    # dataset.rgb:        Generator to load RGB stereo pairs (cam2, cam3)
    # dataset.velo:       Generator to load velodyne scans as [x,y,z,reflectance]

    print("Shape of the ground-truth poses: " + str(len(poses_w_imu)))
    print("\nShape of timestamps: " + str(len(odometry_data.timestamps)))
    print("=======================================")

    print("\nExample of ground-truth pose: " + str(poses_w_imu[0]))
    print("\nExample of timestamps: " + str(odometry_data.timestamps[0:10]))
    print("=======================================")

    # extract timestamps
    timestamps = np.array(list(map(lambda x: x.total_seconds(), odometry_data.timestamps))).reshape(-1, 1)

    # print(timestamps.shape)

    # extra Rotations and Translations
    (R_w_imu, p_w_imu) = zip(*map(lambda pose: (pose[0:3, 0:3], pose[0:3, 3]), poses_w_imu))
    print("\nExample of extracted rotation: " + str(R_w_imu[0]))
    print("\nExample of extracted translation: " + str(p_w_imu[0]))

    # note, here we use hamilton quaternion 
    # with qx qy qz qw format 
    q_w_imu = np.array(list(map(utils.matrix_to_quaternion, R_w_imu))) # qx qy qz qw
    p_w_imu = np.array(p_w_imu)

    # Construct dataset and output
    dataset = pd.DataFrame(np.hstack((timestamps, p_w_imu, q_w_imu))) # (timestamp(s) tx ty tz qx qy qz qw)
    dataset.to_csv(output_path, sep=' ', header=False, index=False)
 








if __name__ == '__main__':
    typer.run(main)
