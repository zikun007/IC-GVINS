#!/usr/bin/env python3
"""Convert IC-GVINS and KAIST urban39 reference trajectories to TUM for evo."""

import argparse
import csv
import re
from pathlib import Path

import numpy as np

GPS_EPOCH_UNIX = 315964800
GPS_LEAP_SECONDS = 18
WEEK_SECONDS = 604800


def vehicle_to_imu(path):
    text = path.read_text()
    rotation = [float(value) for value in re.search(r"R:\s*(.*)", text).group(1).split()]
    translation = [float(value) for value in re.search(r"T:\s*(.*)", text).group(1).split()]
    return np.asarray(rotation).reshape(3, 3), np.asarray(translation)


def rotation_matrix_to_quaternion(matrix):
    """Return an xyzw quaternion for a proper 3x3 rotation matrix."""
    trace = np.trace(matrix)
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        axis = int(np.argmax(np.diag(matrix)))
        if axis == 0:
            scale = 2.0 * np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
            qw = (matrix[2, 1] - matrix[1, 2]) / scale
            qx = 0.25 * scale
            qy = (matrix[0, 1] + matrix[1, 0]) / scale
            qz = (matrix[0, 2] + matrix[2, 0]) / scale
        elif axis == 1:
            scale = 2.0 * np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
            qw = (matrix[0, 2] - matrix[2, 0]) / scale
            qx = (matrix[0, 1] + matrix[1, 0]) / scale
            qy = 0.25 * scale
            qz = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = 2.0 * np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
            qw = (matrix[1, 0] - matrix[0, 1]) / scale
            qx = (matrix[0, 2] + matrix[2, 0]) / scale
            qy = (matrix[1, 2] + matrix[2, 1]) / scale
            qz = 0.25 * scale
    quaternion = np.asarray((qx, qy, qz, qw))
    return quaternion / np.linalg.norm(quaternion)


def write_tum(path, poses):
    with path.open("w") as stream:
        for timestamp, position, quaternion in poses:
            stream.write(f"{timestamp:.9f} {position[0]:.9f} {position[1]:.9f} {position[2]:.9f} "
                         f"{quaternion[0]:.9f} {quaternion[1]:.9f} {quaternion[2]:.9f} {quaternion[3]:.9f}\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path, help="IC-GVINS trajectory.csv")
    parser.add_argument("reference", type=Path, help="KAIST global_pose.csv")
    parser.add_argument("vehicle_to_imu", type=Path, help="KAIST Vehicle2IMU.txt")
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()

    r_vehicle_imu, t_vehicle_imu = vehicle_to_imu(args.vehicle_to_imu)
    reference = []
    with args.reference.open(newline="") as stream:
        for row in csv.reader(stream):
            if row:
                matrix = np.asarray([float(value) for value in row[1:]]).reshape(3, 4)
                reference.append((int(row[0]), matrix[:, :3] @ t_vehicle_imu + matrix[:, 3],
                                  matrix[:, :3] @ r_vehicle_imu))
    if not reference:
        raise SystemExit("reference trajectory is empty")

    origin = reference[0][1]
    # global_pose is UTM ENU; IC-GVINS state positions are local NED.
    enu_to_ned = np.asarray(((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0)))
    # KAIST vehicle/IMU axes are FLU, while IC-GVINS consumes FRD.
    frd_to_flu = np.diag((1.0, -1.0, -1.0))
    reference_tum = []
    for timestamp, position, rotation_enu_imu_flu in reference:
        east, north, up = position - origin
        rotation_ned_imu_frd = enu_to_ned @ rotation_enu_imu_flu @ frd_to_flu
        quaternion = rotation_matrix_to_quaternion(rotation_ned_imu_frd)
        reference_tum.append((timestamp / 1e9, (north, east, -up), quaternion))

    week = int((reference[0][0] / 1e9 + GPS_LEAP_SECONDS - GPS_EPOCH_UNIX) // WEEK_SECONDS)
    estimate_tum = []
    with args.trajectory.open() as stream:
        for line in stream:
            values = line.split()
            if len(values) == 8:
                sow, x, y, z, qx, qy, qz, qw = map(float, values)
                timestamp = sow + week * WEEK_SECONDS + GPS_EPOCH_UNIX - GPS_LEAP_SECONDS
                estimate_tum.append((timestamp, (x, y, z), (qx, qy, qz, qw)))
    if not estimate_tum:
        raise SystemExit("IC-GVINS trajectory contains no poses")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_tum(args.output_dir / "reference_imu_ned.tum", reference_tum)
    write_tum(args.output_dir / "estimate_imu_ned.tum", estimate_tum)
    print(f"wrote {len(reference_tum)} reference and {len(estimate_tum)} estimate poses")


if __name__ == "__main__":
    main()
