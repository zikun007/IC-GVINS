#!/usr/bin/env python3
"""Generate an IC-GVINS configuration for the supplied KAIST urban39 data."""

import argparse
import re
from pathlib import Path

import numpy as np
import yaml


def parse_transform(path):
    text = path.read_text()
    rotation = [float(value) for value in re.search(r"R:\s*(.*)", text).group(1).split()]
    translation = [float(value) for value in re.search(r"T:\s*(.*)", text).group(1).split()]
    return np.asarray(rotation).reshape(3, 3), np.asarray(translation)


def opencv_values(path, key):
    text = path.read_text()
    match = re.search(rf"{key}: !!opencv-matrix.*?data:\s*\[([^]]+)\]", text, re.DOTALL)
    if not match:
        raise ValueError(f"could not read {key} from {path}")
    return [float(value) for value in match.group(1).replace("\n", " ").split(",") if value.strip()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--output-path", default="./output/urban39", help="IC-GVINS result directory")
    args = parser.parse_args()

    calibration = args.dataset / "urban39-pankyo_calibration" / "calibration"
    camera_matrix = opencv_values(calibration / "left.yaml", "camera_matrix")
    distortion = opencv_values(calibration / "left.yaml", "distortion_coefficients")
    r_vehicle_camera, t_vehicle_camera = parse_transform(calibration / "Vehicle2Stereo.txt")
    r_vehicle_imu, t_vehicle_imu = parse_transform(calibration / "Vehicle2IMU.txt")
    _, t_vehicle_vrs = parse_transform(calibration / "Vehicle2VRS.txt")

    # KAIST vehicle / IMU is FLU; IC-GVINS expects body coordinates in FRD.
    flu_to_frd = np.diag([1.0, -1.0, -1.0])
    r_imu_camera = flu_to_frd @ r_vehicle_imu.T @ r_vehicle_camera
    t_imu_camera = flu_to_frd @ r_vehicle_imu.T @ (t_vehicle_camera - t_vehicle_imu)
    t_imu_vrs = flu_to_frd @ r_vehicle_imu.T @ (t_vehicle_vrs - t_vehicle_imu)
    quaternion_xyzw = [*rotation_matrix_to_quaternion_xyzw(r_imu_camera)]

    config = {
        "outputpath": args.output_path,
        "is_make_outputdir": True,
        "initlength": 1,
        "imudatarate": 100,
        "iswithearth": True,
        "antlever": t_imu_vrs.round(8).tolist(),
        # Keep the noise model used by the upstream KAIST configuration.  It is
        # an estimator tuning value (including vehicle vibration), rather than
        # a direct copy of the MTi-300 bench noise-density specification.
        "imumodel": {"arw": 0.1, "vrw": 0.1, "gbstd": 50.0, "abstd": 50.0, "corrtime": 1.0},
        # Keep GNSS outage simulation disabled for the complete sequence.
        "isusegnssoutage": False,
        "gnssoutagetime": 0,
        "gnssthreshold": 20,
        # Receiver covariance in RTK-fixed mode is only a few centimetres and
        # excludes lever-arm, synchronization and model errors.  A realistic
        # floor prevents those errors from immediately rejecting good GNSS.
        # Receiver covariance is formal RTK precision and omits antenna,
        # timestamp and model uncertainty present in this sequence.
        "gnss_std_floor": 0.5,
        "gnss_adaptive_reweighting": False,
        "is_use_visualization": True,
        "track_check_histogram": False,
        "track_min_parallax": 20,
        "track_max_interval": 0.5,
        "track_max_features": 200,
        "reprojection_error_std": 1.5,
        "optimize_windows_size": 10,
        "optimize_num_iterations": 20,
        # KAIST provides a calibrated camera/IMU transform and hardware-triggered
        # image timestamps.  Keep both fixed to avoid an unobservable parameter
        # block during visual initialization or tracking loss.
        "optimize_estimate_extrinsic": False,
        "optimize_estimate_td": False,
        "cam0": {
            # IC-GVINS accepts [fx, fy, cx, cy, skew].  Preserve the small
            # non-zero skew from KAIST's raw left-camera calibration.
            "intrinsic": [camera_matrix[0], camera_matrix[4], camera_matrix[2], camera_matrix[5], camera_matrix[1]],
            "distortion": distortion,
            "resolution": [1280, 560],
            "q_b_c": np.round(quaternion_xyzw, 8).tolist(),
            "t_b_c": t_imu_camera.round(8).tolist(),
            "td_b_c": 0.0,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    print(f"wrote {args.output}")


def rotation_matrix_to_quaternion_xyzw(matrix):
    """Return a normalized xyzw quaternion without requiring SciPy."""
    trace = np.trace(matrix)
    if trace > 0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        return np.array([(matrix[2, 1] - matrix[1, 2]) / scale, (matrix[0, 2] - matrix[2, 0]) / scale,
                         (matrix[1, 0] - matrix[0, 1]) / scale, 0.25 * scale])
    index = int(np.argmax(np.diag(matrix)))
    next_index, last_index = (index + 1) % 3, (index + 2) % 3
    scale = 2.0 * np.sqrt(1.0 + matrix[index, index] - matrix[next_index, next_index] - matrix[last_index, last_index])
    quaternion = np.zeros(4)
    quaternion[index] = 0.25 * scale
    quaternion[next_index] = (matrix[next_index, index] + matrix[index, next_index]) / scale
    quaternion[last_index] = (matrix[last_index, index] + matrix[index, last_index]) / scale
    quaternion[3] = (matrix[last_index, next_index] - matrix[next_index, last_index]) / scale
    return quaternion / np.linalg.norm(quaternion)


if __name__ == "__main__":
    main()
