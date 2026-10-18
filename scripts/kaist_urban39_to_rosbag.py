#!/usr/bin/env python3
"""Convert the supplied KAIST urban39 CSV/PNG data into a ROS 2 bag.

Run this script with the ROS 2 Python environment, for example:
  source /opt/ros/jazzy/setup.bash
  /usr/bin/python3 scripts/kaist_urban39_to_rosbag.py /mnt/OpenDataset/kaist-urban39 urban39_bag
"""

import argparse
import csv
import heapq
from pathlib import Path

import numpy as np
from PIL import Image

from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


def stamp(timestamp_ns):
    return Time(sec=timestamp_ns // 1_000_000_000, nanosec=timestamp_ns % 1_000_000_000)


def debayer_bgr(raw, pattern):
    """Small dependency-free bilinear Bayer demosaic; output is BGR8."""
    pattern = pattern.upper()
    layouts = {
        "BGGR": (("B", "G"), ("G", "R")),
        "RGGB": (("R", "G"), ("G", "B")),
        "GRBG": (("G", "R"), ("B", "G")),
        "GBRG": (("G", "B"), ("R", "G")),
    }
    if pattern not in layouts:
        raise ValueError(f"unsupported Bayer pattern: {pattern}")
    h, w = raw.shape
    channels = []
    for color in ("B", "G", "R"):
        mask = np.zeros((h, w), dtype=np.float32)
        layout = layouts[pattern]
        for row in range(2):
            for col in range(2):
                if layout[row][col] == color:
                    mask[row::2, col::2] = 1.0
        values = raw.astype(np.float32) * mask
        padded_values = np.pad(values, 1, mode="edge")
        padded_mask = np.pad(mask, 1, mode="edge")
        total = np.zeros((h, w), dtype=np.float32)
        count = np.zeros((h, w), dtype=np.float32)
        for row in range(3):
            for col in range(3):
                total += padded_values[row:row + h, col:col + w]
                count += padded_mask[row:row + h, col:col + w]
        interpolated = total / np.maximum(count, 1.0)
        channels.append(np.where(mask > 0, raw, interpolated).astype(np.uint8))
    return np.dstack(channels)


def imu_messages(csv_path, flu_to_frd):
    transform = np.array([1.0, -1.0, -1.0]) if flu_to_frd else np.ones(3)
    with csv_path.open(newline="") as stream:
        for row in csv.reader(stream):
            if not row:
                continue
            if len(row) < 14:
                raise ValueError(f"invalid xsens IMU row with {len(row)} columns: {row}")
            timestamp = int(row[0])
            message = Imu()
            message.header.stamp = stamp(timestamp)
            message.header.frame_id = "imu"
            # KAIST Vehicle2IMU is FLU; IC-GVINS consumes front-right-down.
            # xsens_imu.csv columns are timestamp, quaternion[4], Euler[3],
            # gyro[3], acceleration[3], magnetic field[3].
            gyro = np.asarray([float(row[8]), float(row[9]), float(row[10])]) * transform
            accel = np.asarray([float(row[11]), float(row[12]), float(row[13])]) * transform
            message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = gyro
            message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = accel
            yield timestamp, "/imu0", "sensor_msgs/msg/Imu", serialize_message(message)


def gnss_messages(csv_path):
    with csv_path.open(newline="") as stream:
        for row in csv.reader(stream):
            if not row:
                continue
            timestamp = int(row[0])
            message = NavSatFix()
            message.header.stamp = stamp(timestamp)
            message.header.frame_id = "gnss"
            message.status.status = NavSatStatus.STATUS_FIX
            message.status.service = NavSatStatus.SERVICE_GPS
            message.latitude, message.longitude, message.altitude = map(float, row[1:3] + [row[5]])
            # vrs_gps.csv: HDOP, then east/north/up position standard deviations.
            std_e, std_n, std_u = map(float, row[9:12])
            message.position_covariance[0] = std_e * std_e
            message.position_covariance[4] = std_n * std_n
            message.position_covariance[8] = std_u * std_u
            message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            yield timestamp, "/gnss0", "sensor_msgs/msg/NavSatFix", serialize_message(message)


def image_messages(stamps_path, images_dir, bayer_pattern, max_frames):
    with stamps_path.open() as stream:
        for index, line in enumerate(stream):
            if max_frames is not None and index >= max_frames:
                return
            timestamp = int(line.strip())
            image_path = images_dir / f"{timestamp}.png"
            if not image_path.is_file():
                raise FileNotFoundError(f"missing image for timestamp {timestamp}: {image_path}")
            raw = np.asarray(Image.open(image_path))
            bgr = debayer_bgr(raw, bayer_pattern)
            message = ImageMsg()
            message.header.stamp = stamp(timestamp)
            message.header.frame_id = "cam0"
            message.height, message.width = bgr.shape[:2]
            message.encoding = "bgr8"
            message.is_bigendian = False
            message.step = message.width * 3
            message.data = bgr.tobytes()
            yield timestamp, "/cam0", "sensor_msgs/msg/Image", serialize_message(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path, help="KAIST urban39 directory")
    parser.add_argument("output", type=Path, help="new rosbag2 output directory")
    parser.add_argument("--bayer-pattern", default="BGGR", choices=("BGGR", "RGGB", "GRBG", "GBRG"))
    parser.add_argument("--max-frames", type=int, help="write only this many camera frames (for smoke tests)")
    parser.add_argument("--max-duration", type=float, help="write at most this many seconds from the first sample")
    parser.add_argument("--keep-flu", action="store_true", help="do not convert KAIST IMU FLU samples to IC-GVINS FRD")
    args = parser.parse_args()

    if args.output.exists():
        raise SystemExit(f"refusing to overwrite existing output directory: {args.output}")
    data = args.dataset / "urban39-pankyo_data"
    images = args.dataset / "urban39-pankyo_img" / "stereo_left"
    required = (data / "xsens_imu.csv", data / "vrs_gps.csv", data / "stereo_stamp.csv", images)
    if not all(path.exists() for path in required):
        raise SystemExit("dataset does not match the expected urban39 directory layout")

    writer = SequentialWriter()
    writer.open(StorageOptions(uri=str(args.output), storage_id="sqlite3"), ConverterOptions("cdr", "cdr"))
    topic_types = {"/imu0": "sensor_msgs/msg/Imu", "/gnss0": "sensor_msgs/msg/NavSatFix", "/cam0": "sensor_msgs/msg/Image"}
    for topic_id, (topic, message_type) in enumerate(topic_types.items(), start=1):
        writer.create_topic(TopicMetadata(id=topic_id, name=topic, type=message_type, serialization_format="cdr"))

    streams = [
        imu_messages(data / "xsens_imu.csv", not args.keep_flu),
        gnss_messages(data / "vrs_gps.csv"),
        image_messages(data / "stereo_stamp.csv", images, args.bayer_pattern, args.max_frames),
    ]
    heap = []
    for index, source in enumerate(streams):
        try:
            item = next(source)
            heapq.heappush(heap, (item[0], index, item))
        except StopIteration:
            pass

    counts = {topic: 0 for topic in topic_types}
    first_timestamp = heap[0][0] if heap else 0
    while heap:
        _, index, item = heapq.heappop(heap)
        timestamp, topic, _, serialized = item
        if args.max_duration is not None and timestamp - first_timestamp > args.max_duration * 1_000_000_000:
            break
        writer.write(topic, serialized, timestamp)
        counts[topic] += 1
        try:
            next_item = next(streams[index])
            heapq.heappush(heap, (next_item[0], index, next_item))
        except StopIteration:
            pass
    writer.close()
    print("wrote " + ", ".join(f"{topic}: {count}" for topic, count in counts.items()))


if __name__ == "__main__":
    main()
