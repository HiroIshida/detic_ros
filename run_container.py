#!/usr/bin/env python3

# NOTE: please use only standard libraries
import argparse
import os
import shutil
import subprocess
import uuid
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Optional

_DETIC_ROS_ROOT = "/home/user/detic_ws/src/detic_ros"


def add_prefix(file_path: Path, prefix: str) -> Path:
    parent = file_path.parent
    return parent / (prefix + file_path.name)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-inject", type=str, help="injecting launch file or directory path"
    )
    parser.add_argument("-name", type=str, help="launch file name")
    parser.add_argument(
        "-host", type=str, default="pr1040", help="host name or ip-address"
    )
    args = parser.parse_args()

    inject_path_str: Optional[str] = args.inject
    assert inject_path_str is not None
    inject_path = Path(inject_path_str)

    launch_file_name: Optional[str] = args.name
    assert launch_file_name is not None

    prefix_uuidval = str(uuid.uuid4())

    with TemporaryDirectory() as td:
        tmp_launch_path = Path(td) / "launch"

        if inject_path.is_dir():
            shutil.copytree(inject_path, tmp_launch_path)
        else:
            shutil.copyfile(inject_path, tmp_launch_path)

        for file_path in tmp_launch_path.iterdir():
            os.rename(file_path, add_prefix(file_path, prefix_uuidval))

        docker_run_command = """
            docker run \
                -v {tmp_launch_path}:{detic_ros_root}/launch_injected \
                --rm --net=host -it \
                --gpus 1 detic_ros:latest \
                /bin/bash -i -c \
                "source ~/.bashrc; \
                roscd detic_ros; \
                rossetip; rossetmaster {host}; \
                roslaunch detic_ros {launch_file_name}"
                """.format(
            tmp_launch_path=tmp_launch_path,
            detic_ros_root=_DETIC_ROS_ROOT,
            host=args.host,
            launch_file_name=(prefix_uuidval + launch_file_name),
        )
        print(docker_run_command)
        subprocess.call(docker_run_command, shell=True)
