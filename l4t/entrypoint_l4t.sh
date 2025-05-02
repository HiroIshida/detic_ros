#!/bin/bash

source /catkin_ws/devel/setup.bash
source /venv/bin/activate
export PYTHONPATH=/venv/lib/python3.8/site-packages:$PYTHONPATH

exec "$@"
