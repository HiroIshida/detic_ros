python3 run_container.py -host pr1040 -mount ./launch -name sample.launch device:=auto input_image:=/kinect_head/rgb/image_color &
PYTHON_PID=$!
sleep 15
if ps -p $PYTHON_PID > /dev/null; then 
    echo "Container is running"
    kill -9 $PYTHON_PID
    exit 0
else
    echo "Container is not running (unexpected)"
    exit 1
fi
