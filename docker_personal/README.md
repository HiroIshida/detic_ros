## Build image and run container
```bash
docker build -t detic_ros .
docker run -it \
    --name detic_ros_container \
    --net host -it --gpus all \
    -v $HOME/.ssh:/home/user/.ssh \
    detic_ros:latest
```
