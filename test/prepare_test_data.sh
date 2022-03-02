#/bin/bash
dest_picture=$(rospack find detic_ros)/test/data
wget -nc https://web.eecs.umich.edu/~fouhey/fun/desk/desk.jpg -P $dest_picture

dest_rosbag=$(rospack find detic_ros)/test/data/desk.bag
if [ ! -f "$dest_rosbag" ]; then
    wget -nc "https://drive.google.com/uc?export=download&id=1EoMtvH0KaizoM7WT9cJkKFW_hZsR75x0" -O $dest_rosbag
fi
