#/bin/bash
dest=$(rospack find detic_ros)/test/data
wget -nc https://web.eecs.umich.edu/~fouhey/fun/desk/desk.jpg -P $dest
