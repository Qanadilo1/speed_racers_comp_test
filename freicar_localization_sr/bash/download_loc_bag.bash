#!/bin/bash
if [ ! -f ../loc_bags/freicar_loc_2.bag ]; then
    echo "Downloading freicar_loc_2.bag!"
    mkdir ../loc_bags/
    wget http://aisdatasets.informatik.uni-freiburg.de/freicar/bags/freicar_loc_2.bag
    mv freicar_loc_2.bag ../loc_bags/
else
	echo "File exists..."
fi

