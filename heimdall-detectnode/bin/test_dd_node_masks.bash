#!/usr/bin/env bash

echo "Adding a mask with 1 pixel:"
rosservice call /dd/add_mask test_1 1 1 [1]
echo "List of mask labels:"
rosservice call /dd/get_mask_list 
echo "Adding a mask with 6 pixels:"
rosservice call /dd/add_mask test_2 2 3 [1,1,1,1,1,1]
echo "Contents of mask \"test_2\":"
rosservice call /dd/get_mask test_2 
echo "List of mask labels:"
rosservice call /dd/get_mask_list 
echo "Removing all masks:"
rosservice call /dd/remove_mask_list [test_1,test_2]
echo "List of mask labels:"
rosservice call /dd/get_mask_list 
