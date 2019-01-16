# Usage

## Get the Code and Compile

```.sh
mkdir catkin_ws

git clone git@192.168.3.54:root/vision_lane_detection.git

cd vision_lane_detection
git checkout dev

cd ..

git clone git@192.168.3.54:root/trunktech_msgs.git

cd catkin_ws
mkdir src
ln -s $PWD/../vision_lane_detection src/
ln -s $PWD/../trunktech_msgs/hadmap_msgs src/

catkin_make
. devel/setup.bash
```

## Configurations
 - `config/gantry.yaml`
 - `config/proto.txt`

### config/gantry.yaml
 - use [`ipm_calibration`](http://192.168.3.54/kuangfangjun/ipm_calibration) tool to obtain the following two parameters

```
num_pixels_per_meter_x_dir: 30   # number of pixels per meter in the horizontal direction in the IPM image
num_pixels_per_meter_y_dir: 20   # number of pixel per meter in the vertical direction in the IPM image
```

 - Change the camera topic

```
camera_topic: /pylon_camera_node/image_raw/compressed
```

 - refer to the comment in `config/gantry.yaml` for other parameters

### config/proto.txt

 - use `ipm_calibration` tool to obtain the following 9 parameters (`image_to_ipm_tf`)

```
ipm_param {
  width: 448
  height: 448

use_precomputed_H: true

image_to_ipm_tf: -0.23864383
image_to_ipm_tf: -0.59772766
image_to_ipm_tf: 412.28256
image_to_ipm_tf: -0.044471879
image_to_ipm_tf: -1.4135034
image_to_ipm_tf: 689.36011
image_to_ipm_tf: -8.53082e-05
image_to_ipm_tf: -0.0027097312
image_to_ipm_tf: 1
}
```

 - read the comments in `proto/lane_line.proto` for usages of other parameters. In general,
the default parameters are fine.


## Run

```.sh
roslaunch lane_line gantry.launch
```

Subscribe to the image topic `/lane_detection/visualization`
to view the detection results.


