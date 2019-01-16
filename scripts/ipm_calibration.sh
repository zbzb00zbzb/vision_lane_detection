#!/bin/bash


# file: ipm_calibration.sh
# author: Kuang Fangjun <csukuangfj at gmail dot com>
# date June 05, 2018


pitch=8.8
yaw=-4
roll=0.1

cx=480
cy=300

fx=700
fy=${fx}

x1=253
y1=214

x2=621
y2=502

out_width=448
out_height=448

image_dir=$HOME/Desktop/images/done/2018-07-12-16-55-21
ipm_result_dir=$HOME/Desktop/images/done

../build/IPMToolExecutable \
  --pitch=$pitch \
  --yaw=$yaw \
  --roll=$roll \
  --cx=$cx \
  --cy=$cy \
  --fx=$fx \
  --fy=$fy \
  --x1=${x1} \
  --y1=$y1 \
  --x2=$x2 \
  --y2=$y2 \
  --out_width=$out_width \
  --out_height=$out_height \
  --image_dir=${image_dir} \
  --ipm_result_dir=${ipm_result_dir}
