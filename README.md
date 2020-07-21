I use an Anaconda env named "ros" and build/run stuff as follows
```
colcon build && . install/setup.zsh && export PYTHONPATH=$PYTHONPATH:/home/justin/anaconda3/envs/ros/lib/python3.6/site-packages && ros2 run terra_camera camera_publisher
```
