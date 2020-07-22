I use an Anaconda env named "ros" and build/run stuff as follows
```
colcon build && . install/setup.zsh && source terra_camera/rmp_nav/set_envs.sh  && export PYTHONPATH=$PYTHONPATH:/home/justin/anaconda3/envs/ros/lib/python3.6/site-packages && ros2 run terra_camera thing_to_run
```
