# SJSU Robotics Trial Project

```bash
docker run -it  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/ishan/trial/:/trial --device /dev/dri osrf/ros:humble-desktop
```

create a map with create_map.py, save it

```bash 
cp map.txt src/maze_trial/resource/map.txt
```

run all the nodes