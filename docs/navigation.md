# Navigation


## Steps

1) Launch simulation

```
ros2 launch butlerbot_gazebo simulation.launch.py
```

2) Edit slam toolbox config and launch slam toolbox

- change slam toolbox mode in config file `butlerbot_localization/config/mapper_params_online_async.yaml`

```
mode: localization
```

- and launch it
```
ros2 launch butlerbot_localization localization.launch.py use_rviz:=false
```

3) Launch the navigation
```
ros2 launch butlerbot_navigation navigation.launch.py
```

4) Give goal using `2D Goal Pose` 

## [Video](https://drive.google.com/file/d/1KVi663ozBQsKZ5WcuzwBk9A3OM_IcRl1/view?usp=drive_link)