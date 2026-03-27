# 底盘命令说明文档

## 加载底盘硬件连接
```
roslaunch hf_bringup hf_bringup.launch robot_name:=$(robot_name)
```

robot_name可以为hf_0、hf_1、hf_2、hf_3，根据每个机器人而定，例如调试hf_3，则运行:
```
roslaunch hf_bringup hf_bringup.launch robot_name:=hf_3
```

以上这个launch会启动底盘的手柄控制，以及激光雷达相关驱动等等硬件驱动。

## 建图
```
roslaunch hf_carto hf_carto.launch
```

之后使用手柄控制小车建图，建完图运行：

### 终止建图
```
rosservice call /finish_trajectory 0
```

### 保存地图

```
rosservice call /write_state "{filename: 'path/map.pbstream'}"
```

其中 path 为保存路径，例如，保存在home目录下可以为:
```
rosservice call /write_state "{filename: '$(HOME)/mymap.pbstream'}"
```

### 把pbstream地图转换成ros格式地图 以便路径规划使⽤

```
rosrun cartographer_ros cartographer_pbstream_to_ros_map - pbstream_filename=${HOME}/mymap.pbstream - map_filestem=${HOME}/mymap -resolution=0.02
```

## 导航定位

修改hf_navigation.launch中的地图路径文件，包括.pbstream后缀的和.yaml后缀的

之后运行进行导航
```
roslaunch hf_navigation hf_navigation.launch
```

## 标点
```
roslaunch task_communication hf_record_pose.launch 
```

运行之前在该launch文件里修改保存标点位置的json文件路径

## 运动任务接收节点

将多机节点注释掉，运行单机节点

![](image.png)

```
roslaunch task_communication task_comunication.launch 
```

## 模拟接收运动任务
```
rosrun task_communication fake_navigation_in.py
```


**手柄遥控**： 
- **按住** **RB** 使能，左摇杆控制前后左右，右摇杆控制旋转。右摇杆打向左半区为逆时针旋转，打向右半区为顺时针旋转。
- 遥控速度优先级大于导航速度优先级，一旦手柄介入，平台由手柄全权控制。按一下 **LB** 键释放手柄控制权。


**手柄自动充电**
  - 用遥控将平台对准自动充电桩，按住 **RB** 键的同时按住 **A** 键开启充电。之后，先松开 **RB** ，再松开 **A** 键，充电命令将保存进底盘，可以持续充电。