# hero_chassis_controller

基于 rm_template 创建的 ROS 1 Noetic 英雄麦克纳姆轮底盘控制器。
开发和运行环境：Ubuntu 20.04、CLion、catkin-tools、Gazebo 11。

## 四轮速度 PID（选做 4）

通过 pluginlib 导出 `hero_chassis_controller/HeroChassisController`，继承
`controller_interface::Controller<hardware_interface::EffortJointInterface>`。
四个 JointHandle 读取实际角速度、写入力矩，每轮分别持有一个 control_toolbox::Pid。
控制误差为「目标角速度 - 实际角速度」，computeCommand(error, period) 输出 Nm。
参数在 config/controllers.yaml 中逐轮设置；Pid::init(NodeHandle) 同时启用各轮动态调参服务。
初始增益仅用于启动调试，需在目标 Gazebo 环境通过 rqt 实测调整。

只启动这一套控制器，避免示例控制器占用相同的四个关节：

```bash
source ~/chassis_ws/devel/setup.bash
roslaunch hero_chassis_controller run_simulation_and_controller.launch
# 等待模型和控制器加载完成，然后点击 Gazebo 的播放按钮。
rostopic pub -r 10 /controller/hero_chassis_controller/wheel_commands std_msgs/Float64MultiArray "data: [5.0, 5.0, 5.0, 5.0]"
rosrun rqt_reconfigure rqt_reconfigure
rosrun rqt_plot rqt_plot /controller/hero_chassis_controller/left_front_wheel_joint/state/set_point /controller/hero_chassis_controller/left_front_wheel_joint/state/process_value
```

轮序为左前、右前、左后、右后，速度单位 rad/s。命令超时 0.5 s 后目标归零；
力矩不超过模型的 5 Nm，目标转速按同一比例缩放到 50 rad/s 以内。
非法命令、无效控制周期、非有限轮速均有处理。starting() 重置 PID，stopping() 将力矩清零。

## 代码规范（选做 3）

采用 [ROS C++ Style Guide](https://wiki.ros.org/CppStyleGuide)：2 空格缩进、Allman 大括号，
类名 UpperCamelCase、方法 lowerCamelCase、变量 snake_case、成员变量末尾下划线、头文件保护宏。
.clang-format 辅助排版，命名和接口语义需人工审查。

## 仿真加载关系与 CLion

rm_description_for_task 的 xacro 加载 librm_robot_hw_sim.so，
rm_gazebo/RmRobotHWSim 提供 effort 接口和 controller_manager。
spawner 根据 YAML 的 type 查找本包 XML 导出的插件，再调用 init → starting → update → stopping。
实际机器人需要相应 RobotHW 提供同名、同单位的四个 effort 关节。

从已 source /opt/ros/noetic/setup.bash 的终端打开 CLion，导入工作空间并使用 Debug 配置。
构建工具选 catkin-tools（catkin build），不要使用 catkin_make。
在 CLion Attach to Process 中选择 gzserver，在 init()、update() 设置断点。
编译、断点调试及仿真演示须在目标机器由 CLion 实际完成。
