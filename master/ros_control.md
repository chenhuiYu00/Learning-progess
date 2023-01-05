# ros_control

是一系列packages，包括控制器接口(controller interfaces)、控制器管理器(controller managers)、传输( transmissions )和硬件接口(hardware_interfaces)。





### 控制器

****

``` shell
· sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
```

> effort_controllers：给定期望力矩控制关节
>
> ```c
> joint_effort_controller         //接收一个effort输入并发送一个effort输出
> joint_position_controller       //接收位置输入并发送位置输出
> joint_group_position_controller //一次设置多个关节位置
> joint_velocity_controller       //接收速度输入并发送力输出，使用PID控制器。
> ```
>
> 
>
> joint_state_controller：读取关节位置，将注册到hardware_interface :: JointStateInterface的所有资源的状态发布到类型为sensor_msgs / JointState的主题。
>
> ```c
> joint_state_controller
> ```
>
> 
>
> position_controllers： 一次设置一个或多个关节位置
>
> ```c
> joint_position_controller               //接收位置输入并发送位置输出 
> joint_group_position_controller         //一次设置多个关节位置
> ```
>
> 
>
> velocity_controllers： 一次设置一个或多个关节速度
>
> ```c
> joint_position_controller              //使用PID控制器接收位置输入并发送速度输出
> joint_velocity_controller              //接收速度输入并发送速度输出
> joint_group_velocity_controller        //一次设置多个关节速度
> ```
>
> 
>
> joint_trajectory_controllers： 用于为整个轨迹添加附加功能
>
> ```c
> position_controller
> velocity_controller
> effort_controller
> position_velocity_controller
> position_velocity_acceleration_controller
> ```



### 硬件接口

ROS control 将硬件接口与ros_controller中的一个结合使用，以向硬件发送和接受命令。

> · Joint Command Interface - 支持命令关节阵列的硬件接口。请注意，这些命令可以具有任何语义含义，只要它们每个都可以由单个double表示即可，它们不一定是effort命令。要指定此命令的含义，请参见派生类：
>
> ```c
> Effort Joint Interface      //用于指挥基于力的关节
> Velocity Joint Interface    //用于指挥基于速度的关节
> Position Joint Interface    //用于指挥基于位置的关节
> ```
>
> 
>
> · Joint State Interfaces - 用于支持读取命名关节数组的状态，每个关节都有一些位置，速度和作用力（力或扭矩）。
>
> 
>
> · Actuator State Interfaces - 用于支持读取命名的执行器阵列的状态，每个执行器都有一定的位置，速度和作用力（力或扭矩）。
>
> 
>
> · Actuator Command Interfaces
>
> ```c
> Effort Actuator Interface             //作用力执行器接口
> Velocity Actuator                     //速度执行器接口
> Position Actuator Interface           //位置执行器接口
> ```
>
> · Force-torque sensor Interface                           //力扭矩传感器接口
>
> · IMU sensor Interface                                           //IMU传感器接口





###  控制器管理器 Controller Manager

![img](https://img-blog.csdnimg.cn/20200525135022843.png#pic_center)

用于管理多个控制器，实现控制器的加载、运行、停止等操作。



**命令操作**

加载、运行、停止等操作：

``` shell
rosrun controller_manager controller_manager <command> <controller_name>
```

where，<command> 为：

- load
- unload
- start
- stop
- spawn: load & start
- kill: stop & unload




一次操作多个控制器：

``` c
// 加载控制器，但不运行
rosrun controller_manager spawner [--stopped] name1 name2 name3

// 加载控制器，并运行
rosrun controller_manager spawner name1 name2 name3

// 停止控制器，但不卸载
rosrun controller_manager unspawner name1 name2 name3
```



查看某个控制器状态：

```bash
rosrun controller_manager controller_manager <command>
```

where，`<command>` 包括：

- list
- list-types
- reload-libraries
- reload-libraries --restore



launch操作：

<!--加载并启动-->
<launch>
    <node pkg="controller_manager" 
    type="spawner" 
    args="controller_name1 controller_name2" />
</launch>

<!--只加载不启动-->
<launch>
  <node pkg="controller_manager"
    type="spawner"
    args="--stopped controller_name1 controller_name2" />
</launch>



管理器可视化：

```bash
rosrun rqt_controller_manager rqt_controller_manager
```



### 传动系统Transmissions

传动系统是control pipeline中的一个基础要素，它转换*efforts/flow*变量，使其*product - power - remains*不变。传输接口实现将工作/流量变量映射到输出*efforts/flow*变量，同时保持功率。

<transmission name="simple_trans">
	 <!--指定传动类型-->
     <type>transmission_interface/SimpleTransmission</type>
	 <!--传动系统所连接的关节-->
     <joint name="foo_joint">
     	  <!--指定硬件接口，注意，当在Gazebo中加载此传动系统是，值应为EffortJointInterface-->
     	  <!--而在RobotHW中加载此传动系统时，值应为hardware_interface/EffortJointInterface-->
          <hardwareInterface>EffortJointInterface</hardwareInterface>
     </joint>
     <actuator name="foo_motor">
     	  <!--可选，指定关节制动器之间机械减速比，并非所有传动系统都需要此标签-->
          <mechanicalReduction>50</mechanicalReduction>
          <!--可选，指定硬件接口-->
          <hardwareInterface>EffortJointInterface</hardwareInterface>
     </actuator>
</transmission>



### 关节约束Joint Limits

是硬件抽象层中的一块，维护一个**关节限位**的数据结构，这些限位数据可以从机器人的URDF文件中加载，也可以ROS的参数服务器上加载（先用YAML配置文件导入ROS parameter server）.

限位数据包括：关节速度、位置、加速度、加加速度、力矩等方面的限位；

还包含安全作用的位置软限位、速度边界（k_v）和位置边界（k_p）等等。



**两种加载方式**

- URDF文件

  ``` 
  <joint name="$foo_joint" type="revolute">
    <!-- other joint description elements -->
  
    <!-- Joint limits -->
    <limit lower="0.0"
           upper="1.0"
           effort="10.0"
           velocity="5.0" />
  
    <!-- Soft limits -->
    <safety_controller k_position="100"
                       k_velocity="10"
                       soft_lower_limit="0.1"
                       soft_upper_limit="0.9" /> 
  </joint>
  
  ```

  

- YAML文件

  ``` 
  joints_limits:
      foo_joint:
          has_position_limits: true
          min_position: 0.0
          max_position: 1.0
          has_velocity_limits: true
          max_velocity: 2.0
          has_acceleration_limits: true
          max_acceleration: 5.0
          has_jerk_limits: true
          max_jerk: 100.0
          has_effort_limits: true
          max_effort: 5.0
      bar_joint:
          has_position_limits: false
          has_velocity_limits: true
          max_velocity: 4.0
  
  ```
  
  
  
  **区别**

​	① 目前只能通过URDF来指定软限位，URDF不支持加速度和加加速度限制，这些可通过YAML提供。
​	② YAML可覆盖URDF中描述的值
​	③ PID增益和控制器设置必须保存在yaml文件中，该文件通过roslaunch文件加载到参数服务器中+++++++

> 硬限位是用机械加工件去实现设备位置的限制，经常用到的是机加件上安装聚氨酯做硬限位。软限位就是通过电器和软件来实现如光电开关、行程开关等等。通常软限位小于硬限位



**配置joint limits接口**

``` c++
#include <joint_limits_interface/joint_limits_interface.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

class MyRobot
{
public:
  MyRobot() {}

  bool init()
  {
    // Populate pos_cmd_interface_ with joint handles...

    // Get joint handle of interest
    JointHandle joint_handle = pos_cmd_interface_.getHandle("foo_joint");

    JointLimits limits;
    SoftJointLimits soft_limits;
    // Populate with any of the methods presented in the previous example...

    // Register handle in joint limits interface
    PositionJointSoftLimitsHandle handle(joint_handle, // We read the state and read/write the command
                                         limits,       // Limits spec
                                         soft_limits)  // Soft limits spec

    jnt_limits_interface_.registerHandle(handle);
  }

  void read(ros::Time time, ros::Duration period)
  {
    // Read actuator state from hardware...

    // Propagate current actuator state to joints...
  }

  void write(ros::Time time, ros::Duration period)
  {
    // Enforce joint limits for all registered handles
    // Note: one can also enforce limits on a per-handle basis: handle.enforceLimits(period)
    jnt_limits_interface_.enforceLimits(period);

    // Porpagate joint commands to actuators...

    // Send actuator command to hardware...
  }

private:
  PositionJointInterface pos_cmd_interface_;
  PositionJointSoftLimitsInterface jnt_limits_interface_;
};

```

