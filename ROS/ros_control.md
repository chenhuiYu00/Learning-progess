# ros_control

>  一组包，包括控制器接口、控制器管理器、传输和硬件接口

> [github_wiki](https://github.com/ros-controls/ros_control/wiki)



## hardware_interface

本质上是一些变量(电机数据和电机命令)通过interface传来传去，以Handle为单位被Controller修改

> [官方文档：配置一个机器人](http://docs.ros.org/en/noetic/api/hardware_interface/html/c++/)
>
> 通常需要：
>
> 1. 重写init() read() write()，实现硬件获取/发送数据功能
> 2. 注册interface handle， 将变量和joint联系，封装成Handle注册进interface
> 3. 设置transmission，在模型有执行器时需要，配置执行器和关节的传动关系
> 4. 设置jointlimit，可选
> 4. 加载controller， controller通过关节名在interface匹配该关节的handle

### 串口案例

> 一个基础的案例，电脑通过串口获取舵机信息再处理后将命令通过串口发给电机



#### 派生hw

> hw将硬件与电脑相沟通，从硬件读取的信息会保存在成员变量里。它的结构如下

```c++
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>

// 舵机驱动程序
class MyRobotHW : public hardware_interface::RobotHW
{
public:
    MyRobotHW() {}

    void read()
    {
        // 读取舵机状态
        // ......
    }

    void write()
    {
        // 发送命令到舵机
        // ......
    }

private:
    // 舵机状态变量
    double position_;
    double velocity_;
    double effort_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_robot_controller");
    ros::NodeHandle nh;

    MyRobotHW robot_hw;
    controller_manager::ControllerManager cm(&robot_hw);

    ros::Time previous_time = ros::Time::now();
    ros::Rate rate(50); // 50Hz
    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();
        ros::Duration dt = current_time - previous_time;
        cm.update(current_time, dt);

        previous_time = current_time;
rate.sleep();
	}
    return 0；
}
```

> ```c
> 上面的代码中，MyRobotHW 类是舵机驱动程序，它继承自 hardware_interface::RobotHW。 read() 和 write() 方法分别用于读取舵机状态和发送命令到舵机。
> 
> 在主程序中，首先创建一个 ros::NodeHandle 对象，然后创建一个 MyRobotHW 类的实例并传递给 controller_manager。在 while 循环中调用 cm.update() 方法来更新控制器的状态。
> 
> 注意，这只是一个示例代码，实际上你还需要根据你的舵机型号来实现自己的驱动程序，如读取舵机状态，发送命令，以及配置舵机的参数等。
> ```



##### 以USART方式通讯

> https://blog.csdn.net/qq_52785580/article/details/122877511









#### 联系joint

> 由于ros还不知道我们之前保存的变量是什么意思，也不知道是什么关节，所以向ros注册一些interface

##### 从电机获得的数据

Controller manager 通过在 MyRobotHW 类中注册相应的状态变量来知道相应的舵机状态，而节点`joint_state_publisher`在从interface中获取到关结数据后会把它发布在/joint_state话题上。

配置变量与关节的联系的通常步骤是：

- 定义hardware_interface::JointStateHandle对象，说明关节名字和相应变量
- 把该对象注册进hardware_interface::JointStateInterface对象里



例如，注册位置变量：

```c++

MyRobotHW::MyRobotHW()
{
    // Register position variable
    hardware_interface::JointStateHandle state_handle("joint_name", &pos_, &vel_, &eff_);
    jnt_state_interface.registerHandle(state_handle);
}
```

controller_manager 会在启动时调用 MyRobotHW 类中的 registerInterfaces() 方法，来注册所有的状态变量。

```c++
hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle("joint_name"), &cmd_);
jnt_vel_interface.registerHandle(vel_handle);
```

这样 controller_manager 就知道了"joint_name"是对应的舵机速度了，并且它会根据该舵机状态来更新控制器。



##### 发给电机的指令

###### hardware_interface/EffortJointInterface

基于力矩控制的控制器所使用的interface，我们把力矩指令以指针传给JointHandle后注册，之后控制器可以调用这些Handle并修改力矩指令。我们要做的是把指令通过write()发给电机

**控制器本质上也是先获取Handle，再通过Handle里的`setCommand()`函数修改指令而是实现的！我们自己的controller因而需要先获取那些Handle**

以`effort_controllers/JointPositionController`为例，这是一个基于力控制的位置控制器。我们从电机获取位置，发布期望的位置，再经过控制器处理后控制器返回我们要发给电机的力的大小。

基于力控制那就要准备`hardware_interface::EffortJointInterface     effort_joint_interface_;`上面的代码已经说了位置(&pos)和期望(&cmd)部分，对于力部分

```cpp
hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_);
//joint_effort_command_就是控制器返回给我们的指令
	effort_joint_interface_.registerHandle(jointEffortHandle);

registerInterface(&effort_joint_interface_);
```



与rm自定义的GpioInterface类似，它就是基于gpio控制的控制器所使用的interface







#### 使用controller

> 使用控制器之前需要我们在配置文件里面注册控制器，并向控制器加载interface

```c++
//代码里面的控制器
effort_controllers::JointVelocityController ctrl_friction_l_, ctrl_friction_r_;
effort_controllers::JointPositionController ctrl_trigger_;

//“ ”内是配置文件里面定义的控制器名字
ros::NodeHandle nh_friction_l = ros::NodeHandle(controller_nh, "friction_left");
ros::NodeHandle nh_friction_r = ros::NodeHandle(controller_nh, "friction_right");
ros::NodeHandle nh_trigger = ros::NodeHandle(controller_nh, "trigger");

//从hw获取一个interface
effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();

//
return ctrl_friction_l_.init(effort_joint_interface_, nh_friction_l) &&
       ctrl_friction_r_.init(effort_joint_interface_, nh_friction_r) &&
       ctrl_trigger_.init(effort_joint_interface_, nh_trigger);
```



yaml文件的控制器参数

```yaml
  friction_left:
    joint: "left_friction_wheel_joint"
    pid: { p: 0.0007, i: 0.005, d: 0.0, i_clamp_max: 0.01, i_clamp_min: -0.01, antiwindup: true, publish_state: true }
  friction_right:
    joint: "right_friction_wheel_joint"
    pid: { p: 0.0007, i: 0.005, d: 0.0, i_clamp_max: 0.01, i_clamp_min: -0.01, antiwindup: true, publish_state: true }
  trigger:
    joint: "trigger_joint"
    pid: { p: 50.0, i: 0.0, d: 1.3, i_clamp_max: 0.0, i_clamp_min: 0.0, antiwindup: true, publish_state: true }
```





## 两个节点发布器

`robot_state_publisher` 和 `joint_state_publisher` 的主要区别在于它们所发布的信息类型不同。

`robot_state_publisher` 是一个 ROS 节点，用于发布机器人的全局状态，例如位置和姿态。它通常发布在 TF 树中，这样所有关于机器人位置和姿态的信息就能在整个 ROS 系统中被共享和使用。

`joint_state_publisher` 是一个 ROS 节点，用于发布机器人的关节状态。它发布的信息主要是关于关节的位置，速度和力，并且通常被其他 ROS 控制器使用。

总的来说，如果需要发布关于机器人的全局状态信息，应该使用 `robot_state_publisher` ；如果需要发布关于机器人的关节状态信息，应该使用 `joint_state_publisher`。



### joint_state_publisher

> 注册joint state handle，并将它们关联到对应的关节状态变量

```c
    // Register position variable
    hardware_interface::JointStateHandle state_handle("joint_name", &pos_, &vel_, &eff_);
    jnt_state_interface.registerHandle(state_handle);

   //register interface
   registerInterface(&jnt_state_interface);
```



### robot_state_publisher

> 创建一个robot_state_publisher对象并在代码中调用它的update函数，使用自己派生的hardware interface里面的数据来更新关节信息





## 两个控制器管理器

`controller_manager::ControllerManager`是`controller_manager`包中定义的一个类，它封装了所有控制器管理的功能，例如加载控制器，让控制器启动或停止。

`node controller manager`是ROS系统中的一个节点，它通过ROS消息和服务来管理控制器。

实际上，通常使用的是`controller_manager::ControllerManager`类来管理控制器，而不是使用`node controller manager`节点



### ControllerManager类

> 使用controller_manager::ControllerManager类管理控制器可以分为以下几步：
>
> 1. 创建一个controller_manager::ControllerManager对象。
> 2. 在循环中调用ControllerManager对象的loadControllers方法，用于加载控制器。
> 3. 调用ControllerManager对象的update方法，根据控制周期更新控制器的状态。
> 4. 如果需要，调用ControllerManager对象的unloadControllers方法卸载不再使用的控制器。

```c
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_robot_hardware_interface");
  ros::NodeHandle nh;

  // 创建一个MyRobotHW对象，表示机器人的硬件接口
  MyRobotHW robot;

  // 创建一个ControllerManager对象
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(100.0);
  while (ros::ok())
  {
    robot.read();
    cm.update(robot.getTime(), robot.getPeriod());
    robot.write();
    rate.sleep();
  }

  return 0;
}
```



### Controller manager节点

这个不必多说



#### 一些问题

- controller manager包里面的controller manager正常运行，但是spawner则会提示

```c
[main]: Controler Spawner couldn't find the expected controller_manager ROS interface
```



- controller manager没有任何输出，也不载入控制器

1. 可能是代码写错了，检查launch文件

```html
    <!-- controller manager -->
    <node name="controller_manager" pkg="controller_manager"  type="controller_manager" output="screen"
          args="load
	 sg90_mechanical_arm_controller/joint_state_controller
	 sg90_mechanical_arm_controller/middle_rotation_joint_position_controller
	 sg90_mechanical_arm_controller/rotation_baselink_joint_position_controller
	 sg90_mechanical_arm_controller/left3_middle_joint_position_controller
	 	"/>
```

2. interface配置错误

   没有载入transmission文件/在代码中载入transmission



## Gazebo与实际电机的硬件接口

> 在 Gazebo 中加载 URDF 和操作实际电机时，需要使用的硬件接口不同。
>
> Gazebo 模拟的环境可以通过使用 hardware_interface::PositionJointInterface 来实现，但是操作实际电机可能需要使用更高级的硬件接口，如 hardware_interface::PositionActuatorInterface。
>
> 这样的原因是gazebo是简单的关节控制，而实际上可能会用更复杂的方式驱动关节

因此，您需要确保在 URDF 文件中描述的 transmission 中包含适当的硬件接口，并在加载 transmission 时使用相应的硬件接口。





## URDF

### Transmission

> [wiki](https://github.com/ros-controls/ros_control/wiki/transmission_interface)
>
> 告诉控制器机器人执行器的类型以及与相应joint对应的执行器，
>
> <type>硬件接口的类型：说明执行器的性质。例如`hardware_interface/PositionJointInterface`说明是一种基于位置控制的执行器，而这个执行器需要注册positionJoinitInterface
>
> <joint name>：URDF定义的joint的名字。
>
> <mechanicalReduction>减速比：通过注册interface，我们知道轮子转一圈相应的joint的值也会改变。而transmission则是表示轮子转一圈时电机要转多少圈。

我们可以通过launch或代码的方式来载入transmission

- [urdf载入](http://docs.ros.org/en/melodic/api/transmission_interface/html/c++/classtransmission__interface_1_1TransmissionInterfaceLoader.html#details)

注册hardware interface -> RobotTransmissions.load(urdf) -> 从RobotTransmissions获取转换得到的act_joint transmission interface -> propagate()

```c
<?xml version="1.0" ?>
<robot name="sg90_mechanical_arm" xmlns:xacro="http://www.ros.org/wiki/xacro" >

<transmission name="middle_rotation_joint_tran">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="middle_rotation_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="middle_rotation_joint_actr">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

1. 注册hardware interface

只需要注册ActuatorInterface部分，positionJointInterface会自动生成，之后从nh里面拿就行了。硬要注册和化会报错

```
hardware_interface::PositionActuatorInterface position_act_interface_;
hardware_interface::ActuatorStateInterface act_state_interface_;
```



2. 从RobotTransmissions获取转换得到的act_joint transmission interface

```c
  transmission_interface::ActuatorToJointStateInterface *act_to_jnt_state_{};
  transmission_interface::JointToActuatorPositionInterface
      *jnt_to_act_effort_{};

//转换关系已经完成了，但是我们需要手动获取
  act_to_jnt_state_ =
      robot_transmissions_
          .get<transmission_interface::ActuatorToJointStateInterface>();
  jnt_to_act_effort_ =
      robot_transmissions_
          .get<transmission_interface::JointToActuatorPositionInterface>();
```

3. propagate()

**最重要的一步，将转换关系广播到interface中，没有它controller manager就无法载入相应控制器**

```c++
  void read()
  {
    // Read actuator state from hardware
    // ...

    // Propagate current actuator state to joints
    act_to_jnt_state.propagate();//执行器数据转joint
  }

  void write()
  {
    // Porpagate joint commands to actuators
    jnt_to_act_pos.propagate();//joint转执行器

    // Send actuator command to hardware
    // ...
  }
```

> `act_to_jnt_state.propagate();`将电机给的数据(直接从read()中获取的值)通过设定的transmissioin转换为joint的值
>
> `jnt_to_act_pos.propagate();` 控制器通过Handle设置了给joint的命令，现在通过设定的transmission将命令转化为给电机的命令，再通过write()发送





- [代码载入](https://github.com/ros-controls/ros_control/wiki/transmission_interface)

设置type对象，执行器对象 -> 注册act_joint transmission interface -> propagate()

载入URDF的时候需要string描述文件，我们可以在代码中获取launch文件里面的param

```c++
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>

int main(int argc, char** argv)
{
  using namespace transmission_interface;

  // Raw data
  double a_pos;
  double j_pos;

  // Transmission
  SimpleTransmission trans(10.0); // 10x reducer

  // Wrap raw data
  ActuatorData a_data;
  a_data.position.push_back(&a_pos);

  JointData j_data;
  j_data.position.push_back(&j_pos);

  // Transmission interface
  ActuatorToJointPositionInterface act_to_jnt_pos;
  act_to_jnt_pos.registerHandle(ActuatorToJointPositionHandle("trans", &trans, a_data, j_data));

  // Propagate actuator position to joint space
  act_to_jnt_pos.propagate();
}
```



在代码中注册position_joint_interface

```c++
hardware_interface::JointStateHandle rotation_baselink_joint_state_handle(
    "rotation_baselink_joint", &angle_[0], &vel_[1], &effort_[1]);

joint_state_interface_.registerHandle(rotation_baselink_joint_state_handle);
hardware_interface::JointHandle joint_state_handle(
    joint_state_interface_.getHandle("rotation_baselink_joint"), &cmd_[1]);

position_joint_interface_.registerHandle(joint_state_handle);

registerInterface(&position_joint_interface_);
```

```c++
    std::string robot_description;
    // ...load URDF from parameter server or file
    // Perform actual transmission loading
    if (!transmission_loader_->load(robot_description)) {return false;}
```







### 注意

- ROS似乎会载入URDF中注释掉的link joint和transimission，请完全删除它们

- hardware_interface::ActuatorStateInterface会自动生成一个hardware_interface::PositionJointInterface，这个interface给controller用的，controller生成指令后还需要通过transmission生成最终电机命令，这就是`jnt_to_act_pos.propagate();`的工作之一



### 报错

#### Does not expose the required hardware

>  		Actuator 'middle_rotation_joint_motor'does not expose the required hardware interface 'hardware_interface::ActuatorStateInterface'.

除了joint，还需要注册一下acutor。类型可以是

`hardware_interface::ActuatorStateHandle`的作用类似于JointStateHandle，是获取电机数据层

```c
  hardware_interface::ActuatorStateHandle middle_rotation_act_state(
      "middle_rotation_joint_motor", &angle_[2], &vel_[2], &effort_[2]);
  act_state_interface_.registerHandle(middle_rotation_act_state);
```

`hardware_interface::ActuatorHandle`的作用类似于JointHandle，是电机命令层

```c++
  hardware_interface::ActuatorHandle middle_rotation_act_handle(
      act_state_interface_.getHandle("middle_rotation_joint_motor"), &cmd_[2]);
  position_act_interface_.registerHandle(middle_rotation_act_handle);
```

`hardware_interface::PositionActuatorInterface`的作用类似PositionJointInterface，用于注册

```c++
hardware_interface::PositionActuatorInterface position_act_interface_;
//其他类型还有EffortActuatorInterface等，与urdf中的transmission对应

registerInterface(&position_act_interface_);
```



#### Failed to load transmission

> ```c
>     /st_hardware: [TransmissionInterfaceLoader::load]: Failed to load transmission 'middle_rotation_joint_tran'. It contains no valid hardware interfaces.
> ```

这个报错和上面上面的那个“not expose the required hardware“的报错一起出现的，解决上面后发现这个报错还在。

可能是两个handle操控同一个关节是不允许的所以注释掉了一个后正常了

**PositionJointInterface会自动生成，之后从nh里面拿Handle就行了**

```c++
//  hardware_interface::JointStateHandle rotation_baselink_joint_state_handle(
//      "rotation_baselink_joint", &angle_[0], &vel_[1], &effort_[1]);
//  joint_state_interface_.registerHandle(rotation_baselink_joint_state_handle);
//  hardware_interface::JointHandle joint_state_handle(
//      joint_state_interface_.getHandle("rotation_baselink_joint"),
//      &cmd_[1]);
//  position_joint_interface_.registerHandle(joint_state_handle);
hardware_interface::ActuatorStateHandle rotation_baselink_act_state(
    "rotation_baselink_joint_motor", &angle_[0], &vel_[1], &effort_[1]);
act_state_interface_.registerHandle(rotation_baselink_act_state);
hardware_interface::ActuatorHandle rotation_baselink_act_handle(
    act_state_interface_.getHandle("rotation_baselink_joint_motor"),
    &cmd_[1]);
position_act_interface_.registerHandle(rotation_baselink_act_handle);
```

拿handle的部分：

```c++
auto effort_joint_interface = this->get<hardware_interface::EffortJointInterface>();
std::vector<std::string> names = effort_joint_interface->getNames();
for (const auto& name : names)
  effort_joint_handles_.push_back(effort_joint_interface->getHandle(name));
```







#### Replacing previously registered handle

>```c=+
>[ResourceManager<ResourceHandle>::registerHandle]: Replacing previously registered handle 'rotation_baselink_joint_motor' in 'hardware_interface::PositionActuatorInterface'.
>```

发生了重复注册，在代码中

```c
  hardware_interface::ActuatorStateHandle middle_rotation_act_state(
      "middle_rotation_joint_motor", &angle_[2], &vel_[2], &effort_[2]);
  act_state_interface_.registerHandle(middle_rotation_act_state);
  hardware_interface::ActuatorHandle middle_rotation_act_handle(
      act_state_interface_.getHandle("middle_rotation_joint_motor"), &cmd_[2]);
  position_act_interface_.registerHandle(middle_rotation_act_handle);
```

hardware_interface::ActuatorHandle注册时应该从act_state_interface_使用`getHandle()`，而不是直接：

```c
  hardware_interface::ActuatorHandle middle_rotation_act_handle(
      middle_rotation_act_state, &cmd_[2]); //重复注册
```







## Clock

> [wiki](http://wiki.ros.org/Clock)
>
> [同步两台机器的时钟](https://www.shuzhiduo.com/A/l1dyElaAde/)

- 模拟时钟与挂壁时钟

launch设置这个参数使得ros::Time::now()读取模拟时钟

```c
<arg name="use_sim_time" value="true"/>
```



- 启动roscore后/clock话题上就会发布时间了吗？

不一定。启动 roscore 后，如果其他节点（如模拟节点）需要使用 /clock 话题，则会在发布 /clock 话题的数据。因此，启动 roscore 仅仅创建了 ROS 系统的核心服务器，但不会生成 /clock 话题。如果需要 /clock 话题，请启动需要的节点。



- 谁在往这个话题发消息？

某些特定的节点



- 怎么启动这些节点？

启动动一个ROS时钟节点。常见的时钟节点是 ros::WallTimer 和 ros::Rate。您可以使用 ros::WallTimer 设置一个定时器，以在一个固定的频率发布 /clock 话题，或者使用 ros::Rate 限制另一个节点的频率









### 一些问题

- while(ros::ok())语句只执行一次

首先重启roscore，但是我是在终端里启动roscore后才正常。如果在X-terminal里启动就会这样。

但是之后在X-terminal里启动后又没事了

```
之后我发现启动rm_gazebo empty_world.launch后再启动我的start.launch就会这样，盲猜是两个的clock冲突了才导致这样
```











# 代码优化

## 循环注册Handle

我是一个个按关节注册的，但是下面的是用循环替代

```c++
  auto effort_joint_interface = this->get<hardware_interface::EffortJointInterface>();
  std::vector<std::string> names = effort_joint_interface->getNames();
  for (const auto& name : names)
    effort_joint_handles_.push_back(effort_joint_interface->getHandle(name));
```





## 主循环回调化

[来源](https://blog.csdn.net/AFATAR/article/details/107453318?spm=1001.2014.3001.5506)

通过一个类里面的周期定时器调用update()函数，而在代码主循环里面改为：

```c++
{
    ros::init(argc, argv, "single_joint_hardware_interface");
    ros::NodeHandle nh;
    //ros::AsyncSpinner spinner(4);  
    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    ROBOTHardwareInterface ROBOT(nh);
    //spinner.start();
    spinner.spin();
    //ros::spin();
    return 0;
```

不再是while(ros::ok())  ros::spinOnce()  loop_rate.sleep();的形式









# 报错

## 编译时

### /usr/bin/ld: CMakeFiles

```c
/usr/bin/ld: CMakeFiles/st_hardware.dir/src/common/hardware_interface.cpp.o: in function `steering_engine_hw::StRobotHW::loadUrdf(ros::NodeHandle&)':

hardware_interface.cpp:(.text+0x11d6): undefined reference to `urdf::Model::initString(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&)'
```

这种报错的原因是CMakelist文件里面没有包含一些库，例如上面的就是缺少“urdf".

这个报错在写代码时发现不了，只有编译时才会出现



### Clion与终端冲突

这并不会报错，当你在clion编译一次后再修改代码在终端编译，此时修改的代码不会生效还是原来的包。

catkin clean一下即可