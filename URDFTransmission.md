# URDF:Transmission

> 用来描述关节和驱动器之间的关系,功能包括定义关节传动比和连杆之间的关系等。 
>  通过复杂的转化可以把多驱动器和多关节联系在一起。
>
> transmission把TF（ros）的连接关系与仿真平台上的驱动设备（gazebo）联系在了一起，这样，**当物理引擎下，这个驱动设备进行了动作的时候，ros就能知道是哪个TF需要变化**



- 为了让Gazebo可以识别<transmission>标签，需要家在一个gazebo的ros_control插件：

>  *除了传输标签，还需要向URDF添加Gazebo插件，该插件实际解析传输标签并加载适当的硬件接口和控制器管理器。默认情况下，gazebo_ros_control插件非常简单，但它也可以通过附加的插件架构进行扩展，以允许超级用户在ros_control和gazebo之间创建自己的自定义机器人硬件接口.*

```
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/rrbot</robotNamespace>
  </plugin>
</gazebo>
```



``` c++
    //定义关系传递的唯一名字
<transmission name="simple_trans">
//指定类型
  <type>transmission_interface/SimpleTransmission</type>
//指定关节依赖，可定义一个或多个，将会拥有其系列属性
    //关节名
  <joint name="foo_joint">
    //以上关节支持的硬件接口
    <hardwareInterface>EffortJointInterface</hardwareInterface>
  </joint>
//指定连接的致动器，可定义一个或多个，将会拥有以下标签
  <actuator name="foo_motor">
    //（可选）定义电机/关节减速比
    <mechanicalReduction>50</mechanicalReduction>
    //（可选）指定支持的硬件接口
    <hardwareInterface>EffortJointInterface</hardwareInterface>
   </actuator>
</transmission>

```





- hardwareInterface标签的可选项及对应控制器:

```c
//·Joint Command Interfaces           --effort_controllers - 直接控制关节的力/扭矩。 
    Effort Joint Interface                  joint_effort_controller
    Velocity Joint Interface                joint_position_controller
    Position Joint Interface                joint_velocity_controller
//·Joint State Interfaces             --joint_state_controller -读取所有关节位置
                                            joint_state_controller
//·Actuator State Interfaces          --position_controllers   -一次设置1/n个关节
                                            joint_position_controller
                                            joint_group_position_controller
//·Actuator Command Interfaces        --velocity_controllers - 一次设置一个／多个关节速度。
    Effort Actuator Interface               joint_velocity_controller
    Velocity Actuator Interface             joint_group_velocity_controller
    Position Actuator Interface     //--joint_trajectory_controllers -扩展功能用于规划整个轨迹 
                                                position_controller
    											velocity_controller
    											effort_controller
  											  position_velocity_controller
  											  position_velocity_acceleration_controller
        
//·Force-torque sensor Interface            

//·IMU sensor Interface
```




# Plugin

> **transmission只是将两个标签联系在了一起，但是并不代表ros发送一个消息，**gazebo plugin就是在ros和gazebo之间的一座桥梁
>
> *Gazebo Plugins为您的URDF提供了更强大的功能，并可以将ROS消息和服务呼叫与传感器输出和电机输入联系起来。*



``` c++
   //leftJoint和rightJoint要填写你模型中的名称，然后下面的两个wheel separation和wheel diameter也要根据你的轮子相距的距离和轮子的直径 
<gazebo>
        <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>50.0</updateRate>
            <leftJoint>car_base_front_left_wheel</leftJoint>
            <rightJoint>car_base_front_right_wheel</rightJoint>
            <wheelSeparation>1.0</wheelSeparation>
            <wheelDiameter>0.5</wheelDiameter>
            <torque>1.0</torque>
            <commandTopic>cmd_vel</commandTopic>
            <odometryTopic>odom</odometryTopic>
            <odometryFrame>odom</odometryFrame>
            <robotBaseFrame>base_link</robotBaseFrame>
            <publishWheelTF>true</publishWheelTF>
            <publishWheelJointState>true</publishWheelJointState>
            <legecyMode>false</legecyMode>
            <wheelAcceleration>1</wheelAcceleration>
        </plugin>
</gazebo>
```



载入完成后，ros将会多出cmd_vel话题