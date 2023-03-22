# 建模导出URDF

> 对于脚本无法处理的模型转URDF，需要手动建立URDF模型

对于一个urdf，属性设置的顺序是link<inertial>->joint->link<visual>/<collision>

以这种方式设置一个link后再设置这个link的子link



## Fusion360

> 我们需要在fusion中获取模型零部件的属性填入之后的urdf中，选中所需要的零部件右键属性后可以分析出物理数据



## Link

> link是机器人的单位实体，因为过大的stl文件会极大影响运行效果，所以需要对机器人的模型进行简化并保留关键部分以让单个stl文件大小小于2MB

一个link的举例：

```
<link name="trigger">
    <inertial>
        <mass value="0.952"/>
        <origin xyz="-0.012227 -0.000055 0.010137"/>
        <inertia ixx="8.90e-4" ixy="2.012e-6" ixz="1.911e-4" iyy="1.768e-3"
                 iyz="-7.72e-7" izz="2.067e-3"/>
    </inertial>
    <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <mesh filename="package://rm_description/meshes/standard/trigger.stl" scale="0.001 0.001 0.001"/>
        </geometry>
    </visual>
    <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
            <mesh filename="package://rm_description/meshes/standard/trigger.stl" scale="0.001 0.001 0.001"/>
        </geometry>
    </collision>
</link>
```



### inertial

> inertial描述了部件的质量，质心位置以及惯性，对于保留模型核心部分的link，它的质量，质心和惯性依然需要以完整体为准。也就是说简化的只是外观的视觉和碰撞部分。

注意：

- mass的单位是kg，origin单位是m，inertia的值要在fusion360给出的值的基础上乘以`10^-9`，例如689.2 -> 6.892E-07

- 所有质心的origin坐标，inertia惯性是相对于一个joint的(自己处于子link时的所对应的joint)。fusion会提供相对于质心的惯性和相对于原点的惯性，请选择相对于质心的惯性

- 所有joint的origin是相对于父joint的坐标(该joint的父link在作为子link时的joint)

- stl文件出的时候需要将模型单独放在一个原点下并进行偏移，直接在机器人模型上出stl会有位移，从而导致需要修改视觉和碰撞。因此在衍生模型后需要通过偏移让joint点(该link作为子link时对应的joint)移到原点(在原模型中计算joint到原点的偏移量)

  

### visual

> link的视觉部分，在填入inertia和joint后再调整，因为只涉及外观，优先度不高



### collision

> link的碰撞部分，通常和视觉部分一样，在填入inertia和joint后再调整，因为只涉及仿真的碰撞，优先度不高





## Joint

> 连接link的关节，以父link相对于父父link关节的位置为相对位置

```
<joint name="pitch_joint" type="revolute">
    <origin xyz="0 0 0.1772" rpy="${pi} ${pi} ${pi}"/>
    <dynamics damping="0.0" friction="0.1"/>
    <limit effort="1.2" velocity="13.82" lower="${pitch_lower_limit}" upper="${pitch_upper_limit}"/>
    <safety_controller k_velocity="0.1"
                       k_position="100"
                       soft_lower_limit="${pitch_lower_limit+threshold}"
                       soft_upper_limit="${pitch_upper_limit-threshold}"/>
    <parent link="yaw"/>
    <child link="pitch"/>
    <axis xyz="0 1 0 "/>
</joint>
```

例如 link1通过joint1相对于base_link旋转；link2通过joint2相对于link1旋转，则joint2的坐标的xyz是相对于joint1位置的偏移量

> 有些机器人建模需要baselink，有的不用，好奇有什么影响。
>
> 如果想发tf的话是需要base_link的



## Gazebo

gazebo中需要检查质心位置，模型位置

### 用 check_urdf 检查

在编写完 urdf 代码后，先使用 check_urdf 来检查是否出现报错。

### 检查坐标系

打开 gazebo ，勾选 **Edit | Reset World Pose** 将机器人还原为初始位置以及状态，然后再勾选 **View | Trasnparent** 将机器人的模型透明化，

完成后勾选 **View | Link Frames** 检查坐标系朝向是否正确。

### 检查惯性矩

在 gazebo 中勾选 **View | inertia** 查看惯性矩是否正确，检查前要确保质心位置是否准确以及质量大小是否准确，检查标准：

- 粉色立方体是否将模型基本包裹住
- 假如质心不在模型的几何中心，那么粉色立方体应该偏向质心位置。

### 检查碰撞箱

勾选  **View | collision**  检查碰撞箱是否基本将模型包裹住。