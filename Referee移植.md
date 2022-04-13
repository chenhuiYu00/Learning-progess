# Referee移植



## 收发数据

> 目标数据
>
> uint16 chassis_volt
> uint16 chassis_current
> float32 chassis_power
> uint16 chassis_power_buffer
> uint16 shooter_heat_cooling_limit
> uint16 shooter_heat
> uint16 robot_hp
> float32 bullet_speed
> uint16 hurt_armor_id
> uint16 hurt_type
>
> 
>
> time stamp



### referee.cpp

```c++
void Referee::publishData()
{
  if (referee_data_.robot_id_ == rm_common::RobotId::RED_HERO ||
      referee_data_.robot_id_ == rm_common::RobotId::BLUE_HERO)
  {
    referee_pub_data_.shooter_heat = referee_data_.power_heat_data_.shooter_id_1_42_mm_cooling_heat_;
    referee_pub_data_.shooter_heat_cooling_limit = referee_data_.game_robot_status_.shooter_id_1_42_mm_cooling_limit_;
  }
  else
  {
    referee_pub_data_.shooter_heat = referee_data_.power_heat_data_.shooter_id_1_17_mm_cooling_heat_;
    referee_pub_data_.shooter_heat_cooling_limit = referee_data_.game_robot_status_.shooter_id_1_17_mm_cooling_limit_;
  }
  referee_pub_data_.chassis_volt = referee_data_.power_heat_data_.chassis_volt_;
  referee_pub_data_.chassis_current = referee_data_.power_heat_data_.chassis_current_;
  referee_pub_data_.chassis_power = referee_data_.power_heat_data_.chassis_power_;
  referee_pub_data_.chassis_power_buffer = referee_data_.power_heat_data_.chassis_power_buffer_;
  referee_pub_data_.robot_hp = referee_data_.game_robot_status_.remain_hp_;
  referee_pub_data_.hurt_armor_id = referee_data_.robot_hurt_.armor_id_;
  referee_pub_data_.hurt_type = referee_data_.robot_hurt_.hurt_type_;
  referee_pub_data_.bullet_speed = referee_data_.shoot_data_.bullet_speed_;
  referee_pub_data_.stamp = last_get_;

/****
*
*
*
*****/

  referee_pub_.publish(referee_pub_data_);
  super_capacitor_pub_.publish(super_capacitor_pub_data_);

}
```



### 传帧逻辑

**从main函数开始**      main.cpp

```c++
referee = new rm_referee::ChassisGimbalShooterReferee(nh);
//裁判系统调用类并给予句柄，命名空间是rm_referee
```

*****



**ChassisGimbalShooterReferee(nh) **  chassis_gimbal_shooter_referee.cpp

ui是在data下的referee的referee()发的

> 在这里调用了ui生成函数，有这些参数，data_数据将会录入其中
>
> ```
> const std::string& name, Graph* graph, uint8_t main_mode, bool main_flag, uint8_t sub_mode,bool sub_flag
> ```

```c++
void ChassisGimbalShooterReferee::run()
//获取的句柄开始初始化并传数据
void ChassisGimbalShooterReferee::drawUi(const ros::Time &time)
//参数data_ trigger_change_ui  fixed_ui
    data_用于判断机器人类型和向_ui传数据
//这个drawui后还有个data里的drawui，最后调用referee_的sendui
    
  virtual void drawUi(const ros::Time& time)
  {
    data_.referee_.sendUi(time);
  }

//看了其他的，shooter有trigger fixed Ui，gimbal有trigger，flash
//总之进入到了referee_base.h，ui.h
```

****



**Data data_**  referee_base.h

```c++
  virtual void drawUi(const ros::Time& time)
  {
    data_.referee_.sendUi(time);
  }

  Data data_;
//作用是在上文被调用并录入参数，发布ui
//有Date类
```

**class Data**  data.h

```c++
// 有sub
    joint_state_sub_ = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &Data::jointStateCallback, this);
 	 actuator_state_sub_
     dbus_sub_
     command_sender_sub_
// 有pub
    ros::NodeHandle root_nh;
    referee_.referee_pub_ = root_nh.advertise<rm_msgs::Referee>("/referee", 1);
    referee_.super_capacitor_pub_ = root_nh.advertise<rm_msgs::SuperCapacitor>
    referee_.game_robot_status_pub_
    referee_.game_status_pub_
    referee_.capacity_data_pub_
//有设置端口，波特率什么的
//有调用DbusData msg，生成的referee_明显和收发有关
```

**class Referee**  referee.h

```c++
//明显上文的话题发布者是在referee_中定义的
ros::Publisher referee_pub_;
//还有发布UI的功能
  void addUi(const rm_common::GraphConfig& config, const std::string& content, bool priority_flag = false);

  void sendUi(const ros::Time& time);

  void sendInteractiveData(int data_cmd_id, int receiver_id, unsigned char data);

  void clearBuffer()
```



> 回到上文的**ChassisGimbalShooterReferee(nh) **  chassis_gimbal_shooter_referee.cpp 
>
> 来看trigger_change_ui

**trigger_change_ui_->update() ** ui.h 

```c++
//一系列trigger，fixed，flash什么的都由uibase派生
//构造函数载入了Data_的引用，句柄，ui_type
class UiBase
explicit UiBase(ros::NodeHandle& nh, Data& data, const std::string& ui_type);
//对上文Data的引用，还有数据容器，同时似乎有其他功能
Data& data_; 
std::map<std::string, Graph*> graph_vector_;

//接着向各类ui载入数据了
//调用graph.h，引用graph类，看起来和position有关
void setCapacitorData(Graph& graph);
```

**Graph(const XmlRpc::XmlRpcValue& config, Referee& referee, int id);**   graph.h

```c++
//看起来被调用的是.h而不是.cpp
//调用了referee_引用
Referee& referee_;
```

graph.cpp

```c++
//定义了类似起始位置，图像尺寸，颜色，标题等数据，集成了发布的功能
Graph::Graph(const XmlRpc::XmlRpcValue& config, Referee& referee, int id) : referee_(referee)
//获取的config是要发布的数据
    
//调用的referee_在于向其addui()载入参数
referee_.addUi(config_, title_ + content_, priority_flag);
```



**UiBase(nh, data, "trigger_change")**  ui.cpp

```c++
//利用重构函数把ui type载入进去，来进入不同的派生类，其他的type还有fix flash
if (rpc_value[i]["name"] == "chassis")
        graph_vector_.insert(
            std::pair<std::string, Graph*>(rpc_value[i]["name"], new Graph(rpc_value[i]["config"], data_.referee_, 1)));
//现在我们知道ui.cpp里面这个在干嘛了，分别载入config，referee，id；config将会载入referee的addui()里发布
//对应于yaml里面有很多"name"，这个vector可能配置了多种图形，并分配了相应的referee的addui()

//现在遍历vector，即将发布了
void UiBase::add()
    for (auto graph : graph_vector_)
  {
    graph.second->setOperation(rm_common::GraphOperation::ADD);
    graph.second->display();
  }


//在后面其他ui将会派生ui base，实现各自的功能
```





ui.cpp调用graph.cpp生成graph_vector，存储所有待发帧，它的数据从referee_来

参数date_进行订阅，并通过referee类完成发布，其内部的referee是一个referee.cpp，集成了读取，ui，发布

```c++


      else
        graph_vector_.insert(std::pair<std::string, Graph*>(rpc_value[i]["name"],
                                                            new Graph(rpc_value[i]["config"], data_.referee_, id_++)));
```





> 小总结：ui发送和话题发送的逻辑大概清楚了，它们都在referee_里面实现：前者将数据载入addui()后通过sendui()发布；后者在read()里调用publishData()发布
>
> 同时也发现一部分debus的语句未删除，可以优化
