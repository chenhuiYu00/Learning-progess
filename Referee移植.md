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
//执行了之前的referee_base.cpp的 RefereeBase::drawUi(time); 相当于儿子执行了父亲代码
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



*******

```c++
//sendUi()
一块位于rm_common  rm_referee的飞地
void Referee::sendUi(const ros::Time& time)
{
  if (ui_queue_.empty() || time - last_send_ < ros::Duration(0.05))
    return;
  //发送过于频繁，返回
    
  rm_common::GraphData tx_data;
  int data_len = (int)sizeof(rm_common::GraphData);
  tx_data.header_.sender_id_ = referee_data_.robot_id_;
  tx_data.header_.receiver_id_ = client_id_;
  tx_data.config_ = ui_queue_.back().first;
  //设置发送者，接受者id
    
  if (ui_queue_.back().second.empty())
  {
    tx_data.header_.data_cmd_id_ = rm_common::DataCmdId::CLIENT_GRAPH_SINGLE_CMD;
    data_len -= 30;
    //在手册里面数据长度限制是30
  }
  else
  {
    tx_data.header_.data_cmd_id_ = rm_common::DataCmdId::CLIENT_CHARACTER_CMD;
    for (int i = 0; i < 30; i++)
    {
      if (i < (int)ui_queue_.back().second.size())
        tx_data.content_[i] = ui_queue_.back().second[i];
      else
        tx_data.content_[i] = ' ';
    }
  }
  pack(tx_buffer_, (uint8_t*)&tx_data, rm_common::RefereeCmdId::INTERACTIVE_DATA_CMD, data_len);
    //封装
  tx_len_ = k_header_length_ + k_cmd_id_length_ + k_tail_length_ + data_len;
  ui_queue_.pop_back();
  last_send_ = time;
}
```



在sendUi()的pack()后，产生的tx_buffer最后通过referee_base中调用的的write()载入serial

```c++
 data_.serial_.write(data_.referee_.tx_buffer_, data_.referee_.tx_len_);
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







## 补充

多次派生来的referee

```c++
class ChassisGimbalReferee : public RefereeBase
class ChassisGimbalShooterReferee : public ChassisGimbalReferee
class ChassisGimbalShooterCoverReferee : public ChassisGimbalShooterReferee
```





referee.h ,graph.h, ui.h都由peter写的，我修改的应该是ljq的部分

应该恰好就是referee_base.h   与data.h







## 可以优化的地点

1. referee_base.h部分

   ```c++
   namespace rm_referee
   {
   class RefereeBase
   {
   public:
     explicit RefereeBase(ros::NodeHandle& nh);
     virtual void run();
   
   protected:
     virtual void drawUi(const ros::Time& time)
     {
       data_.referee_.sendUi(time);
     }
    //该函数将会在referee_base.cpp中执行，执行方式是在_referee里执行drawUi()
    //_referee里的drawUi()可以去掉
   
     Data data_;
     ros::NodeHandle nh_;
   };
   }  // namespace rm_referee
   ```

   







# UI跟随系统

> 底盘跟随云台

referee_base中不断调用了run(),可以尝试将坐标写入update()。

难度在于该UI是相对运动的，且需要tf坐标树转换，详情可问阵雨

robotMaster论坛的哈工程



## 一份将要发送的UI

```c++
referee_control = new rm_referee::ChassisGimbalShooterCoverReferee(nh);

trigger_change_ui_->update()
```



## 需要的数据

```c++
//数据来源 joint_state_
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
```



```c++
//图像配置
struct GraphConfig
{
  uint8_t graphic_id_[3];
  uint32_t operate_type_ : 3;
  uint32_t graphic_type_ : 3;
  uint32_t layer_ : 4;
  uint32_t color_ : 4;
  uint32_t start_angle_ : 9;
  uint32_t end_angle_ : 9;
  uint32_t width_ : 10;
  uint32_t start_x_ : 11;
  uint32_t start_y_ : 11;
  uint32_t radius_ : 10;
  uint32_t end_x_ : 11;
  uint32_t end_y_ : 11;
}
```





## 新的理解

```c++
std::map<std::string, Graph *> graph_vector_;
//一份图像容器，从yaml拉取所有的图像配置数据

话题数据/joint_state 下的position数组，第10位也是最后一位元素即为yaw_joint相对数据，该数据在初始状态底盘为对齐云台时为0，向左转一直增，向右转一直减。转一圈数值约为6.2
```

```c++
  double cover_yaw_joint_ = yaw_joint_;
  while (abs(cover_yaw_joint_) > 2 * M_PI){
        cover_yaw_joint_ += cover_yaw_joint_>0 ? -2 * M_PI : 2 * M_PI;
     }
        
  return cover_yaw_joint_;//限制yaw_joint 范围大小



if (start_positions_.size() > 1) {
     config_.start_x_ = 960 - 50 * sin(yaw_joint_);//50表示准星半径
     config_.start_y_ = 540 + 50 * cos(yaw_joint_);
    }
if (end_positions_.size() > 1) {
     config_.end_x_ = 960 - 100 * sin(yaw_joint_);//100-50=50表示绘制准线长度
     config_.end_y_ = 540 + 100 * cos(yaw_joint_);
    }
```



```c++
graph.setOperation(rm_common::GraphOperation::UPDATE);               //手动调用
graph.second->setOperation(rm_common::GraphOperation::ADD);          //以下会通过add()调用，也可重载add()
graph.second->display();
```



```c++
//确认问题
 graph.second->updatePosition(0., time);
  //获取yaw_joint数据有问题
```





# 云台操作手ui

> 飞手与云台手共用ui
>
> 需求：能够显示当前飞镖发射架可发射的状态，有一个舱门打开的状态，显示还有多久开门的时间；
>
> 切换到前哨战，基地位置；发射





## 可能需要的数据

> ```
> 飞镖发射口倒计时：cmd_id (0x0105)
> 
> 0 1 15s 倒计时
> typedef __packed struct
> {
> uint8_t dart_remaining_time;
> } ext_dart_remaining_time_t;
> ```
>
> 

> ```c
> 机器人间交互数据
> typedef struct {
>     uint16_t data_cmd_id_;
>     uint16_t sender_id_;
>     uint16_t receiver_id_;
> } __packed InteractiveDataHeader;
> 
>     typedef struct {
>         InteractiveDataHeader header_data_;
>         uint8_t data_;
>     } __packed InteractiveData;
> ```

> ```c
> rm_common::DartClientCmd dart_cmd_data_{};
> typedef struct
> {
>   uint8_t dart_launch_opening_status_;
>   uint8_t dart_attack_target_;
>   uint16_t target_change_time_;
>   uint8_t first_dart_speed_;
>   uint8_t second_dart_speed_;
>   uint8_t third_dart_speed_;
>   uint8_t fourth_dart_speed_;
>   uint16_t last_dart_launch_time_;
>   uint16_t operate_launch_cmd_time_;
> } __packed DartClientCmd;
> ```

> ```c
> rm_common::DartStatus dart_status_data_{};
> typedef struct
> {
>   uint8_t dart_belong_;
>   uint16_t stage_remaining_time_;
> } __packed DartStatus;
> ```

```c++
referee_data_.dart_client_cmd_.
referee_data_.dart_status_.stage_remaining_time_
```



> 飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人
>
> **dart_launch_opening_status_**：当前飞镖发射口的状态
> 1：关闭；
> 2：正在开启或者关闭中
> 0：已经开启
>
> **dart_attack_target_**：飞镖的打击目标，默认为前哨站；
> 0：前哨站；
> 1：基地。
>
> **target_change_time**:切换打击目标时的比赛剩余时间
>
> 单位秒，从未切换默认为 0。
>
> **operate_launch_cmd_time_**：最近一次操作手确定发射指令时的比赛剩余时间
>
> 单位秒, 初始值为 0





# Msg优化



## power_limit_state

> 在power_limit.h文件中，哨兵功率限制30w，工程300w。
>
> 步兵英雄另行判断：
>
> 准备阶段设置为30w；
>
> 当游戏设置的机器人底盘功率上限大于120w时，设置功率限制为yaml文件的数据的burst_power。
>
> 其他情况下，分别匹配TEST，BURST，NORMAL，CHARGE
>
> 非爆发模式下，当电容的功率限制约等于当前底盘功率限制时，校准电容功率限制。



```c++
trigger_change_ui_->update("chassis", chassis_cmd_sender_->getMsg()->mode,
                           chassis_cmd_sender_->power_limit_->getState() == rm_common::PowerLimit::BURST, 0,
                           chassis_cmd_sender_->power_limit_->getState() == rm_common::PowerLimit::CHARGE);
```



旧manual和新manual有chassis_cmd_sender_，届时旧referee可直接获取chassis_cmd_sender

新referee舍弃了chassis_cmd_sender，转而用data_下的新订阅者获取mode



**该变量贯通新referee与新manual，所有逻辑建立于话题发布，若取消可能需要重写**



#### 新的理解

> 该变量是中间变量，沟通power_limit与裁判系统上GameRobotStatus的uint16_t chassis_power_limit;
>
> **由于推导模式的算法可知，可利用前后状态算出status**

让我们从超级电容的msg中的limit_power倒推出status.

```c
referee_data_.game_robot_status_.chassis_power_limit
```



#### 新的问题

> TEST,CHARGE,NORMAL模式仅通过简单的参数运算，但BURST模式涉及的参数数量繁多



## eject

```c++
  void setEject(bool flag)
  {
    eject_flag_ = flag;
    msg_.eject = flag;
  }
  bool getEject() const
  {
    return eject_flag_;
  }
```

使用方式是在manual中触发按键时有相应的

```
  gimbal_cmd_sender_->setEject(true);
```



> ```c++
> class ChassisCommandSender : public TimeStampCommandSenderBase<rm_msgs::ChassisCmd>
>     //命令发布者的话题发布依靠command_send.h,在该类中派生而出的命令发布方式是设置template class type 为ChassisCmd
> ```



#### 新的理解

> eject相关在command.send中

```c++
  void setRate(double scale_yaw, double scale_pitch)
  {
    msg_.rate_yaw = scale_yaw * max_yaw_rate_;
    msg_.rate_pitch = scale_pitch * max_pitch_vel_;
    if (eject_flag_)//eject的判断字
    {
      msg_.rate_yaw *= eject_sensitivity_;
      msg_.rate_pitch *= eject_sensitivity_;
    }
```

、



## burst_mode

```c++
  void setBurstMode(bool burst_flag)
  {
    heat_limit_->setMode(burst_flag);
    msg_.burst_mode = burst_flag;
  }
  bool getBurstMode()
  {
    return heat_limit_->getMode();
  }
```

