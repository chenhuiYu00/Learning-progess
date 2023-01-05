# 雷达UI

> 需求：由雷达手节点发布包含敌机器人和敌机器人动作的msg，在雷达本身订阅到该msg后封装为ui转发给服务端。最终ui由服务端转发给所有选手端

## 要点

- ui接受者不再是自身，而是全体机器人

- 数据很简单，ui为普通文字ui



## 探索

> 在rm_referee的referee.cpp中，功能得到大部分定义与实现，在这里我们能够发现ui被自定义转发给了自己

```c
//机器人获取自身id，该函数在run()中反复执行
void Referee::getRobotInfo()
{
  referee_data_.robot_id_ = referee_data_.game_robot_status_.robot_id_;
  referee_data_.robot_color_ = referee_data_.robot_id_ >= 100 ? "blue" : "red";
  if (referee_data_.robot_id_ != rm_common::RobotId::BLUE_SENTRY &&
      referee_data_.robot_id_ != rm_common::RobotId::RED_SENTRY)
  {
    switch (referee_data_.robot_id_)
    {
      case rm_common::RobotId::BLUE_HERO:
        client_id_ = rm_common::ClientId::BLUE_HERO_CLIENT;
        break;
      case rm_common::RobotId::BLUE_ENGINEER:
        client_id_ = rm_common::ClientId::BLUE_ENGINEER_CLIENT;
        break;
      case rm_common::RobotId::BLUE_STANDARD_3:
        client_id_ = rm_common::ClientId::BLUE_STANDARD_3_CLIENT;
        break;
      case rm_common::RobotId::BLUE_STANDARD_4:
        client_id_ = rm_common::ClientId::BLUE_STANDARD_4_CLIENT;
        break;
      case rm_common::RobotId::BLUE_STANDARD_5:
        client_id_ = rm_common::ClientId::BLUE_STANDARD_5_CLIENT;
        break;
      case rm_common::RobotId::RED_HERO:
        client_id_ = rm_common::ClientId::RED_HERO_CLIENT;
        break;
      case rm_common::RobotId::RED_ENGINEER:
        client_id_ = rm_common::ClientId::RED_ENGINEER_CLIENT;
        break;
      case rm_common::RobotId::RED_STANDARD_3:
        client_id_ = rm_common::ClientId::RED_STANDARD_3_CLIENT;
        break;
      case rm_common::RobotId::RED_STANDARD_4:
        client_id_ = rm_common::ClientId::RED_STANDARD_4_CLIENT;
        break;
      case rm_common::RobotId::RED_STANDARD_5:
        client_id_ = rm_common::ClientId::RED_STANDARD_5_CLIENT;
        break;
    }
  }
}
```

```c
//在sendUi()中，向帧内插入了client_id即上文赋值的id
tx_data.header_.sender_id_ = referee_data_.robot_id_;
tx_data.header_.receiver_id_ = client_id_;
```

