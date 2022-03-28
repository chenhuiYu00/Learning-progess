# Manual操纵



## chassis_gimbal_manual

底盘云台操纵

### 构造函数

```c++
ChassisGimbalManual::ChassisGimbalManual(ros::NodeHandle &nh) : ManualBase(nh)
    //句柄chassis，vel，ui，gimbal
    //获取参数gyro_move_reduction
    //初始化底盘和云台的命令，与ui界面的链接
```



### 命令发布

```c++
void ChassisGimbalManual::sendCommand(const ros::Time &time) 
    //发布chassis，vel，gimbal命令
    //更新时间
```



###  更新

```c++
void ChassisGimbalManual::updateRc() 
    //更新遥控器
    //匹配底盘运动模式，设置速度的xyz参数
    //云台速率
    
void ChassisGimbalManual::updatePc()
    //
```



### 检查裁判系统

```c++
void ChassisGimbalManual::checkReferee()
    //检查云台，底盘的功率限制
```



### 检查键盘

```c++
void ChassisGimbalManual::checkKeyboard()
    //匹配到红蓝方的机器人id
    //前后左右wsad
    //鼠标位置
```



### 绘制UI

```c++
void ChassisGimbalManual::drawUi(const ros::Time &time)
    //capacitor，chassis，spin，armor的数据
```



------

==debus==

### 遥控开关

```c++
void ChassisGimbalManual::remoteControlTurnOff()
    //vel,gimbal,chassis置零setZero()
```



### 左右拨动开关

```c++
void ChassisGimbalManual::rightSwitchDownRise() 
    //右开关拨到下
    //底盘模式：FOLLOW 云台模式：RATE
    //速度置零
    
void ChassisGimbalManual::rightSwitchMidRise()
    //右开关拨到中
    //底盘模式：FOLLOW 云台模式：RATE

void ChassisGimbalManual::rightSwitchUpRise()
    //右开关拨到上
    //底盘模式：FOLLOW 云台模式：RATE
    //速度置零
    //trigger，time，fixed UI配置
    
void ChassisGimbalManual::leftSwitchMidFall()
    //左开关拨到中
    //底盘命令发布：充电
    // chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
    
void ChassisGimbalManual::leftSwitchDownRise()
    //左开关拨到下
    //云台命令发布：RATE
```



### 按键开关

```c++
void ChassisGimbalManual::wPressing()
    //w键按下
    //底盘命令获取，录入速度x轴方向命令
void ChassisGimbalManual::wRelease()
    //w键盘松开
    //底盘命令检查，设置速度x轴方向命令
    //x_scale_ = x_scale_ <= -1.0 ? -1.0 : x_scale_ - 1.0;
    
同理
    //s键：X轴
    // x_scale_ = x_scale_ >= 1.0 ? 1.0 : x_scale_ + 1.0;
    
    //a键：Y轴
    // y_scale_ = y_scale_ <= -1.0 ? -1.0 : y_scale_ - 1.0;
    
    //d键：Y轴
    //y_scale_ = y_scale_ >= 1.0 ? 1.0 : y_scale_ + 1.0;
```



### 鼠标

```c++
void ChassisGimbalManual::mouseMidRise() 
    //控制云台
```

