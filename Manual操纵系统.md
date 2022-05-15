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



## chassis_gimbal_shooter_manual

射击初始化及操纵控制



### 构造函数

```c++
ChassisGimbalShooterManual::ChassisGimbalShooterManual(ros::NodeHandle &nh) : ChassisGimbalManual(nh)
    //节点shooter，detection_switch
    //获取参数trigger_calibration
    //征集扳机供能，自我检查，游戏开始，拨动开关，按键操作，鼠标移动等事件
```



### 启动

```c++
void ChassisGimbalShooterManual::run()
    //设置敌方阵营的颜色
    //调用update()
```



### 检查裁判系统

```c++
void ChassisGimbalShooterManual::checkReferee() 
    //征集发射供电，自我检查，游戏开始事件
```



### 按键操作

```c++
void ChassisGimbalShooterManual::checkKeyboard()
    //e g q f b
    //ctrl+ c v r b
    //shift
    //鼠标左击，鼠标右击
    
void ChassisGimbalShooterManual::sendCommand(const ros::Time &time)
    //发送命令，调用sendCommand()
```



### 远程遥控开关

```c++
void ChassisGimbalShooterManual::remoteControlTurnOff() 
    //关闭遥控
    shooter_cmd_sender_->setZero();
    trigger_calibration_->stop();

void ChassisGimbalShooterManual::remoteControlTurnOn() 
	//开启遥控
```



### 底盘更新

```c++
void ChassisGimbalShooterManual::chassisOutputOn() 
    //更新底盘数据，检查功率限制
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
```



### 发射开关

```c++
void ChassisGimbalShooterManual::shooterOutputOn()
    //设置Mode，重置扳机
```



### 绘制Ui

```c++
void ChassisGimbalShooterManual::drawUi(const ros::Time &time)
    //阵营，装甲板，发射
    //fixed_ui
```



### 更新遥控器，电脑

```c++
void ChassisGimbalShooterManual::updateRc() 
    //更新遥控器操作的发射，云台数据
    //弹速
    
void ChassisGimbalShooterManual::updatePc()
    //更新电脑端的底盘命令
```



### 拨动开关

```c++
void ChassisGimbalShooterManual::rightSwitchDownRise() 
    //右拨动开关下拨
    //底盘充电，停止射击
    
void ChassisGimbalShooterManual::rightSwitchMidRise()
    //右拨动开关中间
    //同上
    
void ChassisGimbalShooterManual::rightSwitchUpRise()
    //右拨动开关上拨
    //同上
    
void ChassisGimbalShooterManual::leftSwitchDownRise()
    //左拨动开关下拨
    //底盘模式：TRACK 停止射击
    
void ChassisGimbalShooterManual::leftSwitchMidRise()
    //左拨动开关中间
    //底盘模式：TRACK 准备射击
    
void ChassisGimbalShooterManual::leftSwitchUpRise()
    //左拨动开关上拨
    //底盘模式：TRACK 获取弹速
    
void ChassisGimbalShooterManual::leftSwitchUpOn(ros::Duration duration) 
    //持续时间大于1s，执行射击
    //小于1s，准备射击
```



### 鼠标事件

```c++
void ChassisGimbalShooterManual::mouseLeftPress()
    //左击，发射一次，计算误差
    
void ChassisGimbalShooterManual::mouseRightPress() 
    //右击，更新云台
    //弹速，子弹花费
```



### 摁键

```c++
void ChassisGimbalShooterManual::gPress() 
    //底盘切换小陀螺模式GYRO和跟随模式FOLLOW
    //shift键：每触发则功率限制为NORMAL
    
void ChassisGimbalShooterManual::ePress()
    //底盘切换扭腰模式TWIST和跟随模式FOLLOW
 
void ChassisGimbalShooterManual::bPress()
    //充电CHARGR
    
##wsad##
void ChassisGimbalShooterManual::wPress()
void ChassisGimbalShooterManual::aPress()
void ChassisGimbalShooterManual::sPress()
void ChassisGimbalShooterManual::dPress()
    //
    
##shift##
void ChassisGimbalShooterManual::shiftPress() 
    //切换底盘运动模式为FOOLOW
    //超级电容加速 功率限制：BRUST
void ChassisGimbalShooterManual::shiftRelease()
    //非小陀螺模式下 功率限制：NORMAL
    
    
##ctrl##
void ChassisGimbalShooterManual::ctrlCPress()
    //锁定装甲板目标 switchArmorTargetType();
    
void ChassisGimbalShooterManual::ctrlVPress() 
    //锁定敌方颜色 switchEnemyColor();
    

void ChassisGimbalShooterManual::ctrlRPress()
    //非英雄机器人切换为小陀螺模式，功率限制：BRUST
    //英雄机器人成功调取目标服务后才转为小陀螺
    
void ChassisGimbalShooterManual::ctrlBPress()
    //暴露等级？
```





### 死亡事件

```c++
void ChassisGimbalShooterManual::robotDie() 
    //设置Mode
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
```







## chassis_gimbal_shooter_cover_manual

承接ChassisGimbalShooterManual，对原有的命令进行扩充

### 构造函数

```c++
ChassisGimbalShooterCoverManual::ChassisGimbalShooterCoverManual(ros::NodeHandle &nh) : ChassisGimbalShooterManual(nh) 
    //定义cover节点，获取参数cover_calibration
    //ctrl+z/q事件
    //检查ctrl事件 void ChassisGimbalShooterCoverManual::checkKeyboard()
```



### 启动，更新，发布，云台

```c++
void ChassisGimbalShooterCoverManual::run()
    //启动，调用update()

void ChassisGimbalShooterCoverManual::updatePc() 
    //更新PC数据，调用serRate()
    
void ChassisGimbalShooterCoverManual::sendCommand(const ros::Time &time)
    //发布，调用sendCommand()
 
void ChassisGimbalShooterCoverManual::gimbalOutputOn()
    //云台输出，调用reset()
```



### 远程遥控

```c++
void ChassisGimbalShooterCoverManual::remoteControlTurnOff() 
   
void ChassisGimbalShooterCoverManual::remoteControlTurnOn() 
```



### 绘制Ui

```C++
void ChassisGimbalShooterCoverManual::drawUi(const ros::Time &time)
    //flash_ui 更新cover
```



### 拨动开关

> //拨动开关判断：左右+位置+位置流状态
> rightSwitchDownRise                                                                 //到达最下面一格
> leftSwitchMidFall                                                                   //从中间一格离开

```c++
void ChassisGimbalShooterCoverManual::rightSwitchDownRise()
void ChassisGimbalShooterCoverManual::rightSwitchMidRise() 
void ChassisGimbalShooterCoverManual::rightSwitchUpRise()
    //仅右拨动开关下拨时，cover命令才会执行发布
```







### 摁键

```c++
void ChassisGimbalShooterCoverManual::ctrlZPress() 
    //设置目标点方位
    //云台模式：DIRECT 底盘模式：FOLLOW 功率限制：NORMAL
    //当cover成功获取状态数据时才会执行命令发布
    
void ChassisGimbalShooterCoverManual::ctrlQPress()
    //重置cover校准
```









