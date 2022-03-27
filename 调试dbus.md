# 调试dbus

> dbus是遥控器，与接收机连接后进而连接串口（NUC）通信，配置中见到的乱码是dbus发送给接收机的，接收机进而发送给串口



### 修改串口的物理映射地址

一个会映射成usbdbus ，另一个映射为usbrefore（裁判系统也通过串口读取）。需要对dbus进行连接，若dbus连接口为ttyusb0，命令口的也要修改为ttyusb0

首先连接到wifi

**代码串：**

``` bash
ssh dynamicx@192.168.1.196                        //连接到hero机器人,密码dynamicx
												//139 全向轮  112 哨兵
```

``` bash
sudo minicom -s                                   //打开配置工具
```

接着选择serial port set up（第三项），回车，按a键，命令口输入ttyUSB1，exit

完成，接着观察是否有接受乱码输出。





