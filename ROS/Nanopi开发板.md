# Nanopi开发板(M1)

> https://wiki.friendlyelec.com/wiki/index.php/NanoPi_M1/zh

![File:NanoPi M1-2.jpg](images/Nanopi开发板/NanoPi_M1-2.jpg)

**管脚定义**

以上图为参考管脚分布：

PIN：2 4 6 ... 36 38 40

PIN：1 3 5 ... 35 37 39

UART0：1 2 3 4

![2023-07-16 00-41-51 的屏幕截图](images/Nanopi开发板/2023-07-16 00-41-51 的屏幕截图.png)

![2023-07-16 00-42-27 的屏幕截图](images/Nanopi开发板/2023-07-16 00-42-27 的屏幕截图.png)



## 配置

- 安装[调用库](https://wiki.friendlyelec.com/wiki/index.php/WiringNP:_NanoPi_NEO/NEO2/Air_GPIO_Programming_with_C/zh)

```bash
git clone https://github.com/friendlyarm/WiringNP
cd WiringNP/
git checkout nanopi-m1
chmod 755 build
./build
```

```bash
gpio readall  #读取已有的硬件资源
```

如果想修改复用的接口，以PWM复用串口为例子，执行 `sudo npi-config `命令，进入Advanced Options菜单，执行其中的Enable/Disable PWM操作。

- cmakelist里添加库依赖

```cmake
#这个命令会查找WiringNP库，并把它的路径存储在变量WIRINGNP中。
find_library(WIRINGNP wiringPi)

## Declare cpp executables
FILE(GLOB ALL_SOURCES "src/*.cpp")
add_executable(${PROJECT_NAME} ${ALL_SOURCES})

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${WIRINGNP}    #WIRINGNP
        -pthread       #这也是个依赖
        )
```

- 在root的bashrc中写入`source 工作空间`

```bash
source '/home/pi/nano_ws/devel/setup.bash'
```

- 用普通用户编译，执行时进入root用户

```bash
cd ~/nano_ws
su
rosrun xxx xxx
```





编译运行





# 功能

## 

## 将GPIO引出为USB

[可以参考以下步骤来将GPIO2引出为USB A口](https://blog.csdn.net/qlexcel/article/details/120860456)[1](https://blog.csdn.net/qlexcel/article/details/120860456)：

1. [在GPIO2上焊接一个USB转接头，连接USB-DP1、USB-DM1、VDD_5V和GND四个管脚](http://wiki.friendlyelec.com/wiki/index.php/NanoPi_NEO2/zh)[2](http://wiki.friendlyelec.com/wiki/index.php/NanoPi_NEO2/zh)。
2. 在系统中加载dwc2驱动模块，使能USB OTG功能。您可以在/boot/boot.cmd中添加如下命令：

```
setenv bootargs "${bootargs} modules-load=dwc2,g_ether"
```

[然后运行`mkimage -C none -A arm -T script -d /boot/boot.cmd /boot/boot.scr`更新启动脚本](https://www.cnblogs.com/netube/p/10159729.html)[3](https://www.cnblogs.com/netube/p/10159729.html)。

1. 重启系统后，您应该可以在/dev目录下看到sda或者sdb等设备文件，表示USB A口已经被识别。