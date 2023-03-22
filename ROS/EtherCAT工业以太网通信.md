# EtherCAT工业以太网通信

EtherCAT是一种实时以太网协议，用于高性能运动控制和工业自动化应用。它通过在实时以太网帧中传输数据来实现实时性，从而使得EtherCAT网络可以支持复杂的、高性能的控制任务。EtherCAT还具有灵活性和可扩展性，可以支持多种不同类型的设备和拓扑结构。相对于传统的工业控制总线，EtherCAT的性能更加优越，可以提供更高的带宽和更低的延迟。

> [EtherCAT卓越的性能](http://www.chinaaet.com/article/3000014479)



- 和CAN通信的区别

二者都是工业级通讯，不过仍然有些区别

1. 数据传输方式：CAN是一种点对点的串行总线，而EtherCAT则是一种以太网网络，通过广播机制实现数据的传输。
2. 带宽：CAN总线的数据传输速率通常为1 Mbps，而EtherCAT网络的数据传输速率通常为100 Mbps或更高。
3. 网络拓扑：CAN总线通常是线性总线，节点之间通过电缆相连。而EtherCAT网络则是一种分布式网络，各个节点通过交换机相连。
4. 实时性能：EtherCAT网络通过在数据包中嵌入控制命令实现实时性能，具有较高的实时性能。而CAN总线的实时性能则不如EtherCAT。

总体来说，EtherCAT网络的数据传输速度更快，实时性能更好，但它需要更高的网络带宽和复杂的网络拓扑结构。而CAN总线则更加简单和稳定，适合于中小规模的工业自动化应用。





## **EtherCAT原理**

> EtherCAT采用主从式通信结构。主站向各个从站发送以太网数据帧，数据帧经过各个从站时经从站分析其中的指令进行数据的提取或插入从站自身的数据，并将**工作计数器**（Working  Counter，WKC）更新。最终与主站事先设置的预期WKC值作比较，以判断是否经过了所有的从站并被正确处理。
>
> ==由于从站对数据帧的接收与编码及转发、数据的提取与插入都是通过硬件完成，所以数据帧在每个从站停留的时间极小，保证了EtherCAT的实时性==。经过最后一个从站处理后的数据帧将返回主站，主站接收并处理后，一次通信完成。这种通信方式不会引起通信通路的堵塞，可保证网络的实时性和正确性



### 强大的实时性

> 依靠分布式通信和WKC等方式，EtherCAT有很强的实时性

```c
1.工作计数器
    在EtherCAT网络中，每个从站设备都有一个工作计数器（Working Counter），用于记录接收到的数据帧的数量。主站会定期向从站发送同步数据帧（Sync Frame），从站在接收到Sync Frame时会将自己的工作计数器加一，并回送给主站。主站可以通过比较接收到的从站回送的工作计数器值和之前保存的值来检查网络中的数据是否正常传输，确保EtherCAT网络的可靠性和稳定性。
2.分布式时钟同步技术
    EtherCAT使用 分布式时钟同步协议(Distributed Clocks Protocol, DC)。该协议通过在EtherCAT网络中广播同步消息来进行时钟同步。
    EtherCAT主节点广播同步消息，从节点接收该消息并进行本地时间戳的更新。这样，即使各个节点之间的传输延迟不同，各个节点的时间戳也可以在全局时间轴上同步。此外，EtherCAT还提供了一个分布式时钟对象，用于在节点之间同步基于时间的应用程序。这些对象可以提供与协调世界时UTC或本地时间相同的时间精度。
	通过分布式时钟同步协议，EtherCAT网络可以实现高精度的时钟同步，从而使得各个节点之间可以按照同步的时间来进行通信和控制。
```



### 原理图

```lua
CPU: 控制器上的中央处理器，负责运行控制器的应用程序。
Master: 控制器中的EtherCAT主站，向所有EtherCAT从站发送EtherCAT帧并处理其响应。
FMM (FPGA Mezzanine Module): 一种FPGA模块，用于处理数字输入和输出信号。
Switch: 交换机，用于连接多个EtherCAT从站。
Slave: 连接到EtherCAT网络的设备。
M (Memory): 存储主站发送的数据和从站响应的数据。
Clock: 用于同步整个EtherCAT网络中所有设备的系统时钟。

						  +-----+      +--------+
                          |     |      |        |
                          | CPU |<---->| Master |
                          |     |      |        |
                          +-----+      +--------+
                             |              |
               +-------------+--------------+-------------+
               |                                          |
            +-----+                                    +-----+
            |     |                                    |     |
            | FMM |                                    | FMM |
            |     |                                    |     |
            +-----+                                    +-----+
               |                                          |
               |                                          |
+---------+    |    +---------+          +----------+     |    +---------+
|         |<---+--->|         |          |          |<----+--->|         |
|  Slave 1|         |  Switch |<-------->|   Slave 2|          |  Slave 3|
|         |<---+--->|         |          |          |<----+--->|         |
+---------+    |    +---------+          +----------+     |    +---------+
               |                                          |
               +--------------------------+---------------+
                                          |
                                       +-----+
                                       |     |
                                       |  M  |
                                       |     |
                                       +-----+
                                          |
                                          V
                                      +-------+
                                      | Clock |
                                      +-------+

```

EtherCAT网络的通信过程如下：

- 主站将数据包发送到EtherCAT网络。
- 数据包在网络中传递，每个从站读取其应答的数据并向网络中写入其数据。
- 主站读取响应并执行必要的计算。
- 主站向网络发送下一个数据包。

通过这种方式，主站可以实现与多个从站之间的高效通信和实时控制，而分布式时钟同步确保整个网络中所有设备的时钟保持同步。





# 配置环境

> clear_driver依赖以下三个包

安装[soem_interface](https://github.com/leggedrobotics/soem_interface) //实现EtherCAT底层功能

> 该包提供了以下功能：
>
> 1. 连接到EtherCAT总线和设备。
> 2. 实现在EtherCAT设备之间读写数据的功能。
> 3. 提供了用于配置EtherCAT从站的接口。
> 4. 提供了一组服务和话题，可以在ROS系统中访问EtherCAT设备。
>
> `soem_interface`包中提供了以下几个主要的类：
>
> 1. `soem_master`: 用于初始化和管理SOEM库的EtherCAT主站对象。
> 2. `soem_slave`: 用于表示EtherCAT从站的类。
> 3. `soem_device`: 用于表示一个具体的EtherCAT设备，并提供了读写设备数据的接口。
> 4. `soem_cat`: 用于表示整个EtherCAT网络，包含了一组从站和设备对象。

```c
git clone git@github.com:leggedrobotics/soem_interface.git
```

安装 [ethercat_sdk_master ](https://github.com/leggedrobotics/ethercat_sdk_master)//实现EtherCAT高层功能

> 1. 实现了EtherCAT Master节点的功能，可以管理和控制所有的EtherCAT从节点。
> 2. 提供了一套API，可以用于读写从节点的对象字典（Object Dictionary），并控制从节点的状态。
> 3. 提供了一套EtherCAT配置文件编辑工具，可以方便地配置从节点的参数。
> 4. 支持多种EtherCAT从节点，例如数字量输入输出模块、模拟量输入输出模块、电机驱动器、IO-Link设备等。
> 5. 支持EtherCAT从节点的自动发现和配置。

```c
git clone git@github.com:leggedrobotics/ethercat_sdk_master.git
```

安装[message_logger ](https://github.com/ANYbotics/message_logger) //实现日志流

> 主要用途是方便调试：
>
> 1. 支持在终端、文件、ROS消息和ROS节点中输出消息。
> 2. 支持消息格式化和颜色编码，使输出消息更易于阅读。
> 3. 支持消息级别，包括DEBUG、INFO、WARN和ERROR，可以根据需要过滤输出。
> 4. 支持ROS参数设置，包括消息级别和输出位置。
> 5. 可以集成到ROS节点中，只需添加一个简单的Logger对象即可。

```c
git clone git@github.com:ANYbotics/message_logger.git
```



之后需要自己实现以下功能：

1. 解析和打包EtherCAT数据帧
2. 实现基于PDO和SDO的数据交换

```c
PDO（Process Data Object）和SDO（Service Data Object）是EtherCAT通信中用于数据交换的两种不同类型的数据对象。
PDO是一种实时数据通信方式，主要用于在EtherCAT网络中进行实时数据交换。PDO直接读取和写入从站的I/O数据，支持高速实时通信，但是数据格式和长度是固定的，不太适合于复杂的数据交换。
SDO是一种用于配置和参数传输的通信方式，支持灵活的数据格式和长度，适用于对从站进行配置和参数传输等操作。SDO通过从站对象字典（OD）来进行配置，能够实现动态修改从站参数，提高了系统的灵活性。】
综合来说，PDO适用于实时数据传输，SDO适用于配置和参数传输。在EtherCAT通信中，一般都会同时使用这两种数据对象来完成数据交换。
```

3. 实现EtherCAT从站的配置和管理

4. 实现错误处理和故障诊断

5. 实现周期性任务的调度和执行

6. 与其他ROS功能包进行集成，如控制器和传感器驱动程序、导航和运动控制等





# 开源的EtherCAT

> https://www.shuzhiduo.com/A/kPzOR9Z7dx/

EtherCAT的主站开发是基于EtherCAT机器人控制系统的开发中非常重要的环节。目前常见开源的主站代码为的[RT-LAB](http://www.rt-labs.com/)开发的[SOEM](http://ethercat.rt-labs.com/ethercat) (Simple OpenSource EtherCAT Master)和[EtherLab](http://www.etherlab.org/)的[the IgH EtherCAT® Master](http://www.etherlab.org/)。使用起来SOEM的简单一些，而the IgH EtherCAT® Master更复杂一些，但对EtherCAT的实现更为完整。

具体比较如下表：

| 功能                            | SOME(Simple OpenSource EtherCAT Master)          | IgH EtherCAT Master                                          |
| ------------------------------- | ------------------------------------------------ | ------------------------------------------------------------ |
| 版本                            | 1.3.0                                            | 1.5.2                                                        |
| 更新日期                        | 2013-02-26                                       | 2013-02-12                                                   |
| 发布公司                        | RT-LAB                                           | EtherLab                                                     |
| 官方网站                        | ethercat.rt-labs.com                             | [www.etherlab.org](http://www.etherlab.org/)                 |
| 支持的操作系统                  | Linux,Windows                                    | Linux                                                        |
| 支持RT内核                      | RTAI, Xenomai                                    | RTAI, Xenomai, RT-Preempt                                    |
| 支持的CPU                       | Freescale i.MX53 Blackfin 5xx Blackfin 6xx Intel | 支持Linux内核的所有CPU                                       |
| 支持的网卡                      | -                                                | 8139too - RealTek 8139C (or compatible) Fast-Ethernet chipsets. •e1000 - Intel PRO/1000 Gigabit-Ethernet chipsets (PCI). •e100 - Intel PRO/100 Fast-Ethernet chipsets. •r8169 - RealTek 8169/8168/8101 Gigabit-Ethernet chipsets. •e1000e - Intel PRO/1000 Gigabit-Ethernet chipsets (PCI Express). |
| CANOpen over EtherCAT (CoE)     | √                                                | √                                                            |
| Vendor over EtherCAT (VoE)      | √                                                | √                                                            |
| Distributed clocks              | √                                                | -                                                            |
| SERCOS over EtherCAT (SoE)      | √                                                | √                                                            |
| Ethernet over EtherCAT (EoE)    | ×                                                | √                                                            |
| File Access over EtherCAT (FoE) | √                                                | √                                                            |
| Safety over EtherCAT (FSoE)     | ×                                                | ×                                                            |







# 代码分析

> 关于clear_drive中的包的代码分析



## clear_foc

> clear_foc是一个基础的etherCAT主机操作类，它派生于官方类ecat_master::EthercatDevice，定义了信息传输的状态量和信息收发函数及对象。
>
> clear_foc包中，最高层的是clearFoc类，它有状态储存容器`DriveState prevDriveState_`，状态期望容器`DriveState targetDriveState_`，状态转换量容器`ModeOfOperation modeOfOperation_`，命令操作符号`Controlword`。
>
> 从机pdo信息：`PdoInfo pdoInfo_；`
>
> 信息读取对象：`Reading reading_;`
>
> 信息发布对象：`Command stagedCommand_;`

函数`getCurrentPdoInfo()` `getReading()` 获取了从机pdo信息和从机电机的数据及状态



### 重载函数

> 官方类ecat_master::EthercatDevice预留的接口，通过重载这些函数来配置基本etherCAT通信

```c++
// pure virtual overwrites
bool startup() override;
void shutdown() override;
void updateWrite() override;
void updateRead() override;
PdoInfo getCurrentPdoInfo() const override { return pdoInfo_; }
```



#### 初始化etherCAT

进入startup()函数后，会等待从机响应，bus_->waitForState()函数会等待特定从站（slave）==进入指定状态==（state），这个过程有最大重试次数和重试时间间隔。

```c++
EthercatBusBase类的一个成员函数，用于等待特定从站（slave）进入指定状态（state）。函数参数的含义如下：
    state：uint16_t类型，表示期望从站进入的状态。
    slave：uint16_t类型，表示从站的地址。
    maxRetries：unsigned int类型，表示最大重试次数。
    retrySleep：double类型，表示每次重试之间的睡眠时间（单位为秒）。
该函数将等待从站进入指定状态，如果从站没有在指定重试次数内进入该状态，则函数会抛出一个异常。函数中会通过发送EtherCAT报文来查询从站的状态，并在需要的情况下进行重试，每次重试后会睡眠retrySleep秒。
```

接着通过bus_->syncDistributedClock0()来==同步从机的分布式时钟==：

```c
函数EthercatBusBase::syncDistributedClock0()是用于同步EtherCAT总线上从站设备的分布式时钟的方法。
它的参数如下：
    slave: 指定要同步的从站设备的ID。
    activate: 指定是否激活同步分布式时钟功能。
    cycleTime: 指定从站设备的同步周期，单位为毫秒(ms)。
    cycleShift: 指定从站设备的时钟相对于总线时钟的偏移量，单位为纳秒(ns)
```

setDriveStateViaSdo(DriveState::Standby)语句==获取从机状态信息==并判断读取是否成功，之后再用sdoVerifyWrite()尝试==向从机发送数据==并检验是否成功

```c++
sendSdoRead是EtherCAT总线上读取对象字典（Object Dictionary）数据的函数。对象字典是EtherCAT设备中用于存储设备相关参数的数据结构，可以包含诸如设备ID、生产厂商信息、设备状态等信息。每个对象字典都可以由其索引和子索引来唯一标识。在sendSdoRead函数中，index和subindex指定要读取的对象字典的索引和子索引。如果completeAccess参数设置为true，则表示读取整个对象字典，否则只读取指定子索引的数据。读取的结果保存在value变量中。

此函数的主要作用是从EtherCAT设备中读取需要的数据，并且更新该设备的状态信息
```

```c
sdoVerifyWrite 函数用于向 EtherCAT 从站写入 SDO 数据并进行确认。它的输入参数为：
    index：SDO 对象的索引。
    subindex：SDO 对象的子索引。
    completeAccess：是否启用完整访问模式（默认为 false）。
    value：要写入的 SDO 数据。
    delay：等待从站确认的时间（单位为秒），默认为 0。
该函数的作用是向 EtherCAT 从站写入 SDO 数据并等待从站确认。它首先使用 sendSdoWrite 函数将数据写入从站的 SDO 对象中，然后等待从站确认。如果从站在 delay 时间内未发送确认消息，则函数返回 false。如果在 delay 时间内收到了从站的确认消息，则返回 true。
    
这个函数可以用于写入从站的配置参数，并确认从站是否正确接收和处理了这些参数。
```

之后调用getHardwarePdoSizes()自动==获取PDO==大小

```c
EthercatBusBase::getHardwarePdoSizes(const uint16_t slave) 函数用于获取特定从站（slave）的硬件PDO（Process Data Object）大小。
在EtherCAT网络中，从站包含一个或多个PDO，PDO是用于实际数据交换的最小单元。硬件PDO是从站的硬件实现，而软件PDO是使用EtherCAT主站配置的。函数getHardwarePdoSizes返回两个值，即输入PDO的大小和输出PDO的大小，以字节为单位。这对于配置EtherCAT从站的软件PDO很有用，因为它需要与硬件PDO匹配。
```

这样etherCAT的初始化完成了



#### 中断etherCAT

shutdown()函数通过向从站发送状态命令，使其进入初始化状态

```c
bus_->setState(EC_STATE_INIT, address_);
```

状态枚举：

```
typedef enum
{
   /** Init state*/
   EC_STATE_INIT           = 0x01,
   /** Pre-operational. */
   EC_STATE_PRE_OP         = 0x02,
   /** Boot state*/
   EC_STATE_BOOT            = 0x03,
   /** Safe-operational. */
   EC_STATE_SAFE_OP        = 0x04,
   /** Operational */
   EC_STATE_OPERATIONAL    = 0x08,
   /** Error or ACK error */
   EC_STATE_ACK            = 0x10,
   EC_STATE_ERROR          = 0x10
} ec_state;
```

> - EC_STATE_INIT：EtherCAT总线初始化，所有从站都处于初始化状态。
> - EC_STATE_PRE_OP：EtherCAT总线处于预运行状态，所有从站已经完成配置但还未进入运行状态。
> - EC_STATE_BOOT：EtherCAT从站已经完成初始化，正在加载固件。
> - EC_STATE_SAFE_OP：EtherCAT从站已经进入安全模式，它不能进行控制但可以执行一些安全相关的操作，例如发送警告信号、停止等。
> - EC_STATE_OPERATIONAL：EtherCAT从站处于运行状态，可以接收和执行主站发来的控制指令。
> - EC_STATE_ACK：当EtherCAT从站收到主站的控制指令并执行成功时，会进入ACK状态。
> - EC_STATE_ERROR：当EtherCAT从站出现错误时，会进入错误状态。