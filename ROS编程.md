# ROS编程

> 记录需要的命令



## rosparam 与命名空间

### rosraram

> param就是个中央注册表，可以进行参数的增删改查．可以在代码中和在ros launch中都可以设置这个参数，参数的名字要是一样的．并且ros launch中的参数会覆盖代码中的参数．



#### roscpp中的使用

roscpp提供了两套，一套是放在ros::param namespace下，另一套是在ros::NodeHandle下，这两套API的操作基本一样。
推荐使用ros::param的方式，因为ros::param给人感觉，增删改善参数是静态的，实例无关的，这更像**参数本质**

>arg传参,实际就在argv后面
>
>```c++
> request.a = argv[1];
> request.b = argv[2];
>```
>
>**roslaunch beginner_tutorials launch_file.launch**
>
>arg的属性是default，所以我们可以在指令中修改参数的值(**注意这正是arg传参的优势所在，可以不修改launch文件的情况下修改参数值，通过指令形式**)，执行命令：
>
>**roslaunch beginner_tutorials launch_file.launch a:=1 b:=2**



#### ros::param方式


    #include<ros/ros.h>
     
    int main(int argc, char **argv)
    {
        ros::init(argc, argv, "param_demo");
        ros::NodeHandle nh;
        int parameter1, parameter2, parameter3, parameter4, parameter5;
        
        // get param
        bool ifget1 = ros::param::get("param1", parameter1);
    
        // set param
        parameter4 = 4;
        ros::param::set("param4", parameter4);
    
        // check param
        bool ifparam6 = ros::param::has("param6");
        if(ifparam6) { 
            ROS_INFO("Param6 exists"); 
        }
        else { 
            ROS_INFO("Param6 doesn't exist"); 
        }     
    
        // delete param
        bool ifdeleted6 = ros::param::del("param6");
        if(ifdeleted6) { 
            ROS_INFO("Param6 deleted"); 
        } 
        else { 
            ROS_INFO("Param6 not deleted"); 
        }
        
        // get all param names.     New in ROS indigo
        std::vector<std::string> keys;
        ros::param::search(keys);
        
        return 0;
    }

#### ros::NodeHandle方式

    #include<ros/ros.h>
     
    int main(int argc, char **argv)
    {
        ros::init(argc, argv, "param_demo");
        ros::NodeHandle nh;
        int parameter1, parameter2, parameter3, parameter4, parameter5;
        
        // get param 1
        bool ifget2 = nh.getParam("param2",parameter2);
        
        // get param 2.  33333 is default value. When no param3, parameter3=33333.
        nh.param("param3", parameter3, 33333);     
    
        // set param
        parameter5 = 5;
        nh.setParam("param5",parameter5);   
    
        // check param
        bool ifparam5 = nh.hasParam("param5");
        if(ifparam5) {
            ROS_INFO("Param5 exists");
        }
        else {
            ROS_INFO("Param5 doesn't exist");
        }     
    
        // delete param
        bool ifdeleted5 = nh.deleteParam("param5");
        if(ifdeleted5) {
            ROS_INFO("Param5 deleted");
        }
        else {
            ROS_INFO("Param5 not deleted");
        }
        
        // get all param names.     New in ROS indigo
        std::vector<std::string> keys;
        nh.getParamNames(keys);
        
        return 0;
    }



### 命名空间

> 有时会看到 ros::NodeHandle n; 和 ros::NodeHandle nh("~"); 两种用法。

ros::NodeHandle n;是全局命名空间，加入的参数为默认命名空间
ros::NodeHandle nh("~");是局部命名空间，读取param时默认进入节点名的命名空间读取



**全局(global)名称**

举例：`/gloabl/name`

首字符是`/`的名称是全局名称。可以在全局范围内直接访问

**相对(relative)名称**

举例： `relative/name`

相对名称是ros提供默认命名空间。不需要开头的左斜杠。如例子中，如果我们设置默认命名空间为relative（ros::NodeHandle nh("relative")），那么在程序中只需要写name即可。如果其他程序想要访问的话，使用/relative/name全局名称来搜索。



#### 示范用例

[原网址](https://www.icode9.com/content-4-822689.html)

用例1：

```cpp
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<learn_topic::person>("person_topic", 1);
```

输出topic名称：`/person_topic`
 解释：没有设置默认命名空间，故全局的解析就是其本身。

用例2：

```cpp
    ros::NodeHandle nh("namespace");
    ros::Publisher pub = nh.advertise<learn_topic::person>("person_topic", 1);
```

输出topic名称：`/namespace/topic_person`
 解释：设置了默认空间，全局解析为：默认空间 + 自定义topic名称

用例3：

```cpp
    ros::NodeHandle nh("namespace");
    ros::Publisher pub = nh.advertise<learn_topic::person>("/person_topic", 1);
```

输出topic名称：`/person_topic`
 解释：设置了默认空间，但是topic首字符为`/`，说明topic本身就是全局名称，默认空间无效

用例4：

```cpp
    ros::init(argc, argv, "cpp_talker");    //节点名cpp_talker
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<learn_topic::person>("person_topic", 1);
```

输出topic名称：`/cpp_talker/person_topic`
 解释：私有名称不使用当前默认命名空间，而是用节点的全局名称作为命名空间，节点的全局名称为`/cpp_talker`







#### 在 launch 文件中载入参数

> 使用`<param>`标签在node标签外设置全局参数
>
> 使用`<param>`标签在node标签里设置局部参数

如下一个lauch文件，有两个serial参数，一个全局的，一个局部

    <launch>
        <!-- global serial -->
        <param name="serial" value="5" />
        <node name="name_demo" pkg="name_demo" type="name_demo" output="screen">
            <!-- local serial -->
            <param name="serial" value="10" />
        </node>
    </launch>

使用两种命名空间句柄获取全局和局部参数的用法如下：

    #include <ros/ros.h>
     
    int main(int argc, char* argv[])
    {
        int serial_number = -1;
        ros::init(argc, argv, "name_demo");
     
        ros::NodeHandle nh_global;
        ros::NodeHandle nh_local("~");
     
        // global namespace
        nh_global.getParam("serial", serial_number);                // get global serial    
        nh_global.getParam("name_demo/serial", serial_number);      // get local serial. add "name_demo/"
    
        // local namespace
        nh_local.getParam("serial", serial_number);                 // get local serial
        nh_local.getParam("/serial", serial_number);                // get global serial. add "/"
     
        ros::spin();
        return 0;
    }



代码修改参数，必须重新编译。
launch文件可以方便的修改参数。

    <launch>
        <!--param-->
        <param name="param1" value="1" />
        <param name="param2" value="2" />
     
        <!--rosparam-->
        <rosparam>   
            param3: 3
            param4: 4
            param5: 5
        </rosparam>
        
        <node pkg="param_demo" type="param_demo" name="param_demo" output="screen" />
    </launch>




### yaml文件

>  当在复杂系统中，每次启动需要很多的参数配置，故使用yaml文件

```c
<rosparam file="***.yaml" command="load" ns="XXX">
    //参数：file:要读取的文件  command：命令  ns：命名空间，可去
```

其中需要指定三个参数，第一是yaml文件位置，第二是使用load犯法，第三可以指定命名空间。

这种方法与在终端使用：`rosparam load ***.yaml`效果相同。



#### 参数维护

> 通过编写程序，维护node中的参数

**设置参数**

```cpp
ros::NodeHandle nh;
nh.setParam("/global_param", 5);
nh.setParam("relative_param", "my_string");
nh.setParam("bool_param", false);
```

**读取参数**

```cpp
ros::NodeHandle nh;
std::string global_name, relative_name, default_param;
// 使用nh句柄来获取参数
if (nh.getParam("/global_name", global_name))
{
  ...
}

// 如果没有读取到，就按照default_value来设置参数
// 所以要注意后两个参数的数据格式保存一致
nh.param<std::string>("default_param", default_param, default_value);
```

**查询参数是否存在**

```cpp
ros::NodeHandle nh;
if (nh.hasParam("my_param"))
{
  ...
}
```

**删除参数**

```cpp
ros::NodeHandle nh;
nh.deleteParam("my_param");
```