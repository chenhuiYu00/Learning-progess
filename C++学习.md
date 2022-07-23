# C++学习



## explicit 关键词

**explicit关键字的作用就是防止类构造函数的隐式自动转换.**

```c++
CxString string2 = 10;    // CxString是类名，定义类的时候先隐式CxString a(10),再string2=a
                          //这样是不行的, 因为explicit关键字取消了隐式转换  
```

 **explicit关键字只对有一个参数的类构造函数有效, 如果类构造函数参数大于或等于两个时, 是不会产生隐式转换的, 所以explicit关键字也就无效了.**

```c++
    explicit CxString(int age, int size) 
```

**也有一个例外, 就是当除了第一个参数以外的其他参数都有默认值的时候, explicit关键字依然有效, 此时, 当调用构造函数时只传入一个参数, 等效于只有一个参数的类构造函数**

```c++
    explicit CxString(int age, int size = 0)  
```





## std::map

> STL是标准C++系统的一组模板类，使用STL模板类最大的好处就是在各种C++编译器上都通用。
>
>     在STL模板类中，用于线性数据存储管理的类主要有vector, list, map 等等。



使用map对象首先要包括头文件,包含语句中必须加入如下包含声明

```c++
#include <map>
std:map<int, CString> enumMap;
```

注意，STL头文件没有扩展名.h

包括头文件后就可以定义和使用map对象了，map对象是模板类，需要关键字和存储对象两个模板参数，这样就定义了一个用int作为关键字检索CString条目的map对象，std表示命名空间，map对象在std名字空间中，为了方便，在这里我仍然使用了CString类，其实应该使用标准C++的std::string类，我们对模板类进行一下类型定义，这样用的方便，当然，不定义也可以，代码如下：

```c++
enumMap[1] = "One";
enumMap[2] = "Two";
//.....
enumMap[1] = "One Edit";
//或者insert方法
enumMap.insert(make_pair(1,"One"));
 
//返回map中目前存储条目的总数用size()方法：
int nSize = enumMap.size();
```





##  延时函数

> https://wenku.baidu.com/view/e7967cf787254b35eefdc8d376eeaeaad1f316ef.html

