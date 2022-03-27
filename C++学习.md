# C++学习



### explicit 关键词

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

