# class RealtimeBuffer

``` c++
#ifndef REALTIME_TOOLS__REALTIME_BUFFER_H_
#define REALTIME_TOOLS__REALTIME_BUFFER_H_

#include <chrono>
#include <mutex>
#include <thread>

namespace realtime_tools
{
  //承接realtime_tools::RealtimeBuffer<Commands> command_;接受特定类
template <class T>
class RealtimeBuffer
{
 public:
  RealtimeBuffer()                                                                 //构造函数
    : new_data_available_(false)
  {
    
    non_realtime_data_ = new T();
    realtime_data_ = new T();
  }

  /**
   * @brief Constructor for objects that don't have
   * a default constructor
   * @param data The object to use as default value
   */
  RealtimeBuffer(const T& data)
  {
    // 分配内存
    non_realtime_data_ = new T(data);
    realtime_data_ = new T(data);
  }
  //析构函数：有数据则释放动态内存
  ~RealtimeBuffer()
  {
    if (non_realtime_data_)
      delete non_realtime_data_;
    if (realtime_data_)
      delete realtime_data_;
  }

  RealtimeBuffer(const RealtimeBuffer &source)                                    //复制构造函数
  {
    // 分配内存
    non_realtime_data_ = new T();
    realtime_data_ = new T();

    // Copy the data from old RTB to new RTB
    writeFromNonRT(*source.readFromNonRT());
  }

  /*!
   * 自定义赋值运算符，使得类之间可以赋值
   */
  RealtimeBuffer &operator =(const RealtimeBuffer& source)
  {
    if (this == &source)
      return *this;

    // Copy the data from old RTB to new RTB
    writeFromNonRT(*source.readFromNonRT());

    return *this;
  }

  T* readFromRT()
  {
    // 检查数据当前是否正在写入（已锁定）
    std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
    if (guard.owns_lock())
    {
      // swap pointers
      if (new_data_available_)
      {
        T* tmp = realtime_data_;
	realtime_data_ = non_realtime_data_;
        non_realtime_data_ = tmp;
	new_data_available_ = false;
      }
    }
    return realtime_data_;
  }

  T* readFromNonRT() const
  {
    std::lock_guard<std::mutex> guard(mutex_);

    if (new_data_available_)
      return non_realtime_data_;
    else
      return realtime_data_;
  }

  void writeFromNonRT(const T& data)
  {
#ifdef NON_POLLING
    std::lock_guard<std::mutex> guard(mutex_);
#else
    std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
    while (!guard.owns_lock()) {
      std::this_thread::sleep_for(std::chrono::microseconds(500));
      guard.try_lock();
    }
#endif

    // copy data into non-realtime buffer
    *non_realtime_data_ = data;
    new_data_available_ = true;
  }

  void initRT(const T& data)
  {
    *non_realtime_data_ = data;    
    *realtime_data_ = data;    
  }

 private:

  T* realtime_data_;
  T* non_realtime_data_;
  bool new_data_available_;

  // 设置为可变，以便可以在常量缓冲区上执行readFromNonRT（）
  mutable std::mutex mutex_;

}; // class
}// namespace

#endif
```

