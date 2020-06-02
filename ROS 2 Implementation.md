###### tags: `Robotics`

# ROS2 DDS 整體架構

ROS2 架構回顧[^1]

[^1]: 講解 ROS2 整體架構的簡報 https://static1.squarespace.com/static/51df34b1e4b08840dcfd2841/t/5ce6c85ca4222fe0ccbd5309/1558628472094/2019-05-07_Current_Status_of_ROS_2.pdf

![](https://i.imgur.com/f6c34oj.png)

specific rmw_adapter: 實作 rmw 的界面，目前主要有 `rmw_fastrtps_cpp` `rmw_connext_cpp` `rmw_opensplice_cpp` 三種，2018 年因為新的 DDS 實作 cyclonedds 出現而新增了 [`rmw_cyclonedds`](https://github.com/ros2/rmw_cyclonedds)

- Executor
- wait()
- rosidl typesupport

## Executor

![](https://i.imgur.com/KRpqMLD.png)

rclcpp, rclpy 為 ROS2 提供給特定語言的函式庫，是以非同步的執行模型接收資料 (subscription)，傳送資料也可以使用非同步的方式，如使用 timer 並註冊對應的 callback，為了實作非同步模型 ROS2 在 rclcpp, rclpy 中都定義了 **Executor** 的類別，而繼承 Executor 的類別更可以提供多執行緒處理 callback 的能力，或是 Mutual Exculsive，限制在多執行緒下單一時間只處理一個 callback。

![ROS2 Executor 繼承圖](http://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1executor_1_1Executor__inherit__graph.png)

ROS2 最常用的 `spin()` 函式則是使用單執行緒的 Executor，使用 `add_node` 將單一個 node 加入 Executor 中。

```c
void
rclcpp::spin(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_ptr);
  exec.spin();
  exec.remove_node(node_ptr);
}
```

```cpp=27 rclcpp/executors/single_threaded_executor.cpp
void
SingleThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::executor::AnyExecutable any_executable;
    if (get_next_executable(any_executable)) {
      execute_any_executable(any_executable);
    }
  }
}
```

其中 `get_next_executable()` 會呼叫 `get_next_ready_executable()` 檢查目前有哪些 callback 可以執行，如果沒有則最後會呼叫 `rcl_wait()` 等待新的 callback 進入。

Response-Time Analysis of ROS 2 Processing Chains under Reservation-Based Scheduling[^2] 這篇論文描述了 Executor 檢查 callback 的行為，以下為示意圖並附上相關程式碼：

[^2]: 探討 ROS2 callback scheduling 的行為 https://t-blass.de/papers/response-time-analysis-of-ros2.pdf

![](https://i.imgur.com/5X6R1pj.png)

```cpp rclcpp/executor.cpp
bool
Executor::get_next_ready_executable(AnyExecutable & any_executable)
{
  // Check the timers to see if there are any that are ready, if so return
  get_next_timer(any_executable);
  if (any_executable.timer) {
    return true;
  }
  // Check the subscriptions to see if there are any that are ready
  memory_strategy_->get_next_subscription(any_executable, weak_nodes_);
  if (any_executable.subscription || any_executable.subscription_intra_process) {
    return true;
  }
  // Check the services to see if there are any that are ready
  memory_strategy_->get_next_service(any_executable, weak_nodes_);
  if (any_executable.service) {
    return true;
  }
  // Check the clients to see if there are any that are ready
  memory_strategy_->get_next_client(any_executable, weak_nodes_);
  if (any_executable.client) {
    return true;
  }
  // Check the waitables to see if there are any that are ready
  memory_strategy_->get_next_waitable(any_executable, weak_nodes_);
  if (any_executable.waitable) {
    return true;
  }
  // If there is no ready executable, return a null ptr
  return false;
}
```

![ROS2 Executor 類別成員圖](http://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1executor_1_1Executor__coll__graph.png)

### Waitset, GuardCondition

從 rcl 層到 rmw_adapter 實作同步的設計模型，亦即等待訊號通知再去執行特定工作，這一類的操作必須使用 Waitset 和 GuardConition。

這兩個物件在 DDS 標準都有定義，多個 GuardCondition 可以連接 (attach) 到一個 Waitset 上，並等待任何一個 GuardCondition 通知，等待 Waitset 的程式就能被喚醒。每一個 DDS 實作可能會自己定義這兩個物件 (如 RTI Connext)，因此 rmw 層提供一系列的物件和函式來包裝，讓 rmw 層以上獨立於任何一個 DDS 實作 (DDS agonstic)。

以下為 rmw 定義的 Waitset, GuardCondition:
```cpp rmw/types.h
typedef struct RMW_PUBLIC_TYPE rmw_guard_condition_t
{
  const char * implementation_identifier;
  void * data;
  rmw_context_t * context;
} rmw_guard_condition_t;

...

typedef struct RMW_PUBLIC_TYPE rmw_guard_conditions_t
{
  /// The number of guard conditions represented by the array.
  size_t guard_condition_count;
  /// Pointer to an array of void * pointers of guard conditions.
  void ** guard_conditions;
} rmw_guard_conditions_t;

typedef struct RMW_PUBLIC_TYPE rmw_wait_set_t
{
  const char * implementation_identifier;
  rmw_guard_conditions_t * guard_conditions;
  void * data;
} rmw_wait_set_t;
```

成員 `data` 指向 DDS 實作自己定義的物件，要使用時再自行轉型 (只有在 rmw_adapter 中才會使用到 `data`，rcl 不會操作這個資料)。

以下參考 `rmw_fastrtps_cpp` 的程式碼，fastrtps 的 DDS 實作中並沒有獨立定義這兩個物件，因此他直接使用 C++ 標準的 mutex 和 condition variable。

```cpp
// rmw_fast_rtps/rmw_fast_rtps_shared_cpp/src/types/custom_wait_set.info.hpp
typedef struct CustomWaitsetInfo
{
  std::condition_variable condition;
  std::mutex condition_mutex;
} CustomWaitsetInfo;



// rmw_fast_rtps/rmw_fast_rtps_shared_cpp/src/types/guard_condition.hpp
class GuardCondition
{
...
private:
  std::mutex internalMutex_;
  std::atomic_bool hasTriggered_;
  std::mutex * conditionMutex_ RCPPUTILS_TSA_GUARDED_BY(internalMutex_);
  std::condition_variable * conditionVariable_ RCPPUTILS_TSA_GUARDED_BY(internalMutex_);
};
```

事實上不只 GuardCondition 可以連接到 Waitset 上，如 subscription, service, client, events 都是可以連接的物件，以下為 ROS2 中呼叫 `rcl_wait()` 最後會執行的函式，同樣取自 `rmw_fastrtps_cpp` 的實作 (`rmw_wait()` -> `__rmw_wait()`)

```cpp
rmw_ret_t
__rmw_wait(
  rmw_subscriptions_t * subscriptions,
  rmw_guard_conditions_t * guard_conditions,
  rmw_services_t * services,
  rmw_clients_t * clients,
  rmw_events_t * events,
  rmw_wait_set_t * wait_set,
  const rmw_time_t * wait_timeout)
{
  ... // attach all objects to waitset

  bool hasData = check_wait_set_for_data(
    subscriptions, guard_conditions, services, clients, events);
  auto predicate = [subscriptions, guard_conditions, services, clients, events]() {
      return check_wait_set_for_data(subscriptions, guard_conditions, services, clients, events);
    };

  bool timeout = false;
  if (!hasData) {
    if (!wait_timeout) {
      conditionVariable->wait(lock, predicate);
    } else if (wait_timeout->sec > 0 || wait_timeout->nsec > 0) {
      auto n = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::seconds(wait_timeout->sec));
      n += std::chrono::nanoseconds(wait_timeout->nsec);
      timeout = !conditionVariable->wait_for(lock, n, predicate);
    } else {
      timeout = true;
    }
  }

 ... // detach all object to waitset

  return timeout ? RMW_RET_TIMEOUT : RMW_RET_OK;
}
```

### rosidl and typesupport

ROS2 message (在 package 一般稱為 rosidl，在使用時直接稱為 msg) 主要由兩個 package 實作，rosidl_generator 和 rosidl_typesupport。

rosidl_generator 產生 ROS2 應用程式所要操作的資料結構，如最簡單的 std_msgs/String.msg
```
# std_msgs/String.msg
string data
```
rosidl_generator 便會產生有 data 資料成員的類別

```
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/String.idl
// generated code does not contain a copyright notice

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct String_
{

...
  // field types and members
  using _data_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _data_type data;

...

// alias to use template instance with default allocator
using String =
  std_msgs::msg::String_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // STD_MSGS__MSG__STRING__STRUCT_HPP_
```

rosidl_generator_c 中有 message_type_support_struct.h 標頭檔，定義了 `rosidl_message_type_support_t`，告訴底層如何操作 ros message，這部份則由 typesupport 來實作。

```cpp
typedef struct rosidl_message_type_support_t rosidl_message_type_support_t;

typedef const rosidl_message_type_support_t * (* rosidl_message_typesupport_handle_function)(
  const rosidl_message_type_support_t *, const char *);

struct rosidl_message_type_support_t
{
  const char * typesupport_identifier;
  const void * data;
  rosidl_message_typesupport_handle_function func;
};

ROSIDL_GENERATOR_C_PUBLIC
const rosidl_message_type_support_t * get_message_typesupport_handle(
  const rosidl_message_type_support_t * handle, const char * identifier);

ROSIDL_GENERATOR_C_PUBLIC
const rosidl_message_type_support_t * get_message_typesupport_handle_function(
  const rosidl_message_type_support_t * handle, const char * identifier);

#define ROSIDL_GET_MSG_TYPE_SUPPORT(PkgName, MsgSubfolder, MsgName) \
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME( \
    rosidl_typesupport_c, PkgName, MsgSubfolder, MsgName)()
```

#### typesupport


另一個 package 叫 rosidl_typesupport 必須實作 ros message 和 DDS 資料結構的轉換，因為每個 DDS 實作的保存、操作資料 API 名稱都不一樣，因此對應的 rmw adapter 必須連帶提供 rosidl_typesupport 這個 package，以 `rmw_fastrtps_cpp` 來說就有提供 rosidl_typesupport_fastrtps_c 和 rosidl_typesupport_fastrtps_cpp，也是使用自動產生程式碼的方式。

typesupport 相關討論
http://docs.ros2.org/latest/developer_overview.html#internal-api-architecture-overview
https://answers.ros.org/question/292170/rosidl-type-support-and-new-language/
https://groups.google.com/forum/#!topic/ros-sig-ng-ros/Nsd53ps3h6I


## RTPS Protocal

DDSI-RTPS 是 DDS 通訊協定，每種 DDS 的實作都必須符合這套協議的規範，所以就算不同的 DDS 之間也能溝通，例如 fastrtps 的 publisher 傳送資料給 cyclonedds 的 subscriber。


以下為 RTPS 封包的結構，RTPS 通常使用 UDP/IP，而一個 RTPS Packet 可以分類成一個 RTPS Header 和一到數個 RTPS SubMessage。

![](https://i.imgur.com/BYOQHcH.png)



---

Guid 是用來分辨一個 DDS Domain 上的某個 DDS Entity，如 DataReader, DataWriter 等，每個 DDS Entity 都有一個獨一無而的 Guid。

![](https://i.imgur.com/sgKw57y.png)

![](https://i.imgur.com/zuQyEDr.png)

```
PB = 7400
DG = 250
PG = 2
d0 = 0
d1 = 10
d2 = 1
d3 = 11
```


