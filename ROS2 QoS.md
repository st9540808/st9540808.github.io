###### tags: `Robotics`

# ROS2 QoS

## Overview

來玩玩 ROS2 的 QoS 設定囉，Qos 讓使用者可以根據自己的應用微調傳輸行為，也能設定使用者定義的 callback function，來接收來自 ROS2 的事件。

若要使用到完整 QoS 建議使用最新版 ROS2 (目前為 Foxy Fitzroy)，雖然某些 QoS 在 Dashing 就已經引入，但因為不同 DDS 版本的 rmw_implementation 的實作進度都不一樣，舊版的 ROS2 可能會直接忽略某些 QoS 設定，例如 cyclonedds 在 Dashing 釋出版本中就不支援 deadline, lifespan, liveliness。這裡的不支援指的是對 ROS2 而言，底層的 DDS 可能早就有實作出來，只是沒有對接到 ROS2 層。

目前 ROS2 提供的 QoS 整理如下

| Policy   | Description |
| -------- | ----------- |
| History  | ==Keep last==: only store up to N samples, configurable via the queue <ins>**depth**</ins> option. <p></p> ==Keep all==: store all samples, subject to the configured resource limits of the underlying middleware.
| Depth | Depth of history queue when specifying **Keet last** |
| Reliability | ==Best effort==: attempt to deliver samples, but may lose them if the network is not robust. <p></p> ==Reliable==: guarantee that samples are delivered, may retry multiple times.
| Durability | ==Transient local==: the publisher becomes responsible for persisting samples for “<ins>late-joining</ins>” subscribers. <p></p> ==Volatile==: no attempt is made to persist samples. |

Dashing 新加入的 QoS

| Policy   | Description |
| -------- | ----------- |
| Deadline  | Duration: the expected maximum amount of time between subsequent messages being published to a topic
| Lifespan | 傳送的訊息可以存活多久。Duration: the maximum amount of time between the publishing and the reception of a message without the message being considered stale or expired (expired messages are silently dropped and are effectively never received). |
| Liveliness | 設定 Lease Duration，超過一定的時間就將 publisher 認定為下線 <br><br> ==Automatic==: the system will consider all of the node’s publishers to be alive for another “lease duration” when any one of its publishers has published a message.  <br><br> ==Manual by topic==: the system will consider the publisher to be alive for another “lease duration” if it manually asserts that it is still alive (via a call to the publisher API). |


:::info
以上 ROS2 支援的 QoS， 事實上只佔 DDS 標準 (OMG DDS 1.4) 中定義的一小部份。

而每種 DDS 實作支援的 QoS ==參數==也不盡相同，標準中只要求所有 DDS 都必須實作出 Minimum profile 定義的 QoS 參數。
:::

### Request-Offer Model

![](https://i.imgur.com/obDAyQM.png)

要讓 Publisher 和 Subscriber 能建立連線，兩者設定的 QoS 必須要相容，DDS 採用的是 Request-Offer 的模型，簡單來說 Publisher 提供的通訊等級必須大於或等於 Subscriber 要求的。詳細的相容表可參考 ROS2 官方文件。

https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/#qos-compatibilities

舉例來說 Reliability QoS 之中若使用 Reliable 會確保訊息一定會被對方收到，而 Best Effort 則不會管這件事。所以直覺上來說 Reliable 提供的等級會比 Best Effort 來的高，如果 Publisher 只能提供 Best Effort 但是 Subscriber 卻要求 Reliable 的傳輸，這時候兩者就不會相容。

| Publisher QoS | Subscriber QoS| Compatible? |
| --------- | -------- | -------- |
| Reliable     | Reliable     | :heavy_check_mark: |
| Reliable     | Best Effort     | :heavy_check_mark:     |
| Best Effort     | Reliable     | :x:     |
| Best Effort     |  Best Effort     | :heavy_check_mark:     |



### Lifespan

ROS2 文件的定義
> Duration: the maximum amount of time between the publishing and the reception of a message without the message being considered stale or expired (expired messages are silently dropped and are effectively never

## QoS Profiles

ROS2 為四種類型的通訊提供了預設的 QoS

### Default profile

ROS2 Publisher, Subscriber 預設使用的 QoS

| Policy   | Description |
| -------- | ----------- |
| History  |  Keep Last  |
| Depth    |  10         |
| Reliability | Reliable | 
| Durability | Volatile |

其他 ROS2 預設提供的 QoS:

### Services / Clients

Services, Clients 為 ROS2 的遠端程序呼叫 (RPC) 的 Server 和 Client，關於 ROS2 的通訊模型的簡介[可參考這篇文章](https://pojenlai.wordpress.com/2012/11/03/ros-topic-service-and-actionlib/)。ROS2 中有為 Service 設定預設使用 QoS。(跟 default profile 完全一樣)


### Sensor data

有一些資料比起確實傳到對方，應用程式會想在最短的時間內獲得資料，或者是允許中途失去資料，如影像、Lidar 等資訊，可以直些套用 Sensor Data Profile 定義的 QoS。

### Parameter

### ROS2 QoS Definition

以下為 ROS2 幾個預先定義的 QoS Profile
```c++
SensorDataQoS::SensorDataQoS(const QoSInitialization & qos_initialization)
: QoS(qos_initialization, rmw_qos_profile_sensor_data)
{}

ParametersQoS::ParametersQoS(const QoSInitialization & qos_initialization)
: QoS(qos_initialization, rmw_qos_profile_parameters)
{}

ServicesQoS::ServicesQoS(const QoSInitialization & qos_initialization)
: QoS(qos_initialization, rmw_qos_profile_services_default)
{}

ParameterEventsQoS::ParameterEventsQoS(const QoSInitialization & qos_initialization)
: QoS(qos_initialization, rmw_qos_profile_parameter_events)
{}

SystemDefaultsQoS::SystemDefaultsQoS(const QoSInitialization & qos_initialization)
: QoS(qos_initialization, rmw_qos_profile_system_default)
{}
```
## Examples

### rclcpp

[rclcpp QoS API Reference](http://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1QoS.html)

OSRF 推薦使用 `rclcpp::QoS` 的類別控制 ROS2 Publisher, Subscriber 的 QoS。 `rclcpp::QoS` 的 constructor 的參數為 Depth，要設定其他 QoS 請使用 cascading 的程式語言技巧，如以下範例指定 Reliabilty 為 Reliable; Durability 為 Transient Local

```cpp
rclcpp::QoS(10).reliable().transient_local()
```

以下示範設定 Publisher, Subscriber 的 QoS，Publisher 設定 Reliable，Subscriber 設定 Best Effort

#### Publisher
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Talker : public rclcpp::Node {
public:
  Talker()
  : Node("talker") {
    pub_ = this->create_publisher<std_msgs::msg::String>("chatter",
                                                         rclcpp::QoS(10).reliable())
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
```
#### Subscriber

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener : public rclcpp::Node
{
public:
  Listener() : Node("listener") {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", rclcpp::QoS(10).best_effort(),
      std::bind(&Listener::callback, this, std::placeholders::_1));
  }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};
```

如果設定其他預設的 QoS 只需要帶入第二個參數，如 `rclcpp::SensorDataQoS()` 物件等。


### rclpy

#### Resources and sample code
- [rclpy QoS API Reference](http://docs.ros2.org/latest/api/rclpy/api/qos.html)
- 官方寫的範例 code
    - https://github.com/ros2/examples/tree/master/rclpy/topics (簡單設定 Depth)
    - https://github.com/ros2/demos/tree/master/demo_nodes_py/demo_nodes_py/topics (設定 QoS)

:::warning
若要寫一個使用 rclpy 開發的 package，可以參考上面兩個範例的目錄結構，注意到 setup.py 和 __init__.py 這兩個檔案是必須要建立的，這樣才能夠用 `ros2 run` 命令執行你寫的 python 程式。
:::

使用 `rclpy.qos.QoSProfile` 類別設定 QoS，以下跟剛剛 rclcpp 的範例一樣建立一個 QoSProfile，並設定 Depth 為 10，Reliability 為 Reliable，Durability 為 Transient Local。要看所有可使用的參數請查閱 rclpy API。

```py    
qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
```

建立好 QoSProfile 就可以帶入 Publisher 或是 Subscriber 囉，以下程式碼示範在 python 使用 ROS2 建立一個 Subscriber 並套用剛剛設定的 QoS
```python
import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.node import Node

from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback, qos)
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

## Why should I care about QoS?

1. Performance

2. Behaviour



## References

- https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/
- Quality of Service Policies for ROS2 Communications: https://roscon.ros.org/2019/talks/roscon2019_qos.pdf
- https://www.slideshare.net/thomas.moulard/roscon-fr-is-ros-2-ready-for-production
- https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html