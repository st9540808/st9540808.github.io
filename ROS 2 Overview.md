# ROS2 Overview

## Distros
ROS2 was first announced at ROSCon 2014

| Distro | Release date |
| ------ | ------------ |
| alpha1 | Aug 31st, 2015 |
| beta1  | Dec 19th, 2016 |
| beta2  | Jul 5th, 2017 |
| beta3  | Sep 13th, 2017 |
| Ardent Apalone (Ver 1) |  Dec 8th, 2017 |

Latest distro and future release
| Name | Release date | EOL date / Support for |
| ----------------- | -------------- | -------------------- |
| ==Dashing Diademata== | May 31st, 2019 | May 2021 |
| Eloquent Elusor | November 22nd, 2019 | 1 year |
| Foxy Fitzroy | June 5th, 2020 | 3+ years (Target Ubuntu 20.04) |


## Why ROS2

Known issues and limits of ROS1

- **Master** (roscore): a major one
- Realtime performance
- Node management
:::info
- Multi-robots capability (Swarm)
Use ROS namespace would sometimes cause problems

:::
- Small processors
- Communication Priority
- Communication Security
- Unreliable network connectivity

ROS1 使用 TCPROS(預設)/UDPROS 的通訊協定，並且需要 master 管理所有系統上的節點，當 master 下線將失去對系統的掌握，且 master 一旦重新啟動，雖然舊有的節點仍然可以溝通，但是之後新增的節點就無法與舊有的節點溝通，這些特性是 ROS1 在從原型機到最後產品化時最大的困難。

![](https://i.imgur.com/ydvaxa3.png)

ROS2 use cases
- Teams of multiple robots (scalability)
- Small embedded platforms: support “bare-metal” micro controllers.
- Real-time systems: make real-time control directly in ROS, including inter-process and inter-machine communication.
- Non-ideal networks: from poor-quality WiFi to ground-to-space communication links.
- **Production environments**: while it is vital that ROS continue to be the platform of choice in the research lab, we want to ensure that ROS-based lab prototypes can evolve into ROS-based products suitable for use in real-world applications.



![](https://i.imgur.com/HqT6Whq.png)




## ROS2 architecture

### ROS1 ROS2 architecture comparsion
![](https://i.imgur.com/9VBoPDp.png)
> left: ROS1 &nbsp;&nbsp;&nbsp;&nbsp; right: ROS2

### ROS2 詳細架構圖
![](https://i.imgur.com/PPxzXNv.png)

### ROS2 Core Stack
* Build System
    * The `ament_package` Package
    * The `ament_cmake` Repository
    * The `ament_lint` Repository
    * Build tools
* Internal ROS Interfaces
    * Type Specific Interfaces
    * ==The `rcl` Repository==
    * ==The `rmw` Repository==
    * The `rosidl` Repository
    * The `rcutils` Repository
* ROS Middleware Implementations
    * Common Packages for DDS Middleware Packages
    * Structure of ROS Middleware Implementations
* ROS Client Interfaces (Client Libraries)
    * ==The `rclcpp` Package==
    * ==The `rclpy` Package==

---

## DDS in ROS2

DDS stands for Data Distribution Service, is a set of specifications standardized by the Object Management
Group (OMG).

| Specification | Description |
|-|-|
| DDS | Specification of the **programming model**, **QoS**, and **language APIs** used to program a Data-Centric Publish-Subscribe application. |
| DDS-RTPS (Data Distribution Service Real-Time Publish-Subscribe Interoperability Wire Protocol) | Specification of the wire protocol used by DDS to exchange information. It includes ==**discovery**==, **data encapsulation**, **reliability**, **multicast**, and many selectable QoS parameters. |

### DDS/RTPS vendors

| Product name | License | RMW implementation | Status |
| ------------ | ------- | ------------------ | ------ |
| eProsima Fast RTPS | Apache 2 | `rmw_fastrtps_cpp` | Full support (for `rmw` interface). **Default `rmw`**. Packaged with binary releases. |
| RTI Connext | commercial, research | `rmw_connext_cpp` | Full support. Support included in binaries, but Connext installed separately. |
| ADLINK Opensplice | Apache 2, commercial | `rmw_opensplice_cpp` | Partial support. Support included in binaries, but OpenSplice installed separately. |

### QoS

QoS policies of ROS2

| Policy   | Description |
| -------- | ----------- |
| History  | ==Keep last==: only store up to N samples, configurable via the queue <ins>**depth**</ins> option. <p></p> ==Keep all==: store all samples, subject to the configured resource limits of the underlying middleware.
| Reliability | ==Best effort==: attempt to deliver samples, but may lose them if the network is not robust. <p></p> ==Reliable==: guarantee that samples are delivered, may retry multiple times.
| Durability | ==Transient local==: the publisher becomes responsible for persisting samples for “<ins>late-joining</ins>” subscribers. <p></p> ==Volatile==: no attempt is made to persist samples. |

### Technologies relevant to ROS
- Zeroconf
- Protocol Buffers
- ZeroMQ (and the other MQs)
- Redis
- WebSockets

---
References:

ROS2 wiki
- Overview of ROS2 concept: https://index.ros.org/doc/ros2/Concepts/Overview-of-ROS-2-concepts/
- Quality of Service Settings: https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/
- Exploring the performance of ROS2: https://ieeexplore.ieee.org/document/7743223/metrics#metrics
- Hypha ROS2 introdution https://drive.google.com/file/d/1MW_w7MS1DNg1EzhprgbJKY2cqmxksPaw/view
- DDS Best Practices https://www.rti.com/hubfs/docs/DDS_Best_Practices_WP.pdf
- ROS core http://docs.ros2.org/crystal/index.html
- Current Status of ROS2 https://static1.squarespace.com/static/51df34b1e4b08840dcfd2841/t/5ce6c85ca4222fe0ccbd5309/1558628472094/2019-05-07_Current_Status_of_ROS_2.pdf
- Autoware migration from ROS1 to ROS2: https://apex-ai.docsend.com/view/bzyahhy