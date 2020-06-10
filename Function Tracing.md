###### tags: `Profiling`

# Function Tracing

:::info
- uftrace: function tracing in userspace
- trace-cmd: front end for ftrace in Linux
:::

## uftrace
uftrace 為 userspace 的函式追蹤器，使用方法跟輸出都類似 Linux 的 ftrace，以下筆記 uftrace 的使用方法。

### Prerequisite

確認被追蹤的函式庫和執行檔都有使用 `-pg` 編譯及連結，若使用 cmake 可以修改以下參數：

- CMAKE_C_FLAGS
- CMAKE_CXX_FLAGS
- CMAKE_EXE_LINKER_FLAGS

若使用 ROS2 的組建工具 colcon 則可以使用以下命令，自動組建可用 uftrace 追蹤的執行檔。
```bash
$ colcon build --symlink-install \
  --cmake-args -DCMAKE_C_FLAGS="-pg" \
               -DCMAKE_CXX_FLAGS="-pg" \
               -DCMAKE_EXE_LINKER_FLAGS="-pg" \
```

### Usage

> 常用指令的 man pages:
> [uftrace-record(1)](https://github.com/namhyung/uftrace/blob/master/doc/uftrace-record.md), [uftrace-report(1)](https://github.com/namhyung/uftrace/blob/master/doc/uftrace-report.md)

紀錄一個命令 :
```bash
$ uftrace record -a -k <command>
```
- `-a`: auto arguments recording
- `-k`: 追蹤 kernel 函數 (需要有 root 權限)

列出效能報表，依照函數花費的時間排序
```bash
$ uftrace report
```

Ouput example (紀錄 ROS2 應用程式的熱點):
```cpp
$ uftrace report
  Total time   Self time       Calls  Function
  ==========  ==========  ==========  ====================
    2.036  m    2.036  m         580  linux:schedule
    2.016  m  163.070 us          54  std::__invoke
    2.016  m   85.230 us          54  std::__invoke_impl
    2.016  m   32.664 us           6  std::thread::_State_impl::_M_run
    2.016  m    5.664 us           6  std::thread::_Invoker::operator()
    2.016  m   22.143 us           6  std::thread::_Invoker::_M_invoke
    1.008  m  493.544 us           3  eprosima::fastrtps::rtps::UDPChannelResource::perform_listen_operation
    1.007  m  392.896 us         113  eprosima::fastrtps::rtps::UDPChannelResource::Receive
    1.007  m  333.519 us         113  asio::basic_datagram_socket::receive_from
    1.007  m   84.633 us         113  asio::datagram_socket_service::receive_from
    1.007  m  548.420 us         113  asio::detail::reactive_socket_service::receive_from
    1.007  m   88.103 us         113  asio::detail::socket_ops::sync_recvfrom
    1.007  m    2.466 ms         113  asio::detail::socket_ops::recvfrom
   44.130  s    2.902 ms         404  rclcpp::sleep_for
   22.766  s    4.360 ms           1  main
   22.707  s    1.428 us           1  eprosima::fastrtps::rtps::ResourceEvent::run_io_service
   22.707  s    1.405 us           1  asio::io_service::run
   22.707  s  179.629 us           1  asio::detail::task_io_service::run
   22.707  s  325.524 us          49  asio::detail::task_io_service::do_run_one
   ...
```

印出函式呼叫關係，適合用在追蹤大型的軟體架構：
```
$ uftrace replay
```
單純使用 `replay` 可能會印出太多函式，以 ROS2 為例一個簡單的 talker  (不斷的發送資料給其他節點) 會印出約近千萬筆函式呼叫的紀錄，因此需搭配 uftrace 提供的過濾功能：

只列出幾個常用的選項，以下旗標可以在 uftrace 的任何子命令使用
- `-F FUNC`: 函式白名單，只列出 FUNC 函式
- `-N FUNC`: 函式黑名單，排除 FUNC 函式
- `-C FUNC`: 列出所有呼叫 FUNC 的函式，也就是列出 FUNC 的 caller
- <mark>`-t TIME`: 以函式的執行時間過濾，只列出執行時間大於 TIME 的函式 (最好用)</mark>

使用範例，列出所有執行時間大於 50us 且函式名稱開頭不是 std 的所有函式：
```
$ uftrace replay -N ^std -t 50us
```

輸出結果，只列出其中一部份：
```
$ uftrace replay -N ^std -t 50us
...
            [ 30561] |   rclcpp::Publisher::publish(0x562cc5ba4150, 0x7ffdcdd97ab0) {
            [ 30561] |     rclcpp::Publisher::do_inter_process_publish(0x562cc5ba4150, 0x7ffdcdd97ab0) {
            [ 30561] |       rcl_publish() {
            [ 30561] |         rcl_publish(0x562cc5ba4168, 0x7ffdcdd97ab0, 0) {
            [ 30561] |           rmw_publish(0x562cc5bf8230, 0x7ffdcdd97ab0, 0) {
            [ 30561] |             rmw_publish() {
            [ 30561] |               rmw_fastrtps_shared_cpp::__rmw_publish() {
            [ 30561] |                 eprosima::fastrtps::Publisher::write() {
            [ 30561] |                   eprosima::fastrtps::PublisherImpl::create_new_change() {
            [ 30561] |                     eprosima::fastrtps::PublisherImpl::create_new_change_with_params() {
            [ 30561] |                       rmw_fastrtps_shared_cpp::TypeSupport::serialize() {
            [ 30561] |                         rmw_fastrtps_cpp::TypeSupport::serializeROSmessage() {
  50.561 us [ 30561] |                           std_msgs::msg::typesupport_fastrtps_cpp::_String__cdr_serialize();
  64.284 us [ 30561] |                         } /* rmw_fastrtps_cpp::TypeSupport::serializeROSmessage */
  72.783 us [ 30561] |                       } /* rmw_fastrtps_shared_cpp::TypeSupport::serialize */
            [ 30561] |                       eprosima::fastrtps::PublisherHistory::add_pub_change() {
            [ 30561] |                         eprosima::fastrtps::rtps::WriterHistory::add_change_() {
            [ 30561] |                           eprosima::fastrtps::rtps::StatefulWriter::unsent_change_added_to_history() {
  66.144 us [ 30561] |                             eprosima::fastrtps::rtps::ChangeForReader_t::ChangeForReader_t();
  55.405 us [ 30561] |                             eprosima::fastrtps::rtps::ReaderProxy::add_change();
  51.435 us [ 30561] |                             eprosima::fastrtps::rtps::AsyncWriterThread::wakeUp();
 211.351 us [ 30561] |                           } /* eprosima::fastrtps::rtps::StatefulWriter::unsent_change_added_to_history */
 245.287 us [ 30561] |                         } /* eprosima::fastrtps::rtps::WriterHistory::add_change_ */
 247.500 us [ 30561] |                       } /* eprosima::fastrtps::PublisherHistory::add_pub_change */
 429.102 us [ 30561] |                     } /* eprosima::fastrtps::PublisherImpl::create_new_change_with_params */
 440.351 us [ 30561] |                   } /* eprosima::fastrtps::PublisherImpl::create_new_change */
 445.366 us [ 30561] |                 } /* eprosima::fastrtps::Publisher::write */
 446.717 us [ 30561] |               } /* rmw_fastrtps_shared_cpp::__rmw_publish */
 447.939 us [ 30561] |             } /* rmw_publish */
 450.116 us [ 30561] |           } = 0; /* rmw_publish */
 457.967 us [ 30561] |         } = 0; /* rcl_publish */
 459.788 us [ 30561] |       } /* rcl_publish */
 460.571 us [ 30561] |     } /* rclcpp::Publisher::do_inter_process_publish */
 461.153 us [ 30561] |   } /* rclcpp::Publisher::publish */
```

### Set Record Whitelist
指定所有要追蹤的函式，以下為 ROS2 使用 fastrtps 當作底層 DDS middleware 時，傳輸和接收一個 ros message 所呼叫的函式 (按照呼叫順序)。
```
// publish side
rcl_publish
eprosima::fastrtps::Publisher::write
eprosima::fastrtps::PublisherHistory::add_pub_change

eprosima::fastrtps::rtps::UDPTransportInterface::ShrinkLocatorLists
eprosima::fastrtps::rtps::NetworkFactory::ShrinkLocatorLists
eprosima::fastrtps::rtps::RTPSMessageGroup::add_data
eprosima::fastrtps::rtps::RTPSMessageGroup::~RTPSMessageGroup

// subscription side
rmw_wait
eprosima::fastrtps::rtps::ReceiverResource::OnDataReceived
eprosima::fastrtps::SubscriberHistory::add_received_change
rclcpp::executor::Executor::execute_subscription
rmw_take_with_info
```

如果直接使用 uftrace 過濾函式名稱，使用過後覺得有些困難，不僅是因為 mcount 會造成 overhead，造成函式之間經過的時間失真，而且過濾規則可能要套用 uftrace 的 TRIGGER 選項，指定呼叫的深度，因此改用 bpftrace 簡單寫一個腳本觀察呼叫時間。

以下為輸出範例，最左邊為 timestamp (單位為 us)，lat 代表函式進入和回傳經過的時間
```
...
29043337757, rmw_wait entry
29043436718, ReceiverResource::OnDataReceived
29043436796, add_received_change
29043436937, rmw_wait exit, lat: 99219 us
29043437122, rmw_take_with_info, lat: 58 us
...
```

## trace-cmd

trace-cmd 為 ftrace 提供一個簡易使用的前端命令，追蹤 Linux kernel 中的函式呼叫。

### 1. Trace ROS2 Applications Function Calls

ftrace 會追蹤整個系統上的核心的函式呼叫，第一個欄位會顯示此呼叫所屬的程式為何 (格式為 comm-pid)，因 ROS2 使用 DDS 進行傳輸，DDS 的通訊協定 DDSI-RTPS 是建立在 UDP 之上，因此觀察 `udp_sendmsg` 了解系統行為及效能，以下為追蹤使用的命令：

```bash
$ trace-cmd record -p function_graph -c -g udp_sendmsg <command>
```
- `-p`: 選擇要使用的 plugin，function_graph 會紀錄某個函式及其所呼叫的函式
- `-c`: 表示要紀錄 child process
- `-g`: 此選項提供給 function_graph，指定要紀錄的函式，這裡紀錄 `udp_sendmsg`

:::info
plugin 總共有 function, function_graph, preemptirqsoff, irqsoff, preemptoff, and wakeup

請參考 [trace-cmd-record(1)](https://linux.die.net/man/1/trace-cmd-record)
:::

### Output Example

```
$ trace-cmd report
```

單行範例，每一行皆代表一個事件，funcgraph_entry 代表函式進入。
```
talker-22190 [002] 567757.342223: funcgraph_entry: |  udp_sendmsg() {
```


以下為修改過後的輸出結果，拿掉 plugin 名稱 (funcgraph_entry, funcgraph_exit)。
```
$ trace-cmd report | cut --complement -c1-10,44-65 | less
...

talker-22196 [009] 567758.376036:              |  udp_sendmsg() {
talker-22196 [009] 567758.376042:              |    lock_sock_nested() {
talker-22196 [009] 567758.376044:              |      _cond_resched() {
talker-22196 [009] 567758.376046:   1.667 us   |        rcu_all_qs();
talker-22196 [009] 567758.376050:   5.500 us   |      }
talker-22196 [009] 567758.376052:   1.724 us   |      _raw_spin_lock_bh();
talker-22196 [009] 567758.376055:   1.666 us   |      __local_bh_enable_ip();
talker-22196 [009] 567758.376058: + 16.475 us  |    }
talker-22196 [009] 567758.376060:   2.093 us   |    __cgroup_bpf_run_filter_sock_addr();
talker-22196 [009] 567758.376064:              |    release_sock() {
talker-22196 [009] 567758.376065:   1.662 us   |      _raw_spin_lock_bh();
talker-22196 [009] 567758.376069:   1.848 us   |      ip4_datagram_release_cb();
talker-22196 [009] 567758.376072:              |      _raw_spin_unlock_bh() {
talker-22196 [009] 567758.376074:   1.666 us   |        __local_bh_enable_ip();
talker-22196 [009] 567758.376077:   4.750 us   |      }
talker-22196 [009] 567758.376079: + 15.150 us  |    }
talker-22196 [009] 567758.376081:   1.896 us   |    security_sk_classify_flow();
talker-22196 [009] 567758.376085:              |    ip_route_output_flow() {
talker-22196 [009] 567758.376086:              |      ip_route_output_key_hash() {
talker-22196 [009] 567758.376088:              |        ip_route_output_key_hash_rcu() {
talker-22196 [009] 567758.376090:   4.407 us   |          fib_table_lookup();
talker-22196 [009] 567758.376097:   1.615 us   |          find_exception();
talker-22196 [009] 567758.376101: + 13.135 us  |        }
talker-22196 [009] 567758.376103: + 16.453 us  |      }
...
talker-22196 [009] 567758.376329:              |    udp_send_skb.isra.47() {
talker-22196 [009] 567758.376331:   1.708 us   |      udp4_hwcsum();
talker-22196 [009] 567758.376334:              |      ip_send_skb() {
talker-22196 [009] 567758.376336:              |        ip_local_out() {
talker-22196 [009] 567758.376337:              |          __ip_local_out() {
talker-22196 [009] 567758.376339:   1.635 us   |            ip_send_check();
talker-22196 [009] 567758.376342:   4.588 us   |          }
talker-22196 [009] 567758.376344:              |          ip_output() {
talker-22196 [009] 567758.376345:              |            ip_finish_output() {
talker-22196 [009] 567758.376347:   2.084 us   |              __cgroup_bpf_run_filter_skb();
talker-22196 [009] 567758.376351:              |              __ip_finish_output() {
talker-22196 [009] 567758.376353:   1.640 us   |                ipv4_mtu();
talker-22196 [009] 567758.376356:              |                ip_finish_output2() {
talker-22196 [009] 567758.376360:              |                  dev_queue_xmit() {
talker-22196 [009] 567758.376362:              |                    __dev_queue_xmit() {
talker-22196 [009] 567758.376364:   1.771 us   |                      netdev_core_pick_tx();
talker-22196 [009] 567758.376368:              |                      validate_xmit_skb() {
talker-22196 [009] 567758.376369:              |                        netif_skb_features() {
talker-22196 [009] 567758.376372:   1.682 us   |                          skb_network_protocol();
talker-22196 [009] 567758.376375:   5.624 us   |                        }
talker-22196 [009] 567758.376377:   1.694 us   |                        skb_csum_hwoffload_help();
talker-22196 [009] 567758.376380:   1.734 us   |                        validate_xmit_xfrm();
talker-22196 [009] 567758.376384: + 15.748 us  |                      }
talker-22196 [009] 567758.376387:              |                      dev_hard_start_xmit() {
...
```

:::info
使用 function_graph 回報的函式執行時間，會受到其內部函式的呼叫次數影響，呼叫次數越多或層數越多，overhead 就越大，若要精準的執行時間要使用以下命令： 
```bash
$ trace-cmd record -p function -l <func_name>
```
:::

### 2. Trace a BPF Helper Calls

TC (Traffic Control) 在 Linux 是用來過濾並排程封包的子系統，在 TC 中也有一個 eBPF hook，可以讓我們掛載自訂 eBPF 程式過濾封包。在 Linux Network Stack 之中呼叫 `cls_bpf_classify` 時就會執行目前註冊的 eBPF 程式。

```
cpus=12
...
   ssh-15986 [002]  2882.242765: funcgraph_entry:                   |  cls_bpf_classify() {
   ssh-15986 [002]  2882.242767: funcgraph_entry:        0.213 us   |    bpf_skb_load_bytes();
   ssh-15986 [002]  2882.242767: funcgraph_exit:         2.816 us   |  }
<idle>-0     [002]  2882.243697: funcgraph_entry:                   |  cls_bpf_classify() {
<idle>-0     [002]  2882.243697: funcgraph_entry:        0.150 us   |    bpf_skb_load_bytes();
<idle>-0     [002]  2882.243698: funcgraph_exit:         1.330 us   |  }
talker-17381 [003]  2882.290333: funcgraph_entry:                   |  cls_bpf_classify() {
talker-17381 [003]  2882.290339: funcgraph_entry:        1.255 us   |    bpf_skb_load_bytes();
talker-17381 [003]  2882.290342: funcgraph_entry:        1.022 us   |    bpf_skb_load_bytes();
talker-17381 [003]  2882.290344: funcgraph_entry:        1.031 us   |    bpf_skb_load_bytes();
talker-17381 [003]  2882.290346: funcgraph_entry:        0.978 us   |    bpf_skb_load_bytes();
talker-17381 [003]  2882.290348: funcgraph_entry:        0.959 us   |    bpf_skb_load_bytes();
talker-17381 [003]  2882.290349: funcgraph_entry:        0.974 us   |    bpf_skb_load_bytes();
talker-17381 [003]  2882.290351: funcgraph_entry:        0.958 us   |    bpf_skb_load_bytes();
talker-17381 [003]  2882.290353: funcgraph_entry:        0.963 us   |    bpf_skb_load_bytes();
talker-17381 [003]  2882.290355: funcgraph_entry:                   |    bpf_ktime_get_ns() {
talker-17381 [003]  2882.290356: funcgraph_entry:        1.271 us   |      ktime_get_mono_fast_ns();
talker-17381 [003]  2882.290358: funcgraph_exit:         3.032 us   |    }
talker-17381 [003]  2882.290359: funcgraph_entry:                   |    bpf_skb_event_output() {
talker-17381 [003]  2882.290361: funcgraph_entry:                   |      perf_misc_flags() {
talker-17381 [003]  2882.290362: funcgraph_entry:        0.923 us   |        kvm_is_in_guest();
talker-17381 [003]  2882.290364: funcgraph_exit:         3.681 us   |      }
talker-17381 [003]  2882.290366: funcgraph_entry:        1.791 us   |      perf_output_begin();
talker-17381 [003]  2882.290369: funcgraph_entry:        1.083 us   |      perf_output_copy();
talker-17381 [003]  2882.290371: funcgraph_entry:        0.948 us   |      perf_output_copy();
talker-17381 [003]  2882.290373: funcgraph_entry:                   |      perf_output_end() {
talker-17381 [003]  2882.290374: funcgraph_entry:                   |        perf_output_put_handle() {
```

#### Dump Kernel Stack

印出呼叫函式時的堆疊。

```
          talker-30890 [010]  6385.274551: function:             cls_bpf_classify
          talker-30890 [010]  6385.274552: kernel_stack:         <stack trace>
=> cls_bpf_classify (ffffffffc0f38585)
=> tcf_classify (ffffffff82f597c0)
=> __dev_queue_xmit (ffffffff82efcc16)
=> dev_queue_xmit (ffffffff82efd1c0)
=> ip_finish_output2 (ffffffff82f786e1)
=> __ip_finish_output (ffffffff82f78a7a)
=> ip_finish_output (ffffffff82f78b6c)
=> ip_mc_output (ffffffff82f7a15d)
=> ip_local_out (ffffffff82f79aeb)
=> ip_send_skb (ffffffff82f7ac99)
=> udp_send_skb.isra.47 (ffffffff82fa9d76)
=> udp_sendmsg (ffffffff82faa6b7)
=> inet_sendmsg (ffffffff82fb9dad)
=> sock_sendmsg (ffffffff82ed79dc)
=> ___sys_sendmsg (ffffffff82ed8b19)
=> __sys_sendmsg (ffffffff82eda363)
=> __x64_sys_sendmsg (ffffffff82eda3bf)
=> do_syscall_64 (ffffffff8260441a)
=> entry_SYSCALL_64_after_hwframe (ffffffff8320008c)
          talker-30890 [010]  6385.274553: function:             bpf_ktime_get_ns
          talker-30890 [010]  6385.274554: kernel_stack:         <stack trace>
=> bpf_ktime_get_ns (ffffffff827e63d5)

```

---
## References:

### uftrace
uftrace-replay: https://github.com/namhyung/uftrace/blob/master/doc/uftrace-replay.md

### ftrace

trace-cmd tutorial: https://jvns.ca/blog/2017/03/19/getting-started-with-ftrace/