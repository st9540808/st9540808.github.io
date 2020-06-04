###### tags: `Profiling`

# CycloneDDS Tracing

As of dashing release

```bash
$ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
```bash
$ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ros2 run <package_name> <executable_name>
```

## Implementation Details

`dds_entity_t` 為一個 int32_t 型別的整數，在 cyclonedds 當作一個 handle，會在 dds_handles.c 此檔案中的函式轉換成對應的 `dds_entity` 物件的指標

dds_handles.c 中有個全域的鎖，控制所有 handle 的轉換。
```c=35
struct dds_handle_server {
  struct ddsrt_hh *ht;
  size_t count;
  ddsrt_mutex_t lock;
  ddsrt_cond_t cond;
};

static struct dds_handle_server handles;
```

ddssrt_hh 定義在 hopscotch.c
```c=33
struct ddsrt_hh {
  uint32_t size; /* power of 2 */
  struct ddsrt_hh_bucket *buckets;
  ddsrt_hh_hash_fn hash;
  ddsrt_hh_equals_fn equals;
};
```

handle 轉換行為：
1. `dds_handle_pin_int()` 會把 `dds_entity_t` 轉換成 `dds_handle_link`
2. `dds_entity_from_handle_link` 再把 `dds_handle_link` 轉換成 `dds_entity`，原理是用常見的 offsetof 計算出物件的記憶體位址


### Hopscotch Hashing[^1]

CycloneDDS 使用 hopscotch 將一個 32-bit 的整數轉換成物件的指標，hopscotch 整合了三種技巧

- Chained hashing
- Linear probing
- Cuckoo hashing

Each entry includes a hop-information word, an H-bit bitmap that indicates which
of the next H − 1 entries contain items that hashed to the current entry’s virtual
bucket

![](https://i.imgur.com/u2LcNAb.png)



## Function Call Graph
參考這個專案在 github 上的 [**issue**]( https://github.com/ros2/rmw_cyclonedds/issues/79)，詳細描述了幾個重要函式的功能

- Publisher Side
```
rmw_publish
dds_write
write_sample_gc
transmit_sample
sendmsg
```

- Subscriber Side

"cracking" 會將 UDP dataframe 的資料內容切割成一到數個 RTPS Submessage，當看到 DATA 這個 Submessage 時會將資料儲存到 Reader History Cache (`rhc_store`)。
```
nn_rmsg_new
recvmsg
"cracking"
deliver_user_data
rhc_store (once per reader)

dds_take
rmw_take
```

使用 uftrace 追蹤函式呼叫
```sh
# uftrace record -a -k ./install/ros_course_demo/lib/ros_course_demo/talker
$ uftrace replay -t 50us -N ^std
```
:::spoiler
### sender side:
```c
            [  1918] |     rclcpp::Publisher::publish(0x55a527a01fd0, 0x7ffc0f388f60) {
            [  1918] |       rclcpp::Publisher::do_inter_process_publish(0x55a527a01fd0, 0x7ffc0f388f60) {
            [  1918] |         rcl_publish() {
            [  1918] |           rcl_publish() {
            [  1918] |             rmw_publish() {
            [  1918] |               rmw_publish() {
            [  1918] |                 dds_write(0x31f6a16b, 0x7ffc0f388f60) {
  50.314 us [  1918] |                   dds_entity_lock(0x31f6a16b, DDS_KIND_WRITER, 0x7ffc0f388920) = 0;
            [  1918] |                   dds_write_impl(0x55a527a05410, 0x7ffc0f388f60, 0x15ffc9ab5cbf0e57, DDS_WR_ACTION_WRITE) {
            [  1918] |                     serdata_rmw_from_sample() {
 146.199 us [  1918] |                       cycser::cycser();
            [  1918] |                       rmw_cyclonedds_cpp::TypeSupport::serializeROSmessage() {
            [  1918] |                         rmw_cyclonedds_cpp::TypeSupport::serializeROSmessage() {
            [  1918] |                           rmw_cyclonedds_cpp::serialize_field() {
            [  1918] |                             cycser::operator<<() {
            [  1918] |                               cycser::serialize() {
            [  1918] |                                 cycser::serialize() {
 113.896 us [  1918] |                                   cycser::resize();
 121.100 us [  1918] |                                 } /* cycser::serialize */
  94.543 us [  1918] |                                 cycser::resize();
 226.920 us [  1918] |                               } /* cycser::serialize */
 228.437 us [  1918] |                             } /* cycser::operator<< */
 230.036 us [  1918] |                           } /* rmw_cyclonedds_cpp::serialize_field */
 232.862 us [  1918] |                         } /* rmw_cyclonedds_cpp::TypeSupport::serializeROSmessage */
 238.160 us [  1918] |                       } /* rmw_cyclonedds_cpp::TypeSupport::serializeROSmessage */
 596.666 us [  1918] |                     } /* serdata_rmw_from_sample */
            [  1918] |                     ddsi_tkmap_lookup_instance_ref(0x55a5279d08c0, 0x55a527a0cab0) {
  97.859 us [  1918] |                       ddsi_tkmap_find(0x55a5279d08c0, 0x55a527a0cab0, 1) = 0x55a5279dd080;
 102.073 us [  1918] |                     } = 0x55a5279dd080; /* ddsi_tkmap_lookup_instance_ref */
            [  1918] |                     write_sample_gc(0x55a5279c9c00, 0x55a527a056a0, 0x55a527a0d950, 0x55a527a0cab0, 0x55a5279dd080) {
            [  1918] |                       write_sample_eot.isra.2.constprop.5() {
            [  1918] |                         whc_default_insert(0x55a527a0dc40, 10, 11, 0, 0x55a527a0cab0, 0x55a5279dd080) {
  58.986 us [  1918] |                           whc_default_insert_seq(0x55a527a0dc40, 10, 11, 0, 0x55a527a0cab0) = 0x55a527a0e630;
 105.006 us [  1918] |                         } = 0; /* whc_default_insert */
  93.393 us [  1918] |                         create_fragment_message(0x55a527a0d950, 11, 0, 0x55a527a0cab0, 0, 0, 0x7ffc0f388170, 1) = 0;
  55.996 us [  1918] |                         create_fragment_message(0x55a527a0d950, 11, 0, 0x55a527a0cab0, 1, 0, 0x7ffc0f388170, 1) = 0;
            [  1918] |                         writer_hbcontrol_piggyback(0x55a527a0d950, 0x7ffc0f388180, 0xec3fb2985ea, 11, 0x7ffc0f388178) {
            [  1918] |                           writer_hbcontrol_note_asyncwrite(0x55a527a0d950, 0xec3fb2985ea) {
            [  1918] |                             resched_xevent_if_earlier(0x55a527a05b10, 0xec4011f66ea) {
            [  1918] |                               ddsrt_cond_signal(0x55a5272e9f50) {
  65.324 us [  1918] |                                 __x64_sys_futex();
  77.335 us [  1918] |                               } /* ddsrt_cond_signal */
 100.028 us [  1918] |                             } = 1; /* resched_xevent_if_earlier */
 104.070 us [  1918] |                           } /* writer_hbcontrol_note_asyncwrite */
 153.544 us [  1918] |                         } = 0x55a527a0bd80; /* writer_hbcontrol_piggyback */
            [  1918] |                         nn_xpack_send(0x55a527a056a0, 1) {
            [  1918] |                           nn_xpack_send_real(0x55a527a056a0) {
            [  1918] |                             addrset_forall_count(0x55a527a02090, &nn_xpack_send1v, 0x55a527a056a0) {
            [  1918] |                               ddsrt_avl_cwalk(&addrset_treedef, 0x55a527a020c0, &addrset_forall_helper, 0x7ffc0f388060) {
            [  1918] |                                 ddsrt_avl_walk(&addrset_treedef, 0x55a527a020c0, &addrset_forall_helper, 0x7ffc0f388060) {
            [  1918] |                                   addrset_forall_helper(0x55a5279f6dd0, 0x7ffc0f388060) {
            [  1918] |                                     nn_xpack_send1v(0x55a5279f6df0, 0x55a527a056a0) {
            [  1918] |                                       nn_xpack_send1(0x55a5279f6df0, 0x55a527a056a0) {
            [  1918] |                                         ddsi_udp_conn_write(0x55a5278a4c80, 0x55a5279f6df0, 7, 0x55a527a133c0, 0) {
            [  1918] |                                           ddsrt_sendmsg(4, 0x7ffc0f387af0, 0, 0x7ffc0f387ae8) {
            [  1918] |                                             __x64_sys_sendmsg() {
 186.295 us [  1918] |                                             } /* __x64_sys_sendmsg */
```
### Receiver side:

```cpp
  13.158 ms [ 31682] |               /* linux:sched-in */
  13.171 ms [ 31682] |             } /* __x64_sys_recvmsg */
  13.174 ms [ 31682] |           } = 0; /* ddsrt_recvmsg */
  13.177 ms [ 31682] |         } = 2236; /* ddsi_udp_conn_read */
            [ 31682] |         handle_submsg_sequence(0x559cfd8295c0, 0x559cfd82b888, 0x559cfd701820, 0x7fed5daf4420, 0x16105b3acd72332e, 0x72e848396e77
  11.750 us [ 31682] |           handle_regular(0x7fed5dafa570, 0x72e848396e77, 0x7fed5daf9c80, 0x7fed5daf9cd0, 0x7fed5daf3ed0, 0, 0x7fed5dafa5c0, 0x7fe
            [ 31682] |           handle_regular(0x7fed5dafa570, 0x72e848396e77, 0x7fed5daf9c80, 0x7fed5dafa210, 0x7fed5daf3ed0, 1, 0x7fed5dafa698, 0x7fe
            [ 31682] |             deliver_user_data_synchronously.isra.10() {
            [ 31682] |               deliver_user_data(0x7fed5dafa628, 0x7fed5dafa5c0, 0, 1) {
            [ 31682] |                 new_sample_from_data(0x7fed5daf3368, 0x559cfd82b888, 0x7fed5dafa628, 4, 0x7fed5daf3390, 0x7fed5dafa5c0, 0, 0x1610
            [ 31682] |                   serdata_rmw_from_ser() {
  53.215 us [ 31682] |                   } /* serdata_rmw_from_ser */
  59.500 us [ 31682] |                 } = 0x7fed4c01af10; /* new_sample_from_data */
            [ 31682] |                 dds_rhc_default_store_wrap(0x559cfd852f10, 0x7fed5daf3370, 0x7fed4c01af10, 0x7fed4c01b6d0) {
  20.654 us [ 31682] |                   dds_rhc_default_store(0x559cfd852f10, 0x7fed5daf3370, 0x7fed4c01af10, 0x7fed4c01b6d0) = 1;
  21.352 us [ 31682] |                 } = 1; /* dds_rhc_default_store_wrap */
  86.383 us [ 31682] |               } = 0; /* deliver_user_data */
  87.076 us [ 31682] |             } /* deliver_user_data_synchronously.isra.10 */
  96.277 us [ 31682] |           } /* handle_regular */
            [ 31682] |           ddsrt_avl_walk(&pwr_readers_treedef, 0x7fed50020e10, &handle_Heartbeat_helper, 0x7fed5daf3ed0) {
  11.268 us [ 31682] |             handle_Heartbeat_helper(0x559cfd85a970, 0x7fed5daf3ed0);
  11.732 us [ 31682] |           } /* ddsrt_avl_walk */
 151.351 us [ 31682] |         } = 0; /* handle_submsg_sequence */
  13.336 ms [ 31682] |       } = 1; /* do_packet */
            [ 31682] |       do_packet(0x559cfd8295c0, 0x559cfd82b888, 0x559cfd701820, 0, 0x559cfd82f300) {
            [ 31682] |         ddsi_udp_conn_read(0x559cfd701820, 0x7fed5daf9cb0, 65536, 1, 0x7fed5daf4420) {
            [ 31682] |           ddsrt_recvmsg(4, 0x7fed5daf4220, 0, 0x7fed5daf4208) {
            [ 31682] |             __x64_sys_recvmsg() {
            [ 31682] |               /* linux:sched-out */

  97.359 ms [ 31665] |                               /* linux:sched-in */
  97.370 ms [ 31665] |                             } /* __x64_sys_futex */
  97.374 ms [ 31665] |                           } /* ddsrt_cond_wait */
  97.375 ms [ 31665] |                         } = 1; /* ddsrt_cond_waituntil */
  97.387 ms [ 31665] |                       } = 1; /* dds_waitset_wait_impl */
  97.388 ms [ 31665] |                     } = 1; /* dds_waitset_wait */
  15.588 us [ 31665] |                 } /* std::remove */
            [ 31665] |                 std::vector::erase() {
  13.480 us [ 31665] |                   std::vector::_M_erase();
  18.090 us [ 31665] |                 } /* std::vector::erase */
 125.757 us [ 31665] |               } /* rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy::remove_null_handles */
  97.895 ms [ 31665] |             } /* rclcpp::executor::Executor::wait_for_work */
            [ 31665] |             rclcpp::executor::Executor::get_next_ready_executable() {
  22.258 us [ 31665] |               rclcpp::executor::Executor::get_next_timer();
            [ 31665] |               rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy::get_next_subscription() {
  35.065 us [ 31665] |                 rclcpp::memory_strategy::MemoryStrategy::get_subscription_by_handle();
  20.685 us [ 31665] |                 rclcpp::memory_strategy::MemoryStrategy::get_group_by_subscription();
  10.335 us [ 31665] |                 rclcpp::memory_strategy::MemoryStrategy::get_node_by_group();
 100.980 us [ 31665] |               } /* rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy::get_next_subscription */
 124.975 us [ 31665] |             } /* rclcpp::executor::Executor::get_next_ready_executable */
  98.066 ms [ 31665] |           } /* rclcpp::executor::Executor::get_next_executable */
            [ 31665] |           rclcpp::executor::Executor::execute_any_executable() {
            [ 31665] |             rclcpp::executor::Executor::execute_subscription() {
            [ 31665] |               rclcpp::Subscription::create_message(0x7ffe1a4bf0f0) {
            [ 31665] |                 rclcpp::message_memory_strategy::MessageMemoryStrategy::borrow_message(0x7ffe1a4bf0a0) {
            [ 31665] |                   std::allocate_shared(0x7ffe1a4bf0a0) {
            [ 31665] |                     std::shared_ptr::shared_ptr(0x7ffe1a4bf0a0, 0x559cfd857480) {
  56.417 us [ 31665] |                     } /* std::shared_ptr::shared_ptr */
  57.045 us [ 31665] |                   } = 0x7ffe1a4bf0a0; /* std::allocate_shared */
  59.126 us [ 31665] |                 } = 0x7ffe1a4bf0a0; /* rclcpp::message_memory_strategy::MessageMemoryStrategy::borrow_message */
  66.038 us [ 31665] |               } = 0x7ffe1a4bf0f0; /* rclcpp::Subscription::create_message */
            [ 31665] |               rcl_take() {
            [ 31665] |                 rmw_take_with_info() {
            [ 31665] |                   rmw_take_with_info() {
            [ 31665] |                     rmw_take_int() {
            [ 31665] |                       dds_take(0x7c9df1f8, 0x7ffe1a4bdb20, 0x7ffe1a4bdb40, 1, 1) {
            [ 31665] |                         dds_read_impl(1, 0x7c9df1f8, 0x7ffe1a4bdb20, 1, 1, 0x7ffe1a4bdb40, 128, 0, 1, 0) {
            [ 31665] |                           dds_rhc_default_take_wrap(0x559cfd852f10, 1, 0x7ffe1a4bdb20, 0x7ffe1a4bdb40, 1, 128, 0, 0) {
            [ 31665] |                             serdata_rmw_to_sample() {
            [ 31665] |                               rmw_cyclonedds_cpp::TypeSupport::deserializeROSmessage() {
            [ 31665] |                                 rmw_cyclonedds_cpp::TypeSupport::deserializeROSmessage() {
            [ 31665] |                                   rmw_cyclonedds_cpp::deserialize_field() {
            [ 31665] |                                     cycdeser::operator>>() {
            [ 31665] |                                       cycdeser::deserialize() {
  12.182 us [ 31665] |                                         std::__cxx11::basic_string::_M_construct(0x7ffe1a4bd6c0, "hello world aaaaaaaaaaaaaaaaaaa
  10.216 us [ 31665] |                                         std::__cxx11::basic_string::operator=(0x559cfd857600, 0x7ffe1a4bd6c0) = 0x559cfd857600;
  28.158 us [ 31665] |                                       } /* cycdeser::deserialize */
  28.399 us [ 31665] |                                     } /* cycdeser::operator>> */
  28.686 us [ 31665] |                                   } /* rmw_cyclonedds_cpp::deserialize_field */
  29.155 us [ 31665] |                                 } /* rmw_cyclonedds_cpp::TypeSupport::deserializeROSmessage */
  30.023 us [ 31665] |                               } /* rmw_cyclonedds_cpp::TypeSupport::deserializeROSmessage */
  33.767 us [ 31665] |                             } /* serdata_rmw_to_sample */
  42.472 us [ 31665] |                           } = 1; /* dds_rhc_default_take_wrap */
  50.831 us [ 31665] |                         } = 1; /* dds_read_impl */
  51.702 us [ 31665] |                       } = 1; /* dds_take */
  53.019 us [ 31665] |                     } /* rmw_take_int */
  53.277 us [ 31665] |                   } /* rmw_take_with_info */
  53.774 us [ 31665] |                 } /* rmw_take_with_info */
  59.299 us [ 31665] |               } /* rcl_take */
```
:::
<br>

目標是要找到 guid 和 sequence number，guid 分辨是哪個 publisher 送出這個訊息，sequence number 用來分辨這是某個特定的 publisher 送出的第幾筆訊息，這兩個資料類型可以當作 ROS Message 的 unique key。

### Sender Side

到傳送資料為止的 Call Stack (DDS Sample 最後會由 `nn_xpack_send1` 送出)，可以看到 ROS2 呼叫 `publish` 和 DDS 真正送出資料都是在同一的執行緒。

```c
[#0] 0x7ffff3973b70 → nn_xpack_send1(loc=0x555555798fc0, varg=0x5555557de900)
[#1] 0x7ffff3992125 → ddsrt_avl_walk(td=0x7ffff3bc09e0 <addrset_treedef>, tree=0x5555557ce610, f=0x7ffff393ba50 <addrset_forall_helper>, a=0x7ffffffeb180)
[#2] 0x7ffff39929aa → ddsrt_avl_cwalk(td=0x7ffff3bc09e0 <addrset_treedef>, tree=0x5555557ce610, f=0x7ffff393ba50 <addrset_forall_helper>, a=0x7ffffffeb180)
[#3] 0x7ffff393cf6c → addrset_forall_count(as=0x5555557ce5e0, f=0x7ffff3973d30 <nn_xpack_send1v>, arg=0x5555557de900)
[#4] 0x7ffff397449f → nn_xpack_send_real(xp=0x5555557de900)
[#5] 0x7ffff397552d → nn_xpack_send(xp=0x5555557de900, immediately=0x1)
[#6] 0x7ffff397098d → transmit_sample_lgmsg_unlocked(prd=0x0, isnew=0x1, nfrags=0x2, serdata=0x5555557d9e80, plist=0x0, seq=0x2, whcst=0x7ffffffeb2a0, wr=0x5555557dffe0, xp=0x5555557de900)
[#7] 0x7ffff397098d → transmit_sample_unlocks_wr(prd=0x0, isnew=0x1, serdata=0x5555557d9e80, plist=0x0, seq=0x2, whcst=0x7ffffffeb2a0, wr=0x5555557dffe0, xp=0x5555557de900)
[#8] 0x7ffff397098d → write_sample_eot(ts1=0x555555799c80, xp=0x5555557de900, wr=0x5555557dffe0, serdata=0x5555557d9e80, tk=0x5555557e2d20, gc_allowed=0x1, end_of_txn=0x0, plist=<optimized out>)
[#9] 0x7ffff3970b1a → write_sample_gc(ts1=0x555555799c80, xp=<optimized out>, wr=0x5555557dffe0, serdata=0x5555557d9e80, tk=0x5555557e2d20)
[#1] 0x7ffff398c8d6 → dds_write_impl(wr=0x5555557de670, data=0x7ffffffec010, tstamp=<optimized out>, action=DDS_WR_ACTION_WRITE)
[#2] 0x7ffff398c9fd → dds_write(writer=<optimized out>, data=0x7ffffffec010)
[#3] 0x7ffff3be25f2 → rmw_publish(publisher=<optimized out>, ros_message=<optimized out>, allocation=<optimized out>)
[#4] 0x7ffff78720c0 → rcl_publish(publisher=0x5555557c9cf8, ros_message=0x7ffffffec010, allocation=0x0)
[#5] 0x555555560b1e → rclcpp::Publisher<std_msgs::msg::String_<std::allocator<void> >, std::allocator<void> >::do_inter_process_publish(this=0x5555557c9ce0, msg=0x7ffffffec010)
[#6] 0x55555555f2a4 → rclcpp::Publisher<std_msgs::msg::String_<std::allocator<void> >, std::allocator<void> >::publish(this=0x5555557c9ce0, msg=@0x7ffffffec010)
[#7] 0x55555555cae2 → main(argc=0x1, argv=0x7ffffffec188)
```

可追蹤的函式：

whc: writer histroy cache 存放所有要傳送的 DDS Sample

- `whc_default_insert` 第三個參數 `seqno_t seq` 為目前 DDS Sample 的序列號
```C
static int whc_default_insert (struct whc *whc, seqno_t max_drop_seq, seqno_t seq, struct nn_plist *plist, struct ddsi_serdata *serdata, struct ddsi_tkmap_instance *tk);
```

所有跟 writer history cache 相關的 callback 都被定義在 whc_ops 結構體中。當 DDS 要傳送一筆資料，必須插入一個 DDS Sample 到 whc 中，而 `whc_default_insert` 會被當作一個 callback 以 `whc_insert` 呼叫，如 q_transmit.c 列出的程式碼片段所示。

```C
// dds_whc.c
static const struct whc_ops whc_ops = {
  .insert = whc_default_insert,
  ...
};

// q_whc.h
inline int whc_insert (struct whc *whc, seqno_t max_drop_seq, seqno_t seq, struct nn_plist *plist, struct ddsi_serdata *serdata, struct ddsi_tkmap_instance *tk) {
  return whc->ops->insert (whc, max_drop_seq, seq, plist, serdata, tk);
}
```
q_transmit.c
```c
static int insert_sample_in_whc(...)
{
    ...
  if (!do_insert)
    res = 0;
  else if ((insres = whc_insert (wr->whc, writer_max_drop_seq (wr), seq, plist, serdata, tk)) < 0)
    res = insres;
  else
    res = 1;
    ...
}
```

以上為觀看原始碼推測程式執行的行為，但是實際上由於編譯器最佳化，`insert_sample_in_whc` 整個函式都被優化掉了，證據就是用 nm 看 libddsc.so (cyclone DDS 動態函式庫，已下載 debuginfo) 並沒有看到  insert_sample_in_whc 的 symbol，反組譯他的 caller `write_sample_eot` 也沒有呼叫這這個函式。

以下為 write_sample_eot 的程式碼片段 (eot 代表 end of transmission)
```c
static int write_sample_eot (struct thread_state1 * const ts1, struct nn_xpack *xp, struct writer *wr, struct nn_plist *plist, struct ddsi_serdata *serdata, struct ddsi_tkmap_instance *tk, int end_of_txn, int gc_allowed)
{
  struct q_globals const * const gv = wr->e.gv;
  int r;
  seqno_t seq;
  nn_mtime_t tnow;

...
  /* Always use the current monotonic time */
  tnow = now_mt ();
  serdata->twrite = tnow;

  seq = ++wr->seq;

...

  if ((r = insert_sample_in_whc (wr, seq, plist, serdata, tk)) < 0) {
      ...
  }
  else if (addrset_empty (wr->as) && (wr->as_group == NULL || addrset_empty (wr->as_group))) {
      ...
  }
  else
  {
    if (xp)
    {
      nn_plist_t plist_stk, *plist_copy;
      struct whc_state whcst, *whcstptr;
      if (plist == NULL)
        plist_copy = NULL;
      else
      {
        plist_copy = &plist_stk;
        nn_plist_copy (plist_copy, plist);
      }
      if (wr->heartbeat_xevent == NULL)
        whcstptr = NULL;
      else
      {
        whc_get_state(wr->whc, &whcst);
        whcstptr = &whcst;
      }
      transmit_sample_unlocks_wr (xp, wr, whcstptr, seq, plist_copy, serdata, NULL, 1);
      if (plist_copy)
        nn_plist_fini (plist_copy);
    }
    else
    {
      if (wr->heartbeat_xevent)
        writer_hbcontrol_note_asyncwrite (wr, tnow);
      enqueue_sample_wrlock_held (wr, seq, plist, serdata, NULL, 1);
      ddsrt_mutex_unlock (&wr->e.lock);
    }
...

drop:
  /* FIXME: shouldn't I move the ddsi_serdata_unref call to the callers? */
  ddsi_serdata_unref (serdata);
  return r;
}
```

#### Send Call Trace 筆記
- `transmit_sample_unlocks_wr` 會將這筆訊息傳遞出去
- `write_sample_eot` 會被編譯器最佳化變成其他  symbol (我在 apt 下載的函式庫為 `write_sample_eot.constprop.4`)
- `write_sample_gc` 的參數 `struct writer *wr` 和 
`dds_write_impl` 的參數 `dds_writer *wr` 兩者資料成員的 whc 指向相同物件 (表達式分別為 `wr->whc`, `wr->m_whc`)

:::success
因此最後決定追蹤 `write_sample_gc` 的 enter/exit，並紀錄 `writer` 的結構體中 seq 的數值，當呼叫 `write_sample_gc` 就可以大約知道 DDS sample 存入 writer history cache 的時間。
:::

### Receiver Side


當一個 DDS Sample 傳送到 DataReader 的時候的 Stack trace，`dds_rhc_default_store` 代表這個 DDS Sample 正要插入 rhc (reader history cache)
```cpp
[#0] 0x7ffff343cfd0 → dds_rhc_default_store(rhc=0x5555557d3c60, wrinfo=0x7fffe36fb470, sample=0x7fffd0001e70, tk=0x7fffd00019c0)
[#1] 0x7ffff34282be → ddsi_rhc_store(tk=<optimized out>, sample=0x7fffd0001e70, wrinfo=0x7fffe36fb470, rhc=<optimized out>)
[#2] 0x7ffff34282be → deliver_user_data(sampleinfo=0x7fffe3701698, fragchain=0x7fffe3701618, rdguid=0x0, pwr_locked=0x1)
[#3] 0x7ffff34287b9 → deliver_user_data_synchronously(rdguid=0x0, sc=<optimized out>)
[#4] 0x7ffff3428f3c → handle_regular(rst=0x7fffe37015c8, tnow=<optimized out>, rmsg=0x7fffe3700d38, msg=0x7fffe3700d98, sampleinfo=0x7fffe36fbfd0, fragnum=0xffffffff, rdata=0x7fffe3701618, deferred_wakeup=0x7fffe36fbfb8)
[#5] 0x7ffff342afe4 → handle_Data(deferred_wakeup=0x7fffe36fbfb8, datap=0x7fffe3700db0 "", sampleinfo=0x7fffe36fbfd0, size=<optimized out>, msg=0x7fffe3700d98, rmsg=0x7fffe3700d38, tnow=<optimized out>, rst=0x7fffe37015c8)
[#6] 0x7ffff342afe4 → handle_submsg_sequence(ts1=0x55555579fd80, gv=0x5555557a3208, conn=0x55555579bf30, srcloc=0x7fffe36fc520, tnowWC=<optimized out>, tnowE=<optimized out>, src_prefix=0x7fffe3700d70, dst_prefix=0x0, msg=0x7fffe3700d68 "RTPS\002\003\001\017\362\226\017\001", len=0x860, submsg=0x7fffe3700d98 "\025\005,\b", rmsg=0x7fffe3700d38)
[#7] 0x7ffff342c936 → do_packet(ts1=0x55555579fd80, gv=0x5555557a3208, conn=0x55555579bf30, guidprefix=0x0, rbpool=0x5555557af430)
[#8] 0x7ffff342cd23 → recv_thread(vrecv_thread_arg=<optimized out>)
[#9] 0x7ffff342e63b → create_thread_wrapper(ptr=0x5555557af480)
```

- `handle_regular` 第一，第四和第五個參數分別為
    - `struct receiver_state *rst`
    - `Data_DataFrag_common_t *msg`
    - `struct nn_rsample_info *sampleinfo`
- 注意 `sampleinfo->rst` 和第一個參數 rst 指向相同物件
- `sampleinfo->pwr` 有好用的 guid 資訊，但是也可以讀取 `rst->src_guid_prefix` + `msg->writerId` 重建成一個完整的 guid

結構如下 (並非定義在同一個檔案，以下只是列舉所有結構體)
```cpp
struct receiver_state {
  ddsi_guid_prefix_t src_guid_prefix;       /* 12 */
  ddsi_guid_prefix_t dst_guid_prefix;       /* 12 */
  struct addrset *reply_locators;         /* 4/8 */
  int forme;                              /* 4 */
  nn_vendorid_t vendor;                   /* 2 */
  nn_protocol_version_t protocol_version; /* 2 => 44/48 */
  ddsi_tran_conn_t conn;                  /* Connection for request */
  nn_locator_t srcloc;
  struct q_globals *gv;
};

typedef struct Data_DataFrag_common {
  SubmessageHeader_t smhdr;
  uint16_t extraFlags;
  uint16_t octetsToInlineQos;
  ddsi_entityid_t readerId;
  ddsi_entityid_t writerId;
  nn_sequence_number_t writerSN;
} Data_DataFrag_common_t;

struct nn_rsample_info {
  seqno_t seq;
  struct receiver_state *rst;
  struct proxy_writer *pwr;
  uint32_t size;
  uint32_t fragsize;
  nn_wctime_t timestamp;
  nn_wctime_t reception_timestamp; /* OpenSplice extension -- but we get it essentially for free, so why not? */
  unsigned statusinfo: 2;       /* just the two defined bits from the status info */
  unsigned pt_wr_info_zoff: 16; /* PrismTech writer info offset */
  unsigned bswap: 1;            /* so we can extract well formatted writer info quicker */
  unsigned complex_qos: 1;      /* includes QoS other than keyhash, 2-bit statusinfo, PT writer info */
};
```

所有對追蹤有幫助的函式：
```cpp
static ssize_t ddsi_udp_conn_read (ddsi_tran_conn_t conn, unsigned char * buf, size_t len, bool allow_spurious, nn_locator_t *srcloc)
static void handle_regular (struct receiver_state *rst, nn_etime_t tnow, struct nn_rmsg *rmsg, const Data_DataFrag_common_t *msg, const struct nn_rsample_info *sampleinfo, uint32_t fragnum, struct nn_rdata *rdata, struct nn_dqueue **deferred_wakeup)

static int handle_submsg_sequence
(
  struct thread_state1 * const ts1,
  struct q_globals *gv,
  ddsi_tran_conn_t conn,
  const nn_locator_t *srcloc,
  nn_wctime_t tnowWC,
  nn_etime_t tnowE,
  const ddsi_guid_prefix_t * const src_prefix,
  const ddsi_guid_prefix_t * const dst_prefix,
  unsigned char * const msg /* NOT const - we may byteswap it */,
  const size_t len,
  unsigned char * submsg /* aliases somewhere in msg */,
  struct nn_rmsg * const rmsg
)
```

由 gdb 印出此結構體的資料
```
{
  smhdr = {
    submessageId = 0x15, 
    flags = 0x5, 
    octetsToNextHeader = 0x82c
  }, 
  extraFlags = 0x0, 
  octetsToInlineQos = 0x10, 
  readerId = {
    u = 0x1304
  }, 
  writerId = {
    u = 0x1003
  }, 
  writerSN = {
    high = 0x0, 
    low = 0x13
  }
}
```


#### Receive Call Trace 筆記

1. Cyclone 有一個專屬的執行緒用來接收資料 ，函式為 `recv_thread`
2. 呼叫 `ddsi_udp_conn_read` -> `ddsrt_recvmsg` 讀取資料 (應為 blocking call)
    - 注意 `ddsi_udp_conn_read` 的參數 `ddsi_tran_conn_t conn` 會以資料成員的形式傳入之後呼叫的函式 `handle_regular`
    - `handle_regular` 的參數 `struct receiver_state *rst` 中表達式 `rst->conn` 和上面的 conn **指向相同的物件**
4. 向 udp 讀取完資料後呼叫`handle_submsg_sequence` 將資料分成好幾個 DDS submessage
    - `handle_submsg_sequence` 的參數 `srcloc` 有傳送這筆資料的 ip:port (0x9b59 = 39769)
<br>
    ```gdb
    gef➤  p *srcloc
    $10 = {
      kind = 0x1, 
      port = 0x9b59, 
      address = '\000' <repeats 12 times>, "\177\000\000\001"
    }
    ```
    ```bash
    $ netstat -ulpn
    ...
    udp        0      0 0.0.0.0:39769           0.0.0.0:*             11343/talker   
    ```
    - `rst->conn` 的資料成員 `struct ddsi_tran_base` 裡面有接收方的埠號 (0xa495 = 42133)
<br>
    ```c
    gef➤  p *sampleinfo->rst.conn
    $14 = {
      m_base = {
        m_port = 0xa495, 
        m_trantype = 0x1, 
        m_multicast = 0x0,
        ....
    ```
    ```bash
    $ netstat -ulpn
    ...
    udp   422400      0 0.0.0.0:42133           0.0.0.0:*           14606/listener   
    ```
    ![](https://i.imgur.com/zJlzjty.png)
<br>
    - `handle_regular` 的參數中可讀取 `struct nn_rsample_info *sampleinfo` 得到 guid + seqnum
    4. 不同 QoS 可能以不同行為呼叫函式，如果把 talker-listener 一對一傳輸從 reliable 改成 best_effort，會觀察到 `deliver_user_data` 會從呼叫一次變成兩次，而這兩次都是在 `handle_regular` 不同位置被呼叫
    
        以下用 bpftrace 印出 user stack，第一個數字代表 bios time (從開機到現在的時間)，單位是 ns，可以看到 `handle_regular` 分別在不同的位移呼叫 `deliver_user_data`。 +940, +1598 為回傳時執行的指令位址 (call 之後的第一個指令)
    ```c
    22803568717773, 
        deliver_user_data+0
        handle_regular+940
        handle_submsg_sequence+5810
        do_packet+694
        recv_thread+195
        create_thread_wrapper+53
        os_startRoutineWrapper+25
        start_thread+219
    22803568736585, 
        deliver_user_data+0
        handle_regular+1598
        handle_submsg_sequence+5810
        do_packet+694
        recv_thread+195
        create_thread_wrapper+53
        os_startRoutineWrapper+25
        start_thread+219
    ```
    
### `dds_take` when multiple samples in rhc

當指定 History 為 Keep Last 的時候，可以設定另一個 QoS 叫做 Depth，這會在 publisher 和 subscriber 分別控制 send queue 和 receive queue 的深度。



### Functions, Structures for ROS2 layer in Receive Side

建立一個 subscriber
- `rmw_create_subscription` 回傳一個 `rmw_subscription_t` 物件

`rmw_subscription_t` 的定義：
```c
typedef struct RMW_PUBLIC_TYPE rmw_subscription_t
{
  const char * implementation_identifier;
  void * data;
  const char * topic_name;
} rmw_subscription_t;
```
`rmw_subscription_t` 的資料成員 data 指向一個 DDS 實作的物件，以 cyclone DDS 來說會指向 `struct CddsSubscription`，其中就有一個 subscriber 的 handle。
```c
struct CddsSubscription
{
  dds_entity_t subh;
  dds_entity_t rdcondh;
  struct ddsi_sertopic * sertopic;
};
```

[^1]: http://people.csail.mit.edu/shanir/publications/disc2008_submission_98.pdf