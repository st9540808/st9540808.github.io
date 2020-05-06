###### tags: `Profiling`
# Misc
## Measure Execution Time of eBPF Programs

### Average Running time
在 kernel 版本 5.1 之後已經支援紀錄 eBPF 程式的執行時間，適合用在掛載到 kprobe, uprobe 的 eBPF 程式

使用方法
- 設定 kernel 參數 `sysctl kernel.bpf_stats_enabled=1`
- 下載 Linux 原始碼，切換到 tools/bpf/bpftool 目錄下編譯 bpftool 工具

範例

對 `do_sys_getuid()` 插入一個 kprobe 和 kretprobe，並分別掛載 eBPF 程式
```bash
# ./bpftool prog
...
25: kprobe  name do_sys_getuid  tag ec2398ff1c0ade66  gpl run_time_ns 49182 run_cnt 123
        loaded_at 2020-03-30T15:00:46+0800  uid 0
        xlated 152B  jited 104B  memlock 4096B  map_ids 29,28
26: kprobe  name do_ret_sys_getu  tag 32cd9d9976bc11be  gpl run_time_ns 21013 run_cnt 123
        loaded_at 2020-03-30T15:00:46+0800  uid 0
        xlated 936B  jited 534B  memlock 4096B  map_ids 29,28,27
```

重要資訊
- run_time_ns: eBPF 程式的總執行時間，單位為 ns
- run_cnt: eBPF 程式總共被執行了幾次，對 kprobe 而言就是對應的 kernel function 被呼叫了幾次

### ftrace
當 eBPF 程式是掛載到 kernel 內部的某個 hook，可以用 ftrace 觀察其執行時間。

範例 `cls_bpf`

## Time Offset

- ntpd 強制對時
```bash
$ sudo service ntp stop
$ sudo ntpd -gq
$ sudo service ntp start
```

- Clock drift on raspberry pi (沒有 RTC)

![](https://i.imgur.com/K72KfTp.png)


## Stack Bufferoverflow

![](https://i.imgur.com/tWWcQBU.jpg)

### int 0x80
使用此指令可以在 Linux x86 或 Linux x86_64 機器上呼叫 system call

| Syscall # | Param 1 | Param 2 | Param 3 | Param 4 | Param 5 | Param 6 |
| --- | --------- | ------- | ------- | ------- | ------- | ------- | ------- |
| eax |ebx |ecx |edx | esi | edi | ebp

| Return value |
| ------------ |
| eax |

### Charts

:::spoiler
```
Highcharts.chart('container', {

    chart: {
        type: 'boxplot'
    },

    title: {
        text: 'ROS2 Latency'
    },

    legend: {
        enabled: false
    },

    xAxis: {
        categories: ['16 probes', '2 probes'],
        title: {
            text: 'Probes Number'
        }
    },

    yAxis: {
        title: {
            text: 'Latency (ms)'
        },
    },

    series: [{
        name: 'Observations',
        data: [
            [0.250,   1.187,   1.230,   1.280,   1.447],
            [0.1630,  0.8427,  0.8725,  0.8922,  1.0000],
        ],
        tooltip: {
            headerFormat: '<em>Experiment {point.key}</em><br/>'
        }
    },]

});
```
:::
---

![](https://i.imgur.com/s5ZuU6W.png)

---

![](https://i.imgur.com/UrJAKoY.png)

---

![](https://i.imgur.com/AuaP6OF.png)

---

![](https://i.imgur.com/J6Tc7zh.png)



## References
- Memory part 7: Memory performance tools: https://lwn.net/Articles/257209/