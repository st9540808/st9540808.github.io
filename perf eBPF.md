



###### tags: `Profiling`

# Performance Evaluation: perf, eBPF

profiling 有很多種方式，傳統上會使用編譯器在程式中插入 hook，藉此追蹤程式的函式、call graph 等，工具如 gprof, gcc 旗標 finstrument-functions 都是用此方式，他們的缺點是對程式的 overhead 會很高。另一種較為低成本的方式為 sample-based profiling。

perf 和 eBPF 為 Linux 系統中用來做效能分析的重要工具，以下將介紹者兩個工具的架構及使用方法。進行效能分析時常會用到兩種方式 sample-based profiling 及 tracing：

- sample-based profiling: 以某個頻率對系統或程式取樣，如果某個函式被取樣比重最高，那很有機會是程式的熱點。
- tracing: 紀錄每次事件的發生

一般來說 sample-based profiling 的成本會比 tracing 低，根據事件發生的頻率，tracing 可能會造成很大的 overhead。

:::info
以下提供一個簡單的 overhead 計算公式，可用在 profiling 和 tracing：

> Overhead = (frequency × action performed) / CPUs

action performed
- profiling: 每一次取樣時產生的成本
- tracing: 每一次紀錄事件產生的成本
:::

## perf: The Offical Linux Profiler
perf support many profiling/tracing features

- CPU	Performance	Monitoring Units (PMU) also called Performance Monitoring Counters (PMCs) (**Hardware events**)
    - e.g. branch-misses,cache-misses,cpu-cycles,instructions
- OS related events (**Software events**)
    - e.g. context-switches, page-faults
- ==Statically defined tracepoints== (**tracepoints**)
- ==User	and	kernel	dynamic	tracing==	(**uprobes** and **kprobes**)
- Kernel	line	and	local	variable	tracing	
- Efficient	in-kernel	counts	and	filters	
- Stack	tracing,	libunwind
- Code	annotation	

當紀錄有關硬體的效能事件時，必須與硬體的 PMU (Performance Monitoring Unit) 互動，以 intel 處理器來說，PMU 是由 MSRs (Model-Specific Registers) 所控制，並使用 WRMSR, RDMSR 指令操作，MSR 控制目前要紀錄的事件，以及事件發生的次數。

其他軟體事件會由作業系統實作，如 tracepoints, kprobes 。

所有與效能有關的事件在 Linux 中都稱為 perf_events，詳細的事件列表可使用 `perf list` 查看。要注意使用某些事件可能需要更高的權限。



### perf 的兩種主要模式

- `perf stat`
- `perf record`

![](https://i.imgur.com/oAAAIpe.png)



### perf stat: Counting events for a program

- Uses	an	efficient	in-kernel	counter,	and	prints	the	results

perf stat 紀錄硬體事件會使用到 PMU counting mode 統計事件發生的次數。

perf stat 只能執行一個程式並得知與效能相關的統計結果，如統計程式中發生了多少次 cache-misses，但無法得知是程式中的哪個函式觸發此事件，因此會使用 perf record 對程式做更深入的分析。

### perf record: Profiling program

在這種模式下紀錄到的事件不會是真實發生的次數，因為是透過<mark>**取樣**</mark>，如果是紀錄硬體事件會用到 PMU Sampling mode

perf record 有兩種方式可以指定多久取樣一次，對事件設定一個週期 (period) 或設定一個頻率 (frequency) 進行取樣。

- **Period** `-c` or `--count` : 指定==事件發生多少次之後才進行取樣==。當紀錄硬體事件時 Linux 會在 PMU 設定一個數值，當 PMU 裡的計數器溢位時會觸發中斷，並用 instruction pointer 等暫存器紀錄程式狀態， 如指定每發生一百萬次 cache misses 取樣一次可以使用
    `perf record -e 'cache-misses' -c 1000000 -- ./profiled_program`
- **Freqency** `-F` or `--freq`: 指定==一秒要取樣的次數==，這種模式下 Linux 會根據事件產生的快慢，動態調整取樣週期，盡量達到使用者指定的頻率。


下文引用 perf_event_open(2)
> <u>**sample_period**</u>, <u>**sample_freq**</u>
> 
> A  "sampling" event is one that ==generates an overflow notification every N events==, where N is given by **sample_period**.  A sampling event has sample_period > 0.  When an overflow occurs, requested data is recorded in the mmap buffer.  The sample_type field controls what data is recorded on each overflow.
>
> **sample_freq** can be used if you wish to use frequency rather than period.  In this case, you set the freq flag. ==The kernel will adjust the sampling period== to try and achieve the desired rate.  The rate of  adjustment  is  a timer tick


下圖示範 Linux 根據頻率調整取樣週期，以兩個硬體事件 Branch, instruction 為例，x 軸為 perf 取樣的次數，只列出前面 100 個取樣，y 軸為 Linux 設定的事件取樣週期，可以看到在不同的時間點，Linux 指定的取樣週期都不一樣。
    ![](https://i.imgur.com/Mf1jbeJ.png)

對使用者來說用 `-F` 會比較方便而且非常有用，因為不需要知道某個事件一秒鐘會發生幾次，如果取樣到的事件太少，再增加頻率就可以。

以下為各種常見軟硬體事件一秒產生的次數，根據使用者執行的 workload 會有所差距 [^1]

[^1]: BPF Performance Tools Ch18

![](https://i.imgur.com/k8zSpro.png)


要注意的是由於現代處理器的設計都有 pipeline，而且從計數器溢位到觸發中斷也有延遲，因此 perf 紀錄的指令位址不一定會是觸發事件的指令，這種現象通常稱為 skid。同時 speculative execution 會執行到錯誤預測的分支，這也會讓這些指令產生的事件加到 PMU 的計數器中。

:::info
perf 也支援更進階 profiling 的操作，如 PEBS (Precise Event-based Sampling) 可以解決以上的問題，PEBS 直接使用硬體的緩衝區，精準紀錄 PMU 事件發生當下的 instruction pointer。

可參考
*Advanced Hardware Profiling and Sampling (PEBS, IBS, etc.): Creating a New PAPI Sampling Interface*
:::

以下用示意圖說明 perf record 取樣的方法 (雖然以下是用取樣 stack，但可以應用在不同的 perf_event)

perf record 藉由一個使用者以 `-F` 參數設定一秒中要取樣的次數，當程式從 CPU 置換出去時則不取樣。

![](https://i.imgur.com/bNhI409.png)

### Stack Sampling (perf record)

perf record 也可以對程式的 stack 做取樣，

基本用法，`-g` 指定 stack trace，`--` 後面接著要 profile 的程式：

![](https://i.imgur.com/nFMEcg1.png)
<br>

### Hands-on Examples

http://www.brendangregg.com/perf.html

Branden Gregg 在他的 blog 提供非常多現成的命令，可參考他的文章。

以下提供一些範例

- 每發生 100 次 Last Level Cache miss 時取樣一次 stack (整個系統的程式)
    
    ```
    # perf record -e LLC-load-misses -c 100 -ag -- sleep 5 
    ```
    
    
### perf 的實作方式
![](https://i.imgur.com/QSEH2Og.png)


### Flame Graph

當要了解一個大型程式架構的效能時，可以用 Flame Graph 視覺化效能瓶頸。
每一個 Linux 版本可以用的方式都不太一樣，以 Flame Graph 在 github 上使用手冊的說明是使用 Linux 2.6 的流程

![](https://i.imgur.com/yT45FRH.png)

:::success
編譯器在為某些硬體最佳化時，可能會將 frame pointer register 當作 general purpose register 使用，這會讓 perf 在走訪堆疊時失敗 (Broken Stacks)，因此編譯時必須加上 `-fno-omit-frame-pointer`

也可使用 `perf record -g dwarf`
:::


## eBPF

BPF 最早在 1992 年由 Steven McCanne 和 Van Jacobson 提出，包含 BPF 指令集和一個 **in-kernel VM**，用來在不同的硬體架構上過濾網路封包。後來社群發現 BPF 的能力可以用來做效能分析，因此對 Linux kernel 和 BPF 做了很大的改善，目前都通稱現在的版本為 eBPF (或直接稱作 BPF)，而以前提出的版本則稱作 cBPF。

以下為 cBPF 和 eBPF 的差異，eBPF 不但有更多的暫存器也有更大的 stack，但最重要的是 eBPF 的 Map，eBPF 程式能夠寫入資料到 Map，同時 userspace 也能存取 Map 的資料，因此提供了一個在 kernel space 和 user space 交換資料的機制，同時也把 "**狀態**" 這的概念帶到 eBPF 中，這是 cBPF 所沒有的。

![](https://i.imgur.com/qPdCn0z.png)

### 使用 eBPF 的專案
as of 2020:
![](https://i.imgur.com/GzaoEHQ.png)


BPF 時間表： https://github.com/iovisor/bcc/blob/master/docs/kernel-versions.md

- 3.15: Optimization of BPF interpreter instruction set
- 3.18: Linux eBPF was released (bpf syscall), Tables (a.k.a. Maps)
- 3.19: Socket support
- 4.1: BPF attached to kprobes (能夠將 eBPF 註冊到 kprobe 上)
- 4.7: BPF attached to tracepoints	
- 4.9: BPF attached to perf events
- 4.10: cgroups support
- 4.18: bpfilter

以上的時間表可以看到 eBPF 可以註冊 (或掛載，原文為 attach) 到 Linux 不同的地方，又可稱為 hook point，可以用一張圖總結所有 eBPF 能夠註冊到的地方，如 kprobe, uprobe, perf_events (包含下圖的 PMCs 和 Software Envets) 等，每一個可註冊的點都對應一種 eBPF 程式的型態，意思是 eBPF 的 function prototype 和可以使用的 bpf helper function 可能會有所差異，可參考 https://github.com/iovisor/bcc/blob/master/docs/kernel-versions.md#program-types

![](https://i.imgur.com/1zlckrW.png)
<br>

要執行 eBPF 需要 kernel 的支援，以下解釋幾個重要的元件：

- interpreter: in-kernel VM 用來安全的執行 eBPF bytecode，可開啟 JIT 加速 (ubuntu 18.04 預設為開啟)
- hook point: eBPF 的掛載點，當事件觸發時會執行掛載到 hook point 的 eBPF 程式
- Map: array, hash 等資料結構，eBPF 程式和 userspace 都可以存取資料
- helper: eBPF 可以呼叫的函式，根據 eBPF 程式型態會有所差異


以下解釋 eBPF 的註冊以及執行過程，注意到這些瑣碎的操作都可用 bcc 工具處理。

![](https://i.imgur.com/mbTo9Bt.png)

1. generate: 使用工具鏈如 clang, llvm 產生 eBPF bytecode
2. load: 掛載 eBPF 程式到對應的 hook point，這會經過一個 verifier，確保程式沒有迴圈、變數有正確初始化、沒有 unaligned access 的記憶體操作，最後由 BPF interpreter 執行 eBPF。

> 假設要掛載一個 eBPF 到 kprobe 上，首先 userspace 程式會呼叫 `perf_event_open(2)` 回傳一個 fd (file descriptor)，這個 fd 再傳入 `ioctl(2)` 使用 PERF_EVENT_IOC_SET_BPF 參數加上要掛載的 eBPF 程式的 fd (由 `bpf(2)` 產生)

3. 輸出資料到 userspace (兩種方式)
    - perf_output: 利用 ftrace 框架的 ring buffer (/sys/kernel/debug/tracing/trace_pipe) 進行輸出，或者使用 perf ring buffer
    - async read: 存取 map

### eBPF Program Model

![](https://i.imgur.com/kq2oGxG.png)


## eBPF Examples

目前主要有兩種編寫以及掛載 eBPF 程式的工具，bcc 和 bpftrace。bcc 使用 python 為前端，而 bpftrace 則是使用自己的語法，有點類似 awk，筆者兩個工具都有使用過，而他們都有各自的優缺點

個人認為熟悉 bpftrace 的語法之後，可以讓你更快部屬 eBPF，適合做簡單的測試。而 bcc 能作到比 bpftrace 更精細的操作，如讀取 user-space  的資料，操作 `sk_buff` 等，但代價就是寫更多程式碼。

### bcc

`freq.py`： 紀錄 sendmsg 系統呼叫每秒發生的次數並印出

```python
#!/usr/bin/env python
from bcc import BPF
from time import sleep

prog = """
BPF_TABLE("array", u32, u32, stats, 1);
int hello_world(void *ctx) {
  u32 key = 0, value = 0, *val;
  val = stats.lookup_or_init(&key, &value);
  lock_xadd(val, 1);
  return 0;
}
"""

b = BPF(text=prog)

# getting shared kernel map
stats_map = b.get_table("stats")
execve_fnname = b.get_syscall_fnname("sendmsg")
b.attach_kprobe(event=execve_fnname, fn_name="hello_world")

for x in range(0, 20):
    stats_map[ stats_map.Key(0) ] = stats_map.Leaf(0)
    sleep(1)
    print "Total ", execve_fnname, " per second =", stats_map[ stats_map.Key(0) ].value;
```

`lock_xadd()` 是用來原子式寫入變數
 
> Since the defined array map is global, the accounting needs to use an atomic operation, which is defined as lock_xadd(). LLVM maps __sync_fetch_and_add() as a built-in function to the BPF atomic add instruction, that is, BPF_STX | BPF_XADD | BPF_W for word sizes.
>
> ref: https://docs.cilium.io/en/v1.7/bpf/


### bpftrace

跟 `freq.py` 一樣計算並印出每秒呼叫 sendmsg 的次數
```
# bpftrace -e 'tracepoint:syscalls:sys_enter_sendmsg { @sendmsg = count(); }
  interval:s:1 { print(@sendmsg); clear(@sendmsg); }'             
```

## References

perf:
- Brendan Gregg FlameGraphs: http://www.brendangregg.com/FlameGraphs/cpuflamegraphs.html
- Kernel Recipes 2017: Using Linux perf at Netflix: https://www.slideshare.net/brendangregg/kernel-recipes-2017-using-linux-perf-at-netflix?from_action=
- Performance Monitoring Unit: http://rts.lab.asu.edu/web_438_2012/project_final/CSE_598_Performance_Monitoring_Unit.pdf
- perf silde by brendan gregg: https://www.slideshare.net/brendangregg/scale2015-linux-perfprofiling/63
- A Study of Linux Perf and Slab Allocation Sub-Systems: https://core.ac.uk/download/pdf/144148979.pdf
- https://s3.amazonaws.com/connect.linaro.org/yvr18/presentations/yvr18-416.pdf

eBPF:
- eBPF Basics:
    - https://www.slideshare.net/MichaelKehoe3/ebpf-basics-149201150
    - https://elinux.org/images/d/dc/Kernel-Analysis-Using-eBPF-Daniel-Thompson-Linaro.pdf
- eBPF/XDP:
    - http://evcomp.dcc.ufmg.br/wp-content/uploads/eBPF-XDP.pdf
    - https://www.netronome.com/blog/bpf-ebpf-xdp-and-bpfilter-what-are-these-things-and-what-do-they-mean-enterprise/
- Fast Packet Processing with eBPF and XDP: Concepts, Code, Challenges, and Applications: https://dl.acm.org/doi/fullHtml/10.1145/3371038#Bib0001
- KOSS slide

