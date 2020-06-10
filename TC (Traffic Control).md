###### tags: `Networking`

# TC (Traffic Control)

Overview 介紹 tc 的功能和目的，不詳細解釋 tc 內部的元件或實作，改編自  Criteo 的文章。[^1]
## Overview[^1][^2]

網路跟其他電腦元件如 CPU, RAM 一樣都是一項資源，CPU 由排程器達成分時多工，讓不同應用使用 CPU 資源，而網路在 Linux 中是由 tc 子系統管理，tc 架構中包含了一系列的演算法，可以讓使用者調整網路流量、延遲、抖動等。使用情境例如

- 保留 1Gpbs 頻寬給 HTTP (*traffic classification)*
- 給予 DNS 封包流量優先權 (*prioritization*)
- 限制 brust size (*rate shaping*)


tc 可以處理出境 (egress) 和入境 (ingress) 的封包，以下圖為 engress 的示意圖，可以看到 queueing discipline (qdisc) 夾在 IP stack 和 driver queue 中間，qdisc 是 tc 最重要的功能，當 IP stack 產生封包時 (可能是來自本機的應用程式或來自其他網路界面) 會經過 qdisc 這個物件，由 qdisc 做上述三種類型的網路流量管理，最後把封包放到 driver queue 再由 NIC 傳輸。

<!-- ![](https://i.imgur.com/tXizJTz.png) -->
![](https://i.imgur.com/urkAxGx.png)


### Element of Traffic Control

#### 1. Classification

網路流量管理的一的最基本的功能是要能夠分類以及過濾封包，包括入境及出境，藉由讀取通訊協定的表頭 (如 IP, TCP Header) 或封包內容 (如 HTTP)，決定要對這個封包採取什麼樣的動作。在 tc 裡已經有相當多現成的 filter 工具可以直接使用，也可以自己編寫一個 eBPF 程式做更細緻的分類。

以下為一個 Classification 的示意圖，封包經過一個 filter 決定要採取哪個 Policy
![](https://i.imgur.com/Pij6qG0.png)

:::info
近年來因為網路頻寬的升高，也增加了高速過濾封包的需求，現在某些 SmartNIC 可以做 Hardware offloading，意思是把原本過濾封包的 eBPF 程式，轉譯成網卡可執行的指令，直接在硬體上執行。得益於網卡的硬體加速，netronome 的網卡一秒可以過濾近六千萬個封包 (60Mpp/s)！
https://www.netronome.com/blog/bpf-ebpf-xdp-and-bpfilter-what-are-these-things-and-what-do-they-mean-enterprise/
:::

#### 2. Scheduling

## TC Objects[^3]

以下講解 tc 的主要元件，包括 qdics, class, filter, policy。

1. qdisc (Queueing Discipline)
2. class
3. filter, policy (有些書上會寫 classifier, action)


## Functions


### Add a Filter

filter 只能加到 qdisc，要加到 class 必須先有 qdisc
filters are: per interface + per qdisc + per protocal (+ per priority)

```
# tc filter add dev eth0 parent 100: protocal ipv6 matchall \
  action drop
```

命令解釋：
- matchall 為 filter 的名稱，會匹配所有封包
- matchall 之前的 protocal 代表要匹配的協議，是由 tc 架構提供的過濾功能

### Network Emulation (netem)

netem 可以模擬真實網路會遇到各種狀況


[^1]: Demystification of TC:
https://medium.com/criteo-labs/demystification-of-tc-de3dfe4067c2

[^2]: Queueing in the Linux Network Stack
https://www.linuxjournal.com/content/queueing-linux-network-stack

[^3]: Linux Network Traffic - Implementation Overview https://www.almesberger.net/cv/papers/tcio8.pdf