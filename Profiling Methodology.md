###### tags: `Profiling`

# Performance Evalutaion Methodology

:::info
Linux system performance by Brendan Gregg

http://www.brendangregg.com/Slides/LISA2019_Linux_Systems_Performance.pdf

![](https://i.imgur.com/e15bBoG.png)

:::

## Overview

Common questions in performance evalutaion

1. What is the function consuming most cycles? and its context (who are calling it?)
2. What is the application consuming most cycles
3. What library is most used?
4. Do we have a lot of TLB page walks? On which functions, binaries?

### Intel SkylakeX core PMU

PEBS: Precise sampling to eliminate interrupt-based sampling IP-skid
- Sample recorded by microcode to virtal memory buffer
- 1 interrupt per buffer full: **significant overhead reduction** = increased sampling frequency
- Capture IP, machine state, data address for ld/st

## Top-Down Analysis

Look at the events below listed by perf

```bash
  topdown-fetch-bubbles OR cpu/topdown-fetch-bubbles/ [Kernel PMU event]
  topdown-recovery-bubbles OR cpu/topdown-recovery-bubbles/ [Kernel PMU event]
  topdown-slots-issued OR cpu/topdown-slots-issued/  [Kernel PMU event]
  topdown-slots-retired OR cpu/topdown-slots-retired/ [Kernel PMU event]
  topdown-total-slots OR cpu/topdown-total-slots/    [Kernel PMU event]
```

Perf adds these events to enable user to do top down microarchitecture performance analysis[^1]. Proposed by Ahmad Yasin at intel, the method allows us to investigate how a workload or an application is utilizing hardware resources. This can be done at a very low cost since it uses on-chip PMU directly.

[^1]: A Top-Down method for performance analysis and counters architecture

## Tracing Distrubuted Systems

- machine-centric: Difficult to show the relations and dependencies between multiple nodes and servers.
- workflow-centric: also known as end-to-end tracing. For example, a request-based distributed service, a trace records the work done in servers.
    - Notable works: Stardust, X-Trace
    - Google’s Dapper, Cloudera’s HTrace, Twitter’s Zipkin

End-to-end tracing capability:
- anomaly detection
- diagnosis of steady-state correctness and performance problems
- profiling (done by sampling trace)
- resource-usage attribution

### Approach to End-to-End Tracing

- Metadata propagation
- Schema-based
- Black-box inference

## References
- Hardware Performance Monitoring Landscape:　https://protools19.github.io/slides/Eranian_KeynoteSC19.pdf
- https://zhuanlan.zhihu.com/p/60940902
- https://software.intel.com/content/www/us/en/develop/documentation/vtune-cookbook/top/methodologies/top-down-microarchitecture-analysis-method.html