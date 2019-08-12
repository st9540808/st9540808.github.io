---
title: Hello World (My First Blog Post)
date: 2019-08-06 10:07:25
categories:
  - cat1
  - cat4
tags: test1
toc: true
---

Welcome to [Hexo](https://hexo.io/)! This is your very first post. Check [documentation](https://hexo.io/docs/) for more info. If you get any problems when using Hexo, you can find the answer in [troubleshooting](https://hexo.io/docs/troubleshooting.html) or you can ask me on [GitHub](https://github.com/hexojs/hexo/issues).

# Quick Start (H1)
## Another header (H2)
### Create a new post (H3)
#### h4
##### h5
###### h6

$a < 3$

==測試的字串 `git commit`==

hexo code block

{% codeblock mytitle lang:c http://localhost:4000/2019/08/01/hello-world/ src line_number:true highlight:true first_line:1155 mark:1156,1159-1162,1167 %}
static struct smp_hotplug_thread softirq_threads = {
  .store              = &ksoftirqd,
  .thread_should_run  = ksoftirqd_should_run,
  .thread_fn          = run_ksoftirqd,
  .thread_comm        = "ksoftirqd/%u",
};

static __init int spawn_ksoftirqd(void)
{
  register_cpu_notifier(&cpu_nfb);

  BUG_ON(smpboot_register_percpu_thread(&softirq_threads));
  /* this is a very looooooooooooooooooooooooooooooooooooooooooooooog comment */

  return 0;
}
early_initcall(spawn_ksoftirqd);
{% endcodeblock %}

c code

```c
static int igb_alloc_q_vector(struct igb_adapter *adapter,
                              int v_count, int v_idx,
                              int txr_count, int txr_idx,
                              int rxr_count, int rxr_idx)
{
  /* ... */

  /* allocate q_vector and rings */
  q_vector = kzalloc(size, GFP_KERNEL);
  if (!q_vector)
          return -ENOMEM;

  /* initialize NAPI */
  netif_napi_add(adapter->netdev, &q_vector->napi, igb_poll, 64);

  /* ... */
```

css code

```css
body {
  background-image: url("img_tree.png");
  background-repeat: no-repeat;
  background-position: right top;
  margin-right: 200px;
}
```

python code

```python
print("Prime numbers between",lower,"and",upper,"are:")

for num in range(lower,upper + 1):
   # prime numbers are greater than 1
   if num > 1:
       for i in range(2,num):
           if (num % i) == 0:
               break
       else:
           print(num)
```

bash code

```bash
$ hexo new "My New Post"
```

More info: [Writing](https://hexo.io/docs/writing.html)

### Run server

```bash
$ hexo server
```

More info: [Server](https://hexo.io/docs/server.html)

### Generate static files

```bash
$ hexo generate
```

More info: [Generating](https://hexo.io/docs/generating.html)

### Deploy to remote sites

``` bash
$ hexo deploy
```

C code

``` c
for (int i = 0; i < 100; i++) {
    do {
        sqrt(2);
    } while (0);
}
```

More info: [Deployment](https://hexo.io/docs/deployment.html)
