---
title: Install IgH EtherCAT Master on Raspberry Pi & Backup
date: 2018-01-27 19:39:30
tags: "Raspberry Pi"
permalink: install-igh-ethercat-master-on-raspberry-pi-backup
---

{% raw %}
<h1>Install</h1>
<p>Referring to the installation guide in the Igh maunal. It's pretty simple to install the Igh EtherCAT on raspi. Below records my steps to install Igh.</p>
<h4>1. update kernel and download kernel source</h4>
<div class="highlight"><pre><span></span>$ sudo rpi-update
$ reboot
$ git clone https://github.com/notro/rpi-source
$ python rpi-source
</pre></div>


<p style="line-height:1.0"></p>

<h4>2. If rpi-source failed because of complier version, you need to change to the one that compile kernel. e.g.</h4>
<!-- language: shell -->

<div class="highlight"><pre><span></span>$ sudo apt-get install -y gcc-4.8
$ sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.6 <span class="m">20</span>
$ sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 <span class="m">50</span>
</pre></div>


<p><p style="line-height:1.0"></p></p>
<h4>3. download ethercat</h4>
<!-- language: shell -->

<div class="highlight"><pre><span></span>$ wget https://etherlab.org/download/ethercat/ethercat-1.5.2.tar.bz2
$ tar jxf ethercat-1.5.2.tar.bz2
</pre></div>


<p><p style="line-height:1.0"></p></p>
<h4>4. intall automake and kernel source</h4>
<!-- language: shell -->

<div class="highlight"><pre><span></span>$ sudo apt-get install automake autoconf libtool
$ sudo apt-get install linux-source
</pre></div>


<p><p style="line-height:1.0"></p></p>
<h4>5. start install IgH EtherCAT</h4>
<!-- language: shell -->

<div class="highlight"><pre><span></span>$ ./bootstrap
$ ./configure
$ make all modules
$ sudo make modules_install install
$ sudo depmod
</pre></div>

<blockquote><strong>Note here:</strong> if <code>make install</code> failed due to FPU problem (softfp ABI?), use <code>touch</code>  on the failed target.</blockquote>

<p style="line-height:1.0"></p>

<h4>6. create soft link</h4>
<!-- language: shell -->

<div class="highlight"><pre><span></span>$ sudo ln -s /opt/etherlab/etc/init.d/ethercat /etc/init.d/ethercat
$ sudo mkdir /etc/sysconfig
$ sudo cp /opt/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat
</pre></div>


<p><p style="line-height:1.0"></p></p>
<h4>7. config ethercat, you need to fill in MASTER0_DEVICE with <strong>eth0</strong> MAC address, and fill in DEVICE_MODULES with "generic"</h4>
<!-- language: shell -->

<div class="highlight"><pre><span></span>$ ifconfig
$ sudo vim /etc/sysconfig/ethercat
</pre></div>


<p><p style="line-height:1.0"></p></p>
<h4>8. finish up</h4>
<!-- language: shell -->

<div class="highlight" style="margin-bottom: 20px"><pre><span></span>$ sudo <span class="nb">echo</span> <span class="nv">KERNEL</span><span class="o">==</span><span class="se">\&quot;</span>EtherCAT<span class="o">[</span><span class="m">0</span>-9<span class="o">]</span>*<span class="se">\&quot;</span>, <span class="nv">MODE</span><span class="o">=</span><span class="se">\&quot;</span><span class="m">0664</span><span class="se">\&quot;</span> &gt; /etc/udev/rules.d/99-EtherCAT.rules
$ ln -s /opt/etherlab/bin/ethercat /usr/local/bin/ethercat
</pre></div>

<hr>

<h1>Backup</h1>
<p>After we install the Igh EtherCAT, it might be faster that we use the backup image on the new raspi machine.</p>
<p>Find out which devices are currently available:</p>
<div class="highlight"><pre><span></span>$ df -h
</pre></div>


<p>use command <code>dmesg</code> to confirm that new device you inserted is SD card.
<strong>Always make sure that <em>if</em> is the device to read and <em>of</em> is the device to write.</strong></p>
<p>To backup SD card: (it might take a while)</p>
<div class="highlight"><pre><span></span>$ sudo dd <span class="k">if</span><span class="o">=</span>/dev/sdc <span class="nv">of</span><span class="o">=</span>/SDCardBackup.img
</pre></div>


<p>To restore the image:</p>
<div class="highlight"><pre><span></span>$ sudo dd <span class="nv">bs</span><span class="o">=</span>4M <span class="k">if</span><span class="o">=</span>/SDCardBackup.img <span class="nv">of</span><span class="o">=</span>/dev/sdc
</pre></div>


<p>Before ejecting the SD card, make sure that your Linux PC has completed writing:</p>
<div class="highlight"><pre><span></span>$ sudo sync
</pre></div>


<p>Don't forget to fill in MAC address of the new machine.</p>
{% endraw %}