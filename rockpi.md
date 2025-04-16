### wifi
start hotports
```bash  
nmcli  device wifi connect mySSID password '12345678'   #  nmcli connectio up mySSID
nmcli device wifi hotspot con-name ap001 ifname wlp3s0 ssid rock-pi-4c password 12345678 #mcli connection up ap001
```
### connect
```bash
ssh rock@10.24.2.1  #ustb-wifi rock@rock-4c-plus
ssh rock@192.168.94.25  #vivoS9
ssh rock@192.168.3.16  #ros_1st
```
ssh no password
```bash
ssh-keygen
ssh-copy-id -i ~/.ssh/id_rsa.pub rock@10.24.2.1
```

### change source
```
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ jammy main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ jammy-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ jammy-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ jammy-security main restricted universe multiverse
```

### git
set config
```bash
git config --global http.proxy socks5://127.0.0.1:7890
git config --global https.proxy socks5://127.0.0.1:7890
```

### set gnome 

```bash
sudo systemctl set-default multi-user.target # open default cli 
sudo systemctl set-default graphical.target  # open default gnome
```
back to gnome :`Ctrl+Alt+F1`

back to cli:` Ctrl+Alt+F3`

### ROS 
install Dependency package
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```

### set my driver 
```dtc -I dts -O dtb -o spi-lcd35.dtbo spi-lcd35.dts ```
