### change source
```
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ jammy main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ jammy-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ jammy-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ jammy-security main restricted universe multiverse
```

### set my driver 
```dtc -I dts -O dtb -o spi-lcd35.dtbo spi-lcd35.dts ```

### set gnome 

```bash
sudo systemctl set-default multi-user.target # open default cli 
sudo systemctl set-default graphical.target  # open default gnome
```
back to gnome :`Ctrl+Alt+F1`

back to cli:` Ctrl+Alt+F3`

### connect
```bash
ssh rock@10.24.36.75  ustb-wifi
ssh rock@192.168.94.25  vivoS9
```

