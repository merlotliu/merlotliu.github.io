# Linux开发环境搭建

## 相关下载链接

1.安装Linux系统（虚拟机安装、云服务器）

https://releases.ubuntu.com/bionic/

2.安装XSHELL、XFTP

https://www.netsarang.com/zh/free-for-home-school/

3.安装Visual Studio Code

https://code.visualstudio.com/

## 1 ssh环境配置

在安装完Ubuntu之后，即可进行ssh环境的配置。

输入以下命令安装`ssh-server`：

```shell
sudo apt install openssh-server
```

然后使用以下命令查看`sshd`服务是否启动

```shell
ps -e | grep sshd
```

如果启动了，就可以使用`xshell`或其他方式连接了。

如果没有启动，可以使用以下命令启动服务，再使用上面命令查看：

```shell
sudo service sshd start
```

## 2 xshell连接

先在服务器使用`ifconfig`查看ip：

```
ifconfig
```

打开xshell，`文件-新建`打开以下，对话框。

填写名称（随便填），协议选择`SSH`，主机填写上面获取的ip地址（局域网内使用私有ip，云服务器填写公网ip），下面填写端口号（一般是22，如果是映射过来的填写映射的端口号）。

![image-20220917222537131](../../.gitbook/assets/linux-1-environment-setup.assets/image-20220917222537131.png)

确定之后，填写用户名、密码即可登录。

此外，windows或其他系统的命令行，使用`ssh`也可以连接，如：

```
//ssh <ip:port>
ssh 192.168.0.34:22
```

## 3 Visual Studio Code连接

### 基础连接

打开vs code，在拓展插件中，搜索`remote`，安装`remote-development`。

安装完之后就会看到一个**电脑**样式的图标，就在**拓展插件**图标下面。

![image-20220917223330436](../../.gitbook/assets/linux-1-environment-setup.assets/image-20220917223330436.png)

点击这个图标，进入`remote explorer`，在上面选择`SSH Targets`。

![image-20220917223644257](../../.gitbook/assets/linux-1-environment-setup.assets/image-20220917223644257.png)

点击**齿轮状图标（配置）**，选择第一个，然后填写`Host`（别名），`Hostname`（ip地址），`User`（登陆的用户名）。当然，还可以添加条目，如端口号（默认22）。

![image-20220917224055689](../../.gitbook/assets/linux-1-environment-setup.assets/image-20220917224055689.png)

![image-20220917224032070](../../.gitbook/assets/linux-1-environment-setup.assets/image-20220917224032070.png)

配置完成后，点击电脑右侧图标，连接并打开新窗口。

![image-20220917224331637](../../.gitbook/assets/linux-1-environment-setup.assets/image-20220917224331637.png)

第一次，会要求选择系统，我们在这里选择`Linux`。

然后他会询问是否继续，点击`Continue`。

接着需要输入用户密码，输入并回车即可。

然后就可以打开文件夹，点击打开文件夹即可查看可以打开的位置，默认为家目录。

![image-20220917224621646](../../.gitbook/assets/linux-1-environment-setup.assets/image-20220917224621646.png)

### 密钥设置

为了方便使用，避免每次都需要在vs code 输入密码。

我们首先打开命令行cmd，输入以下内容默认回车，即可完成密钥生成。在用户目录下的`.ssh`目录下即可看到生成的公钥和私钥。

```
ssh-keygen -t rsa
```

然后使用xshell连接到服务器，同样输入以上命令，并切换到`.ssh`目录下。

接着创建并打开文件`authorized_keys`。

将第一步创建的机器中的公钥复制到这里保存并退出。

```
cd .ssh
vim authorized_keys
```

