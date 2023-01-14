---
layout: post
title: Mac Install Brew
date: 2023-01-05 17:54:00 +0800
updated: 2023-01-05 17:54:00 +0800
tags: [mac, brew]
categories: [mac]
comments: true
---

# Mac 安装包管理工具 brew

## 终端

打开终端的方式：

方法1: `command + space` 打开聚焦，输入`终端`或`terminal`，回车即可打开；

方法2：`访达`-> `应用程序`-> `实用工具`-> `终端` ;

### 快捷键设置

mac 貌似并没有打开终端的快捷键，不过我们可以借助第三方软件实现，在这里我们使用的是 `Thor Launcher`，在 app store 搜索即可下载，打开之后添加终端，设置快捷键即可。

## brew 安装

国内镜像安装：

```shell
/bin/zsh -c "$(curl -fsSL https://gitee.com/cunkai/HomebrewCN/raw/master/Homebrew.sh)"
```

回车后根据提示操作即可，中途需要使用 git 下载一些资源，如果没有下载 git，会提示安装，git 安装完，重新执行上述命令即可。

完成后，输入以下命令验证：

```shell
merlotliu@lu-mbp ~ % brew -v
Homebrew 3.6.15
Homebrew/homebrew-core (git revision 2933fe57a1a; last commit 2022-12-12)
Homebrew/homebrew-cask (git revision f1e5143d96; last commit 2022-12-12)
```

值得注意的是，如果正常显示版本，却还有其他异常信息，根据提示操作即可。

此外，上述命令的默认安装路径为 `/opt/homebrew/`。

## brew 卸载

```shell
/bin/zsh -c "$(curl -fsSL https://gitee.com/cunkai/HomebrewCN/raw/master/HomebrewUninstall.sh)"
```

## 使用

### 查找软件路径

```shell
% brew --prefix <software>
```

或使用以下命令查看 brew 的默认配置：

```
% brew config | grep HOMEBREW
HOMEBREW_VERSION: 3.6.15
HOMEBREW_PREFIX: /opt/homebrew # 软件默认安装路径
HOMEBREW_BOTTLE_DOMAIN: https://mirrors.ustc.edu.cn/homebrew-bottles
HOMEBREW_CASK_OPTS: []
HOMEBREW_MAKE_JOBS: 8
```

## Reference 

1. [https://zhuanlan.zhihu.com/p/372576355](https://zhuanlan.zhihu.com/p/372576355)
1. http://t.zoukankan.com/EasonJim-p-7875920.html
