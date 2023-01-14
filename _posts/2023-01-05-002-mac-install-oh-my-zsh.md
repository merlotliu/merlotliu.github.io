---
layout: post
title: Mac Install Oh-My-Zsh
date: 2023-01-05 17:54:00 +0800
updated: 2023-01-05 17:54:00 +0800
tags: [mac, oh-my-zsh]
categories: [mac]
comments: true
---

# Mac 安装 Oh My Zsh

官网：https://ohmyz.sh

## 安装

官网给出的安装命令：

```shell
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
```

但由于网络问题，一直报错，原因因该就是连接不上服务器，所以将后面 github 的地址修改为国内的镜像即可。具体如下：

```shell
sh -c "$(curl -fsSL https://gitee.com/mirrors/oh-my-zsh/raw/master/tools/install.sh)"
```

如果之前安装了没有删除干净或其他原因，可能会出错：

```shell
The $ZSH folder already exists (/Users/user/.oh-my-zsh).
You'll need to remove it if you want to reinstall.
```

根据提示删除对应目录即可：`rm -rf ~/.oh-my-zsh`

## 卸载

```shell
sh ~/.oh-my-zsh/tools/uninstall.sh
```

## 配置

### 主题

安装完之后，默认的主题为 `robbyrussell`：

```shell
➜  ~ 
```

使用`open ~/.zshrc` 修改配置文件 `~/.zshrc` ：

```properties
# Set name of the theme to load --- if set to "random", it will
# load a random theme each time oh-my-zsh is loaded, in which case,
# to know which specific one was loaded, run: echo $RANDOM_THEME
# See https://github.com/ohmyzsh/ohmyzsh/wiki/Themes
ZSH_THEME="robbyrussell" # CHANGE TO YOUT THEME
```

其他更多主题请参阅：https://github.com/ohmyzsh/ohmyzsh/wiki/Themes

### 插件

找到以下 plugins 选项，在后面的括号内添加插件，多个插件用空格隔开，git 已作为默认插件加载。

```properties
# Which plugins would you like to load?
# Standard plugins can be found in $ZSH/plugins/
# Custom plugins may be added to $ZSH_CUSTOM/plugins/
# Example format: plugins=(rails git textmate ruby lighthouse)
# Add wisely, as too many plugins slow down shell startup.
plugins=(git)
```

其他更多插件请参阅：https://github.com/ohmyzsh/ohmyzsh/wiki/Plugins

## Reference

1. https://blog.csdn.net/qwe641259875/article/details/107201760/
2. https://blog.csdn.net/u012039040/article/details/123279213
3. https://github.com/ohmyzsh/ohmyzsh/wiki/Themes
4. https://github.com/ohmyzsh/ohmyzsh/wiki/Plugins
