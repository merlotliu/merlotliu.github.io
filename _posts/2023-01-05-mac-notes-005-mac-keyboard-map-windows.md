---
layout: post
title: Mac 外接 Windows 键盘映射修改
date: 2023-01-05 17:54:00 +0800
updated: 2023-01-05 17:54:00 +0800
tags: [mac]
categories: [mac] 
comments: true
---

## 下载映射软件 karabiner-elements

官方网址：https://karabiner-elements.pqrs.org

## Add rules

1. 选择 `Complex Modifications` -> 点击下方的 `Add rule`;

2. 接着点击下方的 `Import more rules from the internet （Open a web browser）`;

3. 在出现的网页中输入 `ctrl + c` -> 选择出现的 `Windows shortcuts on macOS` -> `Import`;

## 系统设置

接着在系统的`隐私与安全性` -> `输入监控` 中添加以下内容：

![image-20221215221104974](.gitbook/mac-keyboard-map-win/image-20221215221104974.png)

前两项位于 `/Library/Application Support/org.pqrs/Karabiner-Elements/bin` 文件夹下，后一项在 `应用程序 `中可以找到。

## Reference

1. https://blog.csdn.net/sdf57/article/details/125517142
2. https://cloud.tencent.com/developer/article/1873835
