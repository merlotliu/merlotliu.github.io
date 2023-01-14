---
layout: post
title: Typora for Mac
date: 2023-01-01 17:54:00 +0800
updated: 2023-01-01 17:54:00 +0800
tags: [mac, typora]
categories: [mac]
comments: true
---

## 从命令行启动

在 mac 中，使用 `open -a Typora` 或 `open /Applications/Typora.app` 可以打开 Typora 。

在后面跟上需要打开的文件，即可使用 Typora 打开文件，如：

```
open -a Typora template.md
open /Applications/Typora.app template.md
```

为了更为简便的使用命令，可以在 .zshrc 中添加（使用 bash 的话就是 .bashrc）：

```
alias typora="open -a Typora"
```

`source .zshrc` 使其生效，然后就可以通过 `typora template` 的方式打开文件了。

## Reference 

1. https://www.jianshu.com/p/d607bc4a2314
