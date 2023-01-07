# Mac 更新后 git clone 报错

## 错误描述

使用 `git` 后出现以下错误信息：

```
xcrun: error: invalid active developer path (/Library/Developer/CommandLineTools), missing xcrun at: /Library/Developer/CommandLineTools/usr/bin/xcrun
```

## 解决方法

输入以下命令安装相关插件：

```
xcode-select --install
```

安装弹出的插件即可。

## Reference

1. http://www.manongjc.com/detail/50-jefqpqgejfuzmqs.html