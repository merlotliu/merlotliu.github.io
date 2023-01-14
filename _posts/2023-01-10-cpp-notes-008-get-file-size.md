---
layout: post
title: 获取文件大小
date: 2023-01-10 12:28:54 +0800
updated: 2023-01-10 12:28:54 +0800
tags: [cpp]
categories: [cpp]
comments: true
---

## 函数 & 宏

- open()：打开一个文件，返回文件描述符；
- lseek()：设置根据 flag 设置文件偏移，SEEK_SET 相对于文件头，SEEK_END 相对于文件尾，SEEK_CUR 相对于当前位置，返回相对于文件头的偏移量；
- close()：关闭文件描述符；

## 逻辑

主要就是通过 lseek 的返回来确定文件的大小。

所以，很显然我们再通过返回值的判断，即可确定文件是否为空。



## 代码

```c++
#include <unistd.h>
#include <fcntl.h>

long long get_file_size( const char* p ) {
  int fd = open( p, O_RDONLY );
  if( -1 == fd ) {
  	return -1;
  }
  long long file_size = lseek(fd, 0, SEEK_END);
  close(fd);
  return file_size;
}
```



## 测试

```c++
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

int main(void) {
    const char *p = "./src.txt";
    int fd = open(p, O_WRONLY | O_CREAT, 0664);
    write(fd, "This is src.txt.\n", 17);
    close(fd);
    printf("%lld\n", lu::filesystem::get_file_size(p));
  	printf("%lld\n", lu::filesystem::get_file_size(""));

    return 0;
}
```



## 结果

```
17
-1
```



## Reference 

1. C/Cpp 标准文档
