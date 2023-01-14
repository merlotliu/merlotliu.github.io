---
layout: post
title: 判断文件是否为空
date: 2023-01-09 15:57:13
updated: 2023-01-09 15:57:13
tags: [cpp]
categories: [cpp]
comments: true
---

## 函数 & 宏

- fopen() ：打开一个文件，返回 FILE 指针；
- fgetc() ：从 FILE 指针获取一个字符，到达文件末尾则返回 EOF；
- EOF ：标识文件末尾，并非一个字符；
- fclose() ：从 FILE 指针关闭一个文件；

## 逻辑

### 方法1:

打开文件，读取一个字符，如果是 EOF 说明文件为空，反之不为空。此外，只有文件存在且 fgetc 第一个读到的是 EOF 才认为文件为空，返回 true，其余情况均返回 false。

### 方法2：

获取文件大小，如果文件大小为 0，说明为空。

具体可以用 lseek 或 stat 来实现，这一部分可以参见标准文档或我的另外一片博客。

## 代码

```c++
#include <stdio.h>

bool is_empty( const char* p ) {
  FILE* fp = fopen( p, "r" );
  if( fp == NULL ) return false;
  int flag = ( EOF == fgetc( fp ) );
  fclose( fp );
  return flag;
}
```

## 测试

1. 创建两个文件，一个名为 `not_empty.txt`, 一个为 `empty.txt`；
2. 在`not_empty.txt` 写入任意几个字符；
3. 运行以下代码：

```c++
int main( int argc, char* argv[] ) {
    const char* file_is_not_empty = "./not_empty.txt";
    const char* file_is_empty = "./empty.txt";

    bool b = is_empty( file_is_not_empty );
    printf( b ? "%s is empty\n" : "%s is not empty\n", file_is_not_empty );
    b = is_empty( file_is_empty );
    printf( b ? "%s is empty\n" : "%s is not empty\n", file_is_empty );
    return 0;
}
```

## 结果

```shell
./not_empty.txt is not empty
./empty.txt is empty
```

## Reference 

1. C 标准文档
