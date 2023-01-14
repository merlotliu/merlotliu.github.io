---
layout: post
title: 合并文件
date: 2023-01-10 12:28:54
updated: 2023-01-10 12:28:54
tags: [cpp]
categories: [cpp]
comments: true
---

## 函数 & 宏

- open()：打开一个文件，返回文件描述符；
- read()：从文件描述符，读取 size 长度数据到 buffer，返回读取的长度；
- write()：将 buffer 中 size 长度的数据写入到 fd，返回写入的长度；
- lseek()：设置根据 flag 设置文件偏移，SEEK_SET 相对于文件头，SEEK_END 相对于文件尾，SEEK_CUR 相对于当前位置，返回相对于文件头的偏移量；
- remove()：根据路径移除文件；
- close()：关闭文件描述符；



## 逻辑

事实上，合并文件和复制文件的内容大同小异。主要是复制文件是一个文件到另外一个文件，而合并文件是多个文件输入到同一个文件。

这里的操作是，将多个文件合并为一个新文件。根据这一点，其实我们可以引申出将多个文件追加到一个文件后面的操作。



## 代码

```c++
bool combine(int src_cnt, const char *src_path[], const char* dst_path) {
    int fout = open(dst_path, O_WRONLY | O_CREAT, 0664);
    if(-1 == fout) return false;
    
    long total_size = 0;
    char buf[BUFSIZE];
    for(int i = 0; i < src_cnt; i++) {
        int fin = open(src_path[i], O_RDONLY);
        if(fin != -1) {
            int len = 0;
            while((len = read(fin, buf, sizeof(buf))) > 0) {
                write(fout, buf, len);
            }
            total_size += lseek(fin, 0, SEEK_END);
            close(fin);
        } else {
            printf("open file '%s' failed\n", src_path[i]);
            return false;
        }
    }
    
    if(lseek(fout, 0, SEEK_END) != total_size) {
        printf("generate file '%s' failed\n", dst_path);
        close(fout);
        remove(dst_path);
        return false;
    }
    close(fout);
    return true;
}
```



## 测试

创建三个文件 `src1.txt、src2.txt、src3.txt`，分别写一些内容，随后将他们按照数字顺序，合并成 `combine.txt`，打印合并之后的文件内容：

```c++
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

int main(void) {
    int src_cnt = 3;
    const char *src_path[src_cnt];
    src_path[0] = "./src1.txt";
    src_path[1] = "./src2.txt";
    src_path[2] = "./src3.txt";
    int fd = open(src_path[0], O_WRONLY | O_CREAT, 0664);
    write(fd, "This is src1.txt.\n", 18);
    close(fd);
    fd = open(src_path[1], O_WRONLY | O_CREAT, 0664);
    write(fd, "This is src2.txt.\n", 18);
    close(fd);
    fd = open(src_path[2], O_WRONLY | O_CREAT, 0664);
    write(fd, "This is src3.txt.\n", 18);
    close(fd);
    const char dst_path[] = "./combine.txt";
    lu::filesystem::combine(src_cnt, src_path, dst_path);
    fd = open(dst_path, O_RDONLY);
    char buf[1024];
    read(fd, buf, 1024);
    printf("%s\n", buf);
    close(fd);

    return 0;
}
```



## 结果

```
This is src1.txt.
This is src2.txt.
This is src3.txt.
```



## Reference 

1. C/Cpp 标准文档
