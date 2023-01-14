---
layout: post
title: 复制文件
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

读取写入的过程很简单，读到数据为 0 时，也就到达文件末尾了。

lseek 在这里的作用是，判断最后两个文件的大小，如果文件大小不一样，则说明有问题，将目标文件移除，返回 false；

此外，如果文件已经存在，则会被覆盖。



## 代码

```c++
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

#define BUFSIZE 1024

namespace lu {
namespace filesystem {

bool copy_file(const char* src_path, const char* dst_path) {
    int fout = open(dst_path, O_WRONLY | O_CREAT, 0664);
    if(-1 == fout) return false;
    int fin = open(src_path, O_RDONLY);
    if(-1 == fin) {
        printf("open file '%s' failed\n", src_path);
        return false;
    }
    char buf[BUFSIZE];
    int len = 0;
    while((len = read(fin, buf, sizeof(buf))) > 0) {
        write(fout, buf, len);
    }

    if(lseek(fout, 0, SEEK_END) < lseek(fin, 0, SEEK_END)) {
        printf("generate file '%s' failed\n", dst_path);
      	close(fin);
        close(fout);
        remove(dst_path);
        return false;
    }
    close(fin);
    close(fout);

    return true;
}
```



## 测试

创建名为 `src.txt` 的文件，写入 `This is src file.` ，定义目标文件路径，调用函数拷贝之后，输出拷贝文件的内容：

```c++
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

int main(void) {
    const char src_path[] = "./src.txt";
    int fin = open(src_path, O_RDWR | O_CREAT, 0664);
    char buf[] = "This is src file.";
    write(fin, buf, sizeof(buf) - 1);
    printf("src file :\n%s\n", buf);

    const char dst_path[] = "./dst.txt";
    lu::filesystem::copy_file(src_path, dst_path);
    int fout = open(src_path, O_RDONLY);
    bzero(buf, sizeof(buf));
    read(fout, buf, sizeof(buf));
    printf("dst file :\n%s\n", buf);

    return 0;
}
```

**Notes**：值得注意的是，我们在写入 buf 时，长度为 sizeof(buf) -1 ，这是为什么？原因是 `""` 定义的字符串常量会在末尾添加 '\0' ，这样一来如果按 buf 长度写入，实际写入的东西会变成 `This is src file.\0` ，`\0` 在一些环境下显示成 NULL，此时文件也会被认为是二进制文件。



## 结果

```
src file :
This is src file.
dst file :
This is src file.
```



## Reference 

1. C/Cpp 标准文档
