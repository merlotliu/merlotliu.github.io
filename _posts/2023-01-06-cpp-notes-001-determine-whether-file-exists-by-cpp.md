---
layout: post
title: 使用 C++ 判断文件是否存在
date: 2023-01-06 17:27:53
updated: 2023-01-06 17:27:53
tags: [cpp]
categories: [cpp]
comments: true
---

## 代码

```c++
#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <fstream>

inline bool exists_ifstream (const std::string& name) {
    std::ifstream fin(name.c_str());
    return f.good();
}

inline bool exists_fopen (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    }
   	return false;
}

inline bool exists_access (const std::string& name) {
    return ( access( name.c_str(), F_OK ) != -1 );
}

inline bool exists_stat (const std::string& name) {
    struct stat buffer;   
    return (stat (name.c_str(), &buffer) == 0); 
}
```

在参考资料的评测中，`stat` 的性能最好。

## Reference 

1. [c++ 判断文件是否存在的几种方法](https://blog.csdn.net/guotianqing/article/details/100766120)
1. [Fastest way to check if a file exists using standard C++/C++11,14,17/C?](https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exists-using-standard-c-c11-14-17-c)
