---
layout: post
title: 字符串大小写转化
date: 2023-01-10 12:28:54
updated: 2023-01-10 12:28:54
tags: [cpp]
categories: [cpp]
comments: true
---

## 逻辑

大小写的转化逻辑很简单：

- 全部转化小写：判断是否为大写（在 A ~ Z 之间），将对应的字母加上小写字母和大写的差值（'a' - 'A'）；
- 全部转化大写：判断是否为大写（在 a ~ z 之间），将对应的字母加上大写字母和小写的差值（'A' - 'a'）；



## 代码

```c++
#include <string>

const char* lower_case(char* s) {
    char *ptr = s;
    while (*ptr != '\0') {
        if(*ptr >= 'A' && *ptr <= 'Z') *ptr += 'a' - 'A';
        ptr++;
    }
    return s;
}

const char* lower_case(std::string& s) {
    char *ptr = &s[0];
    while (*ptr != '\0') {
        if(*ptr >= 'A' && *ptr <= 'Z') *ptr += 'a' - 'A';
        ptr++;
    }
    return s.c_str();
}

const char* upper_case(char* s) {
    char *ptr = s;
    while (*ptr != '\0') {
        if(*ptr >= 'a' && *ptr <= 'z') *ptr += 'A' - 'a';
        ptr++;
    }
    return s;
}

const char* upper_case(std::string& s) {
    char *ptr = &s[0];
    while (*ptr != '\0') {
        if(*ptr >= 'a' && *ptr <= 'z') *ptr += 'A' - 'a';
        ptr++;
    }
    return s.c_str();
}
```



## 测试

```c++
#include <stdio.>

int main(void) {
    std::string str = "AbAbCcDd";
    printf("%s\n", str.c_str());
    printf("%s\n", lower_case(str));
    printf("%s\n", upper_case(str));
    return 0;
}
```



## 结果

```
AbAbCcDd
ababccdd
ABABCCDD
```



## Reference 

1. C/Cpp 标准文档
