---
layout: post
title: 获取当前时间
date: 2023-01-10 12:28:54
updated: 2023-01-10 12:28:54
tags: [cpp]
categories: [cpp]
comments: true
---

## 函数 & 宏

- time()：返回 time_t 对象的 Epoch 时间；
- localtime_r(): 将 Epoch 时间转化为当地的日历时间，存储在 buf ，并返回指向 buf 的指针；

## 逻辑

获取时间的过程很单，调用 api 两三行就搞定了。

主要是 tm 结构，和时间格式的输出。tm 结构如下：

```c++
struct tm {
  int tm_sec;     /* seconds (0 - 60) */
  int tm_min;     /* minutes (0 - 59) */
  int tm_hour;    /* hours (0 - 23) */
  int tm_mday;    /* day of month (1 - 31) */
  int tm_mon;     /* month of year (0 - 11) */
  int tm_year;    /* year - 1900 */
  int tm_wday;    /* day of week (Sunday = 0) */
  int tm_yday;    /* day of year (0 - 365) */
  int tm_isdst;   /* is summer time in effect? */
  char *tm_zone;  /* abbreviation of timezone name */
  long tm_gmtoff; /* offset from UTC in seconds */
};
```

如，我需要的时间日期格式为 `YYYY-MM-DD hh-mm-ss`，根据以下代码即可获得。



## 代码

```c++
#include <timer.h>

#include <string>

std::string get_cur_datetime() {
    time_t time_s = time(NULL);
    struct tm cur;
    localtime_r(&time_s, &cur);
    char buf[BUFSIZE];
    snprintf(buf, BUFSIZE, "%04d-%02d-%02d %02d:%02d:%02d", 
        cur.tm_year + 1900, cur.tm_mon + 1, cur.tm_mday, cur.tm_hour, cur.tm_min, cur.tm_sec);
    return buf;
}
```



## 测试

```c++
int main(void) {
    printf("%s\n", get_cur_datetime().c_str());
    return 0;
}
```



## 结果

```
2023-01-10 13:13:26
```



## Reference 

1. C/Cpp 标准文档
