---
layout: post
title: 获取当前用户的家目录
date: 2023-01-10 12:28:54
updated: 2023-01-10 12:28:54
tags: [cpp]
categories: [cpp]
comments: true
---

## 函数 & 宏

- sysconf()：根据给定的宏获取系统变量；
- _SC_GETPW_R_SIZE_MAX：配合 sysconf() 获取存储用户信息结构的缓冲区最大值；
- getuid()：获取当前用户 id；
- getpwuid_r()：根据传入的用户 id，获取用户信息存入 buffer 数组；

## 逻辑

先获取所需缓冲区的大小，并声明缓冲区。

根据传入的用户 id，在系统记录 password 的文件（如：/etc/passwd）中查找相关用户信息，存入准备好的 buffer 数组中。成功后，result 指向存储数据的缓冲，passwd 结构中的指针变量指向缓冲（如 pw_name 等值存放在 buffer 数组中），整形变量直接存储相关信息。

可以通过在获取用户信息成功后，清空 buffer ，并在清空 buffer 前后打印 passwd 变量和 result 的值，来判断 buffer 中存放的值。

struct passwd结构如下：

```c++
struct passwd {
  char    *pw_name;       /* user name */
  char    *pw_passwd;     /* encrypted password */
  uid_t   pw_uid;         /* user uid */
  gid_t   pw_gid;         /* user gid */
  time_t  pw_change;      /* password change time */
  char    *pw_class;      /* user access class */
  char    *pw_gecos;      /* Honeywell login info */
  char    *pw_dir;        /* home directory */
  char    *pw_shell;      /* default shell */
  time_t  pw_expire;      /* account expiration */
  int     pw_fields;      /* internal: fields filled in */
};
```



## 代码

```c++
std::string get_home_dir() {
    int bufsize;
    if ((bufsize = sysconf( _SC_GETPW_R_SIZE_MAX )) == -1)
        return "";

    char buffer[bufsize];
    struct passwd pwd, *result = NULL;
    if (getpwuid_r(getuid(), &pwd, buffer, bufsize, &result) != 0 || !result)
        return "";

    return std::string(pwd.pw_dir);
}
```



## 测试

```c++
int main(void) {
    printf("%s\n", get_home_dir().c_str());
    return 0;
}
```



## 结果

```c++
/Users/usrname
```



## Reference 

1. C/Cpp 标准文档
