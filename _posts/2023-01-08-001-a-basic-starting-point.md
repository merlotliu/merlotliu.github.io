---
layout: post
title: 001 A Basic Starting Point
date: 2023-01-08 12:54:16
updated: 2023-01-08 12:54:16
tags: ~
categories: ~
permalinks: ~
comments: true
---

# Step 1 : 初识 CMake

下面是不同 Section 的一些说明：

- 命令 & 变量：Exercise 中使用的相关命令和变量。
- 编辑：每一个 Step 中的 **编辑**  一节下的文件中，都包含了一个或多个 TODO 注释。每一个 TODO 代表改变或添加一行或两行代码。读者应该根据 TODO 后面的数字，按顺序完成。
- 开始：**开始** 一节将给出一些指导信息帮助完成 Exercise。
- 构建 & 运行：给出从构建到运行 Exercise 的具体步骤。
- 解答：给出每个 Exercise 期望的解答步骤；

值得注意的是，每一个步骤都是以下一个步骤的基础。例如，Step2 的开始基于 Step1 的正确完成。

## Exercise 1 - 构建一个最基础的 CMake 项目

最基础的 CMake 项目仅包含单个源文件。对于这样的项目，仅需要在 `CMakeLists.txt` 包含 3 条命令即可。

**Notes**：CMake 支持大小写混写，但推荐使用小写。

### 目标

理解如何创建一个简单的 CMake 项目

### 命令 & 变量

- `cmake_minimum_required()` ：所有项目最开始的 `CMakeLists.txt` 必须从这条命令开始，用其指定 CMake 的最低版本。这是为了确保后面使用的 CMake 函数能够被正常的编译运行。
  所以，在构建别人的项目，发现 cmake 版本过低，可以通过修改这一参数尝试是否能够编译通过。当然这是 cmake 无法或难以升级新版本的下下策。

- `project() ` : 设置项目的名称、语言或版本信息。建议紧跟在 `cmake_minimum_required()` 之后。
- `add_executable()` ：使用特定的源代码，生成一个可执行程序。

### 编辑

-  `CMakeLists.txt`

### 开始

tutorial.cxx 的源代码在 Cmake 教程的相关目录中提供（如果没有下载请先下载 [cmake-3.25.1-tutorial-source](https://cmake.org/cmake/help/latest/_downloads/c1300c13296fa23e6753fbab2d04d7a4/cmake-3.25.1-tutorial-source.zip)），可用于计算数字的平方根。此文件无需在此步骤中进行编辑。

在同一目录中有一个 CMakeLists.txt 文件，请完成TODO 1 到 TODO 3 的相关内容。

### 构建 & 运行

完成 TODO 1 到 TODO 3 的内容后，开始构建项目。理论上，是可以直接在 Step1 目录下 cmake ，但这样会污染源代码。所以，建议先创建一个文件夹用于存放编译后的内容，具体步骤如下：

创建 build 文件夹并切换到目录下：

```shell
cd Step1 && mkdir build && cd build
```

创建 Makefile 以其其他编译信息：

```shell
cmake ..
```

编译生成可执行文件：

```shell
cmake --build .
#or
make
```

测试并运行可执行文件 Tutorial：

```shell
./Tutorial 
./Tutorial 10
./Tutorial 81
```

### 解答

CMakeLists.txt

```cmake
# TODO 1
cmake_minimum_required(VERSION 3.10)
# TODO 2
project(Tutorial)
# TODO 3
add_executable(Tutorial tutorial.cxx)
```

## Exercise 2 - 指定 C++ 版本

cmake 提供了一些以 `CMAKE_` 开头的特殊变量。为了避免重名，自定义变量请与其区别开。

通过 CMAKE_CXX_STANDARD 和 CMAKE_CXX_STANDARD_REQUIRED 可以用来指定构建项目的 c++ 标准。

### 目标

添加 C++11 特性相关 api

### 命令 & 变量

- `CMAKE_CXX_STANDARD`：如果设置，将作为`CXX_STANDARD` 的默认值；
- `CMAKE_CXX_STANDARD_REQUIRED`：如果设置，将作为`CXX_STANDARD_REQUIRED` 的默认值；
- `set()`：为变量设置值；

### 编辑

- `CMakeLists.txt`
- `tutorial.cxx`

### 构建 & 运行

构建

```
cd Step1/build
cmake ..
make
```

运行

```
./Tutorial 
./Tutorial 10
./Tutorial 81
```

### 解答

tutorial.cxx

```
// #include <cstdlib> // TODO 5: Remove this line

	// convert input to double
  // TODO 4: Replace atof(argv[1]) with std::stod(argv[1])
  // const double inputValue = atof(argv[1]);
  const double inputValue = std::atof(argv[1]);
```

CMakeLists.txt

```
# TODO 6: Set the variable CMAKE_CXX_STANDARD to 11
#         and the variable CMAKE_CXX_STANDARD_REQUIRED to True
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
```

## Exercise 3 - 添加版本号并配置头文件

### 目标



### 命令 & 变量

- `<PROJECT-NAME>_VERSION_MAJOR`
- `<PROJECT-NAME>_VERSION_MINOR`
- `configure_file()`
- `target_include_directories()`

### 编辑

- `CMakeLists.txt`
- `tutorial.cxx`

### 开始



### 构建 & 开始



### 解答



## Reference 

1. [CMake 官方文档](https://cmake.org/cmake/help/latest/guide/tutorial/index.html)
