
简体中文 | [English](./README.en.md) 

<h1 align="center">NLink Unpack</h1>

<div align="center">

![Logo](http://ftp.nooploop.com/media/image/nooploop.png)

NLink Unpack 用于支持Nooploop产品，如[LinkTrack](https://www.nooploop.com/)，[LinkTrack AOA](http://www.nooploop.com/linktrack-aoa)，[TOFSense](https://www.nooploop.com/tofsense)，纯C语言编写，用户可用于构建协议解析代码

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)](https://dev.azure.com/ant-design/ant-design-pro/_build/latest?definitionId=1?branchName=master) ![Test Status](https://img.shields.io/badge/test-passing-brightgreen)


</div>

## Notice
- 如编译错误，请确保编译器已经开启C99支持
- NLink协议数据遵循小端模式，本示例代码默认运行于小端模式，大端模式不可用
- 对于变长协议，如NLink_LinkTrack_NodeFrame1，动态节点部分采用了堆内存分配，如节点数量较多，请确保堆空间足够

## Usage

使用示例参考[example.c](./example.c)文件

## License

源码基于[BSD 3-Clause license](LICENSE)发布


