
[简体中文](./README.md) | English

<h1 align="center">NLink Unpack</h1>

<div align="center">

![Logo](http://ftp.nooploop.com/media/image/nooploop.png)

NLink Unpack is used for products of Nooploop，like [LinkTrack](https://www.nooploop.com/)，[LinkTrack AOA](http://www.nooploop.com/linktrack-aoa)，[TOFSense](https://www.nooploop.com/tofsense.Written in pure C language, can be used for building protocol parsing code

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)](https://dev.azure.com/ant-design/ant-design-pro/_build/latest?definitionId=1?branchName=master) ![Test Status](https://img.shields.io/badge/test-passing-brightgreen)


</div>

## Notice

- In case of compilation errors, make sure the compiler has turned on C99 support
- NLink protocol data follows the Little-Endian mode, this example code runs in the mode too, and the Big-Endian mode is not supported
- For variable length protocols, such as NLink_LinkTrack_NodeFrame1, heap memory allocation is used for dynamic nodes. If there are many nodes, please ensure enough heap space

## Usage

Refer to [example.c](./example.c) to get start

## License

The source code is released under a [BSD 3-Clause license](LICENSE).


