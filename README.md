# GEARdrones

<img src="Document/drones.png" style="zoom: 67%;" />

> GEARdrones是一个专为**低成本无人机编队控制**所开发的，集**飞行控制器**、**UWB相对定位算法**、**软件上位机**于一体的四旋翼无人机系统。
>
> 演示视频：[自研无人机集群？理工男的大学生活也可以很精彩！](https://www.bilibili.com/video/BV1JY4y1o76U)
>
> 本项目主要包含以下内容：
>
> ​	1.基于STM32F1和F4的两个版本的飞行控制器嵌入式软件工程
>
> ​	2.基于STM32F1的遥控器嵌入式软件工程
>
> ​	3.无人机的PCB工程(正在整理)、外壳模型
>
> ​	4.相对定位算法的Matlab仿真文件
>
> ​	5.基于PYQT5和MQTT开发的软件上位机

### 参考开发环境：

​	嵌入式软件开发环境：Keil MDK5 5.29

​	编译器：V5.06 build750

​	MQTT服务器：mosquito/EMQX

​	MATLAB 2022B

​	Python 3.8

### 参考文献：

> [1] Guo K, Xie L. INFRASTRUCTURE-FREE COOPERATIVE RELATIVE LOCALIZATION FOR UAVS IN GPS-DENIED ENVIRONMENTS[J]. International Journal of Robotics and Automation, 2021, 36(6).
>
> [2] Nguyen T M, Zaini A H, Guo K, et al. An ultra-wideband-based multi-UAV localization system in GPS-denied environments[C]//2016 International Micro Air Vehicles Conference. 2016, 6: 1-15.
>
> [3] Guo K, Qiu Z, Miao C, et al. Ultra-wideband-based localization for quadcopter navigation[J]. Unmanned Systems, 2016, 4(01): 23-34.
>
> [4] Guo K, Li X, Xie L. Ultra-wideband and odometry-based cooperative relative localization with application to multi-UAV formation control[J]. IEEE transactions on cybernetics, 2019, 50(6): 2590-2603.

### 参考开源库：

[SRML:华南虎嵌入式软件中间件层库](https://github.com/scutrobotlab/srml)
