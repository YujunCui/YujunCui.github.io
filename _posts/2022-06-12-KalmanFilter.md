---
layout: article
title: Yolov5-DeepSort之卡尔曼滤波
tags: TeXt
---
# 卡尔曼滤波
卡尔曼滤波算法有预测和更新两个主要过程，共 5 个公式。
## 1. 预测
根据上一个时刻 (k-1 时刻) 的后验估计值来估计当前时刻 (k时刻) 的状态，得到当前时刻 (k时刻) 的先验估计值. 估计的对象有两个，分别是状态和协方差矩阵。
- 状态预测公式1：
<div align=center><img src="..\assets\images\kalman\1卡尔曼状态预测公式.png" style="zoom:80%;" /></div>
​        或者简化为：
<div align=center><img src="..\assets\images\kalman\1卡尔曼状态预测公式_简化.png" style="zoom:80%;" /></div>
- 协方差预测公式2：
<div align=center><img src="..\assets\images\kalman\2卡尔曼协方差预测公式.png" style="zoom:80%;" /></div>

公式1是根据上一时刻的状态和控制变量来推测此刻的状态，公式1实际上是对运动系统的建模的过程。*x*表示状态向量; *u*表示控制或加速度向量; *w*表示预测过程的噪声，服从高斯分布; *k*表示时间; *F*表示状态转移矩阵，常用来对目标的运动建模，其模型可能为匀速直线运动或者匀加速运动，当状态转移矩阵不符合目标的状态转换模型时，滤波会很快发散; *B*表示控制矩阵; ^表示是估计值，并不是真实值，因为真实值无法获得; 上标-表示先验值，没有上标-表示后验值。

公式2是根据上一时刻的协方差矩阵来推测此刻的协方差矩阵，估计状态的不确定度。*P*表示状态向协方差矩阵;*Q*表示系统过程的协方差，该参数被用来表示状态转换矩阵与实际过程之间的误差，也叫状态转移协方差矩阵; 上标*T*表示矩阵的转置。


## 2. 更新
使用当前时刻 (k时刻) 的测量值来更正预测阶段估计值，得到当前时刻 (k时刻) 的后验估计值。

- 卡尔曼增益公式3：
<div align=center><img src="..\assets\images\kalman\3卡尔曼增益公式.png" style="zoom:80%;" /></div>
- 状态更新公式4：
<div align=center><img src="..\assets\images\kalman\4卡尔曼状态更新公式.png" style="zoom:80%;" /></div>
- 协方差更新公式5：
<div align=center><img src="..\assets\images\kalman\5卡尔曼协方差更新公式.png" style="zoom:80%;" /></div>

公式3计算卡尔曼增益或卡尔曼系数*K*。*H*表示状态变量到测量(观测)的转换矩阵，表示将状态和观测连接起来的关系，卡尔曼滤波里为线性关系，它负责将 m 维的状态空间转换到 n 维(m≥n)的测量空间，使之符合数学计算，例如在我们只有位置传感器而没有速度传感器的情况下，我们直接能得到的测量值或观测值只有位置，但是可以利用位置和时间的关系间接得到速度，因此位置和速度都可以作为状态变量，这也就是状态空间维度≥测量空间维度; *R*表示测量噪声协方差矩阵; 上标-1表示矩阵的逆。

公式4更新状态的后验估计值，卡尔曼增益*K*用来加权，即是选择相信预测值多一点还是选择相信测量值多一点。 *z*表示测量值(观测值)。当测量噪声*R*很大时，意味着测量不准确，我们选择相信预测值；当测量噪声*R*很小时，意味着测量较为准确，我们选择相信测量值。


Enjoy! :ghost: :ghost: :ghost:

<!--more-->

---

If you like TeXt, don't forget to give me a star. :star2:

[![Star This Project](https://img.shields.io/github/stars/kitian616/jekyll-TeXt-theme.svg?label=StZZars&style=social)](https://github.com/kitian616/jekyll-TeXt-theme/)

