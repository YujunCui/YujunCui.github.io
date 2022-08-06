---
layout: article
title: Yolov5-DeepSort之马氏距离
tags: TeXt
---
# 马氏距离
**马氏距离**(Mahalanobis distance)是由印度统计学家马哈拉诺比斯(P. C. Mahalanobis)提出的，表示点与一个分布之间的距离。它是一种有效的计算两个未知样本集的相似度的方法。与[欧氏距离](https://baike.baidu.com/item/欧氏距离/1798948)不同的是，它考虑到各种特性之间的联系。

+ 马氏距离可以定义为两个服从同一分布并且其协方差矩阵为**Σ**的随机变量***x***与***y***的差异程度：

<div align=center><img src="https://raw.githubusercontent.com/YujunCui/YujunCui.github.io/master/assets/images/kalman/马氏距离.png" style="zoom:80%;" /></div>

​       等价实现推理：
<div align=center><img src="https://raw.githubusercontent.com/YujunCui/YujunCui.github.io/master/assets/images/kalman/马氏距离等价实现.png" style="zoom:80%;" /></div>
+ 如果协方差矩阵为单位矩阵，马氏距离就简化为欧式距离；如果协方差矩阵为对角阵，其也可称为正规化的马氏距离。

- - -
## 1. 代码详解

上述马氏距离的理论部分，但是在代码实践中，原作者做了一些改进，即等价推理部分。本人在学习过程中对代码做了详细的注释，并论证了该等价过程，记录在本博客中，与大家分享。

```python
"""
# 卡尔曼滤波分为两个阶段：预测阶段 和 更新阶段
预测阶段：根据上一时刻(t-1时刻)的后验估计值来估计当前时刻(t时刻)的先验估计值
    Eq1. 状态预测公式:     x^~(t) = F*x^(t-1) + B*u(t-1)  + w(t-1) 
                         或简化 x^~(t) = F*x^(t-1)
    Eq2. 协方差预测公式:   P~(t) = F*P(t-1)*F.T + Q
更新阶段：使用当前时刻(t时刻)的测量值来更正预测阶段估计值，得到当前时刻(t时刻)的后验估计值
    Eq3. 卡尔曼增益:      K(t) = P~(t)*H.T*(H*P~(t)*H.T + R).I
    Eq4. 状态更新公式:    x^(t) = x^(t-1) + K(t)*(z(t) - H*x^~(t))
    Eq5. 协方差更新公式:   P(t) = (I - K(t)*H)*P~(t)
"""
class KalmanFilter(object):
    def __init__(self):
        pass
    
    # 状态、协方差 向测量空间映射
    def project(self, mean, covariance):
        std = [self._std_weight_position * mean[3],
               self._std_weight_position * mean[3],
               1e-1,
               self._std_weight_position * mean[3]]
        # 初始化测量过程中噪声矩阵 R, 即 innovation_cov, 该噪声矩阵与检测框的高相关 ，它是一个4x4的对角矩阵
        innovation_cov = np.diag(np.square(std))

        # Eq4. 状态更新公式: x^(t) = x^(t-1) + K(t)*(z(t) - H*x^~(t))
        # Eq4. 中将 状态向量 映射到 测量空间 H*x^~(t)
        mean = np.dot(self._update_mat, mean)
        # Eq3. 卡尔曼增益: K(t) = P~(t) * H.T * (H * P~(t) * H.T + R).I
        # Eq3. 中将协方差矩阵 P~(t) 映射到测量空间 H * P~(t) * H.T
        covariance = np.linalg.multi_dot((self._update_mat, covariance, self._update_mat.T))

        return mean, covariance + innovation_cov  # H * x^(t)； H * P~(t) * H.T + R
    
    # 马氏距离
    def gating_distance(self, mean, covariance, measurements, only_position=False):
        mean, covariance = self.project(mean, covariance)
        if only_position:
            mean, covariance = mean[:2], covariance[:2, :2]
            measurements = measurements[:, :2]
        
        # 将 埃尔米特（实数即对称）正定的矩阵 分解为下/上三角矩阵
        cholesky_factor = np.linalg.cholesky(covariance)
        d = measurements - mean  # Xi - Xj
        # scipy.linalg.solve_triangular解下三角线性方程组 a*x = b 方程中的 x，（假定a是一个上/下三角矩阵）
        z = scipy.linalg.solve_triangular(cholesky_factor, d.T, lower=True, check_finite=False, 			                                           overwrite_b=True)
        squared_maha = np.sum(z * z, axis=0)  # Dmaha**2 = z.T * z = z**2, 见等价推理

        return squared_maha

```



```
本文作者： 崔玉君
版权声明： 转载请注明出处！
```



Enjoy! :ghost: :ghost: :ghost:

<!--more-->

---

If you like TeXt, don't forget to give me a star. :star2:

[![Star This Project](https://img.shields.io/github/stars/kitian616/jekyll-TeXt-theme.svg?label=StZZars&style=social)](https://github.com/kitian616/jekyll-TeXt-theme/)