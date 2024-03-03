# AdaptorPhysX



项目运算架构已经完成，具体的模块业务表现待完成

AdaptorPhysX项目是使用CPU多线程(Unity Burst & Jobs系统)实现PBD (Position Based Dynamics)算法来模拟具有体积或表面的可变形物体(绳、布、软体、流体)，并根据PBD计算规则实现计算框架。您可以在此项目的基础上完成自己的算法扩展。

有关 PBD 算法参考论文: https://matthias-research.github.io/pages/publications/posBasedDyn.pdf

## 环境要求

请安装ECS框架，但是Entities可不安装，请按下文方式在Unity安装相关依赖

> Windows – Package Manager – Add package from git URL - input: com.unity.jobs 

我用的版本，理论上相近的版本也能运行，未来有大更新会酌情更新

- com.unity.burst - 1.8.9
- com.unity.jobs  - 0.70.0-preview.7
- com.unity.mathematics - 1.2.6

##### Unity

使用2022.3.13版本进行开发，在本版本有一个Unity关于范型的显示Bug，导致Constraint无法正常在Inspector操作，只能等待Unity官方后续更新。

问题参考：https://github.com/Mustenaka/AdaptorPhysX/issues/1

## 模块

本项目包含以下模块:

![AdaptorPhysX-architecture](../Pic/AdaptorPhysX-architecture.png)

## 效果

##### 绳索:

单一绳索, 1024粒子, 1轮迭代, 长度约束，Jobs加速

![rope-100p-constraint-distance](../Pic/Effect/Rope/job-rope-1024.gif)

## 我的博客

如果你感兴趣，可以访问我的博客了解一下PBD，URL：https://www.mustenaka.cn/index.php/2023/09/06/pbd-method-learn-01/

> AdaptorPhysX 全称: Adaptive Physics Extension
>
> 命名空间及缩写: APEX.
