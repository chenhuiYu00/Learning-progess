# PWM与力矩指令转换

PWM（脉冲宽度调制）是一种通过调节占空比来控制电机转速和力矩的方法。占空比就是指在一个周期内，信号处于高电平的时间占据整个信号周期的百分比。例如，如果一个周期为10ms，高电平为5ms，那么占空比就是50%。

要将力矩指令转换为PWM占空比，您需要知道电机的一些参数，如电阻R、电感L、反电动势系数Ke和力矩系数Kt。您还需要知道电机的输入电压Ea和期望的力矩T。

根据基本的电机[方程](https://zhuanlan.zhihu.com/p/309276208)，我们可以得到以下关系：
$$
E=IR+LdtdI+Keω
\\
T=KtI
$$
其中I是电机电流，ω是电机角速度。

如果忽略电感的影响，我们可以简化为：
$$
E=IR+Keω
$$
将第二个方程代入第一个方程，我们可以得到：
$$
E=T×R/Kt+Keω
$$
解出T，我们可以得到：
$$
T=Kt(E−Keω)/R
$$
如果我们假设ω是恒定的，那么我们可以看出T和Ea成正比。也就是说，如果我们增加输入电压，那么力矩也会增加。

但是我们不能无限制地增加输入电压，因为这样会导致过热或损坏电机。所以我们需要用PWM来控制输入电压的平均值。

假设我们有一个恒定的直流电源Vd，我们用PWM来开关它。那么输入电压的平均值就是：
$$
E=Vd×m
$$
其中m是占空比。所以我们可以将力矩方程改写为：
$$
T=Kt(Vd×m−Keω)/R
$$
如果我们已知期望的力矩T和其他参数，那么我们可以解出占空比m：
$$
m=KtVdT+RVdKeω
$$
这就是将力矩指令转换为PWM占空比的公式。当然，这里做了一些简化的假设，实际情况可能更复杂。您还需要考虑PWM的频率、电流纹波、反电动势、续流二极管等[因素](https://blog.csdn.net/aa804738534/article/details/104841178)[3](https://blog.csdn.net/aa804738534/article/details/104841178)[4](https://blog.csdn.net/xuewei_Li/article/details/87907052)[5](https://zhuanlan.zhihu.com/p/617868624)。





## 获取Kt，Ke，R

电机的Kt、Ke、R等参数一般可以从电机的规格书或者产品目录中找到。如果没有这些资料，您也可以通过一些测量方法来获得这些参数。😊

测量电阻R的方法是：分离电机电源线，连接任意两相到电阻测量仪（如：万用表）并让第三相悬空，记录相到相间的电阻值，该值的一半为每相电阻值[1](https://www.infineon.com/dgdl/Infineon-How-to-Measure-Motor-Parameters-PP-v01_00-CN.pdf?fileId=5546d462566bd0c701568cd2e6694248)。

测量电感L的方法是：分离电机电源线，连接任意两相到电感测量仪（如：RCL测量仪）并设置为1kHz，让第三相悬空，记录相到相间的电感值，该值一半为每相的电感值[1](https://www.infineon.com/dgdl/Infineon-How-to-Measure-Motor-Parameters-PP-v01_00-CN.pdf?fileId=5546d462566bd0c701568cd2e6694248)。

测量反电动势系数Ke的方法是：分离电机电源线，连接任意两相端口到示波器电压探头，让第三相悬空，用手拨动电机，记录所产生正弦波的频率和峰峰值，使用公式[1](https://www.infineon.com/dgdl/Infineon-How-to-Measure-Motor-Parameters-PP-v01_00-CN.pdf?fileId=5546d462566bd0c701568cd2e6694248)：
$$
Ke=2πfVpeak_ph
$$
其中Vpeak_ph是正弦波的峰值，f是正弦波的频率。

测量转矩系数Kt的方法是：根据转矩系数Kt和反电动势系数Ke之间的关系[2](https://zhuanlan.zhihu.com/p/57481014)：
$$
Kt=2πPP60Ke
$$
其中PP是电机极对数。





## 论文

> 上面的有问题，看看论文
>
> **选取合适的PWM频率**：
>
> https://www.machinedesign.com/materials/article/21125511/controlling-brushed-dc-motors-using-pwm

关于电机的空间模型，我们不能直接将 Kt×I=转矩 ，因为我们还需要考虑反电动势和粘性阻力等等,但用T 约等于 Kt*I 也许可行

[相关](https://zhuanlan.zhihu.com/p/341200614)