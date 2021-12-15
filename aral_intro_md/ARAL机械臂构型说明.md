# ARAL机械臂构型介绍

## 1\. 机械臂构型定义

ARAL 共定义了 8 种机械臂构型，其结构体定义如下：

```c++
enum class RobotConfiguration: int
{
    LEFT,           // 肩部朝左
    RIGHT,          // 肩部朝右
    UP,             // 肘部朝上
    DOWN,           // 肘部朝下
    FLIP,           // 腕部翻转
    NOFLIP,         // 腕部不翻转

    NONE    = -1,
    LUF     = 0,
    LUN     = 1,
    LDF     = 2,
    LDN     = 3,

    RUF     = 4,
    RUN     = 5,
    RDF     = 6,
    RDN     = 7
};
```



## 2\. Aubo i5 构型示例

| 构型   | 定义     |          示例图（除标注关节角外，其余关节角均为 0）          |
| ------ | -------- | :----------------------------------------------------------: |
| LEFT   | q_1 <= 0 | <img src="../../../resources/pics/aral_export_pic/Screenshot from 2021-12-13 15-36-23.png" alt="Screenshot from 2021-12-13 15-36-23" style="zoom:80%;" />       q1 = -74.44 |
| RIGHT  | q_1 > 0  | <img src="../../../resources/pics/aral_export_pic/Screenshot from 2021-12-13 15-28-57.png" alt="Screenshot from 2021-12-13 15-28-57" style="zoom:80%;" />       q1 = 124.78 |
| UP     | q_3 <= 0 | <img src="../../../resources/pics/aral_export_pic/Screenshot from 2021-12-13 15-41-14.png" alt="Screenshot from 2021-12-13 15-41-14" style="zoom:80%;" />       q3 = -59.25 |
| DOWN   | q_3 > 0  | <img src="../../../resources/pics/aral_export_pic/Screenshot from 2021-12-13 15-38-23.png" alt="Screenshot from 2021-12-13 15-38-23" style="zoom:80%;" />      q3 = 110.62 |
| FLIP   | q_5 <= 0 | <img src="../../../resources/pics/aral_export_pic/Screenshot from 2021-12-13 15-44-47.png" alt="Screenshot from 2021-12-13 15-44-47" style="zoom:80%;" />    q3 = -138.19 |
| NOFLIP | q_5 > 0  | <img src="../../../resources/pics/aral_export_pic/Screenshot from 2021-12-13 15-43-11.png" alt="Screenshot from 2021-12-13 15-43-11" style="zoom:80%;" />     q3 = 119.67 |


