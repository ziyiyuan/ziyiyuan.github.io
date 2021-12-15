# 工具和工件重力补偿接口说明

## 基于末端传感器 的重力补偿基本流程和接口如下：

#### 1. 建立机器人算法接口实例

```C++
/**
 * @brief 创建机器人算法库实例
 * @param modelName：机器人对应的URDF描述文件
 * @param config：通过配置文件创建不同的特性（reserved）
 * @return 算法库实例指针，如果失败，返回 nullptr
 * ARAL算法库文件可以通过dlopen在运行时打开, 通过调用CreateRLIntfacePtr函数得到基类接口指针
 */
extern ARAL::RLIntfacePtr CreateARALIntfacePtr(const char* model, const char* config = NULL);

```

#### 2. 设置传感器相对于法兰坐标系的位姿；

```c++
    /**
     * @brief 设置末端传感器在机器人法兰坐标系下的位姿
     * @param pose: 位置和姿态数组信息
     * @param type：位置表示XYZ三个方向的偏移；姿态有多种表示方式，可根据实际的情况选择不同的表示。
     */
    virtual void mdlSetEndSensorPose(const double* pose, const PoseDescription type = PoseDescription::POS_RPY) = 0;
```

#### 3. 设置 tcp 相对于法兰坐标系的位姿；

```C++
    /**
     * @brief 设置工具(整体)的位姿
     * @param pose: 位置和姿态数组信息
     * @param type：位置表示XYZ三个方向的偏移；姿态有多种表示方式，可根据实际的情况选择不同的表示。
     */
    virtual void mdlSetToolPose(const double* pose, const PoseDescription type = PoseDescription::POS_RPY) = 0;
```

#### 4. 标定传感器（得到工具在传感器坐标系的质量和质心）

#####  a.  三点法静态标定

```c++
 /**
     * @brief 根据末端6维力/力矩传感器的信息标定工具和传感器的属性
     * @param joints： 输入三种不同构型对应的机器人关节角，三种构型的姿态差异越大越好
     * @param measurement：在三种构型下测得的传感器的原始裸数据
     * @param result: 1)offset:传感器的零点偏置值
     *                2）com： 工具在传感器坐标系下的质心位置
     *                3）mass: 工具的质量
     *                4)angle: 机器人底座在世界坐标系下的姿态角。
     * @return
     */
    virtual int calibToolAndSensor(const RLJntArray joints[FT_SENSOR_CALIB__NUM], const RLWrench measurement[FT_SENSOR_CALIB__NUM], FtSensorCalibrationResult& result) = 0;
```

#####　b. 动态标定

设置轨迹配置信息

```c++
    /**
     * @brief 动力学标定的配置信息
     * @param config: 具体的配置信息(参考CalibConfig结构体)
     * @param excitation_trajectory: 辨识的激励轨迹(需要运行该轨迹得到测量数据)
     * @param trajPara: 激励轨迹参数
     * @return: 返回值 < 0, 表示计算失败(需要检查配置信息中的约束条件是否过于严格或者相互矛盾)
     */
    virtual int calibDynamicConfig(const CalibConfig& config, DoubleVecVec& excitation_trajectory, DoubleVec& trajPara) = 0;
```

```c++
/**
     * @brief 动力学参数辨识(参考DynCalibType结构体)
     * @param type: 辨识的类型
     * @param curPos: 测量的关节角度位置
     * @param curCur: 测量的关节电流(单位mA), 如果没有就置空
     * @param curSen: 测量的传感器值，如果为本体辨识， 则为底座传感器的值； 如果为工具辨识，则为末端六维力传感器的测量值；如果没有就置空
     * @param paraEst: 输出的估计参数， 具体的数值意义根据不同的type有不同的定义
     * @return: 返回值 < 0 表示计算失败
     */
    virtual int calibDynamicPara(const DynCalibType& type, const DoubleVecVec& curPos, const DoubleVecVec& curCur, const DoubleVecVec& curSen, DoubleVec& paraEst) = 0;

```

#### 5.设置传感器偏置

```c++
    /**
     * @brief 更新末端六维力传感器的偏置，在对传感器标定完成后需要设置该信息
     *        (如果不设置,则在力控时传给算法的数据必须是减去偏置后的信息)
     * @param ftData：传感器在6个方向的偏置
     */
    virtual void rsUpdateEndFTSensorOffset(const RLWrench& ftData) = 0;
```



#### 6.设置工具在传感器坐标系下的质量和质心

```c++
    /**
     * @brief 设置工具的动力学参数, 参考坐标系为末端传感器坐标系
     * @param m： 工具质量
     * @param com：工具质心, 维度为3
     * @param inertial：维度为6, 如果没有则置零。
     */
    ARAL_INT_EXTENSION virtual void mdlSetToolInertial(const double m, const double* com, const double* inertial) = 0;
```



#### 7. 设置工件在工具坐标系下的质量和质心；

工件变化时，需要更改此参数；

```c++
    /**
     * @brief 设置工件的动力学参数, 参考坐标系为工具坐标系(一般用于pick的场景)
     * @param m: 工件的质量
     * @param com: 工件的质心
     * @param inertial: 工件在参考坐标系下的惯性参数；
     */
    virtual void mdlSetWorkpieceInertial(const double m, const double* com, const double* inertial) = 0;
```



#### 8. 设置当前机械臂状态，

```C++
	/**
     * @brief 更新机器人关节的实时状态信息.
     * @param q: 关节位置，单位rad.
     * @param qd: 关节速度，单位rad/s, 如果没有则置空.
     * @param qdd: 关节加速度，单位rad/s^2，如果没有则置空.
     */
    virtual void rsUpdatJointPVA(const double* q, const double* qd = NULL, const double* qdd = NULL) = 0;
```



#### 9. 更新传感器状态信息；

```c++
    /**
     * @brief 更新末端六维力传感器的信息，在力控时每个周期都需更新
     * @param ftData：传感器在6个方向的力和力矩
     */
    virtual void rsUpdateEndFTSensor(const RLWrench& ftData) = 0;
```

#### 10. 得到在 tcp 处产生的外力；

```c++
    /**
     * @brief 获得机械臂工具末端作用点的Wrench
     * @param wrench: 6个方向的力和力矩
     */
    virtual READ_ONLY RLWrench rsGetTCPWrench() = 0;
```



#### 10. 判断是否发生碰撞。

```
用户自己设定阈值判断
```



## 调用示例

#### 基于三点法静态标定

```C++
    TEST(testCalGravity)
    {
        double M1 = 1.;
        double M2 = 2.;
        double C1[] = {0.01,0.02,0.03};
        double C2[] = {0.1,0.2,0.3};
        double i1[6] = {0.0139 , 0.0140, 0.0224, 0., 0., 0.};// IXX, IYY, IZZ, IXY, IXZ ,IYZ
        double i2[6] = {0.5 , 0.6, 0.7, 0., 0., 0.};
        double i[6] = {0.};

        // 1.
        RLIntfacePtr robot = CreateARALIntfacePtr("aubo_i5");
        // 2
        double Tfs[] = {0.01,0.02,0.03,M_PI/2,0,0};
        robot->mdlSetEndSensorPose(Tfs, PoseDescription::POS_RPY);
        //3
        double Tft[] = {0.05,0.01,0.03,M_PI/2,0,0};
        robot->mdlSetToolPose(Tft, PoseDescription::POS_RPY);
        //4
        FtSensorCalibrationResult res;
        RLJntArray poses[3];
        poses[0] ={ -15*M_PI/180, 15*M_PI/180,75*M_PI/180,60*M_PI/180,80*M_PI/180,0};
        poses[1] ={ -36*M_PI/180,27*M_PI/180,95*M_PI/180,-27*M_PI/180,0*M_PI/180,0};
        poses[2] ={-36*M_PI/180,21*M_PI/180,100*M_PI/180,-6*M_PI/180,90*M_PI/180,0};
        RLWrench vec[3];
        vec[0] ={0.479004, -40.4279, -0.670605, 2.59429, -0.0958008, 0.0536484};
        vec[1] ={43.8768, 5.84385, 0.958008, -0.134121, 2.59812, -3.78221};
        vec[2] ={0.383203, -2.29922, 42.2481, 4.01597, -0.183938, 0.149449};
        robot->calibToolAndSensor(poses, vec, res);
        //5
        RLWrench sensor_offset = {0.0};
        robot->rsUpdateEndFTSensorOffset(res.offset);
        //    robot->rsUpdateEndFTSensorOffset(sensor_offset);
        //6
        robot->mdlSetToolInertial(res.mass, res.com.data(), i);
        //    robot->mdlSetToolInertialFromFTSensor(M2, C2, i2);
        //7
        //    robot->mdlSetWorkpieceInertial(M2, C2, i2);
        robot->mdlSetWorkpieceInertial(M2, C2, i);
        //8
        double jnt[] = {0.6641,    0.3930,   -1.1606,    0.0172,   -1.5708,    0.6641};
        robot->rsUpdateJointPVA(jnt, jnt, jnt);
        //9
        RLWrench Sensor_raw_data = {0.0};
        robot->rsUpdateEndFTSensor(Sensor_raw_data);
        //10
        RLWrench wrench_tip = robot->rsGetTCPWrench();  
    }
```

#### 动态标定

```c++
 TEST(testToolCalibrationBasedOnEndSensor)
    {
        int ret = 0;
        // 1. 初始化机器人实例
        RLIntfacePtr robot = CreateARALIntfacePtr("aubo_i5", lib_config.c_str());

        // 2．设置传感器在法兰坐标系的位姿
        double Tfs[] = {0, 0, 0.04, 0, 0, 0};
        robot->mdlSetEndSensorPose(Tfs, PoseDescription::POS_RPY);

        //3．设置工具坐标系子法兰坐标系的位姿
        double Tft[] = {0.05, 0.01, 0.14, M_PI/2, 0, 0};
        robot->mdlSetToolPose(Tft, PoseDescription::POS_RPY);

        //４．标定传感器
        //４.1 设置轨迹属性
        CalibConfig config;
        config.type = DynCalibType::TOOL_END_SENSOR;　//设置标定类型为基于末端传感器对工具进行标定
        config.move_axis = {3, 4}; //只运动４，５关节
        config.init_joint = {0, 0, 0, 0, 0, 0}; // 运动初始位置

        config.upper_joint_bound = {2., M_PI/2};　//关节运动位置上限
        config.lower_joint_bound = {-2, -M_PI/2};　//关节运动位置上限

        DoubleVecVec trajectory;
        std::vector<double> coeff;
    	 //４.2 优化得到激励轨迹
        ret = robot->calibDynamicConfig(config, trajectory, coeff);    //对辨识轨迹进行优化
     
     	// trajectory为激励轨迹， 可以以servoJ或离线运动的方式驱动机械臂运动， 然后采集对应的关节位置、电流和末端力传感器的信息
     
     　　//４.３ 采集当前电流　curPos　和传感器数值　curSen，　其中curCur设置为空，工具估计参数为　paraEst
        DoubleVecVec curPos, curCur, curSen;
        DoubleVec paraEst;
        paraEst.reserve(16);
        ret = robot->calibDynamicPara(DynCalibType::TOOL_END_SENSOR, curPos, curCur, curSen, paraEst);

        //５. 设置传感器偏置
        RLWrench sensor_offset = {paraEst[10], paraEst[11], paraEst[12], paraEst[13], paraEst[14], paraEst[15]};
        robot->rsUpdateEndFTSensorOffset(sensor_offset);

        //６. 设置工具动力学参数
        double Mt = paraEst[0];
        double Ct[3] = {paraEst[1], paraEst[2], paraEst[3]};
        double It[6] = {paraEst[4], paraEst[5], paraEst[6], paraEst[7], paraEst[8], paraEst[9]};// IXX, IYY, IZZ, IXY, IXZ ,IYZ
        robot->mdlSetToolInertialFromFTSensor(Mt, Ct, It);

        //７. 设置工件动力学参数
        double Mw= 0;
        double Cw[3], Iw[6];
        robot->mdlSetWorkpieceInertial(Mw, Cw, Iw);

        //８. 更新关节角,速度，加速度
        double q[6], qd[6], qdd[6];
        robot->rsUpdateJointPVAT(q, qd, qdd);

        //９. 更新传感器数据
        RLWrench Sensor_raw_data = {0.0};
        robot->rsUpdateEndFTSensor(Sensor_raw_data);

        //10. 得到作用在tcp末端的力
        RLWrench wrench_tip = robot->rsGetTCPWrench();
    }
```



## 参数设置说明

1. 只补偿重力的情况下，工具和工件的惯性张量可以不设置，默认为0；
2. 只补偿重力的情况下，机械臂的速度，加速度均设为0；
3. 所有参数均采用国际单位制. m , rad, kg, 等