# spacebot
ZJU Comprehensive Practice of Robots and Intelligent Systems - Task 1

浙江大学 机器人与智能系统综合实践 作业1

文件内容：

- SpaceRobot.ttt: Coppeliasim仿真文件
- forward_kinematics.py: 正运动学计算，其中radian为弧度版本，degree为角度版本
- inverse_kinematics.py: 逆运动学数值解计算，其中radian为弧度版本，degree为角度版本
- calc_rct.m: 用来验证DH参数表正确性以及可视化DH参数建模的matlab文件
- IK.py: 逆运动学解析解计算
- traj_planning.py: 六次多项式轨迹规划，以及左右基底切换
- trans_matrix_calculation.m: 1个关节的T矩阵计算
- trans_matrix.m: 7个关节的T矩阵计算
- mdh_reference.m: 逆运动学辅助代码，用来计算从2到7的齐次矩阵
- main.py: 主函数

正逆运动学结果已经在相关代码中离线计算好，在traj_planning当中直接代入计算结果。
