MuJoCo MPC 汽车仪表盘项目
项目信息

    学号: 202011178
    
    姓名: 胥煜卿
    
    班级: 计科2305班
    
    完成日期: 2025年12月27日

项目概述

本项目基于MuJoCo MPC物理引擎，实现了一个汽车仪表盘可视化系统。系统通过从MuJoCo仿真环境中提取车辆实时数据（速度、转速、油量等），使用OpenGL渲染技术将这些数据以直观的仪表盘形式显示在3D仿真场景中。项目包含了完整的C++实现，包括数据提取模块、仪表盘渲染模块，并集成到MuJoCo MPC的主渲染流程中。

环境要求

操作系统: Ubuntu 22.04 LTS

编译器: gcc 11.3.0

CMake: 3.22.1

MuJoCo MPC: 最新版本

依赖库: OpenGL, GLFW, Eigen, OpenBLAS



## 编译和运行

### 编译步骤

```
# 克隆MuJoCo MPC项目
cd ~
git clone https://github.com/google-deepmind/mujoco_mpc.git
cd mujoco_mpc
```

```
# 复制我的代码文件到相应目录
cp path_to_my_files/dashboard_data.h mjpc/
cp path_to_my_files/dashboard_render.h mjpc/
cp path_to_my_files/dashboard_render.cc mjpc/

# 修改CMakeLists.txt添加新文件（如果需要）
# 编辑mjpc/CMakeLists.txt，添加dashboard_render.cc到源文件列表

# 编译项目
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j4
```

### 运行

```
# 运行带有仪表盘的汽车场景
./bin/mjpc --mjcf=../mjpc/tasks/car/task.xml
```

## 功能说明

### 已实现功能

    速度表：实时显示车辆速度（0-240 km/h），带颜色分段指示
    
    转速表：模拟发动机转速（0-8000 RPM），带安全区域提示
    
    油量表：显示剩余油量（0-100%），带低油量警告
    
    实时数据更新：从MuJoCo仿真中获取速度数据并实时更新
    
    美观的UI设计：圆形仪表盘、彩色刻度、指针动画效果

进阶功能

    UI美化：纹理效果的仪表盘、平滑的指针动画
    
    警告提示：转速过高红色警告、油量过低黄色警告
    
    数据模拟：模拟油量消耗、发动机转速与速度的关系



文件说明

    dashboard_data.h: 仪表盘数据结构定义和声明
    
    dashboard_render.h: 仪表盘渲染类定义和声明
    
    dashboard_render.cc: 仪表盘渲染具体实现
    
    car_model.xml: 自定义车辆模型文件
    
    task.xml: 包含MPC任务的场景配置文件

已知问题

    仪表盘位置目前固定在屏幕中心，未完全跟随车辆位置
    
    在某些低配机器上可能帧率较低
    
    字体渲染功能受限，仅使用MuJoCo内置文本渲染

参考资料

    MuJoCo官方文档: https://mujoco.readthedocs.io/
    
    MuJoCo MPC GitHub仓库: https://github.com/google-deepmind/mujoco_mpc
    
    OpenGL教程: https://learnopengl.com/
    
    C++编程指南
