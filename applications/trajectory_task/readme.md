1. 创建一个独立的FreeRtos任务，Entry Function 为 trajectory_task
2. 将trajectory_task.cpp, trajectory_task.hpp放在applications\路径下
3. 编辑CMakeLists.txt
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    applications/trajectory_task.cpp # <- 添加这一行
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    applicaitons/ # <- 添加这一行
)
```
4. 烧录程序后，在plotter_task中打印以下变量，观察曲线
   
```cpp
plotter.plot(yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc);
```
其中，yaw和pitch为云台需要跟踪的角度曲线，vel速度项、acc加速度项可选择性使用

5. 注意代码中的变量禁止与yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc重名
6. 注意单位：角度 rad、速度 rad/s、加速度 rad/s^2