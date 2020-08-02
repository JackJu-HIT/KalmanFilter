# KalmanFilter
基于自己之前搭好的ROS机器人平台，自己写了个卡尔曼滤波算法对IMU的加速度xyz进行滤波处理。
涉及到eigen库
# 架构
他会接受/imu话题的imu信息
发布/imu_kalman的imu信息（处理后的加速度信息xyz）
# 环境要求
你需要配置ROS环境，
# 运行方法
rosrun kalman_filter kalman_filter 
# 具体细节
可以参见我的博客

