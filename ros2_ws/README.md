# ROS2 Workspace

## buildの通し方
0. OpenCV4.8.0をC++用インストール
1. 
```bash
colcon build --packages-select htmd_manager_msgs main_msgs
```
2. 
```bash
source install/setup.bash
```
3. 
```bash
colcon build
```