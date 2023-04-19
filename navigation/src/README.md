# 配置
1. 将两个 py 文件放入 navigation/src 下
2. 在 navigation/src 文件夹下右键打开终端，分别授予两个 py 文件执行权限： 
    chmod +x track_points.py
    chmod +x follow_points.py
# 使用 
1. 记录点坐标
    - 按照导航.txt 中内容启动所有小车导航所需节点
    - 终端中运行 rosrun navigation track_points.py，首先需要输入文件名，该文件名将用来保存点的信息，文件将被保存在 navigation/src/results 下
    - 使用遥控器控制小车前进，到达目标位置先使小车停止，然后在上面运行 rosrun 的终端中输入 1 即可记录位置
    - 记录结束输入 0 结束程序运行

2. 按照记录导航
    - 按照导航.txt 中内容启动所有小车导航所需节点
    - 在终端中输入 rosrun navigation follow_points.py _filename:= 然后在 navigation/src/results 下将所需要执行的点记录文件拖入到终端中，按回车开始执行
    - 正常情况下小车将开始导航