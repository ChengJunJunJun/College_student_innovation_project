"""
Author: Jun Cheng
Date: 2024.4.6

此程序可以实现以下功能：
1，将点云格式从.csv转换成.pcd文件并保存
2，求解旋转矩阵并保存，方便构建数据集
3，利用旋转矩阵实现配准并可视化

更新：
完成由于旋转自动产生的位移矩阵计算；

注意！！！！ 必须先绕Z轴旋转，之后是X，Y；

是将目标点云配准到源点云，所以测量的第一个是目标点云，第二个才是源点云。
"""

import open3d
import pandas as pd
import numpy as np
from function import euler_to_rotation_matrix, \
    calculate_accompany_t, \
    points_to_pcd, \
    visualize_transformed_cloud, \
    transform_and_visualize, \
    save_homogeneous_matrix

"""
下面是要输入的路径，请输入这五个；
"""
Csv_path_1 = 'test/test1.csv'
Pcd_path_1 = 'test/test1.pcd'
Csv_path_2 = 'test/test2.csv'
Pcd_path_2 = 'test/test2.pcd'
Homogeneous_Transformation_Matrix_path = 'test/test12.csv'
points_to_pcd(csv_path=Csv_path_1, pcd_path=Pcd_path_1)
points_to_pcd(csv_path=Csv_path_2, pcd_path=Pcd_path_2)

Z_angle = 10  # 偏航角Z
X_angle = 15  # 滚转角X
Y_angle = 15  # 俯仰角Y

R = euler_to_rotation_matrix(Z_angle, X_angle, Y_angle)
T_accompany= calculate_accompany_t(X_angle,Y_angle)
T = np.asarray([0, 0, 0]) + T_accompany

# 目标点云
target_pcd = open3d.io.read_point_cloud(Pcd_path_1)
target_points = np.asarray(target_pcd.points)

# 源点云
source_pcd = open3d.io.read_point_cloud(Pcd_path_2)
source_points = np.asarray(source_pcd.points)



# 用于可视化
transform_and_visualize(source_points, source_pcd, target_pcd, R, T)

"""
保存齐次矩阵；首先需要可视化一下，看看配准效果，之后保存转换矩阵到指定位置；
"""
# save_homogeneous_matrix(R, T, Homogeneous_Transformation_Matrix_path)
# visualize_transformed_cloud(Homogeneous_Transformation_Matrix_path, source_points, target_pcd)