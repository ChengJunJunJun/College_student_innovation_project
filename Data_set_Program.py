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
请输入数据
"""
Z_angle = 10  # 偏航角Z
X_angle = 15  # 滚转角X
Y_angle = 15  # 俯仰角Y

# 请注意，第一个点云是目标点云
Csv_path_1 = 'test/test1.csv'
Pcd_path_1 = 'test/test1.pcd'
# 第二个点云是源点云
Csv_path_2 = 'test/test2.csv'
Pcd_path_2 = 'test/test2.pcd'
# 保存旋转矩阵
Homogeneous_Transformation_Matrix_path = 'test/1-2.csv'




"""
1, 第一个函数，实现单纯的可视化，看看能不能配起来
2，第二个函数，实现保存可以配起来的齐次平移旋转矩阵
3，第三个函数，可视化保存下来的矩阵乘源点云，用于测试
"""
# transform_and_visualize(Csv_path_1, Csv_path_2, Pcd_path_1, Pcd_path_2, Z_angle, X_angle, Y_angle)
# save_homogeneous_matrix(Homogeneous_Transformation_Matrix_path, Z_angle, X_angle, Y_angle)
visualize_transformed_cloud(Homogeneous_Transformation_Matrix_path, Pcd_path_1, Pcd_path_2)