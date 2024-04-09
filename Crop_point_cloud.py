"""
Author: Jun Cheng
Date: 2024.3.15

"""
import numpy as np
import open3d as o3d

from function import croppointcloud, visualize_transformed_cloud



"""
保存下来处理好的点云文件
"""
Pcd_path_1 = 'test/test1.pcd' # 目标点云
Pcd_path_2 = 'test/test2.pcd' # 源点云
Homogeneous_Transformation_Matrix_path = 'test/1-2.csv'



# visualize_transformed_cloud(Homogeneous_Transformation_Matrix_path, Pcd_path_1, Pcd_path_2)
# croppointcloud(Pcd_path_2, np.array([-0.616 ,-2.083 ,-0.534]), np.array([ 0.26  ,-1.416  ,0.491]), 'test/test2_cropped.pcd', should_save = False)