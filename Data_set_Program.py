"""
Author: Jun Cheng
Date: 2024.4.1

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


# 对点云文件的格式转换
def points_to_pcd(csv_path, pcd_path):
    # 读取 CSV 文件并提取第 8、9、10 列数据
    data = pd.read_csv(csv_path, usecols=[7, 8, 9])

    # 将数据写入 PCD 文件
    with open(pcd_path, 'w') as f:
        # 写入 PCD 文件的头部信息
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write("WIDTH {}\n".format(len(data)))
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write("POINTS {}\n".format(len(data)))
        f.write("DATA ascii\n")

        # 写入点云数据
        for i in range(len(data)):
            f.write("{} {} {}\n".format(data.iloc[i, 0], data.iloc[i, 1], data.iloc[i, 2]))

    print("PCD 文件已创建")


# 完成对旋转矩阵的求解
def euler_to_rotation_matrix(yaw, roll, pitch):
    # Convert degrees to radians
    yaw_rad = np.radians(yaw)   # z
    pitch_rad = np.radians(pitch)  # y
    roll_rad = np.radians(roll) # x

    # Calculate rotation matrix
    r_z = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0],
                    [np.sin(yaw_rad), np.cos(yaw_rad), 0],
                    [0, 0, 1]])

    r_x = np.array([[1, 0, 0],
                    [0, np.cos(roll_rad), -np.sin(roll_rad)],
                    [0, np.sin(roll_rad), np.cos(roll_rad)]])

    r_y = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                    [0, 1, 0],
                    [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])

    # Combine the rotations in ZXY order
    
    r = np.dot(r_z, np.dot(r_x, r_y))
    return r

def calculate_accompany_t(alpha,beta):
    """
    alpha: 绕X轴的角度，注意正负
    beta: 绕Y轴的角度，注意正负
    """
    angle_x = np.radians(alpha)
    angle_y = np.radians(beta)
    t = np.array([np.sin(angle_y) * 0.1014, -((0.1014*np.cos(angle_y)+0.015)*np.sin(angle_x)), -(0.11640-((0.1014*np.cos(angle_y)+0.015)*np.cos(angle_x)))])
    return t

# 点云变换函数
def transform(pc, r, t: object = None) -> object:
    pc = np.dot(pc, r.T)
    if t is not None:
        pc = pc + t
    return pc


"""
下面是要输入的路径，请输入这四个；
"""
Csv_path_1 = 'test/test1.csv'
Pcd_path_1 = 'test/test1.pcd'
Csv_path_2 = 'test/test2.csv'
Pcd_path_2 = 'test/test2.pcd'
points_to_pcd(csv_path=Csv_path_1, pcd_path=Pcd_path_1)
points_to_pcd(csv_path=Csv_path_2, pcd_path=Pcd_path_2)

Z_angle = 10  # 偏航角Z
X_angle = 15  # 滚转角X
Y_angle = 15  # 俯仰角Y

rotation_matrix = euler_to_rotation_matrix(Z_angle, X_angle, Y_angle)

R = rotation_matrix
T_accompany= calculate_accompany_t(X_angle,Y_angle)
T = np.asarray([0, 0, 0]) + T_accompany

"""
保存齐次矩阵；首先需要可视化一下，看看配准效果，之后保存转换矩阵到指定位置；
"""
np.vstack((R, [0, 0, 0]))
np.hstack((T, np.array([1])))
Homogeneous_Transformation_Matrix = np.hstack((np.vstack((R, [0, 0, 0])), np.hstack((T, np.array([1])))[:, np.newaxis]))
df = pd.DataFrame(Homogeneous_Transformation_Matrix)
# df.to_csv('Aircraft_tail/Tail_1_to_Tail_2.csv', index=False, header=False)

# 目标点云
target_pcd = open3d.io.read_point_cloud(Pcd_path_1)
target_points = np.asarray(target_pcd.points)

# 源点云
source_pcd = open3d.io.read_point_cloud(Pcd_path_2)
source_points = np.asarray(source_pcd.points)

# 将源点云变换到目标点云坐标系
transform_points = transform(source_points, R, T)
pcd_transform = open3d.geometry.PointCloud()
pcd_transform.points = open3d.utility.Vector3dVector(transform_points)

# 分别定义点云的颜色
source_pcd.paint_uniform_color([0, 1, 0])  # 绿色
target_pcd.paint_uniform_color([0, 0, 1])  # 蓝色
pcd_transform.paint_uniform_color([1, 0, 0])  # 红色

# 创建窗口对象
vis = open3d.visualization.Visualizer()
# 创建窗口，设置窗口标题
vis.create_window(window_name="point_cloud")
# 设置点云渲染参数
opt = vis.get_render_option()
# 设置背景色（这里为白色）
opt.background_color = np.array([255, 255, 255])
# 设置渲染点的大小
opt.point_size = 2.0
# 添加点云
""""""""
aabb = target_pcd.get_axis_aligned_bounding_box()   # 用一个轴对称方框把点云框起来；
box_points = aabb.get_box_points()
print(np.asarray(box_points))

aabb.color = (1, 0, 0)
mesh_frame = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])  # 生成坐标系，方便之后的裁剪；

""""""""
vis.add_geometry(source_pcd)
vis.add_geometry(aabb)
vis.add_geometry(mesh_frame)
vis.add_geometry(target_pcd)
vis.add_geometry(pcd_transform)
vis.run()
