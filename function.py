"""
Author: Jun Cheng
Date: 2024.4.6
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

def visualize_transformed_cloud(csv_path, Pcd_path_1, Pcd_path_2):
    """_summary_

    Args:
        csv_path (路径): 保存旋转矩阵的路径
        source_points (源点云): 源点云
        target_pcd (目标点云): 目标点云
    """
    # 目标点云和源点云
    target_pcd = open3d.io.read_point_cloud(Pcd_path_1)
    target_points = np.asarray(target_pcd.points)
    source_pcd = open3d.io.read_point_cloud(Pcd_path_2)
    source_points = np.asarray(source_pcd.points)

    aabb1 = target_pcd.get_axis_aligned_bounding_box()   # 用一个轴对称方框把点云框起来；
    min_bound = aabb1.get_min_bound()
    max_bound = aabb1.get_max_bound()

    print("target_pcd Min bound:", min_bound)
    print("target_pcd Max bound:", max_bound)

    aabb2 = source_pcd.get_axis_aligned_bounding_box()   # 用一个轴对称方框把点云框起来；
    min_bound = aabb2.get_min_bound()
    max_bound = aabb2.get_max_bound()

    print("source_pcd Min bound:", min_bound)
    print("source_pcd Max bound:", max_bound)

    aabb1.color = (0, 0, 1) # 蓝色
    aabb2.color = (0, 1, 0) # 绿色

    mesh_frame = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])  # 生成坐标系，方便之后的裁剪；

    # 读取齐次旋转平移矩阵
    Homogeneous_Transformation_Matrix = pd.read_csv(csv_path, header=None).values

    # 将源点云转换为齐次坐标
    source_cloud_homogeneous = np.hstack((source_points, np.ones((source_points.shape[0], 1))))

    # 使用齐次旋转平移矩阵对源点云进行变换
    transformed_source_cloud = np.dot(source_cloud_homogeneous, Homogeneous_Transformation_Matrix.T)

    # 去除齐次坐标的最后一列，得到变换后的源点云
    transformed_source_cloud = transformed_source_cloud[:, :3]

    # 创建一个Point_Cloud对象
    transformed_source_cloud_pcd = open3d.geometry.PointCloud()

    # 将numpy数组转换为Point_Cloud
    transformed_source_cloud_pcd.points = open3d.utility.Vector3dVector(transformed_source_cloud)

    # 设置点云的颜色
    transformed_source_cloud_pcd.paint_uniform_color([1, 0, 0])  # 红色
    source_pcd.paint_uniform_color([0, 1, 0])  # 绿色
    target_pcd.paint_uniform_color([0, 0, 1])  # 蓝色

    # 创建一个Visualizer对象
    vis = open3d.visualization.Visualizer()
    
    # 创建窗口，设置窗口标题
    vis.create_window(window_name="point_cloud")
    # 设置点云渲染参数
    opt = vis.get_render_option()
    # 设置背景色（这里为白色）
    opt.background_color = np.array([255, 255, 255])
    # 设置渲染点的大小
    opt.point_size = 2.0

    # 添加几何体到Visualizer
    vis.add_geometry(transformed_source_cloud_pcd)
    vis.add_geometry(target_pcd)
    vis.add_geometry(source_pcd)
    vis.add_geometry(aabb2)
    vis.add_geometry(aabb1)
    vis.add_geometry(mesh_frame)

    # 运行Visualizer
    vis.run()

    # 销毁Visualizer窗口
    vis.destroy_window()


def transform_and_visualize(Csv_path_1, Csv_path_2, Pcd_path_1, Pcd_path_2, Z_angle, X_angle, Y_angle):
    
    points_to_pcd(csv_path=Csv_path_1, pcd_path=Pcd_path_1)
    points_to_pcd(csv_path=Csv_path_2, pcd_path=Pcd_path_2)


    # 目标点云和源点云
    target_pcd = open3d.io.read_point_cloud(Pcd_path_1)
    target_points = np.asarray(target_pcd.points)
    source_pcd = open3d.io.read_point_cloud(Pcd_path_2)
    source_points = np.asarray(source_pcd.points)

    R = euler_to_rotation_matrix(Z_angle, X_angle, Y_angle)
    T_accompany= calculate_accompany_t(X_angle,Y_angle)
    T = np.asarray([0, 0, 0]) + T_accompany

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
    # 添加点云到窗口

    """"""""
    vis.add_geometry(source_pcd)
    vis.add_geometry(target_pcd)
    vis.add_geometry(pcd_transform)
    vis.run()
    vis.destroy_window()


def save_homogeneous_matrix(csv_path, Z_angle, X_angle, Y_angle):
    R = euler_to_rotation_matrix(Z_angle, X_angle, Y_angle)
    T_accompany= calculate_accompany_t(X_angle,Y_angle)
    T = np.asarray([0, 0, 0]) + T_accompany
    np.vstack((R, [0, 0, 0]))
    np.hstack((T, np.array([1])))
    Homogeneous_Transformation_Matrix = np.hstack((np.vstack((R, [0, 0, 0])), np.hstack((T, np.array([1])))[:, np.newaxis]))
    df = pd.DataFrame(Homogeneous_Transformation_Matrix)
    df.to_csv(csv_path, index=False, header=False)
    print(f'已经保存齐次旋转平移矩阵{csv_path}')


def croppointcloud(Pcd_path, min_bound, max_bound, save_path, should_save = False):
    
    
    cropped_box = open3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    """
    下面这段可以改进一下
    """
    pcd = open3d.io.read_point_cloud(Pcd_path)
    cropped_point_cloud = pcd.crop(cropped_box)
    pcd = open3d.io.read_point_cloud(Pcd_path)
    mesh_frame = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])
    """
    实现可视化效果，从而选择是不是要对点云进行裁剪
    注：坐标轴X，Y，Z分别对应颜色红，绿，蓝
    """
    pcd.paint_uniform_color([0, 1, 0])  # 绿色
    cropped_point_cloud.paint_uniform_color([0, 0, 1])  # 蓝色
    cropped_box.color = (1, 0, 0)  # 红色
    vis = open3d.visualization.Visualizer()
    vis.create_window(window_name="point_cloud")
    opt = vis.get_render_option()
    opt.background_color = np.array([255, 255, 255])
    opt.point_size = 2.0
    vis.add_geometry(pcd)
    vis.add_geometry(cropped_box)
    vis.add_geometry(cropped_point_cloud)
    vis.add_geometry(mesh_frame)
    vis.run()
    # 销毁Visualizer窗口
    vis.destroy_window()
    if should_save:
        open3d.io.write_point_cloud(save_path, cropped_point_cloud)