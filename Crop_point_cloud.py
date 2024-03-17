"""
Author: Jun Cheng
Date: 2024.3.15

"""
import numpy as np
import open3d as o3d
"""
主要输入部分
"""

Pcd_path = 'Aircraft_tail/Tail_1.pcd'
cropped_box = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.375, -2.89, -0.282), max_bound=(0.74, 0, 1.045))
"""
下面这段可以改进一下
"""
pcd = o3d.io.read_point_cloud(Pcd_path)
cropped_point_cloud = pcd.crop(cropped_box)
pcd = o3d.io.read_point_cloud(Pcd_path)
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])
"""
实现可视化效果，从而选择是不是要对点云进行裁剪
注：坐标轴X，Y，Z分别对应颜色红，绿，蓝
"""
pcd.paint_uniform_color([0, 1, 0])  # 绿色
cropped_point_cloud.paint_uniform_color([0, 0, 1])  # 蓝色
cropped_box.color = (1, 0, 0)  # 红色
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="point_cloud")
opt = vis.get_render_option()
opt.background_color = np.array([255, 255, 255])
opt.point_size = 2.0
vis.add_geometry(pcd)
vis.add_geometry(cropped_box)
vis.add_geometry(cropped_point_cloud)
vis.add_geometry(mesh_frame)
vis.run()

"""
保存下来处理好的点云文件
"""
#o3d.io.write_point_cloud("Aircraft_tail/Tail_1_cropped.pcd", cropped_point_cloud)
