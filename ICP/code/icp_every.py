import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import icp

def load_point_cloud(filename):
    """加载点云数据"""
    return o3d.io.read_point_cloud(filename)


def visualize_registration(points_0, points_1, transformed_points_1, i):
    """使用 draw_geometries 分别显示配准前和配准后的点云"""
    
    # 配准前
    target_cloud_before = o3d.geometry.PointCloud()
    source_cloud_before = o3d.geometry.PointCloud()
    target_cloud_before.points = o3d.utility.Vector3dVector(points_0)
    target_cloud_before.paint_uniform_color([0, 0, 1])  # 蓝色
    source_cloud_before.points = o3d.utility.Vector3dVector(points_1)
    source_cloud_before.paint_uniform_color([1, 0, 0])  # 红色
    
    print("Displaying point clouds before registration...")
    o3d.visualization.draw_geometries(
        [target_cloud_before, source_cloud_before],
        window_name=f"{i} and {i+1} Before Registration",
        width=800,
        height=600
    )

    # 配准后
    target_cloud_after = o3d.geometry.PointCloud()
    aligned_cloud_after = o3d.geometry.PointCloud()
    target_cloud_after.points = o3d.utility.Vector3dVector(points_0)
    target_cloud_after.paint_uniform_color([0, 0, 1])  # 蓝色
    aligned_cloud_after.points = o3d.utility.Vector3dVector(transformed_points_1)
    aligned_cloud_after.paint_uniform_color([1, 0, 0])  # 绿色
    
    print("Displaying point clouds after registration...")
    o3d.visualization.draw_geometries(
        [target_cloud_after, aligned_cloud_after],
        window_name=f"{i} and {i+1} After Registration",
        width=800,
        height=600
    )



def main():
    transform_all =[]
    for i in range(9):
    # 读取点云
        ply_first = load_point_cloud(f"{i}.ply")
        ply_second = load_point_cloud(f"{i+1}.ply")

        # 提取点云数据并取二维坐标
        points_0 = np.asarray(ply_first.points)[:, :2]
        points_1 = np.asarray(ply_second.points)[:, :2]

        # ICP算法进行相邻点云配准
        tform = icp.icp(points_1, points_0)
        transform_all.append(tform)

        # 应用变换到ply_i+1
        transformed_points_1 = np.dot(tform[:2, :2], points_1.T).T + tform[:2, 2]

        points_0_visual = np.hstack((points_0, np.zeros((points_0.shape[0], 1))))
        points_1_visual = np.hstack((points_1, np.zeros((points_1.shape[0], 1))))
        transformed_points_1_visual = np.hstack((transformed_points_1, np.zeros((transformed_points_1.shape[0], 1))))

        # 在Open3D中显示配准前后的点云
        visualize_registration(points_0_visual, points_1_visual, transformed_points_1_visual, i )

        #input("Press Enter to continue to the next iteration...")
    print(transform_all)
    

if __name__ == "__main__":
    main()
