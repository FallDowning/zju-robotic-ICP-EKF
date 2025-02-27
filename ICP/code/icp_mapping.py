import numpy as np
import open3d as o3d
import icp

def load_point_cloud(filename):
    """加载点云数据"""
    return o3d.io.read_point_cloud(filename)

def merge_point_clouds(laser_map, curr_ply,tolerance):
    """合并点云，只保留与已存在点云中距离大于0.01的点"""
    # 将当前点云转换为numpy数组
    curr_ply_array = curr_ply

    # 如果 laser_map 为空，则直接返回 curr_ply
    if laser_map.size == 0:
        return curr_ply_array
    # 初始化一个空的列表，用于存储合并后的点
    merged_points = []
    # 遍历当前点云中的每一个点
    for point in curr_ply_array:
        # 计算当前点与激光地图中的所有点的距离
        distances = np.linalg.norm(laser_map - point, axis=1)
        
        # 如果距离大于0.01，则将该点添加到合并列表
        if np.all(distances > tolerance):
            merged_points.append(point)

    # 将列表转换为numpy数组并合并
    merged_points = np.vstack((laser_map, np.array(merged_points)))

    return merged_points

def main():
    # 读取初始点云
    laser_map = load_point_cloud('0.ply')

    laser_map_points = np.asarray(laser_map.points)[:, :2]  # 只保留 x 和 y 坐标
    init_points = laser_map_points
    
    tform = np.eye(3)
    transform_all = []
    tform_real = np.eye(3)
    trajectory_points = []
    trajectory_points.append(tform[:2, 2])
    for i in range(1, 10):

        # 读取当前点云
        curr_ply = load_point_cloud(f'{i}.ply')
        curr_points = np.asarray(curr_ply.points)[:, :2]  # 只保留 x 和 y 坐标
        
        tform = np.dot(tform_real,tform)

        # 为了当前帧和第0帧匹配的准确性，需要将当前帧的点云做一些变换（本质相当于将第0帧融合后的点云变换到i-1帧坐标系下）
        curr_points = np.dot(tform[:2, :2], curr_points.T).T + tform[:2, 2]


        # ICP算法进行当前点云和第0帧点云的配准
        tform_real = icp.icp(curr_points, laser_map_points)

        # 记录机器人相对于世界坐标系的位姿
        transform_all.append(tform)
        # 记录机器人不同帧的位置
        trajectory_points.append(tform[:2, 2])  

        # 将当前帧点云变换到世界坐标系下
        transformed_points = np.dot(tform_real[:2, :2], curr_points.T).T + tform_real[:2, 2]

        # 合并点云
        laser_map_points = merge_point_clouds(laser_map_points, transformed_points, 0.01)

    tform = np.dot(tform_real,tform)
    # 记录机器人相对于世界坐标系的位姿
    transform_all.append(tform)
    # 记录机器人不同帧的位置
    trajectory_points.append(tform[:2, 2])  

     # 将合并后的点云转换为三维格式
    laser_map_points_3d = np.hstack((laser_map_points, np.zeros((laser_map_points.shape[0], 1))))

    # 创建Open3D点云对象
    
    laser_map_pcd = o3d.geometry.PointCloud()
    laser_map_pcd.points = o3d.utility.Vector3dVector(laser_map_points_3d)
    laser_map_pcd.paint_uniform_color([0, 0, 1])  # 蓝色表示激光地图
    # 显示合并后的点云
    o3d.visualization.draw_geometries([laser_map_pcd])

    # 轨迹点
    trajectory_points_3d = np.array([[x, y, 0] for x, y in trajectory_points])

    # 创建轨迹点云对象
    trajectory_pcd = o3d.geometry.PointCloud()
    trajectory_pcd.points = o3d.utility.Vector3dVector(trajectory_points_3d)
    trajectory_pcd.paint_uniform_color([1, 0, 0])  # 红色表示轨迹点

    # 创建 LineSet 对象以连接轨迹点
    lines = [[i, i + 1] for i in range(len(trajectory_points_3d) - 1)]
    colors = [[1, 0, 0] for _ in range(len(lines))]  # 红色表示轨迹线条

    # 创建 LineSet，并设置点和线条
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(trajectory_points_3d)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    # 显示轨迹点云和连线
    o3d.visualization.draw_geometries([laser_map_pcd, trajectory_pcd, line_set], window_name="Point Cloud with Connected Trajectory")

    for i in range(10):
        print (f"第{i}帧位姿：",transform_all[i])
    
if __name__ == "__main__":
    main()
