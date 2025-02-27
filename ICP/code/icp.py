import numpy as np

def compute_transformation(source, target):
    """计算点云之间的变换矩阵"""
    # 计算中心坐标
    centroid_source = np.mean(source, axis=0)
    centroid_target = np.mean(target, axis=0)

    # 去中心化
    source_centered = source - centroid_source
    target_centered = target - centroid_target

    # 计算协方差矩阵
    covariance_matrix = np.dot(source_centered.T, target_centered)

    # SVD分解
    U, _, Vt = np.linalg.svd(covariance_matrix)

    # 计算旋转矩阵
    rotation = np.dot(Vt.T, U.T)

    # 计算平移
    translation = centroid_target - np.dot(rotation, centroid_source)

    # 构造变换矩阵
    transformation = np.eye(3)
    transformation[:2, :2] = rotation
    transformation[:2, 2] = translation[:2]
    return transformation

def nearest_neighbors(source, target):
    """找到目标点云中与源点云最近的点"""
    distances = np.linalg.norm(target[:, None] - source, axis=2)  # 计算距离
    indices = np.argmin(distances, axis=0)  # 找到最近点的索引
    return indices

def icp(source, target, max_iterations=100, tolerance=1e-6):
    """实现ICP算法"""
    prev_error = float('inf')
    transformation_matrix = np.eye(3)

    for i in range(max_iterations):
        # 1. 在目标点云中找到源点云的最近点
        source_temp = np.dot(transformation_matrix[:2, :2], source.T).T + transformation_matrix[:2, 2]

        indices = nearest_neighbors(source_temp, target)  # 找到最近邻索引

        # 2. 根据最近点的索引来找到匹配的点
        matched_points_source = source_temp
        matched_points_target = target[indices]

        # 3. 计算变换矩阵
        transformation = compute_transformation(matched_points_source, matched_points_target)

        # 4. 更新变换矩阵
        transformation_matrix = np.dot(transformation, transformation_matrix)

        # 5. 计算当前的平均误差
        distances = np.linalg.norm(matched_points_source - matched_points_target, axis=1)
        current_error = np.mean(distances)
        if np.abs(current_error-prev_error) < tolerance:
            break
        prev_error = current_error
        
    return transformation_matrix