import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

# 문 형태의 포인트 클라우드 생성 함수
def generate_door_point_cloud(width=1.0, height=2.0, depth=0.1, n_points=1000, noise_level=0.0):
    """문과 주변 벽을 표현하는 포인트 클라우드를 생성"""
    wall_points = np.random.rand(n_points // 2, 3)
    wall_points[:, 0] = wall_points[:, 0] * (width + 2 * depth) - (width / 2 + depth)
    wall_points[:, 1] = wall_points[:, 1] * height
    wall_points[:, 2] = wall_points[:, 2] * depth - depth

    door_points = np.random.rand(n_points // 2, 3)
    door_points[:, 0] = door_points[:, 0] * width - width / 2
    door_points[:, 1] = door_points[:, 1] * height
    door_points[:, 2] = 0

    cloud = np.vstack([wall_points, door_points])
    cloud += noise_level * np.random.randn(cloud.shape[0], 3)
    
    return cloud

# 포인트 클라우드를 회전시키는 함수
def rotate_point_cloud(cloud, angle_x=0, angle_y=0, angle_z=0):
    """포인트 클라우드를 회전시키는 함수"""
    cos_x, sin_x = np.cos(angle_x), np.sin(angle_x)
    cos_y, sin_y = np.cos(angle_y), np.sin(angle_y)
    cos_z, sin_z = np.cos(angle_z), np.sin(angle_z)
    
    R_x = np.array([[1, 0, 0],
                    [0, cos_x, -sin_x],
                    [0, sin_x, cos_x]])
    
    R_y = np.array([[cos_y, 0, sin_y],
                    [0, 1, 0],
                    [-sin_y, 0, cos_y]])
    
    R_z = np.array([[cos_z, -sin_z, 0],
                    [sin_z, cos_z, 0],
                    [0, 0, 1]])
    
    R = R_z @ R_y @ R_x
    
    return cloud @ R.T

# ICP를 이용한 포인트 클라우드 정렬
def perform_icp(source_points, target_points):
    """ICP를 수행하여 source_points를 target_points에 맞추는 함수"""
    # Open3D 포인트 클라우드 객체 생성
    source = o3d.geometry.PointCloud()
    target = o3d.geometry.PointCloud()
    
    source.points = o3d.utility.Vector3dVector(source_points)
    target.points = o3d.utility.Vector3dVector(target_points)
    
    # ICP 수행
    reg_icp = o3d.pipelines.registration.registration_icp(
        source, target, 0.05, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    
    return reg_icp.transformation, reg_icp.inlier_rmse

# 포인트 클라우드 A와 B 생성
A = generate_door_point_cloud(width=1.0, height=2.0, depth=0.1, n_points=500, noise_level=0.0)
# B = rotate_point_cloud(A, angle_x=np.pi/6, angle_y=np.pi/6, angle_z=np.pi/6)
# B += 0.05 * np.random.randn(*A.shape)
B = generate_door_point_cloud(width=1.2, height=2.3, depth=0.2, n_points=500, noise_level=0.001)

# ICP를 이용한 정렬 수행
transformation, inlier_rmse = perform_icp(A, B)

# 결과 출력
print(f"ICP Transformation Matrix:")
print(transformation)
print(f"Inlier RMSE: {inlier_rmse:.6f}")

# 포인트 클라우드 시각화
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# A 포인트 클라우드 시각화 (파란색)
ax.scatter(A[:, 0], A[:, 1], A[:, 2], c='blue', label='Point Cloud A', alpha=0.6, s=10)

# B 포인트 클라우드 시각화 (빨간색)
ax.scatter(B[:, 0], B[:, 1], B[:, 2], c='red', label='Point Cloud B', alpha=0.6, s=10)

# 그래프 레이블 및 설정
ax.set_title('Visualization of Door-Shaped Point Cloud A and B')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

plt.show()
