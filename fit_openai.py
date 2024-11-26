import numpy as np
from geomdl import NURBS
from geomdl import fitting
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import trimesh

# 生成示例点云
np.random.seed(0)
num_points = 100
points = np.random.rand(num_points, 3)

mesh_name = "./data/simplied_closet.ply"
mesh = trimesh.load(mesh_name, force='mesh', process=True, maintain_order=False)

verts = np.array(mesh.vertices[:730])
verts = verts.tolist()
points = verts


# 设置控制点的数量和曲面的度数
size_u = 4  # U方向控制点数量
size_v = 4  # V方向控制点数量
degree_u = 2  # U方向的度数
degree_v = 2  # V方向的度数

# 使用最小二乘法近似表面
surface = fitting.approximate_surface(points, size_u, size_v, degree_u, degree_v)

# 评估拟合后的曲面
surface.evaluate()

# 可视化点云和拟合的NURBS曲面
def generate_triangle_mesh(surface, u_steps=10, v_steps=10):
    mesh_points = []
    for i in range(u_steps + 1):
        for j in range(v_steps + 1):
            u = i / u_steps
            v = j / v_steps
            point = surface.evaluate_single((u, v))
            mesh_points.append(point)

    triangles = []
    for i in range(u_steps):
        for j in range(v_steps):
            idx1 = i * (v_steps + 1) + j
            idx2 = idx1 + 1
            idx3 = idx1 + (v_steps + 1)
            idx4 = idx3 + 1
            
            triangles.append((idx1, idx2, idx3))
            triangles.append((idx2, idx4, idx3))

    return np.array(mesh_points), triangles

# 生成三角网格
mesh_points, triangles = generate_triangle_mesh(surface)

# 可视化
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='b', label='Point Cloud')

# 绘制三角形
for tri in triangles:
    tri_points = mesh_points[list(tri)]
    # 使用 Poly3DCollection 来绘制三维多边形
    poly3d = [[tri_points[0], tri_points[1], tri_points[2]]]
    ax.add_collection3d(Poly3DCollection(poly3d, alpha=0.5, edgecolor='g'))

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.show()