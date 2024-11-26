'''
    Code to load mesh obj with UV map and generate independent parts based on UV maps.
'''

import numpy as np
import pdb
import os
from plyfile import PlyData, PlyElement

def save_verts_to_ply(verts, save_path):
    out_file = open(save_path, 'w')

    head_strs = ['ply\n', 'format ascii 1.0\n']
    ele_1 = ['element vertex ' + str(verts.shape[0]) + '\n',
             'property float x\n',
             'property float y\n',
             'property float z\n',
             'property uchar red\n',
             'property uchar green\n',
             'property uchar blue\n']
    ele_2 = ['element face ' + str(0) + '\n',
             'property list uchar int vertex_index\n']
    tail_strs = ['end_header\n']

    # Write Header
    out_file.writelines(head_strs)
    out_file.writelines(ele_1)
    out_file.writelines(ele_2)
    out_file.writelines(tail_strs)

    ##############
    # Write output
    ##############
    # First, write vertex positions
    for i in range(verts.shape[0]):
        out_file.write(str(verts[i][0]) + ' ')
        out_file.write(str(verts[i][1]) + ' ')
        out_file.write(str(verts[i][2]) + ' ')

    out_file.close()


def load_mesh_with_uv_from_obj(path: str, is_quad=False):
    obj_file = open(path, 'r')
    verts = []
    faces = []
    uvs = []
    uv_faces = []
    face_num = 3
    if is_quad:
        face_num = 4
    print(f'Begin to load mesh obj file: {path}')
    while True:
        line = obj_file.readline()
        line = line.replace('\n', '')
        if len(line) == 0:
            break
        if line.startswith('v '):
            strs = line.split(' ')
            v = [float(strs[idx]) for idx in range(1, 4)]
            verts.append(v)
        elif line.startswith('vt '):
            strs = line.split(' ')
            vt = [float(strs[idx]) for idx in range(1, 3)]
            uvs.append(vt)
        elif line.startswith('f '):
            strs = line.split(' ')
            fv = []
            fuv = []
            for idx in range(1, face_num+1):
                str = strs[idx]
                sstr = str.split('/')
                fv.append(int(sstr[0]) - 1)
                fuv.append(int(sstr[1]) - 1)
            faces.append(fv)
            uv_faces.append(fuv)
    obj_file.close()
    print(f'Complete to load mesh obj file: {path}')
    return verts, faces, uvs, uv_faces


def get_independent_component(v_graph):
    comps = []
    visited = [False] * len(v_graph)
    for begin_v in range(len(v_graph)):
        if visited[begin_v]:
            continue
        visited[begin_v] = True
        que = []
        comp = {begin_v: v_graph[begin_v]}
        que.append(begin_v)
        while len(que) > 0:
            v = que.pop()
            neighs = v_graph[v]
            for neigh in neighs:
                if visited[neigh]:
                    continue
                visited[neigh] = True
                comp[neigh] = v_graph[neigh]
                que.append(neigh)
        # print(f'#vertex in component {len(comps)}:', len(comp))
        comps.append(comp)
    return comps


def get_v_graph(v_num, edges):
    v_graph = [None] * v_num
    for e_id in range(edges.shape[0]):
        edge = edges[e_id]
        if v_graph[edge[0]] is None:
            v_graph[edge[0]] = set()
        v_graph[edge[0]].add(edge[1])
        if v_graph[edge[1]] is None:
            v_graph[edge[1]] = set()
        v_graph[edge[1]].add(edge[0])
    return v_graph


def get_edges_np(faces):
    if faces.shape[1] == 4:
        edges = np.concatenate(
            [
                faces[:, [0, 1]],
                faces[:, [1, 2]],
                faces[:, [2, 3]],
                faces[:, [3, 0]],
            ],
            axis=0,
        )
    else:
        edges = np.concatenate(
            [
                faces[:, [0, 1]],
                faces[:, [1, 2]],
                faces[:, [2, 0]],
            ],
            axis=0,
        )
    edges.sort()
    edges = np.unique(edges, axis=0)
    return edges


def get_independent_componet_of_uv(uvs: np.ndarray, uv_faces: np.ndarray):
    uv_edge = get_edges_np(uv_faces)
    uv_graph = get_v_graph(len(uvs), uv_edge)
    uv_comps = get_independent_component(uv_graph)
    # pdb.set_trace()
    return uv_comps

# Map the id of uv verts to the id of geometry verts
def get_uv_vert_idx_map(faces, uv_faces, uvs):
    uv_verts_num = len(uvs)
    uv_to_geo_id_map = np.zeros((uv_verts_num), dtype=np.int32)
    for i in range(uv_verts_num):
        for j in range(3):
            uv_to_geo_id_map[uv_faces[i][j]] = faces[i][j]

    return uv_to_geo_id_map


def dump_components_into_pcd(comps, uv_to_geo_id_map, verts, out_folder_dir, min_part_size=1):
    os.makedirs(out_folder_dir, exist_ok=True)
    ply_folder = os.path.join(out_folder_dir, "ply")
    pcd_folder = os.path.join(out_folder_dir, "pcd")
    os.makedirs(ply_folder, exist_ok=True)
    os.makedirs(pcd_folder, exist_ok=True)
    verts = np.asarray(verts)
    for idx, comp in enumerate(comps):
        geo_vids = []
        unique_vert = {}
        for vid in comp:
            geo_vid = uv_to_geo_id_map[vid]
            geo_vids.append(geo_vid)
            unique_vert[geo_vid] = 1

        # Skip part that has very few vertices
        if len(unique_vert) < min_part_size:
            continue
        cur_verts = verts[geo_vids]
        file_ply_name = os.path.join(ply_folder, f"part_{idx}.ply")
        file_pcd_name = os.path.join(pcd_folder, f"part_{idx}.pcd")
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cur_verts)
        vert_colors = np.random.rand(1, 3)
        vert_colors = np.repeat(vert_colors, repeats=cur_verts.shape[0], axis=0)
        pcd.colors = o3d.utility.Vector3dVector(vert_colors)
        o3d.io.write_point_cloud(file_ply_name, pcd)
        o3d.io.write_point_cloud(file_pcd_name, pcd)
        print(f"Saved part vertices to {file_ply_name} and {file_pcd_name}")
        # pdb.set_trace()


if __name__ == "__main__":
    data_dir = "./data/chair1_instantmesh/chair1.obj"
    verts, faces, uvs, uv_faces = load_mesh_with_uv_from_obj(data_dir)
    uv_to_geo_id_map = get_uv_vert_idx_map(faces, uv_faces, uvs)
    # pdb.set_trace()
    uv_faces = np.array(uv_faces)
    uv_comps = get_independent_componet_of_uv(uvs, uv_faces)
    dump_components_into_pcd(uv_comps, uv_to_geo_id_map, verts, "./output/chair1")

