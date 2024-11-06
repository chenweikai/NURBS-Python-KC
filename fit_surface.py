#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    Examples for the NURBS-Python Package
    Released under MIT License
    Developed by Onur Rauf Bingol (c) 2018

    A cubic-quadratic Bezier surface
"""

import numpy as np
import os
import pdb

import trimesh
import open3d as o3d

from geomdl import BSpline
from geomdl import utilities
from geomdl.visualization import VisVTK


# Fix file path
os.chdir(os.path.dirname(os.path.realpath(__file__)))

# Create a BSpline surface instance (Bezier surface)
surf = BSpline.Surface()

# load mesh as control points
mesh_name = "./data/simplied_closet.ply"
mesh = trimesh.load(mesh_name, force='mesh', process=True, maintain_order=False)

verts = np.array(mesh.vertices[:730])
verts = verts.tolist()

# load pcd file from PCL library () as control points
pcd_dir = "./data/bunny.pcd"
pcd = o3d.io.read_point_cloud(pcd_dir)
pcd_pts = np.asarray(pcd.points)
verts = pcd_pts[:395]
verts = verts.tolist()
# pdb.set_trace()

# Set up the Bezier surface
surf.degree_u = 3
surf.degree_v = 2
# control_points = [[0, 0, 0], [0, 4, 0], [0, 8, -3],
#                   [2, 0, 6], [2, 4, 0], [2, 8, 0],
#                   [4, 0, 0], [4, 4, 0], [4, 8, 3],
#                   [6, 0, 0], [6, 4, -3], [6, 8, 0]]
# surf.set_ctrlpts(control_points, 4, 3)
# pdb.set_trace()

control_points = verts
surf.set_ctrlpts(control_points, 79, 5)
surf.knotvector_u = utilities.generate_knot_vector(surf.degree_u, surf.ctrlpts_size_u)
surf.knotvector_v = utilities.generate_knot_vector(surf.degree_v, surf.ctrlpts_size_v)


# Set sample size
surf.sample_size = 25

# Evaluate surface
surf.evaluate()

# import pdb
# pdb.set_trace()

# Plot the control point grid and the evaluated surface
vis_comp = VisVTK.VisSurface()
surf.vis = vis_comp
surf.render()

pass
