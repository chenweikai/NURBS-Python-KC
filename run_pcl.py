'''
    Run PCL Nurbs fitting point cloud from python scripts.
'''

import os
import pdb

from utils import walklevel

pcd_dir = "./output/chair1/pcd"
pcd_files = walklevel(pcd_dir, 0, False)

out_dir = "./output/chair1/recon"
os.makedirs(out_dir, exist_ok=True)

for file in pcd_files:
    base_name = os.path.basename(file)
    subj_name = os.path.splitext(base_name)[0]
    out_recon_name = os.path.join(out_dir, subj_name + ".ply")
    command = f"./pcl_example_nurbs_fitting_surface {file} {out_recon_name}"
    os.system(command)
    # pdb.set_trace()
