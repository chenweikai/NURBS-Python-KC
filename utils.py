import numpy as np
import io
import copy
import os
import pdb
import trimesh
import math
from PIL import Image
import cv2
from plyfile import PlyData, PlyElement


def walklevel(input_dir, depth=1, is_folder=False):
    """Return the files/folders at the specified level under the input directory
    """
    stuff = os.path.abspath(os.path.expanduser(os.path.expandvars(input_dir)))
    output = []
    for root, dirs, files in os.walk(stuff):
        if root[len(stuff):].count(os.sep) == depth:
            if is_folder:
                output.append(root)
            else:
                for f in files:
                    output.append(os.path.join(root,f))
    return output