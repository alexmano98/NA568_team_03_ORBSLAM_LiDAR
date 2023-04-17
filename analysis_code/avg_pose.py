import sys
import os

import numpy as np
import matplotlib.pyplot as plt

def extract_pose(line):
    # Get r and t
    mat = np.zeros((3,4))
    data = line.split()
    for i, d in enumerate(data):
        mat[i // 4, i % 4] = d
    r = mat[:, :3]
    t = mat[:, -1]
    return r, t

def pose_to_line(r, t):
    mat = np.zeros((3,4))
    mat[:,:3] = r
    mat[:,-1] = t
    line = ""
    for i in range(3):
        for j in range(4):
            line += str(mat[i,j]) + " "
    line = line[:-1] + "\n"
    return line
# def isRotationMatrix(M):
#     tag = False
#     I = np.identity(M.shape[0])
#     if np.all((np.matmul(M, M.T)) == I) and (np.linalg.det(M)==1): tag = True
#     return tag    

def isRotationMatrix(R):
    # square matrix test
    if R.ndim != 2 or R.shape[0] != R.shape[1]:
        return False
    should_be_identity = np.allclose(R.dot(R.T), np.identity(R.shape[0], float))
    should_be_one = np.allclose(np.linalg.det(R), 1)
    return should_be_identity and should_be_one

lit_orb_folder = "Litamin2-ORB/"
orb_lit_folder = "ORB-Litamin2/"
out_path = "both/"

lit_orb_files = []
orb_lit_files = []
for (dirpath, dirnames, filenames) in os.walk(lit_orb_folder):
    lit_orb_files.extend(filenames)
    break
for (dirpath, dirnames, filenames) in os.walk(orb_lit_folder):
    orb_lit_files.extend(filenames)


for file_1, file_2 in zip(lit_orb_files, orb_lit_files):
    out_data = []
    print("files:")
    print(file_1, file_2)
    with open(lit_orb_folder + file_1) as f1, open(orb_lit_folder + file_2) as f2:
        for line1, line2 in zip(f1,f2):
            # Extract pose from each
            r1, t1 = extract_pose(line1)
            r2, t2 = extract_pose(line2)

            avg_t = (t1 + t2) / 2

            avg_r = (r1 + r2) / 2
            u, s, vh = np.linalg.svd(avg_r)
            r = u @ vh
            # print(r1)
            # print(r2)
            # print(r)
            assert isRotationMatrix(r)
            # assert False
            out_data.append((r,avg_t))

    f = open(out_path + file_1, "w")
    for r, t in out_data:
        line = pose_to_line(r, t)
        f.write(line)
    f.close()

