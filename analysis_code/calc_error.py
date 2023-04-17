import os
import numpy as np
import json

def calc_translation_error(rel_pose):
    return np.linalg.norm(rel_pose[:3, 3])

def calc_rot_error(rel_pose):
    tr = np.trace(rel_pose[:3,:3])
    domain = (tr - 1) / 2
    if domain > 1:
        domain = 1.0
    elif domain < -1:
        domain = -1.0
    rot_error = np.arccos(domain) * 180 / np.pi
    # if domain < -1 or domain > 1:
    #     print("DOMAIN ERROR:" + str((tr - 1) / 2))
    #     assert False
    return rot_error

def calc_rel_pose(est_pose, gt_pose):
    # print(gt_pose)
    # print(est_pose)
    # assert False
    rel_pose = np.linalg.inv(gt_pose) @ est_pose
    return rel_pose

def extract_pose(line):
    # Get r and t
    mat = np.eye(4)
    data = line.split()
    for i, d in enumerate(data):
        mat[i // 4, i % 4] = d
    # r = mat[:, :3]
    # t = mat[:, -1]
    return mat

ground_truth_folder = "data_odometry_poses/"
est_folder = "pure_Litamin2/"

gt_files = []
est_files = []
for (dirpath, dirnames, filenames) in os.walk(ground_truth_folder):
    gt_files.extend(filenames)
    break
for (dirpath, dirnames, filenames) in os.walk(est_folder):
    est_files.extend(filenames)
    break

mean_t_error = []
mean_rot_error = []

for file_1, file_2 in zip(gt_files, est_files):
    t_error = []
    rot_error = []
    out_data = []
    print("files:")
    print(file_1, file_2)
    with open(ground_truth_folder + file_1) as f1, open(est_folder + file_2) as f2:
        for line1, line2 in zip(f1,f2):
            # print(line1)
            # Extract pose from each
            gt_pose = extract_pose(line1)
            est_pose = extract_pose(line2)

            rel_pose = calc_rel_pose(est_pose, gt_pose)

            t_error.append(calc_translation_error(rel_pose))
            rot_error.append(calc_rot_error(rel_pose))

    # print(t_error)

    mean_t_error.append((file_1, np.mean(t_error)))
    mean_rot_error.append((file_2, np.mean(rot_error)))

with open("mean_t_errors.json", "w") as f:
    json.dump(mean_t_error, f)

with open("mean_rot_errors.json", "w") as f:
    json.dump(mean_rot_error, f)