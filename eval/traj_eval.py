import numpy as np
import argparse


from eval_traj_utils import read_kitti_format_poses,read_kitti_format_calib,apply_kitti_format_calib,absolute_error,relative_error


def main(gt_file, preb_file, calib_file):
    calib = read_kitti_format_calib(calib_file)
    gt_poses = read_kitti_format_poses(gt_file)
    preb_poses = read_kitti_format_poses(preb_file)
    preb_poses = apply_kitti_format_calib(preb_poses, calib["Tr"])
    rot_rmse, tran_rmse, align_mat = absolute_error(np.array(gt_poses), np.array(preb_poses))
    drift_ate, drift_are = relative_error(np.array(gt_poses), np.array(preb_poses))
    print("Average Translation Error       (%):",drift_ate)
    print("Average Rotational Error (deg/100m):",drift_are*100)
    print("Absoulte Trajectory Error       (m):",tran_rmse)
    print("Absoulte Rotation Error       (deg):",rot_rmse)






if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='KITTI Evaluation toolkit')
    parser.add_argument('--gt_file',     type=str, default='./ground_truth_pose',  help='file of the ground truth odometry')
    parser.add_argument('--pred_file',     type=str, default='./pred_pose',  help='file path of the ground truth odometry')
    parser.add_argument('--calib_file',type=str, default='calib.txt')
    args = parser.parse_args()

    main(args.gt_file, args.pred_file, args.calib_file)