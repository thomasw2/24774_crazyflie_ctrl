import numpy as np
import rosbag
import matplotlib.pyplot as plt

if __name__ == "__main__":
    bag_kf = rosbag.Bag('rosbags/exp1_defc_defe_tile.bag')
    pose_x_kf = []
    pose_y_kf = []
    max_points = 500
    for topic, msg, t in bag_kf.read_messages(topics=['/cf1/pose/']):
        pose_x_kf.append(msg.pose.position.x)
        pose_y_kf.append(msg.pose.position.y)
    pose_x_kf_var = np.var(pose_x_kf[0:max_points])
    pose_y_kf_var = np.var(pose_y_kf[0:max_points])

    print(len(pose_x_kf))

    bag_rd = rosbag.Bag('rosbags/pure_rd_cov.bag')
    pose_x_rd = []
    pose_y_rd = []
    for topic, msg, t in bag_rd.read_messages(topics=['/cf1/pose/']):
        pose_x_rd.append(msg.pose.position.x)
        pose_y_rd.append(msg.pose.position.y)
    pose_x_rd_var = np.var(pose_x_rd)
    pose_y_rd_var = np.var(pose_y_rd)

    kf_x_weight = pose_x_rd_var/(pose_x_rd_var + pose_x_kf_var)
    kf_y_weight = pose_y_rd_var/(pose_y_rd_var + pose_y_kf_var)

    rd_x_weight = pose_x_kf_var/(pose_x_rd_var + pose_x_kf_var) 
    rd_y_weight = pose_y_kf_var/(pose_y_rd_var + pose_y_kf_var) 

    print("kf_x_weight", kf_x_weight)
    print("kf_y_weight", kf_y_weight)
    print("")
    print("rd_x_weight", rd_x_weight)
    print("rd_y_weight", rd_y_weight)
    print("")
    print("pose_x_kf_var", pose_x_kf_var)
    print("pose_y_kf_var", pose_y_kf_var)
    print("")
    print("pose_x_rd_var", pose_x_rd_var)
    print("pose_y_rd_var", pose_y_rd_var)