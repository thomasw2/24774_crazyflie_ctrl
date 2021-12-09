import numpy as np
import rosbag
import matplotlib.pyplot as plt

if __name__ == "__main__":
    #bag_cuse = rosbag.Bag('exp05_cuse.bag')
    #bag_kf = rosbag.Bag('exp05_kf.bag')
    #bag_rd = rosbag.Bag('exp05_rd.bag')

    bag_cuse = rosbag.Bag('exp05_cuse_fly.bag')
    bag_kf = rosbag.Bag('exp05_kf_fly.bag')
    bag_rd = rosbag.Bag('exp05_rd_fly.bag')

    bag_cuse_2 = rosbag.Bag('exp05_cuse_fly_2.bag')
    bag_kf_2 = rosbag.Bag('exp05_kf_fly_2.bag')
    bag_rd_2 = rosbag.Bag('exp05_rd_fly_2.bag')

    bag_cuse_3 = rosbag.Bag('exp05_cuse_fly_3.bag')
    bag_kf_3 = rosbag.Bag('exp05_kf_fly_3.bag')
    bag_rd_3 = rosbag.Bag('exp05_rd_fly_3.bag')

    pose_x_cuse = []
    pose_y_cuse = []
    pose_x_kf = []
    pose_y_kf = []
    pose_x_rd = []
    pose_y_rd = []

    for topic, msg, t in bag_cuse.read_messages(topics=['/cf1/pose/']):
        pose_x_cuse.append(msg.pose.position.x)
        pose_y_cuse.append(msg.pose.position.y)
    
    for topic, msg, t in bag_kf.read_messages(topics=['/cf1/pose/']):
        pose_x_kf.append(msg.pose.position.x)
        pose_y_kf.append(msg.pose.position.y)

    for topic, msg, t in bag_rd.read_messages(topics=['/cf1/pose/']):
        pose_x_rd.append(msg.pose.position.x)
        pose_y_rd.append(msg.pose.position.y)

    # for topic, msg, t in bag_cuse_2.read_messages(topics=['/cf1/pose/']):
    #     pose_x_cuse.append(msg.pose.position.x)
    #     pose_y_cuse.append(msg.pose.position.y)
    
    # for topic, msg, t in bag_kf_2.read_messages(topics=['/cf1/pose/']):
    #     pose_x_kf.append(msg.pose.position.x)
    #     pose_y_kf.append(msg.pose.position.y)

    # for topic, msg, t in bag_rd_2.read_messages(topics=['/cf1/pose/']):
    #     pose_x_rd.append(msg.pose.position.x)
    #     pose_y_rd.append(msg.pose.position.y)

    # for topic, msg, t in bag_cuse_3.read_messages(topics=['/cf1/pose/']):
    #     pose_x_cuse.append(msg.pose.position.x)
    #     pose_y_cuse.append(msg.pose.position.y)
    
    # for topic, msg, t in bag_kf_3.read_messages(topics=['/cf1/pose/']):
    #     pose_x_kf.append(msg.pose.position.x)
    #     pose_y_kf.append(msg.pose.position.y)

    # for topic, msg, t in bag_rd_3.read_messages(topics=['/cf1/pose/']):
    #     pose_x_rd.append(msg.pose.position.x)
    #     pose_y_rd.append(msg.pose.position.y)
  
    plt.scatter(pose_x_kf, pose_y_kf, label='KF')
    plt.scatter(pose_x_rd, pose_y_rd, label='RD')
    plt.scatter(pose_x_cuse, pose_y_cuse, label='CUSE')
    plt.xlim(-0.3, 0.3)
    plt.ylim(-0.3, 0.3)
    plt.legend()


    plt.show()


