import os
import sys
import numpy as np
#import rosbag_pandas

if __name__ == "__main__":
   
    #dataframe = rosbag_pandas.bag_to_dataframe('defcov_tile.bag')
    #data = dataframe.to_numpy()
    bag = rosbag.Bag('defcov_tile.bag')
    print(bag)



