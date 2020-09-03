#!/usr/bin/env python
# coding: utf-8

# In[13]:


import pyrealsense2 as rs
import time
from plyfile import PlyData, PlyElement
import numpy as np
from sklearn.cluster import KMeans
import random
import math


# In[15]:


pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
align = rs.align(rs.stream.color)

#Start streaming with default recommended configuration
pipe.start(config);


for x in range(5):
  pipe.wait_for_frames()
  time.sleep(2)

# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()

# Wait for the next set of frames from the camera
frames = pipe.wait_for_frames()
frames = align.process(frames)
# Fetch color and depth frames
depth = frames.get_depth_frame()
color = frames.get_color_frame()

# Tell pointcloud object to map to this color frame
pc.map_to(color)

# Generate the pointcloud and texture mappings
points = pc.calculate(depth)

print("Saving to file...")
points.export_to_ply("test0902_3.ply", color)
print("Done")
pipe.stop()


# In[24]:


a = [1, 2, 3]
b = [0.1, 0.2, 0.3]
c = [0.2, 0.4, 0.6]
print [sum(x) for x in zip(a, c)]


# In[ ]:




