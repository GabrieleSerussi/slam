# Pose Graph Optimization for SLAM using g2o

## Overview
This repository contains code to generate a g2o log file for  **Pose Graph Optimization (PGO)** in the context of **Simultaneous Localization and Mapping (SLAM)**. The primary goal is to optimize the robot's trajectory and landmark positions using the **g2o** library. Specifically, the code tracks the robot's position and the landmarks it observes (which are represented by AprilTags).

## Purpose
- **SLAM**: SLAM is a fundamental problem in robotics where a robot simultaneously estimates its own pose (position and orientation) and constructs a map of its environment.
- **Pose Graph Optimization**: PGO aims to refine the robot's estimated trajectory by minimizing the error between observed measurements (such as odometry and landmark positions) and predicted measurements based on the graph structure.
- **g2o**: The **g2o** library provides tools for solving nonlinear optimization problems, including PGO. It efficiently handles large-scale graph-based optimization.

## Code Details
1. **Position Tracking**:
    - The code subscribes to odometry data (from wheel encoders or other sensors) to estimate the robot's position (x, y, yaw).
    - It maintains a history of robot positions (`current` and `old`).
    - When significant movement occurs (translation or rotation), it creates vertices and edges in the pose graph.

2. **Landmark Observations (AprilTags)**:
    - The robot detects AprilTags (landmarks).
    - For each detected tag:
        - Transforms its position from the camera frame to the odom frame.
        - Creates vertex entries for new tags and edge entries connecting the robot's position to the tag's position.

3. **Output File**:
    - The code generates a **g2o** file (e.g., "out.g2o") that encodes the pose graph information.
    - This file can be used for further optimization and analysis.
