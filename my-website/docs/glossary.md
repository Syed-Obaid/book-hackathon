---
id: glossary
title: Glossary
sidebar_label: Glossary
description: Terminology and definitions for Physical AI and Humanoid Robotics
keywords: [glossary, robotics, ROS 2, AI, terminology]
---

# Glossary

Alphabetically sorted terms and definitions used throughout the Physical AI & Humanoid Robotics Hackathon Guide.

---

## G

### Gazebo

Open-source 3D robotics simulator that provides realistic physics simulation (gravity, friction, collisions) for testing robot behaviors in virtual environments before hardware deployment. Gazebo Garden is the LTS version used in this guide.

**Related:** Digital Twin, Simulation, Physics Engine

---

## I

### IMU (Inertial Measurement Unit)

Sensor that measures acceleration and angular velocity using accelerometers and gyroscopes. Common in robotics for estimating robot orientation and detecting motion. Subject to drift over time requiring calibration or sensor fusion.

**Related:** Sensor Simulation, Sim-to-Real Gap

### Isaac Sim

NVIDIA's robotics simulation platform built on Omniverse, providing GPU-accelerated physics, photorealistic rendering, and AI training capabilities via Isaac Gym for reinforcement learning.

**Related:** Reinforcement Learning, Digital Twin, Simulation

---

## J

### Jetson

NVIDIA's embedded AI computing platform for edge deployment. Jetson Orin Nano (8GB) and Orin NX (16GB) are used in this guide for deploying trained AI models to physical robots with real-time inference constraints.

**Related:** Edge AI, Deployment, TensorRT

---

## M

### mAP (Mean Average Precision)

Performance metric for object detection models, measuring the average precision across all object classes and IoU (Intersection over Union) thresholds. Higher mAP indicates better detection accuracy.

**Related:** Object Detection, Perception, YOLO

---

## R

### ROS 2 (Robot Operating System 2)

Open-source middleware framework for robot software development, providing inter-process communication (topics, services, actions), hardware abstraction, and package management. ROS 2 Humble is the LTS version used in this guide.

**Related:** URDF, Node, Topic, Publisher, Subscriber

### RViz

3D visualization tool for ROS 2 that displays robot models, sensor data (point clouds, camera images), transforms (TF), and planning trajectories in real-time. Essential for debugging robot state and verifying URDF models.

**Related:** URDF, ROS 2, Visualization

---

## S

### Sim-to-Real (Gap)

The performance degradation observed when transferring AI policies trained in simulation to physical robots. Typical degradation is 10-20% due to unmodeled dynamics, sensor noise, and physics inaccuracies. Mitigation strategies include domain randomization and system identification.

**Related:** Reinforcement Learning, Isaac Sim, Gazebo, Domain Randomization

---

## U

### URDF (Unified Robot Description Format)

XML-based format for defining robot kinematics, geometry, and visual appearance. Specifies joints (revolute, prismatic, fixed), links (visual, collision, inertial properties), and transforms between coordinate frames.

**Related:** ROS 2, Robot Model, Kinematics

---

## V

### VLA (Vision-Language-Action)

Multi-modal AI pipeline that processes natural language commands and visual input to generate robot action sequences. Combines speech recognition (ASR), large language models (LLMs) for intent parsing, and action planning for embodied AI control.

**Related:** LLM, Embodied AI, Natural Language Processing, Action Planning

---

## Notes

- **First Use**: Terms are linked on first use in each chapter to this glossary
- **APA Citations**: Technical definitions reference peer-reviewed sources in [References](./references.md)
- **Updates**: New terms added as modules are developed
