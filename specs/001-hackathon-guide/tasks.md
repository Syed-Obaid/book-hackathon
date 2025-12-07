# Tasks: Physical AI & Humanoid Robotics Hackathon Guide

**Input**: Design documents from `/specs/001-hackathon-guide/`
**Prerequisites**: plan.md (✓), spec.md (✓), research.md (✓), data-model.md (✓), quickstart.md (✓)

**Project Type**: Documentation/educational content (Docusaurus static site)
**Tests**: Build validation, link checking, constitution compliance (not unit tests)

**Organization**: Tasks grouped by user story (module) to enable independent writing and testing of each module.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (module) this task belongs to (e.g., US1=Module 1, US2=Module 2)
- Include exact file paths in descriptions

## Path Conventions

- `/my-website/docs/`: Markdown content (modules, chapters)
- `/my-website/static/img/`: Images and diagrams
- `/my-website/examples/`: Code examples (separate from Docusaurus build)
- `/my-website/src/components/`: Custom React components for MDX

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Initialize Docusaurus project, configure GitHub Pages deployment, and set up project scaffolding.

- [X] T001 Create Docusaurus project: `npx create-docusaurus@latest my-website classic` in repository root
- [X] T002 Install plugins: `npm install @docusaurus/theme-mermaid @docusaurus/plugin-ideal-image docusaurus-lunr-search --save` in my-website/
- [X] T003 [P] Configure Mermaid in my-website/docusaurus.config.js (add theme-mermaid to presets)
- [X] T004 [P] Configure Ideal Image plugin in my-website/docusaurus.config.js (max width 1000px, lazy loading enabled)
- [X] T005 [P] Configure Lunr search in my-website/docusaurus.config.js (offline search)
- [X] T006 Create GitHub Actions workflow: my-website/.github/workflows/deploy.yml (build on push to main, deploy to gh-pages)
- [X] T007 [P] Configure sidebars.js with 5 module categories (ROS 2 Foundation, Digital Twin, AI Perception, VLA, Capstone)
- [X] T008 [P] Update docusaurus.config.js metadata (title: "Physical AI & Humanoid Robotics Hackathon Guide", tagline, organizationName, projectName)
- [X] T009 Test local build: `cd my-website && npm start` (verify localhost:3000 loads)
- [X] T010 Test production build: `npm run build` (verify build/ directory generated with 0 errors)

**Checkpoint**: Docusaurus project initialized, localhost preview working, GitHub Actions ready for deployment

---

## Phase 2: Foundational Content (Blocking Prerequisites)

**Purpose**: Create cross-module content that ALL chapters will reference (glossary, notation, references, chapter template).

**⚠️ CRITICAL**: No module-specific chapter writing can begin until this phase is complete.

- [X] T011 Create glossary: my-website/docs/glossary.md (structure: alphabetically sorted terms with definitions)
- [X] T012 [P] Add 10 foundational glossary terms: URDF, ROS 2, IMU, mAP, Sim-to-Real, VLA, Gazebo, Isaac Sim, Jetson, RViz
- [X] T013 Create notation guide: my-website/docs/notation.md (mathematical symbols: q for joint angles, T for transforms, J for Jacobian)
- [X] T014 Create references file: my-website/docs/references.md (APA 7th edition template, alphabetically sorted)
- [X] T015 [P] Add 15 foundational references: ROS 2 Humble docs, Isaac Sim docs, Gazebo docs, Unity Robotics Hub, 5 sim-to-real papers (Peng 2018, Tobin 2017, Sadeghi 2017, Muratore 2021, Akkaya 2019), Siciliano textbook, Lynch textbook, Jetson docs, RealSense docs, Unitree Go2
- [X] T016 Create chapter template: specs/001-hackathon-guide/contracts/chapter-template.md (6 sections: Learning Objectives, Prerequisites, Content, Summary, Exercises, References)
- [X] T017 Create examples directory structure: my-website/examples/ with subdirectories module1/ through module5/
- [X] T018 [P] Create static image directories: my-website/static/img/module1/ through my-website/static/img/module5/
- [X] T019 Create homepage: my-website/src/pages/index.js (custom landing page with course overview, 5 module cards, hardware requirements)
- [X] T020 Validate foundational content: Run `npm run build` and verify glossary, notation, references pages render correctly

**Checkpoint**: Foundation ready - module-specific chapter writing can now begin in parallel

---

## Phase 3: User Story 1 - ROS 2 Foundation for Humanoid Control (Priority: P1) 🎯 MVP

**Goal**: Students can install ROS 2, create workspaces, define URDF models, write publisher/subscriber nodes, and visualize robots in RViz.

**Independent Test**: Student can create ROS 2 workspace, define 3-DOF arm URDF, publish joint commands, and visualize in RViz within 2 hours.

### Module 1 Outline and Structure

- [ ] T021 [US1] Create module outline: specs/001-hackathon-guide/contracts/module1-outline.md (6 chapters: Overview, Installation, URDF Basics, Nodes/Topics, Services/Actions, Exercises)
- [ ] T022 [US1] Create module directory: my-website/docs/module1/ with _category_.json (label: "Module 1: ROS 2 Foundation", position: 1)

### Module 1 Chapters (Can write in parallel after T021-T022)

- [ ] T023 [P] [US1] Write chapter: my-website/docs/module1/overview.md (500 words: What is ROS 2, why roboticists use it, key concepts preview, motivation)
- [ ] T024 [P] [US1] Write chapter: my-website/docs/module1/installation.md (800 words: Ubuntu 22.04 check, ROS 2 Humble apt installation, workspace setup with colcon, troubleshooting)
- [ ] T025 [P] [US1] Write chapter: my-website/docs/module1/urdf-basics.md (1000 words: URDF XML structure, joint types, link properties, coordinate frames, 3-DOF arm example, common pitfalls)
- [ ] T026 [P] [US1] Write chapter: my-website/docs/module1/nodes-topics.md (900 words: Publisher-subscriber pattern, writing Python nodes, topic communication, ROS 2 launch files)
- [ ] T027 [P] [US1] Write chapter: my-website/docs/module1/services-actions.md (700 words: Request-reply patterns, service definitions, action servers, when to use each)
- [ ] T028 [P] [US1] Write chapter: my-website/docs/module1/exercises.md (1100 words: 18 exercises total - 3 per chapter, graded project rubric)

### Module 1 Code Examples

- [ ] T029 [P] [US1] Create code example: my-website/examples/module1/ros2-basics/install_ros2.sh (bash script for apt installation, tested on Ubuntu 22.04)
- [ ] T030 [P] [US1] Create code example: my-website/examples/module1/ros2-basics/create_workspace.sh (workspace initialization script)
- [ ] T031 [P] [US1] Create code example: my-website/examples/module1/urdf-models/simple_arm.urdf (3-DOF arm with comments explaining joint/link syntax)
- [ ] T032 [P] [US1] Create code example: my-website/examples/module1/ros2-basics/urdf_publisher.py (Python node publishing joint states, requires rclpy>=3.3.0)
- [ ] T033 [P] [US1] Create code example: my-website/examples/module1/ros2-basics/joint_controller.py (Subscriber node listening to joint commands)
- [ ] T034 [P] [US1] Create code example: my-website/examples/module1/ros2-basics/urdf_visualizer.launch.py (ROS 2 launch file for RViz visualization)
- [ ] T035 [US1] Create README: my-website/examples/module1/ros2-basics/README.md (setup instructions, expected output for all 6 examples)
- [ ] T036 [US1] Create requirements.txt: my-website/examples/module1/ros2-basics/requirements.txt (rclpy>=3.3.0, numpy>=1.24.0)
- [ ] T037 [US1] Test Tier 1 validation: Run flake8 on all Module 1 Python files (syntax check)

### Module 1 Diagrams

- [ ] T038 [P] [US1] Create Mermaid diagram (inline): ROS 2 high-level architecture in overview.md (nodes, topics, DDS layer)
- [ ] T039 [P] [US1] Create Mermaid diagram (inline): Workspace directory tree in installation.md
- [ ] T040 [P] [US1] Create Mermaid diagram (inline): Kinematic tree visualization in urdf-basics.md
- [ ] T041 [P] [US1] Create Excalidraw diagram: Coordinate frame conventions (DH parameters) → export to my-website/static/img/module1/dh-frames.svg
- [ ] T042 [P] [US1] Create Mermaid diagram (inline): ROS 2 node graph (publisher-subscriber) in nodes-topics.md
- [ ] T043 [US1] Optimize all SVG diagrams: Run `svgo my-website/static/img/module1/*.svg --multipass` (target < 100KB per file)

### Module 1 Validation

- [ ] T044 [US1] Add 15 glossary terms from Module 1: node, topic, service, action, URDF, link, joint, TF, RViz, colcon, package.xml, launch file, rclpy, subscriber, publisher
- [ ] T045 [US1] Add citations to references.md: ROS 2 Humble docs, URDF specification, RViz docs, colcon docs, Siciliano textbook (DH parameters)
- [ ] T046 [US1] Constitution check: Verify all 6 chapters have Learning Objectives, Prerequisites, Content, Summary, Exercises (3+), References
- [ ] T047 [US1] Word count check: Verify total Module 1 word count is 5000-6000 words, no single page > 2000 words
- [ ] T048 [US1] Build test: Run `npm run build` and verify Module 1 renders without errors
- [ ] T049 [US1] Link check: Verify all internal links (glossary, notation, references) and external links (ROS 2 docs) are valid

**Checkpoint**: Module 1 complete and independently functional. Student can follow guide to install ROS 2, create URDF, and visualize in RViz.

---

## Phase 4: User Story 2 - Digital Twin Simulation (Priority: P2)

**Goal**: Students can spawn robots in Gazebo with realistic physics, integrate Unity for high-fidelity rendering, simulate sensors (IMU, depth camera), and understand sim-to-real gap.

**Independent Test**: Student spawns humanoid in Gazebo, applies forces, observes collision/gravity responses, integrates depth camera, and measures sensor accuracy within 2% error margin.

### Module 2 Outline and Structure

- [ ] T050 [US2] Create module outline: specs/001-hackathon-guide/contracts/module2-outline.md (6 chapters: Overview, Gazebo Setup, Physics Simulation, Unity Integration, Sensor Simulation, Exercises)
- [ ] T051 [US2] Create module directory: my-website/docs/module2/ with _category_.json (label: "Module 2: Digital Twin Simulation", position: 2)

### Module 2 Chapters

- [ ] T052 [P] [US2] Write chapter: my-website/docs/module2/overview.md (500 words: Why simulation, Gazebo vs Unity tradeoffs, physics engines, sim-to-real gap introduction)
- [ ] T053 [P] [US2] Write chapter: my-website/docs/module2/gazebo-setup.md (800 words: Gazebo Garden installation, world files, spawning URDF models, GUI navigation)
- [ ] T054 [P] [US2] Write chapter: my-website/docs/module2/physics-sim.md (1000 words: Gravity, collision detection, friction, contact forces, ODE vs Bullet solvers)
- [ ] T055 [P] [US2] Write chapter: my-website/docs/module2/unity-integration.md (900 words: Unity 2022.3 LTS setup, ROS-TCP-Connector, real-time rendering at 60 FPS, human-robot interaction scenarios)
- [ ] T056 [P] [US2] Write chapter: my-website/docs/module2/sensor-sim.md (700 words: IMU, depth camera (RealSense D435i), lidar, sensor noise parameters, quantifying sim-to-real gap)
- [ ] T057 [P] [US2] Write chapter: my-website/docs/module2/exercises.md (1100 words: 18 exercises - sensor calibration, physics tuning, Unity scene creation)

### Module 2 Code Examples

- [ ] T058 [P] [US2] Create code example: my-website/examples/module2/gazebo-worlds/simple_world.sdf (Gazebo world with obstacles)
- [ ] T059 [P] [US2] Create code example: my-website/examples/module2/gazebo-worlds/spawn_robot.launch.py (ROS 2 launch file to spawn URDF in Gazebo)
- [ ] T060 [P] [US2] Create code example: my-website/examples/module2/gazebo-worlds/physics_config.yaml (ODE solver configuration)
- [ ] T061 [P] [US2] Create code example: my-website/examples/module2/unity-scenes/HumanoidDemoScene.unity (Unity scene with humanoid and environment)
- [ ] T062 [P] [US2] Create code example: my-website/examples/module2/unity-scenes/RosTcpConnectorSetup.cs (C# script for ROS-TCP-Connector)
- [ ] T063 [P] [US2] Create code example: my-website/examples/module2/sensor-sim/imu_sim.py (IMU data publisher with drift parameters)
- [ ] T064 [P] [US2] Create code example: my-website/examples/module2/sensor-sim/depth_camera_sim.py (RealSense D435i simulation with noise)
- [ ] T065 [US2] Create README: my-website/examples/module2/README.md (setup for Gazebo Garden, Unity 2022.3, sensor plugins)
- [ ] T066 [US2] Test Tier 1+2 validation: Syntax check + Gazebo headless spawn test

### Module 2 Diagrams

- [ ] T067 [P] [US2] Create Excalidraw diagram: Lab hardware setup for Gazebo workstation → export to my-website/static/img/module2/gazebo-hardware.svg
- [ ] T068 [P] [US2] Create Mermaid diagram (inline): Gazebo physics pipeline in physics-sim.md
- [ ] T069 [P] [US2] Create Excalidraw diagram: Unity scene hierarchy → export to my-website/static/img/module2/unity-hierarchy.svg
- [ ] T070 [P] [US2] Create Mermaid diagram (inline): ROS-TCP-Connector data flow in unity-integration.md
- [ ] T071 [P] [US2] Create Mermaid diagram (inline): Sensor simulation architecture in sensor-sim.md
- [ ] T072 [US2] Optimize SVG diagrams for Module 2

### Module 2 Validation

- [ ] T073 [US2] Add 12 glossary terms: Gazebo, Unity, SDF, physics engine, IMU, depth camera, lidar, domain randomization, ODE, Bullet, ROS-TCP-Connector, point cloud
- [ ] T074 [US2] Add citations: Gazebo Garden docs, Unity Robotics Hub, ROS-TCP-Connector GitHub, Tobin 2017 (domain randomization), RealSense docs
- [ ] T075 [US2] Constitution check Module 2 (6 sections per chapter, exercises, citations)
- [ ] T076 [US2] Word count check Module 2 (5000-6000 total, max 2000 per page)
- [ ] T077 [US2] Build test and link check Module 2

**Checkpoint**: Module 2 complete. Student can simulate humanoid robots in Gazebo and Unity with realistic physics and sensors.

---

## Phase 5: User Story 3 - AI-Powered Perception and Navigation (Priority: P3)

**Goal**: Students train RL navigation policies in Isaac Gym, integrate object detection (YOLO), deploy trained models to Gazebo, and implement SLAM for environment mapping.

**Independent Test**: Student trains navigation policy for 10,000 episodes in Isaac Gym, achieves > 85% success rate on obstacle courses, deploys to Gazebo with < 15% degradation.

### Module 3 Outline and Structure

- [ ] T078 [US3] Create module outline: specs/001-hackathon-guide/contracts/module3-outline.md (6 chapters: Overview, Isaac Sim Setup, Object Detection, Navigation RL, SLAM, Exercises)
- [ ] T079 [US3] Create module directory: my-website/docs/module3/ with _category_.json (label: "Module 3: AI Perception & Navigation", position: 3)

### Module 3 Chapters

- [ ] T080 [P] [US3] Write chapter: my-website/docs/module3/overview.md (500 words: AI perception vs traditional planning, reinforcement learning motivation, Isaac Sim capabilities)
- [ ] T081 [P] [US3] Write chapter: my-website/docs/module3/isaac-sim-setup.md (900 words: Isaac Sim 2023.1.1+ installation, OmniGraph, Isaac Gym environments, RTX GPU requirements)
- [ ] T082 [P] [US3] Write chapter: my-website/docs/module3/perception.md (1000 words: YOLO object detection, Mask R-CNN, ROS 2 perception pipelines, mAP evaluation, 15 FPS target)
- [ ] T083 [P] [US3] Write chapter: my-website/docs/module3/navigation-rl.md (1100 words: Isaac Gym PPO implementation, state/action spaces, reward function design, 10,000 episode training, hyperparameters)
- [ ] T084 [P] [US3] Write chapter: my-website/docs/module3/slam.md (800 words: SLAM with lidar/depth camera, 2D occupancy grid, 5cm resolution, cartographer integration)
- [ ] T085 [P] [US3] Write chapter: my-website/docs/module3/exercises.md (1200 words: 18 exercises - train custom RL policy, fine-tune YOLO, integrate SLAM)

### Module 3 Code Examples

- [ ] T086 [P] [US3] Create code example: my-website/examples/module3/isaac-gym/navigation_env.py (Isaac Gym environment for obstacle avoidance)
- [ ] T087 [P] [US3] Create code example: my-website/examples/module3/isaac-gym/train_ppo.py (PPO training script with hyperparameters)
- [ ] T088 [P] [US3] Create code example: my-website/examples/module3/perception/yolo_detector.py (YOLO integration with ROS 2, PyTorch 2.0+)
- [ ] T089 [P] [US3] Create code example: my-website/examples/module3/perception/perception_pipeline.py (ROS 2 node for object detection + localization)
- [ ] T090 [P] [US3] Create code example: my-website/examples/module3/navigation/deploy_policy.py (Deploy Isaac Gym policy to Gazebo)
- [ ] T091 [P] [US3] Create code example: my-website/examples/module3/slam/cartographer_config.yaml (SLAM configuration for ROS 2)
- [ ] T092 [US3] Create README: my-website/examples/module3/README.md (Isaac Sim licensing, RTX GPU setup, pre-trained model weights)
- [ ] T093 [US3] Test Tier 1+2: Syntax check + headless RL training (1000 episodes smoke test)

### Module 3 Diagrams

- [ ] T094 [P] [US3] Create Mermaid diagram (inline): Isaac Sim architecture in isaac-sim-setup.md
- [ ] T095 [P] [US3] Create Mermaid diagram (inline): Perception pipeline (camera → YOLO → ROS 2 topic) in perception.md
- [ ] T096 [P] [US3] Create Mermaid diagram (inline): RL training loop in navigation-rl.md
- [ ] T097 [P] [US3] Create Mermaid diagram (inline): SLAM data flow in slam.md
- [ ] T098 [P] [US3] Create Excalidraw diagram: Isaac Gym environment visualization → export to my-website/static/img/module3/isaac-gym-env.svg
- [ ] T099 [US3] Optimize SVG diagrams Module 3

### Module 3 Validation

- [ ] T100 [US3] Add 15 glossary terms: Isaac Sim, Isaac Gym, reinforcement learning, PPO, YOLO, Mask R-CNN, mAP, SLAM, cartographer, occupancy grid, reward function, state space, action space, OmniGraph, perception
- [ ] T101 [US3] Add citations: Isaac Sim docs, Isaac Gym docs, Peng 2018, Muratore 2021, YOLO paper, cartographer docs, PyTorch docs
- [ ] T102 [US3] Constitution check Module 3
- [ ] T103 [US3] Word count check Module 3 (5000-6000 total)
- [ ] T104 [US3] Build test and link check Module 3

**Checkpoint**: Module 3 complete. Student can train AI navigation policies and deploy to simulation.

---

## Phase 6: User Story 4 - Vision-Language-Action (VLA) Integration (Priority: P4)

**Goal**: Students build VLA pipeline (speech → LLM → action planning → execution), process natural language commands, and execute multi-step manipulation tasks.

**Independent Test**: Student speaks command "Navigate to kitchen and pick up cup," robot parses intent, generates action sequence, and executes with < 3s latency.

### Module 4 Outline and Structure

- [ ] T105 [US4] Create module outline: specs/001-hackathon-guide/contracts/module4-outline.md (5 chapters: Overview, Speech Recognition, LLM Planning, Action Execution, Exercises)
- [ ] T106 [US4] Create module directory: my-website/docs/module4/ with _category_.json (label: "Module 4: VLA Integration", position: 4)

### Module 4 Chapters

- [ ] T107 [P] [US4] Write chapter: my-website/docs/module4/overview.md (500 words: VLA motivation, embodied AI, multi-modal control)
- [ ] T108 [P] [US4] Write chapter: my-website/docs/module4/speech-recognition.md (900 words: Whisper ASR, noise cancellation, 60dB ambient, 92% accuracy target)
- [ ] T109 [P] [US4] Write chapter: my-website/docs/module4/llm-planning.md (1100 words: GPT-4/Llama intent parsing, action sequence generation, handling ambiguity, safety constraints)
- [ ] T110 [P] [US4] Write chapter: my-website/docs/module4/action-execution.md (800 words: ROS 2 action servers, bimanual coordination, 50ms timing tolerance, execution monitoring)
- [ ] T111 [P] [US4] Write chapter: my-website/docs/module4/exercises.md (1200 words: 15 exercises - build custom VLA commands, test edge cases)

### Module 4 Code Examples

- [ ] T112 [P] [US4] Create code example: my-website/examples/module4/vla-pipeline/speech_recognizer.py (Whisper integration, microphone input)
- [ ] T113 [P] [US4] Create code example: my-website/examples/module4/vla-pipeline/intent_parser.py (LLM API call for command parsing)
- [ ] T114 [P] [US4] Create code example: my-website/examples/module4/vla-pipeline/action_planner.py (Action sequence generator)
- [ ] T115 [P] [US4] Create code example: my-website/examples/module4/vla-pipeline/vla_executor.py (ROS 2 action client for execution)
- [ ] T116 [P] [US4] Create code example: my-website/examples/module4/vla-pipeline/full_pipeline.py (End-to-end VLA demo)
- [ ] T117 [US4] Create README: my-website/examples/module4/README.md (microphone setup, LLM API keys, ROS 2 action server requirements)
- [ ] T118 [US4] Test Tier 1+2: Syntax check + mock VLA pipeline test (no API calls)

### Module 4 Diagrams

- [ ] T119 [P] [US4] Create Mermaid diagram (inline): VLA pipeline architecture in overview.md (speech → LLM → planner → executor)
- [ ] T120 [P] [US4] Create Mermaid diagram (inline): Speech recognition flow in speech-recognition.md
- [ ] T121 [P] [US4] Create Mermaid diagram (inline): LLM prompt structure in llm-planning.md
- [ ] T122 [P] [US4] Create Mermaid diagram (inline): Bimanual coordination timeline in action-execution.md
- [ ] T123 [US4] Optimize SVG diagrams Module 4

### Module 4 Validation

- [ ] T124 [US4] Add 10 glossary terms: VLA, ASR, Whisper, LLM, intent parsing, action sequence, bimanual, execution monitoring, embodied AI, multi-modal
- [ ] T125 [US4] Add citations: Whisper paper, GPT-4 docs, Llama docs, ROS 2 action server docs
- [ ] T126 [US4] Constitution check Module 4
- [ ] T127 [US4] Word count check Module 4 (4500-5500 total)
- [ ] T128 [US4] Build test and link check Module 4

**Checkpoint**: Module 4 complete. Student can build VLA pipeline for natural language robot control.

---

## Phase 7: User Story 5 - Capstone Project and Sim-to-Real Deployment (Priority: P5)

**Goal**: Students complete comprehensive capstone integrating all prior modules, deploy trained AI to Jetson Orin, demonstrate sim-to-real transfer, and submit technical report.

**Independent Test**: Student designs custom humanoid task, trains in simulation, deploys to Jetson with < 200ms inference latency, completes live demo with < 2 human interventions.

### Module 5 Outline and Structure

- [ ] T129 [US5] Create module outline: specs/001-hackathon-guide/contracts/module5-outline.md (5 chapters: Overview, Project Requirements, Sim-to-Real Gap, Edge Deployment, Evaluation Rubric)
- [ ] T130 [US5] Create module directory: my-website/docs/module5/ with _category_.json (label: "Module 5: Capstone Project", position: 5)

### Module 5 Chapters

- [ ] T131 [P] [US5] Write chapter: my-website/docs/module5/overview.md (500 words: Capstone expectations, integrating all 4 modules, portfolio demonstration)
- [ ] T132 [P] [US5] Write chapter: my-website/docs/module5/project-requirements.md (1000 words: URDF model, Gazebo world, training logs, deployment script, technical report structure)
- [ ] T133 [P] [US5] Write chapter: my-website/docs/module5/sim-to-real.md (1100 words: Quantifying degradation, sensor noise, dynamics mismatch, calibration, citing 5 papers)
- [ ] T134 [P] [US5] Write chapter: my-website/docs/module5/deployment.md (1000 words: Jetson Orin setup, TensorRT optimization, memory constraints (6GB), inference latency benchmarks)
- [ ] T135 [P] [US5] Write chapter: my-website/docs/module5/rubric.md (900 words: Grading criteria - technical completeness, simulation accuracy, deployment success, documentation quality, live demo)

### Module 5 Code Examples

- [ ] T136 [P] [US5] Create capstone template: my-website/examples/module5/capstone-template/ (directory structure for student submissions)
- [ ] T137 [P] [US5] Create code example: my-website/examples/module5/capstone-template/tensorrt_convert.py (Convert PyTorch model to TensorRT)
- [ ] T138 [P] [US5] Create code example: my-website/examples/module5/capstone-template/jetson_deploy.py (Deploy model to Jetson Orin with memory profiling)
- [ ] T139 [P] [US5] Create code example: my-website/examples/module5/capstone-template/benchmark_inference.py (Measure inference latency, target < 200ms)
- [ ] T140 [P] [US5] Create code example: my-website/examples/module5/capstone-template/technical_report_template.md (4000-6000 word structured report)
- [ ] T141 [US5] Create README: my-website/examples/module5/README.md (Jetson Orin Nano setup, power configuration, RealSense USB 3.0 connection)
- [ ] T142 [US5] Test Tier 3 (manual): Document Jetson deployment with screenshots

### Module 5 Diagrams

- [ ] T143 [P] [US5] Create Excalidraw diagram: Hardware setup (Jetson + RealSense + robot) → export to my-website/static/img/module5/hardware-setup.svg
- [ ] T144 [P] [US5] Create Mermaid diagram (inline): Capstone project workflow in project-requirements.md
- [ ] T145 [P] [US5] Create Mermaid diagram (inline): Sim-to-real gap sources in sim-to-real.md
- [ ] T146 [P] [US5] Create Mermaid diagram (inline): TensorRT optimization pipeline in deployment.md
- [ ] T147 [US5] Optimize SVG diagrams Module 5

### Module 5 Validation

- [ ] T148 [US5] Add 8 glossary terms: Jetson Orin, TensorRT, edge AI, inference latency, model quantization, sim-to-real transfer, dynamics mismatch, technical report
- [ ] T149 [US5] Add citations: Jetson Orin docs, TensorRT docs, Sadeghi 2017, Akkaya 2019, capstone rubric template
- [ ] T150 [US5] Constitution check Module 5
- [ ] T151 [US5] Word count check Module 5 (4500-5500 total)
- [ ] T152 [US5] Build test and link check Module 5

**Checkpoint**: All 5 modules complete. Full hackathon guide ready for students.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements affecting multiple modules, final quality checks, and deployment readiness.

- [ ] T153 [P] Update homepage: my-website/src/pages/index.js (add module completion progress bars, hardware comparison table via MDX)
- [ ] T154 [P] Create about page: my-website/src/pages/about.md (course credits, instructors, license)
- [ ] T155 [P] Create MDX component: my-website/src/components/HardwareSpec.jsx (sortable table comparing Jetson Orin Nano/NX/AGX)
- [ ] T156 [P] Create MDX component: my-website/src/components/PerformanceMetrics.jsx (FPS/latency charts for benchmarks)
- [ ] T157 Spell check: Run `cspell` with custom robotics dictionary (add terms: URDF, RealSense, Gazebo, etc.)
- [ ] T158 Accessibility audit: Run Lighthouse on 10 random pages (target accessibility score ≥ 95)
- [ ] T159 Performance audit: Run Lighthouse on homepage + 5 module landing pages (target performance ≥ 90, LCP < 2.5s)
- [ ] T160 SEO optimization: Add Open Graph tags, sitemap.xml, robots.txt to my-website/docusaurus.config.js
- [ ] T161 Image optimization: Verify all images in static/img/ are < 500KB, run `svgo` on SVGs
- [ ] T162 APA citation validation: Check all references in docs/references.md with Scribbr APA checker (0 formatting errors)
- [ ] T163 Create contributing guide: my-website/CONTRIBUTING.md (link to specs/001-hackathon-guide/quickstart.md)
- [ ] T164 Create LICENSE: my-website/LICENSE (MIT license for open-source educational content)
- [ ] T165 Create project README: my-website/README.md (project overview, contributor setup, deployment instructions)
- [ ] T166 Final build test: Run `npm run build` with 0 errors, 0 warnings
- [ ] T167 Final link check: Run link checker on entire site (0 broken internal/external links)
- [ ] T168 Deploy to GitHub Pages: Merge feature branch to main, verify GitHub Actions deploys to gh-pages
- [ ] T169 Post-deployment check: Verify site live at GitHub Pages URL, test navigation across all 5 modules
- [ ] T170 Tag release: Git tag `v1.0-ros2-humble` for stable release

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all module writing
- **Modules (Phases 3-7)**: All depend on Foundational phase completion
  - Modules can proceed in parallel (US1-US5 independent writing)
  - Or sequentially in priority order (P1 → P2 → P3 → P4 → P5)
- **Polish (Phase 8)**: Depends on desired modules being complete (minimum P1 for MVP)

### User Story (Module) Dependencies

- **Module 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other modules
- **Module 2 (P2)**: Can start after Foundational (Phase 2) - Builds on ROS 2 knowledge from M1 but chapters independently writable
- **Module 3 (P3)**: Can start after Foundational (Phase 2) - Requires M1+M2 concepts for students but chapters independently writable
- **Module 4 (P4)**: Can start after Foundational (Phase 2) - Integrates M1-M3 for students but chapters independently writable
- **Module 5 (P5)**: Can start after Foundational (Phase 2) - Capstone integrates all modules but chapters independently writable

### Within Each Module

- Module outline and directory structure BEFORE chapters
- All chapters for a module can be written in parallel after outline
- Code examples can be created in parallel with chapters
- Diagrams can be created in parallel with chapters
- Validation tasks run after all module content created

### Parallel Opportunities

- **Setup phase**: Tasks T003-T005, T007-T008 can run in parallel
- **Foundational phase**: Tasks T012, T015, T018 can run in parallel
- **Module chapters**: All chapters within a module (T023-T028 for M1) can be written in parallel
- **Code examples**: All examples within a module (T029-T034 for M1) can be created in parallel
- **Diagrams**: All diagrams within a module (T038-T042 for M1) can be created in parallel
- **Modules**: Different writers can work on different modules in parallel (M1, M2, M3, M4, M5)
- **Polish**: Tasks T153-T156 can run in parallel

---

## Parallel Example: Module 1 Chapters

```bash
# Launch all Module 1 chapters together after T021-T022 complete:
Task T023: "Write chapter: my-website/docs/module1/overview.md"
Task T024: "Write chapter: my-website/docs/module1/installation.md"
Task T025: "Write chapter: my-website/docs/module1/urdf-basics.md"
Task T026: "Write chapter: my-website/docs/module1/nodes-topics.md"
Task T027: "Write chapter: my-website/docs/module1/services-actions.md"
Task T028: "Write chapter: my-website/docs/module1/exercises.md"

# Launch all Module 1 code examples together:
Task T029: "Create code example: install_ros2.sh"
Task T030: "Create code example: create_workspace.sh"
Task T031: "Create code example: simple_arm.urdf"
Task T032: "Create code example: urdf_publisher.py"
Task T033: "Create code example: joint_controller.py"
Task T034: "Create code example: urdf_visualizer.launch.py"
```

---

## Implementation Strategy

### MVP First (Module 1 Only)

1. Complete Phase 1: Setup (Docusaurus initialized)
2. Complete Phase 2: Foundational (Glossary, references, templates ready)
3. Complete Phase 3: Module 1 (ROS 2 Foundation)
4. **STOP and VALIDATE**: Test Module 1 with students (can they install ROS 2 and visualize URDF?)
5. Deploy to GitHub Pages for pilot testing

### Incremental Delivery

1. Setup + Foundational → Empty guide with infrastructure
2. Add Module 1 → Deploy/Demo (Students can learn ROS 2) ← MVP!
3. Add Module 2 → Deploy/Demo (Students can now simulate)
4. Add Module 3 → Deploy/Demo (Students can now train AI)
5. Add Module 4 → Deploy/Demo (Students can now do VLA)
6. Add Module 5 → Deploy/Demo (Students can now do capstone) ← Full course

### Parallel Writing Strategy

With multiple writers:

1. Team completes Setup + Foundational together (1 week)
2. Once Foundational done:
   - Writer A: Module 1 (ROS 2)
   - Writer B: Module 2 (Simulation)
   - Writer C: Module 3 (AI)
   - Writer D: Module 4 (VLA)
   - Writer E: Module 5 (Capstone)
3. Modules complete independently, merge to main, deploy incrementally

---

## Notes

- **[P] tasks**: Different files, can run in parallel
- **[Story] label**: Maps task to specific module (US1=Module 1, US2=Module 2, etc.)
- **Documentation project adaptation**: "Tests" mean build validation, link checking, constitution compliance (not unit tests)
- **Code examples**: Must be tested (Tier 1: syntax, Tier 2: functional, Tier 3: manual screenshots)
- **Word counts**: Total per module 4000-6000 words, max 2000 per chapter
- **Constitution check**: Every chapter needs 6 sections (Learning Objectives, Prerequisites, Content, Summary, Exercises, References)
- **Commit strategy**: Commit after each chapter or logical group (e.g., all Module 1 chapters)
- **Build validation**: Run `npm run build` frequently, especially before marking module complete
- **Accessibility**: Every image needs descriptive alt text, verified by Lighthouse audit
