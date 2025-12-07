# Implementation Plan: Physical AI & Humanoid Robotics Hackathon Guide

**Branch**: `001-hackathon-guide` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-hackathon-guide/spec.md`

## Summary

Create a comprehensive educational guide (4000-6000 words per module) teaching advanced AI students how to build, simulate, and deploy humanoid robotics systems using ROS 2, Gazebo/Unity simulation, NVIDIA Isaac AI capabilities, and Vision-Language-Action (VLA) models. The guide follows a 5-module progression over 12 weeks, culminating in a capstone project where students deploy trained AI to edge hardware (Jetson Orin). Content will be delivered via Docusaurus static site hosted on GitHub Pages, following spec-driven development workflow with Claude Code assistance.

**Technical approach**: Docusaurus for documentation architecture + SpecKit Plus for spec-driven writing + GitHub Pages for deployment. Each module structured with learning objectives, prerequisites, core concepts, code examples (tested and version-pinned), exercises, and APA citations.

## Technical Context

**Language/Version**: Markdown/MDX for content, JavaScript/Node.js 18+ for Docusaurus build system
**Primary Dependencies**: Docusaurus 3.x (static site generator), React 18+ (for MDX components), Prism.js (syntax highlighting), Mermaid (diagrams)
**Storage**: Git repository for version control, static assets in `/static/img/`, code examples in `/examples/`, APA citations in `/docs/references.md`
**Testing**: Docusaurus build validation (`npm run build`), link checker (broken links), spell checker, Lighthouse performance/accessibility
**Target Platform**: GitHub Pages static hosting (gh-pages branch), browser-based reading (desktop/mobile)
**Project Type**: Documentation/educational content (Docusaurus static site)
**Performance Goals**: Page load < 3s, Largest Contentful Paint < 2.5s, Cumulative Layout Shift < 0.1 (Lighthouse)
**Constraints**: 4000-6000 words per module (max 2000 words per page), images < 500KB, APA citation format, all code examples must be tested
**Scale/Scope**: 5 modules × 4-6 sections each = ~25-30 pages, 12-week timeline, ~100 code examples, ~50 diagrams

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Content Accuracy & Technical Rigor

- [x] **Mathematical equations validated**: ROS 2 kinematics (DH parameters, Jacobians) cited from Siciliano & Khatib (2016)
- [x] **Code examples tested**: All ROS 2, Gazebo, Isaac Sim examples runnable with specified versions
- [x] **Citations required**: ROS 2 docs, NVIDIA Isaac docs, peer-reviewed papers for algorithms
- [x] **Version specifications**: ROS 2 Humble, Gazebo Garden, Isaac Sim 2023.1.1+, PyTorch 2.0+
- [x] **No speculative claims**: Performance metrics (FPS, latency) validated on specified hardware (RTX 4070 Ti, Jetson Orin)

### II. Educational Clarity & Accessibility

- [x] **Prerequisites declared**: Each module lists prior chapters + external knowledge (Python, ML fundamentals, linear algebra)
- [x] **Concept introduction pattern**: (1) motivation (why learn ROS 2?) → (2) simple example (3-DOF arm) → (3) formal definition (pub-sub pattern) → (4) application (humanoid control)
- [x] **Target audience**: Advanced AI students with Python/ML background but no robotics experience
- [x] **Measurable learning objectives**: "Students can create ROS 2 workspace and visualize URDF in RViz within 2 hours"
- [x] **Worked examples**: Minimum one per concept (e.g., URDF model, Gazebo spawn, Isaac Gym training loop)
- [x] **Diagrams required**: Lab setup (hardware connections), system architecture (ROS 2 node graph), data flow (VLA pipeline)
- [x] **Glossary terms linked**: First use links to `/docs/glossary.md` (URDF, IMU, mAP, sim-to-real gap)

### III. Consistency & Standards (NON-NEGOTIABLE)

- [x] **Terminology via glossary**: `docs/glossary.md` single source of truth
- [x] **Code formatting**: PEP 8 for Python, ROS 2 conventions (snake_case for nodes/topics)
- [x] **Chapter structure template**:
  1. Learning Objectives (3-5 bullet points)
  2. Prerequisites (prior chapters + external knowledge)
  3. Content (Introduction → Core Concepts → Examples → Applications)
  4. Summary (key takeaways)
  5. Exercises (3+: conceptual, computational, implementation)
  6. References (APA format)
- [x] **Voice**: Second person for tutorials ("you will create a workspace"), third person for theory ("the robot computes inverse kinematics")
- [x] **Notation**: Mathematical symbols in `docs/notation.md` (q for joint angles, T for transforms, J for Jacobian)
- [x] **Units**: SI units (meters, radians) except domain conventions (degrees for joint angles with explicit note)

### IV. Docusaurus Structure & Quality

- [x] **One concept per page**: Max 2000 words per `.md` file (split long modules into subsections)
- [x] **Sidebar organization**: Fundamentals (Module 1-2) → Intermediate (Module 3) → Advanced (Module 4-5)
- [x] **Metadata required**: `title`, `description`, `keywords`, `sidebar_position` in frontmatter
- [x] **Relative links**: `[Digital Twin](../module2/gazebo.md)` not absolute URLs
- [x] **Assets naming**: `/static/img/module1/ros2-node-graph.svg` not `/static/img/fig1.png`
- [x] **Alt text**: "ROS 2 node graph showing publisher-subscriber pattern with joint state topic" not "image of robot"
- [x] **Search optimization**: Keywords in H1/H2, first paragraph, metadata

### V. Code Example Quality

- [x] **Language specification**: ```python not ``` in fenced blocks
- [x] **Complete examples**: Full scripts with imports, not fragments (unless marked "excerpt from examples/module1/urdf_publisher.py")
- [x] **Comments explain WHY**: `# Publish at 10Hz to match Gazebo physics timestep` not `# Set rate to 10`
- [x] **Dependencies with versions**: `# Requires: rclpy>=3.3.0, numpy>=1.24.0`
- [x] **Repository structure**: `/examples/module1/ros2-basics/urdf_publisher.py` with `README.md`
- [x] **Test coverage**: Each example includes validation (e.g., `ros2 topic echo /joint_states` shows expected output)
- [x] **Safety warnings**: "Ensure e-stop accessible before running on physical robot"
- [x] **Standard libraries**: ROS 2, NumPy, PyTorch, OpenCV (avoid obscure packages)

### VI. Deployment & Publishing Standards

- [x] **Main branch**: Production-ready content deployed to GitHub Pages
- [x] **Feature branches**: `001-hackathon-guide` for development, PR-based review
- [x] **Build gates**: Docusaurus `npm run build` succeeds without warnings
- [x] **Link checker**: No broken internal/external links
- [x] **Spell check**: Technical terms in dictionary
- [x] **Image optimization**: < 500KB per file, `svgo` for SVGs
- [x] **Performance targets**: Initial load < 3s, LCP < 2.5s, CLS < 0.1
- [x] **SEO**: Open Graph tags, sitemap, robots.txt
- [x] **Versioning**: Major revisions tagged (v1.0, v2.0)
- [x] **Redirects**: Deprecated URLs redirect to updated content

**Constitution Check Status**: ✅ PASSED - All 6 principles adhered to. No violations requiring justification.

## Project Structure

### Documentation (this feature)

```text
specs/001-hackathon-guide/
├── plan.md              # This file (/sp.plan output)
├── spec.md              # Feature requirements (completed)
├── research.md          # Phase 0: Technology research, best practices
├── data-model.md        # Phase 1: Content entities (modules, chapters, examples)
├── quickstart.md        # Phase 1: Getting started guide for contributors
├── contracts/           # Phase 1: Module outlines, chapter templates
│   ├── module1-outline.md
│   ├── module2-outline.md
│   ├── module3-outline.md
│   ├── module4-outline.md
│   ├── module5-outline.md
│   └── chapter-template.md
└── tasks.md             # Phase 2: Granular writing tasks (/sp.tasks - NOT created by /sp.plan)
```

### Source Code (repository root - Docusaurus project)

```text
my-website/              # Docusaurus root (created by npx create-docusaurus)
├── docs/                # Markdown content source
│   ├── intro.md         # Landing page
│   ├── glossary.md      # Terminology (single source of truth)
│   ├── notation.md      # Mathematical symbols
│   ├── references.md    # APA citations (centralized)
│   ├── module1/         # ROS 2 Foundation
│   │   ├── _category_.json
│   │   ├── overview.md
│   │   ├── installation.md
│   │   ├── workspace.md
│   │   ├── urdf-basics.md
│   │   ├── nodes-topics.md
│   │   └── exercises.md
│   ├── module2/         # Digital Twin Simulation
│   │   ├── _category_.json
│   │   ├── overview.md
│   │   ├── gazebo-setup.md
│   │   ├── physics-sim.md
│   │   ├── unity-integration.md
│   │   ├── sensor-sim.md
│   │   └── exercises.md
│   ├── module3/         # AI Perception & Navigation
│   │   ├── _category_.json
│   │   ├── overview.md
│   │   ├── isaac-sim-setup.md
│   │   ├── perception.md
│   │   ├── navigation-rl.md
│   │   ├── slam.md
│   │   └── exercises.md
│   ├── module4/         # VLA Integration
│   │   ├── _category_.json
│   │   ├── overview.md
│   │   ├── speech-recognition.md
│   │   ├── llm-planning.md
│   │   ├── action-execution.md
│   │   └── exercises.md
│   └── module5/         # Capstone Project
│       ├── _category_.json
│       ├── overview.md
│       ├── project-requirements.md
│       ├── sim-to-real.md
│       ├── deployment.md
│       └── rubric.md
├── static/              # Static assets
│   └── img/
│       ├── module1/     # ROS 2 diagrams
│       ├── module2/     # Gazebo/Unity screenshots
│       ├── module3/     # Isaac Sim visualizations
│       ├── module4/     # VLA pipeline diagrams
│       └── module5/     # Hardware setup photos
├── examples/            # Tested code examples (outside Docusaurus build)
│   ├── module1/
│   │   ├── ros2-basics/
│   │   │   ├── README.md
│   │   │   ├── urdf_publisher.py
│   │   │   └── joint_controller.py
│   │   └── urdf-models/
│   │       └── simple_arm.urdf
│   ├── module2/
│   │   ├── gazebo-worlds/
│   │   └── unity-scenes/
│   ├── module3/
│   │   ├── isaac-gym/
│   │   └── perception/
│   ├── module4/
│   │   └── vla-pipeline/
│   └── module5/
│       └── capstone-template/
├── src/                 # Custom React components (MDX)
│   ├── components/
│   │   ├── CodeSandbox.jsx     # Interactive code editor
│   │   ├── HardwareSpec.jsx    # Hardware requirement tables
│   │   └── PerformanceMetrics.jsx
│   └── pages/
│       ├── index.js             # Custom homepage
│       └── about.md
├── sidebars.js          # Sidebar configuration
├── docusaurus.config.js # Main Docusaurus config
├── package.json         # Node dependencies
├── .github/
│   └── workflows/
│       └── deploy.yml   # GitHub Actions for gh-pages deployment
└── README.md            # Project overview for contributors
```

**Structure Decision**: Docusaurus documentation project with modular content organization. Each module (5 total) gets own directory under `/docs/` with 4-6 subsections per module. Code examples stored separately in `/examples/` with README + tests, then referenced/embedded in docs. Static diagrams in `/static/img/[module]/`. Custom React components in `/src/components/` for interactive elements (hardware specs, performance tables). GitHub Actions workflow deploys to `gh-pages` branch automatically on merge to `main`.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations identified. Constitution Check passed all 6 principles.

## Phase 0: Research & Technology Selection

**Objective**: Resolve all technical unknowns, establish best practices for Docusaurus documentation, and research credible sources for robotics content.

### Research Tasks

1. **Docusaurus Best Practices**
   - **Question**: What Docusaurus theme and plugins optimize for technical education (code highlighting, diagrams, search)?
   - **Research approach**: Review Docusaurus showcase (docusaurus.io/showcase), analyze top-rated technical documentation sites (React docs, Kubernetes docs, TensorFlow docs)
   - **Output**: Theme selection rationale, plugin recommendations (search, diagrams, code sandboxes)

2. **MDX vs. Markdown Decision**
   - **Question**: When to use MDX (interactive React components) vs. plain Markdown for robotics content?
   - **Research approach**: Evaluate MDX overhead (build time, complexity) vs. benefits (interactive hardware specs, embedded simulators, performance tables)
   - **Output**: Decision matrix with use cases for each format

3. **ROS 2 Documentation Standards**
   - **Question**: What citation format and versioning approach for ROS 2 APIs (changing rapidly)?
   - **Research approach**: Review ROS 2 documentation style guide, analyze how official tutorials handle API versioning
   - **Output**: Citation template for ROS 2 docs, strategy for handling API changes across Humble/Iron/Jazzy

4. **NVIDIA Isaac Sim Licensing**
   - **Question**: Can Isaac Sim screenshots/code be used in open-source educational content?
   - **Research approach**: Review NVIDIA Isaac Sim EULA, developer program terms, contact NVIDIA DevRel if unclear
   - **Output**: Licensing confirmation, attribution requirements

5. **APA Citation Management**
   - **Question**: What tool/workflow for managing ~50+ APA citations (papers, docs, GitHub repos)?
   - **Research approach**: Evaluate Zotero (open-source), BibTeX integration, manual Markdown approach
   - **Output**: Citation workflow, template for `docs/references.md`

6. **Code Example Validation Strategy**
   - **Question**: How to automate testing of ROS 2/Gazebo/Isaac Sim examples (some require GUI/GPU)?
   - **Research approach**: Research ROS 2 launch file testing, Gazebo headless mode, Isaac Sim Docker containers
   - **Output**: CI/CD strategy for code validation, test harness design

7. **Sim-to-Real Gap Quantification**
   - **Question**: What peer-reviewed papers provide data on sim-to-real performance degradation for RL policies?
   - **Research approach**: Search Google Scholar for "sim-to-real transfer robotics" + "domain randomization" + "Isaac Sim"
   - **Output**: Curated list of 5-10 papers with citation-worthy data (e.g., 15-20% degradation typical)

8. **Hardware Setup Diagrams**
   - **Question**: What diagramming tool for hardware connections (Jetson + RealSense + robot)?
   - **Research approach**: Evaluate Mermaid.js (inline Markdown), Excalidraw (hand-drawn), Fritzing (circuit diagrams)
   - **Output**: Diagramming tool selection, template for lab setup diagrams

9. **GitHub Pages Performance Optimization**
   - **Question**: How to achieve < 3s page load with large code examples and diagrams?
   - **Research approach**: Review Docusaurus performance docs, analyze Lighthouse reports from similar sites
   - **Output**: Optimization checklist (lazy loading images, code splitting, CDN for assets)

10. **Accessibility Compliance**
    - **Question**: What WCAG AA requirements apply to educational robotics content?
    - **Research approach**: Review WCAG 2.1 AA guidelines, analyze accessibility best practices for code-heavy documentation
    - **Output**: Accessibility checklist (alt text, color contrast, keyboard navigation, screen reader support)

### Expected Outcomes (research.md)

**File**: `specs/001-hackathon-guide/research.md`

**Structure**:
```markdown
# Research: Physical AI & Humanoid Robotics Hackathon Guide

## Technology Decisions

### 1. Docusaurus Theme & Plugins

**Decision**: Use Docusaurus Classic theme with plugins: `@docusaurus/theme-mermaid`, `@docusaurus/plugin-ideal-image`, `docusaurus-lunr-search`

**Rationale**:
- Classic theme proven for technical docs (React, Kubernetes use it)
- Mermaid plugin enables inline diagrams (system architectures, data flow)
- Ideal-image plugin auto-optimizes images (< 500KB target)
- Lunr search provides offline search without external service

**Alternatives considered**:
- Custom theme: Rejected due to 12-week timeline constraint
- Algolia DocSearch: Rejected due to manual approval process, Lunr sufficient for ~30 pages

### 2. MDX vs. Markdown

**Decision**: Use Markdown (.md) for all content, MDX (.mdx) only for 3 pages: hardware specs, performance benchmarks, interactive exercises

**Rationale**:
- Markdown simpler for writers, faster build times (6 of 30 pages have MDX overhead)
- MDX reserved for pages needing React components (hardware comparison tables, embedded CodeSandbox)
- 90% of content (tutorials, concepts) requires no interactivity

**Alternatives considered**:
- Full MDX: Rejected due to build time penalty, unnecessary complexity for static content

[... continue for all 10 research tasks ...]

## Credible Sources Identified

### ROS 2 Documentation
- Official: https://docs.ros.org/en/humble/ (ROS 2 Humble Hawksbill)
- Citation format: Robot Operating System 2. (2023). ROS 2 Humble Documentation. Retrieved from https://docs.ros.org/en/humble/

### NVIDIA Isaac Sim
- Official: https://docs.nvidia.com/isaac/doc/index.html
- Licensing: NVIDIA Developer Program EULA permits educational use with attribution
- Citation format: NVIDIA Corporation. (2023). Isaac Sim Documentation (Version 2023.1.1). Retrieved from https://docs.nvidia.com/isaac/

### Peer-Reviewed Papers (Sim-to-Real)
1. Peng, X. B., et al. (2018). "Sim-to-real transfer of robotic control with dynamics randomization." *IEEE Conference on Robotics and Automation (ICRA)*, 3803-3810. DOI: 10.1109/ICRA.2018.8460528
2. Tobin, J., et al. (2017). "Domain randomization for transferring deep neural networks from simulation to the real world." *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 23-30. DOI: 10.1109/IROS.2017.8202133
[... 8 more papers ...]

### Hardware Specifications
- Jetson Orin Nano: https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit
- RealSense D435i: https://www.intelrealsense.com/depth-camera-d435i/

## Code Validation Strategy

**Decision**: 3-tier testing approach
1. **Tier 1 - Syntax check**: Python linting (flake8) + ROS 2 package build (colcon build) - automated in CI
2. **Tier 2 - Functional test**: ROS 2 launch files with assertions (e.g., topic published at 10Hz) - automated for headless examples
3. **Tier 3 - Manual validation**: Gazebo/Isaac Sim GUI examples - screenshots + instructions for human verification

**Rationale**: Full automation impossible for GPU-dependent Isaac Sim examples. Tier 1+2 catch 80% of errors, Tier 3 documented for maintainers.

## Diagramming Tools

**Decision**: Mermaid.js for system diagrams, Excalidraw for hardware schematics

**Rationale**:
- Mermaid.js: Native Docusaurus support, version-controllable (text-based), good for node graphs, sequence diagrams
- Excalidraw: Hand-drawn aesthetic matches educational tone, export to SVG, free and open-source

[... additional sections ...]
```

## Phase 1: Content Architecture & Module Outlines

**Objective**: Define content structure (data model), create detailed module outlines (contracts), and provide quickstart guide for contributors.

### 1.1 Data Model (Content Entities)

**File**: `specs/001-hackathon-guide/data-model.md`

**Content Entities**:

1. **Module** (5 instances: ROS 2, Simulation, AI, VLA, Capstone)
   - **Attributes**: module_id (1-5), title, description, duration_weeks (1-3), prerequisites (list of module_ids), learning_objectives (3-5 bullets), word_count_target (4000-6000)
   - **Relationships**: Contains 4-6 Chapters, precedes next Module
   - **Validation**: Sum of Chapter word_counts ≤ 6000, prerequisites form DAG (no cycles)

2. **Chapter** (25-30 instances across 5 modules)
   - **Attributes**: chapter_id (e.g., "m1-c2"), module_id, title, sidebar_position, word_count (≤ 2000), file_path (e.g., "docs/module1/workspace.md")
   - **Relationships**: Belongs to Module, contains 3+ Exercises, references 5-10 Citations
   - **Validation**: Follows template structure (objectives → prerequisites → content → summary → exercises → references), frontmatter complete

3. **Code Example** (~100 instances)
   - **Attributes**: example_id, module_id, file_path (e.g., "examples/module1/ros2-basics/urdf_publisher.py"), language (Python, XML, YAML), dependencies (list with versions), test_status (passed/failed/manual)
   - **Relationships**: Embedded in Chapter, tested by Test Case
   - **Validation**: Syntax valid, dependencies version-pinned, includes README, comments explain WHY not WHAT

4. **Diagram** (~50 instances)
   - **Attributes**: diagram_id, module_id, file_path (e.g., "static/img/module1/ros2-node-graph.svg"), type (mermaid/excalidraw), alt_text, file_size_kb (< 500)
   - **Relationships**: Embedded in Chapter
   - **Validation**: Alt text descriptive (not "image of robot"), file size < 500KB, SVG optimized

5. **Exercise** (~75-90 instances, 3 per chapter)
   - **Attributes**: exercise_id, chapter_id, type (conceptual/computational/implementation), difficulty (1-3), estimated_time_min (15-60), solution_provided (yes/no)
   - **Relationships**: Belongs to Chapter
   - **Validation**: At least one of each type per chapter, solutions in separate file (not inline)

6. **Citation** (~50-100 instances)
   - **Attributes**: citation_id, type (paper/doc/repo/book), apa_formatted_text, url, access_date
   - **Relationships**: Referenced by Chapter
   - **Validation**: APA 7th edition compliant, URL accessible (link check), unique citation_id

7. **Glossary Term** (~30-50 instances)
   - **Attributes**: term, definition, module_first_use, related_terms (list)
   - **Relationships**: Linked from Chapter
   - **Validation**: Defined in docs/glossary.md, linked on first use per chapter

### 1.2 Module Outlines (Contracts)

**File**: `specs/001-hackathon-guide/contracts/module1-outline.md` (replicate for modules 2-5)

**Example** (Module 1: ROS 2 Foundation):

```markdown
# Module 1 Outline: ROS 2 Foundation for Humanoid Control

**Duration**: Weeks 1-2 (2 weeks)
**Prerequisites**: Python programming, Linux command line
**Learning Objectives**:
1. Install ROS 2 Humble and create workspaces using colcon
2. Define humanoid robot models using URDF syntax (joints, links, collision, visual)
3. Write Python nodes to publish/subscribe to joint state topics
4. Visualize robot configurations in RViz and understand TF transforms

**Word Count Target**: 5000-6000 words (split across 6 chapters)

## Chapter Structure

### 1.1 Overview (500 words)
- **File**: `docs/module1/overview.md`
- **Sidebar Position**: 1
- **Content**:
  - What is ROS 2 and why roboticists use it
  - Key concepts preview: nodes, topics, services, URDF
  - Motivation: "Without ROS 2, every robotics project reinvents communication layer"
- **Diagrams**: 1 (high-level ROS 2 architecture)
- **Code Examples**: 0
- **Exercises**: 1 conceptual ("Explain pub-sub vs. request-reply patterns")

### 1.2 Installation & Workspace Setup (800 words)
- **File**: `docs/module1/installation.md`
- **Sidebar Position**: 2
- **Content**:
  - Ubuntu 22.04 prerequisite check
  - ROS 2 Humble installation (apt-get vs. source build)
  - Creating workspace with colcon: directory structure, package.xml, setup.bash
  - Troubleshooting common errors (missing dependencies, path issues)
- **Diagrams**: 1 (workspace directory tree)
- **Code Examples**: 3
  - `install_ros2.sh` (bash script for apt installation)
  - `create_workspace.sh` (workspace initialization)
  - `package.xml` template
- **Exercises**: 2
  - Computational: "Verify ROS 2 installation with `ros2 doctor` and interpret output"
  - Implementation: "Create workspace and build example package"
- **Citations**: 2 (ROS 2 installation docs, colcon documentation)

### 1.3 URDF Basics: Defining Humanoid Robots (1000 words)
- **File**: `docs/module1/urdf-basics.md`
- **Sidebar Position**: 3
- **Content**:
  - URDF XML structure: `<robot>`, `<link>`, `<joint>`
  - Joint types: revolute, prismatic, fixed
  - Link properties: visual (STL mesh), collision (simplified geometry), inertial (mass, inertia tensor)
  - Coordinate frames and transforms
  - Example: 3-DOF robotic arm URDF
  - Common pitfalls: non-matching parent/child link names, missing inertia
- **Diagrams**: 2
  - Kinematic tree visualization
  - Coordinate frame conventions (DH parameters)
- **Code Examples**: 2
  - `simple_arm.urdf` (complete 3-DOF arm with comments)
  - `urdf_visualizer.launch.py` (ROS 2 launch file for RViz)
- **Exercises**: 3
  - Conceptual: "Explain why collision geometry differs from visual geometry"
  - Computational: "Calculate inertia tensor for cylindrical link (mass 2kg, radius 0.05m, length 0.3m)"
  - Implementation: "Modify simple_arm.urdf to add 4th DOF and visualize in RViz"
- **Citations**: 3 (URDF specification, RViz documentation, Siciliano robotics textbook for DH parameters)

### 1.4 Nodes and Topics: Publisher-Subscriber Pattern (900 words)
[... similar structure ...]

### 1.5 Services and Actions: Request-Reply Patterns (700 words)
[... similar structure ...]

### 1.6 Module 1 Exercises & Project (1100 words)
[... similar structure ...]

## Summary

By end of Module 1, students will have:
- Working ROS 2 Humble installation
- Understanding of URDF for robot modeling
- Ability to write publisher/subscriber nodes in Python
- Familiarity with RViz for visualization

This forms foundation for Module 2 (simulation) and Module 3 (AI integration).
```

**Files to create**:
- `contracts/module1-outline.md` (ROS 2 Foundation)
- `contracts/module2-outline.md` (Digital Twin Simulation)
- `contracts/module3-outline.md` (AI Perception & Navigation)
- `contracts/module4-outline.md` (VLA Integration)
- `contracts/module5-outline.md` (Capstone Project)
- `contracts/chapter-template.md` (Reusable template for all chapters)

### 1.3 Quickstart for Contributors

**File**: `specs/001-hackathon-guide/quickstart.md`

**Content**:
```markdown
# Quickstart: Contributing to Physical AI & Humanoid Robotics Guide

## Prerequisites

- Node.js 18+ and npm 8+
- Git
- Text editor (VS Code recommended)
- Python 3.10+ (for code example validation)
- ROS 2 Humble (optional, for testing examples)

## Setup

1. Clone repository:
   ```bash
   git clone https://github.com/[org]/physical-ai-robotics-guide.git
   cd physical-ai-robotics-guide
   ```

2. Install Docusaurus dependencies:
   ```bash
   cd my-website
   npm install
   ```

3. Start local development server:
   ```bash
   npm start
   ```
   Opens browser at `http://localhost:3000` with hot reload.

## Project Structure

- `/docs/`: Markdown content (modules, chapters)
- `/static/img/`: Images and diagrams
- `/examples/`: Code examples (separate from Docusaurus build)
- `/src/components/`: Custom React components for MDX
- `sidebars.js`: Navigation configuration
- `docusaurus.config.js`: Site configuration

## Writing Workflow

1. **Check specification**: Read `specs/001-hackathon-guide/spec.md` and relevant module outline in `contracts/`
2. **Create feature branch**: `git checkout -b chapter/module1-urdf-basics`
3. **Write content**: Follow chapter template in `contracts/chapter-template.md`
4. **Add frontmatter**:
   ```markdown
   ---
   title: "URDF Basics: Defining Humanoid Robots"
   description: "Learn URDF XML syntax for modeling robot kinematics, joints, and links"
   keywords: [urdf, robot model, joints, links, ros2]
   sidebar_position: 3
   ---
   ```
5. **Test locally**: `npm start` and verify rendering
6. **Validate build**: `npm run build` (must succeed)
7. **Check links**: `npm run check-links` (custom script)
8. **Commit**: `git add docs/module1/urdf-basics.md && git commit -m "docs: add URDF basics chapter"`
9. **Create PR**: `gh pr create --title "Add Module 1 Chapter 3: URDF Basics"`

## Constitution Compliance

Every chapter MUST include:
1. **Learning Objectives** (3-5 bullets)
2. **Prerequisites** (prior chapters + external knowledge)
3. **Content** (Introduction → Core Concepts → Examples → Applications)
4. **Summary** (key takeaways)
5. **Exercises** (min 3: conceptual, computational, implementation)
6. **References** (APA citations)

See `.specify/memory/constitution.md` for full requirements.

## Code Examples

1. **Create example in `/examples/[module]/`**:
   ```
   examples/module1/ros2-basics/
   ├── README.md           # Setup instructions, expected output
   ├── urdf_publisher.py   # Tested, version-pinned dependencies
   └── requirements.txt    # Python packages with versions
   ```

2. **Test example**:
   ```bash
   cd examples/module1/ros2-basics
   pip install -r requirements.txt
   python urdf_publisher.py
   # Verify output matches README
   ```

3. **Embed in docs**:
   ````markdown
   ```python title="examples/module1/ros2-basics/urdf_publisher.py"
   # Requires: rclpy>=3.3.0, numpy>=1.24.0
   import rclpy
   from rclpy.node import Node
   # ... rest of code ...
   ```
   ````

## Diagrams

1. **Mermaid (inline)**:
   ````markdown
   ```mermaid
   graph LR
     A[Publisher Node] -->|/joint_states| B[Subscriber Node]
   ```
   ````

2. **Excalidraw (external)**:
   - Draw in Excalidraw (https://excalidraw.com)
   - Export as SVG
   - Optimize: `svgo static/img/module1/hardware-setup.svg`
   - Embed: `![Hardware setup diagram](../../../static/img/module1/hardware-setup.svg)`

## APA Citations

Add to `docs/references.md`:
```markdown
Robot Operating System 2. (2023). *ROS 2 Humble Documentation*. Retrieved December 7, 2025, from https://docs.ros.org/en/humble/
```

Link in chapter:
```markdown
... as described in the ROS 2 documentation ([ROS 2 Humble](./references.md#ros-2-humble)) ...
```

## Quality Checks

Before PR:
- [ ] `npm run build` succeeds
- [ ] No broken links (`npm run check-links`)
- [ ] Spell check passed
- [ ] Code examples tested
- [ ] Lighthouse score > 90 (performance, accessibility)
- [ ] All images < 500KB
- [ ] APA citations complete

## Getting Help

- Constitution: `.specify/memory/constitution.md`
- Spec: `specs/001-hackathon-guide/spec.md`
- Module outlines: `specs/001-hackathon-guide/contracts/`
- Discord: #physical-ai-guide channel
```

### 1.4 Update Agent Context

Run `.specify/scripts/bash/update-agent-context.sh` to add Docusaurus/robotics context to Claude Code's memory.

**Expected additions to agent context file**:
- Docusaurus 3.x for static site generation
- Markdown/MDX for content authoring
- Mermaid.js for diagrams
- ROS 2 Humble as primary robotics framework
- NVIDIA Isaac Sim for AI simulation
- APA 7th edition citation format
- GitHub Pages deployment workflow

## Phase 2: Task Generation (Deferred to /sp.tasks)

**Note**: Detailed task breakdown occurs in `/sp.tasks` command. This plan provides sufficient structure (module outlines, chapter templates, content entities) for task generation.

**Expected task categories** (preview):
1. **Setup tasks**: Docusaurus initialization, GitHub Pages workflow, project scaffolding
2. **Module 1 tasks**: Write 6 chapters (installation, URDF, nodes/topics, services, exercises)
3. **Module 2 tasks**: Write 6 chapters (Gazebo setup, physics, Unity, sensors, exercises)
4. **Module 3 tasks**: Write 6 chapters (Isaac Sim, perception, navigation, SLAM, exercises)
5. **Module 4 tasks**: Write 5 chapters (speech, LLM planning, action execution, exercises)
6. **Module 5 tasks**: Write 5 chapters (project requirements, sim-to-real, deployment, rubric)
7. **Cross-cutting tasks**: Glossary, notation guide, references, diagrams, code examples
8. **Validation tasks**: Build checks, link validation, spell check, Lighthouse audit

## Decisions Needing Documentation

### 1. Docusaurus Theme Selection

**Options**:
- A) Classic theme (default)
- B) Custom theme (branded for robotics)

**Tradeoffs**:
- A) Classic: Proven, well-documented, used by React/Kubernetes. Low risk, fast setup. Limited visual customization.
- B) Custom: Unique branding, tailored UX. High development cost (weeks), maintenance burden.

**Decision**: **A) Classic theme** with minor CSS overrides for branding. Rationale: 12-week timeline prioritizes content over aesthetics. Classic theme meets all functional requirements (search, responsive, accessible).

### 2. MDX vs. Markdown for Interactive Sections

**Options**:
- A) Markdown only (simple, fast builds)
- B) MDX for all pages (consistent React components)
- C) Hybrid: Markdown default, MDX for interactive pages

**Tradeoffs**:
- A) Markdown: Faster builds, easier for non-developers. No interactivity (hardware comparison tables require static content).
- B) Full MDX: Consistent component use. Slower builds, higher complexity for simple pages.
- C) Hybrid: Best of both. Requires discipline to know when to use each.

**Decision**: **C) Hybrid** - Markdown for 90% of content (tutorials, concepts), MDX only for hardware specs, performance benchmarks, interactive exercises. Rationale: Balances simplicity (Markdown) with capability (MDX components) where needed.

### 3. Navigation Style

**Options**:
- A) Single sidebar (all modules in one nav tree)
- B) Multi-sidebar (separate nav per module)
- C) Top-level tabs (module tabs, chapter sidebar)

**Tradeoffs**:
- A) Single: Simple, all content visible. Can become long/unwieldy with 30 pages.
- B) Multi: Focused navigation per module. Users lose global context (which module am I in?).
- C) Tabs: Clear module separation, compact sidebar. Requires custom nav implementation.

**Decision**: **A) Single sidebar** with collapsible categories per module. Rationale: 30 pages manageable with categories. Users can see full structure at glance. Matches Docusaurus best practices.

### 4. Folder Naming Conventions

**Options**:
- A) Numbered: `01-installation.md`, `02-urdf-basics.md`
- B) Descriptive: `installation.md`, `urdf-basics.md` (sidebar_position in frontmatter)
- C) Hybrid: Folders numbered, files descriptive: `module1/`, `01-installation.md`

**Tradeoffs**:
- A) Numbered files: Obvious order in file explorer. Refactoring requires renaming files.
- B) Descriptive: Clean names, order via metadata. File explorer order meaningless.
- C) Hybrid: Folders show module order, files descriptive. Best clarity.

**Decision**: **B) Descriptive file names** with `sidebar_position` in frontmatter. Rationale: Docusaurus handles ordering via metadata. Descriptive names easier to navigate in editor. Refactoring only requires metadata change, not file renames.

### 5. Versioning Strategy

**Options**:
- A) Versioned docs (v1.0, v2.0 with /docs/version-X.X/)
- B) Rolling updates (single /docs/, git tags for releases)

**Tradeoffs**:
- A) Versioned: Users can access old content if ROS 2 versions diverge. Maintenance burden (fix bugs in multiple versions).
- B) Rolling: Always current, single source of truth. Old content disappears (problematic if students mid-course).

**Decision**: **B) Rolling updates** with git tags (v1.0-ros2-humble, v2.0-ros2-jazzy). Rationale: Course tied to ROS 2 Humble (LTS). By time Humble EOL (2027), course will be redesigned. Git tags allow rollback if needed. Simpler maintenance.

## Testing Strategy

### Build Validation

**Test**: Docusaurus build succeeds without errors/warnings
**Command**: `npm run build`
**Acceptance**: Exit code 0, no console errors, `build/` directory generated
**Frequency**: Every commit (GitHub Actions CI)

### Link Validation

**Test**: No broken internal or external links
**Tool**: `docusaurus-plugin-broken-links` (internal), custom script for external
**Acceptance**: 0 broken links reported
**Frequency**: Pre-merge (PR check)

### Spell Check

**Test**: No spelling errors in technical terms
**Tool**: `cspell` with custom dictionary (`robotics-terms.txt`)
**Acceptance**: 0 unknown words (or added to dictionary with justification)
**Frequency**: Pre-merge (PR check)

### Code Example Validation

**Test**: All Python examples run without errors
**Approach**:
- **Tier 1**: Syntax check (`flake8`) - automated
- **Tier 2**: Functional test (ROS 2 launch files with assertions) - automated for headless
- **Tier 3**: Manual validation (Gazebo/Isaac Sim GUI examples) - documented screenshots
**Acceptance**: Tier 1+2 pass, Tier 3 screenshots match expected output
**Frequency**: Per example (before embedding in docs)

### Performance Validation (Lighthouse)

**Test**: Page load performance meets targets
**Tool**: Lighthouse CI
**Metrics**:
- Performance score ≥ 90
- Initial page load < 3s
- Largest Contentful Paint < 2.5s
- Cumulative Layout Shift < 0.1
**Acceptance**: All metrics pass for homepage + 5 random pages
**Frequency**: Pre-release (before merging to main)

### Accessibility Validation

**Test**: WCAG 2.1 AA compliance
**Tool**: Lighthouse accessibility audit + axe DevTools
**Checks**:
- Alt text on all images
- Color contrast ≥ 4.5:1 for text
- Keyboard navigation works
- Screen reader friendly
**Acceptance**: Lighthouse accessibility score ≥ 95
**Frequency**: Pre-release

### APA Citation Validation

**Test**: All citations follow APA 7th edition format
**Tool**: Manual review + APA checker (Scribbr.com)
**Acceptance**: 0 formatting errors in `docs/references.md`
**Frequency**: Before final review

### Constitution Compliance

**Test**: Each module/chapter meets constitution requirements
**Checklist** (per chapter):
- [ ] Learning objectives stated (3-5 bullets)
- [ ] Prerequisites listed
- [ ] Structure: Intro → Concepts → Examples → Applications → Summary → Exercises → References
- [ ] Min 3 exercises (conceptual, computational, implementation)
- [ ] All code examples tested
- [ ] Diagrams have descriptive alt text
- [ ] APA citations included
- [ ] Word count ≤ 2000 per page
**Acceptance**: All checkboxes pass
**Frequency**: Per chapter (during writing)

### GitHub Pages Deployment Test

**Test**: Automated deployment to gh-pages succeeds
**Tool**: GitHub Actions workflow (`.github/workflows/deploy.yml`)
**Steps**:
1. Trigger on merge to `main`
2. Run `npm run build`
3. Deploy `build/` to `gh-pages` branch
4. Verify site live at `https://[org].github.io/physical-ai-robotics-guide/`
**Acceptance**: Site loads, navigation works, no 404 errors
**Frequency**: Every merge to main (automated)

### Editorial Review

**Test**: Content clarity, correctness, coherence
**Process**:
1. **Clarity**: Peer review by non-robotics developer (can they follow?)
2. **Correctness**: Technical review by robotics expert (formulas, best practices accurate?)
3. **Coherence**: Read full module top-to-bottom (logical flow?)
**Acceptance**: Reviewer approves with < 3 minor comments
**Frequency**: Per module (before marking complete)

## Project Phases

### Phase 0: Research (Weeks 0-1)

**Objectives**:
- Resolve all technology decisions (theme, MDX, diagramming tools)
- Identify credible sources (ROS 2 docs, Isaac Sim docs, peer-reviewed papers)
- Establish code validation strategy

**Deliverables**:
- `research.md` with 10 decision rationales
- Curated list of 50+ APA citations
- Code validation test harness

**Success Criteria**:
- All "NEEDS CLARIFICATION" items resolved
- Docusaurus project initialized with theme/plugins
- First module outline (Module 1) drafted

### Phase 1: Foundation (Weeks 1-2)

**Objectives**:
- Create Docusaurus project scaffolding
- Set up GitHub Pages deployment workflow
- Write glossary, notation guide, references template
- Create chapter template and module outlines

**Deliverables**:
- Docusaurus project running locally (`npm start`)
- GitHub Actions workflow deploying to gh-pages
- `docs/glossary.md`, `docs/notation.md`, `docs/references.md`
- 5 module outlines in `contracts/`
- Chapter template in `contracts/chapter-template.md`
- `quickstart.md` for contributors

**Success Criteria**:
- `npm run build` succeeds
- Site deployed to GitHub Pages (placeholder content)
- All module outlines reviewed and approved
- Agent context updated with Docusaurus/robotics tech

### Phase 2: Analysis (Weeks 3-4)

**Objectives**:
- Expand module outlines into detailed chapter specs
- Research and test all code examples (stub implementations)
- Create hardware setup diagrams
- Draft learning objectives and exercises for each chapter

**Deliverables**:
- 30 chapter specifications (detailed outlines)
- 100 code example stubs (with dependencies, no implementation)
- 50 diagram placeholders (with alt text)
- Exercises catalog (3 per chapter × 30 = 90 exercises)

**Success Criteria**:
- Each chapter spec includes: word count target, diagrams list, code examples list, exercises list, citations list
- Code example stubs pass Tier 1 validation (syntax)
- Diagram placeholders have descriptive alt text

### Phase 3: Synthesis (Weeks 5-12)

**Objectives**:
- Write all 30 chapters following spec-driven workflow
- Implement all 100 code examples and validate
- Create all 50 diagrams
- Integrate with Docusaurus and test deployment

**Deliverables**:
- 30 complete chapters (4000-6000 words per module)
- 100 tested code examples
- 50 optimized diagrams (< 500KB each)
- Fully functional Docusaurus site deployed to GitHub Pages

**Success Criteria**:
- All constitution compliance checks pass
- `npm run build` succeeds with 0 warnings
- Lighthouse scores ≥ 90 (performance, accessibility, SEO)
- Editorial review approved for all modules
- GitHub Pages site live and accessible

## Risk Mitigation

**Risk 1**: Isaac Sim examples cannot be tested in CI (require GPU)
**Mitigation**: Tier 3 manual validation with screenshots. Documented setup instructions for maintainers.

**Risk 2**: ROS 2 API changes break code examples during 12-week timeline
**Mitigation**: Version-pin all dependencies. Test against ROS 2 Humble (LTS until 2027). Add "Last tested" date to examples.

**Risk 3**: Word count exceeds 6000 per module (Docusaurus build slows)
**Mitigation**: Split long chapters into subsections (max 2000 words per file). Use Docusaurus categories for logical grouping.

**Risk 4**: APA citations incomplete or inconsistent
**Mitigation**: Centralized `docs/references.md`, validated with APA checker, peer review before merge.

**Risk 5**: Diagrams not accessible (missing alt text)
**Mitigation**: Accessibility checklist in PR template, Lighthouse audit enforces alt text.

## Next Steps

1. **Run Phase 0 research** (`/sp.plan` generates `research.md` automatically)
2. **Review and approve module outlines** (stakeholder sign-off on contracts)
3. **Proceed to `/sp.tasks`** to break down writing into granular tasks
4. **Begin Phase 1 foundation work** (Docusaurus setup, scaffolding)

---

**Plan Status**: Complete - Ready for task generation via `/sp.tasks`
