---
marp: true
theme: default
paginate: true
size: 16:9
backgroundColor: '#f8fafc'
color: '#1e293b'
header: 'Academic Resources in Robotics'
footer: 'yyf | PolyU | Aug 2025'
math: katex
style: |
  section { 
    font-family: 'Segoe UI', 'Helvetica Neue', Arial, sans-serif;
    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
    color: white;
    font-size: 24px;
  }
  h1 { 
    color: #ffffff; 
    text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
    border-bottom: 3px solid #fbbf24;
    padding-bottom: 10px;
  }
  h2 { 
    color: #fef3c7; 
    border-left: 4px solid #fbbf24;
    padding-left: 15px;
  }
  h3 { color: #fde68a; }
  table {
    background: rgba(255,255,255,0.95);
    color: #1f2937;
    border-radius: 10px;
    overflow: hidden;
    box-shadow: 0 10px 25px rgba(0,0,0,0.2);
    font-size: 1.2em;
  }
  table th {
    background: #374151;
    color: white;
    font-weight: 600;
    font-size: 0.8em;
    padding: 8px;
  }
  table td {
    padding: 6px 8px;
    font-size: 0.75em;
  }
  table tr:nth-child(even) {
    background: rgba(243,244,246,0.8);
  }
  strong { color: #1e40af; }
  ul li { margin: 5px 0; }
  /* Small table style for featured papers */
  .small-table table {
    font-size: 1em;
  }
  .small-table table th {
    font-size: 0.7em;
    padding: 4px;
  }
  .small-table table td {
    padding: 3px 4px;
    font-size: 0.6em;
  }
  
---

# 🤖 Academic Resources in Robotics
### Top-tier Journals & Conferences Overview

---

## 📚 Top-tier Journals

| Abbreviation     | Cycle | Publisher | Description                           |
|------------------|-------|-----------|---------------------------------------|
| **IJRR**         | 30d   | SAGE      | International Journal of Robotics Research |
| **RAS**          | 30d   | Elsevier  | Robotics and Autonomous Systems      |
| **RA-L**         | 30d   | IEEE      | Robotics and Automation Letters      |
| **Science Robotics** | 30d | AAAS    | Science Robotics Journal             |
| **T-RO**         | 90d   | IEEE      | Transactions on Robotics             |
| **AURO**         | 90d   | Springer  | Autonomous Robots                    |
| **Nature Robotics** | 90d | Nature   | Nature Machine Intelligence          |

*Cycle indicates publication frequency in days*


---

## 🏛️ Top-tier Conferences

| Abbreviation | Cycle | Organization | Description                            |
|--------------|-------|--------------|----------------------------------------|
| **ICRA**     | 365d  | IEEE         | Int'l Conf. on Robotics and Automation |
| **IROS**     | 365d  | IEEE & RSJ   | Int'l Conf. on Intelligent Robots and Systems |
| **RSS**      | 365d  | RSS          | Robotics: Science and Systems          |
| **CoRL**     | 365d  | PMLR         | Conference on Robot Learning           |
| **ISRR**     | 730d  | IFRR         | Int'l Symposium of Robotics Research   |

### 📊 Publication Frequency Statistics
- **High-frequency Journals** (30d): IJRR, RAS, RA-L, Science Robotics
- **Medium-frequency Journals** (90d): T-RO, AURO, Nature Robotics  
- **Annual Conferences** (365d): ICRA, IROS, RSS, CoRL
- **Biennial Conference** (730d): ISRR

---

# 📋 Contents


| Month | Year | Status | Papers | Highlights |
|-------|------|--------|--------|------------|
| [**July**](#📄-july-2025-paper-review) | 2025 | ✅ Available | 6 papers | IJRR special collection |
| [**August**](#📄-august-2025-paper-review) | 2025 | 🔄 Coming Soon | TBD | ICRA & IROS selections |
| [**September**](#📄-september-2025-paper-review) | 2025 | 📅 Planned | TBD | CoRL & RSS highlights |



---

# 📄 July 2025 Paper Review
---

## 🔥 Featured Papers

<div class="small-table">

| No. | Venue | Title | Authors | Key Contribution |
|-----|-------|-------|---------|------------------|
| 1 | **IJRR** | RflyMAD: A Dataset for Multicopter Fault Detection and Health Assessment | Le et al. | Comprehensive fault detection dataset |
| 2 | **IJRR** | FusionPortableV2: A Unified Multi-Sensor Dataset for Generalized SLAM | Wei et al. | Multi-platform SLAM dataset |
| 3 | **IJRR** | BRNE: Mixed Strategy Nash Equilibrium for Crowd Navigation | Sun et al. | Bayesian Robot Navigation Engine |
| 4 | **IJRR** | Shared Visuo-Tactile Interactive Perception for Robust Object Pose Estimation | Murali et al. | Visuo-tactile shared perception framework |
| 5 | **IJRR** | Multi-Tactile Sensor Calibration via Motion Constraints | Yu et al. | Motion constraint calibration method |
| 6 | **IJRR** | JVS-SLAM: Joint Vector-Set Distribution SLAM | Inostroza et al. | Unified frontend-backend SLAM |

</div>

---
## 📖 Paper 1: RflyMAD Dataset

**RflyMAD: A Dataset for Multicopter Fault Detection and Health Assessment**

 **IJRR** | Le et al., Beihang University

### 🎯 Problem & Solution
- **Gap**: Lack of public fault detection datasets for multicopters
- **Solution**: Comprehensive dataset bridging simulation and real flight

### 📊 Dataset Overview
- **5,629 Flight Cases**: 2,566 SIL + 2,566 HIL + 497 Real flights
- **11 Fault Types**: Motor, Propeller, Sensors, Environmental  
- **6 Flight Modes**: Hover, Waypoints, Velocity, Circling, Accel/Decel
- **3 Platforms**: X200/X450/X680 multicopters (200mm-680mm)

---

## 📈 RflyMAD Research Details

### 🔬 Data Composition & Scale
<div class="small-table">

| Component | SIL | HIL | Real | Description |
|-----------|-----|-----|------|-------------|
| Motor Faults | 921 | 921 | 231 | 1-4 motors failure |
| Sensor Faults | 690 | 690 | 182 | IMU, GPS, Barometer |
| Environmental | 320 | 320 | - | Wind, Load changes |
| No Fault | 200 | 200 | 84 | Normal operations |

</div>

### 🎯 Research Contributions
- **Comprehensive Coverage**: Bridges simulation and real-world data
- **Multi-modal Data**: ULog, ROS bag, Telemetry, Ground Truth
- **Transfer Learning**: Validates sim-to-real generalization
- **Benchmark Dataset**: First public multicopter fault detection dataset

---

## 📖 Paper 2: FusionPortableV2 Dataset

**FusionPortableV2: A Unified Multi-Sensor Dataset for Generalized SLAM**

**IJRR** | Wei et al., HKUST & UCL

### 🎯 Problem & Solution
- **Gap**: SLAM algorithms lack generalization across platforms and environments
- **Solution**: Unified multi-sensor dataset spanning diverse platforms and scenarios

### 📊 Dataset Overview
- **27 Sequences**: 2.5 hours total, 38.7 km distance
- **4 Platforms**: Handheld, Legged robot, UGV, Vehicle  
- **Multi-sensors**: LiDAR, Stereo cameras, Event cameras, IMU, INS
- **12 Environments**: Campus, underground, highway, multi-layer parking

---

## 📖 Paper 3: BRNE Algorithm

**BRNE: Mixed Strategy Nash Equilibrium for Crowd Navigation**

**IJRR** | Sun et al., Northwestern Univ. & Honda Research Inst.

### 🎯 Problem & Solution
- **Pain Points**: Freezing robot (uncertainty), reciprocal dance (oscillation), real-time failure (O(N³) computation)
- **Solution**: BRNE with mixed strategy Nash equilibrium & Bayesian updates

### 🚀 Core Design
- **Game Model**: Captures human behavior uncertainty
- **Update**: Iterative Bayesian (prior: trajectory; likelihood: collision risk)
- **Theoretical Gain**: Provable global equilibrium
- **Real-time**: O(TM²N²) (5 agents: Jetson; 8 agents: laptop)

---

## 📖 Paper 3: BRNE Algorithm

**Engineering: From Theory to Deployment**

**Key Implementation & Validation**

### 🔧 Technical Details
- **Strategy Representation**: Gaussian Process (GP) sampling (M=50-100 trajectories)
  - Mean: Robot (RRT) / Human (constant velocity)
  - Kernel: Smooth constraint (e.g., RBF)
- **Weight Update**: Init (1/M) → Likelihood ($L \propto \exp(-\gamma\sum R_{ik})$) → Posterior (normalized)

### 📊 Validation Results
- **Simulation (ETH/UCY)**: ↓30-50% collision rate, ↓15-25% navigation time
- **Hardware**: Quadruped (Jetson NX) + 3-5 humans (no freezing/oscillation)
- **Human-Level**: Matches real pedestrian trajectory consistency

---

## 📖 Paper 4: Visuo-Tactile Shared Perception

**Shared Visuo-Tactile Interactive Perception for Robust Object Pose Estimation**

**IJRR** | Murali, Porr, Kaboli 

### 🎯 Problem & Solution
- **Pain Points**: 
  1. Visual-only fails on transparent/specular objects; tactile-only is sparse/local.
  2. Mono-modal shared perception can’t handle cross-modal (vision+touch) mismatch.
- **Solution**: Two-robot shared visuo-tactile framework.


### 🚀 Core Design
- **Shared Perception**: UR5 + Franka Panda (Kinect) share scene data to declutter dense clutter.
- **S-TIQF**: Stochastic Translation-Invariant Quaternion Filter (Bayesian + stochastic optimization).
- **In Situ Calibration**: Visuo-tactile hand-eye calibration with arbitrary objects (no special targets).
- **Active Reconstruction**: Joint information gain criterion for NBV/NBT → reduce redundant actions.


---

## 📖 Paper 4: Visuo-Tactile Shared Perception

**Key Implementation & Validation**

**Technical Details & Experimental Results**

### 🔧 Technical Details
- **Scene Decluttering**: Declutter graph (edges = overlap/proximity; actions = grasp/push) → auto-singulate objects via semantic/grasp affordance networks.
- **Active Reconstruction**:
  - NBV: Hemisphere sampling (Panda reach: 855mm, radius: 550mm) → camera orientation toward object centroid.
  - NBT: Bounding box face sampling → touch direction = face normal.
  - Sensor Selection: Energy cost D(at) (prefer touch for transparent objects via IoU heuristic: IoUpc/rgb < ω).
- **S-TIQF Workflow**: Decouple rotation/translation → Bayesian update (prior: trajectory; likelihood: collision risk) → global optimal pose.


### 📊 Validation Results
- **Simulation (Standard Datasets)**: Outperforms SOTA in pose accuracy for dense visual + sparse tactile point clouds.
- **Real-Robot Tests**: Opaque/transparent/specular objects in dense clutter → no pose estimation failure.
- **Calibration**: In situ method avoids laborious procedures, ensures cross-modal data alignment.
- **Efficiency**: Joint info gain reduces redundant actions → faster reconstruction vs. mono-modal active perception.

---

## 📖 Paper 5: Multi-Tactile Sensor Calibration

**Multi-Tactile Sensor Calibration via Motion Constraints with Tactile Measurements**

**IJRR** | Yu et al., SJTU

### 🎯 Problem & Solution
- **Pain Points**: Multi-finger robots lack encoder-free calibration;No overlapping regions (no shared features like cameras); high-cost encoders unavailable for low-cost/soft hands.
- **Solution**: Calibrate via rigid object’s shared motion.

### 🚀 Core Design
- **Key Constraint**: Grasped object is rigid (shared unique motion for all sensors).
- **Motion Estimation**: Each sensor (e.g., GelSlim) infers object motion via contact pt registration.
- **Calibration Target**: Homogeneous transform matrix X (rotation + translation) between sensors.
- **No Object Prior**: Works for arbitrary object shapes/sizes (no CAD/models needed).


---

## 📖 Paper 5: Multi-Tactile Sensor Calibration

**Key Implementation & Validation**

### 🔧 Technical Details
1. **Object Motion Estimation**:
   - Sensor: GelSlim (vision-based tactile sensor) → captures contact 3D point clouds.
   - Process: Perturb object slightly → improved ICP → get motion matrix M (R + T) per sensor.

2. **Calibration Workflow**:
   - For 2 sensors: M₁ (sensor1’s motion), M₂ (sensor2’s motion), X (sensor2→sensor1 pose).
   - Constraint: M₁X = XM₂ (rigid object motion consistency).
   - Solve: Collect multi-group (M₁,M₂) → overdetermined equations → least squares to get X.

3. **Extension**: 3+ sensors via pairwise calibration (e.g., X₁₂→X₂₃→X₁₃).

---


## 📖 Paper 6: JVS-SLAM

**Combining the SLAM back and front ends with a joint vector-set distribution**

**IJRR** | Inostroza, Adams

### 🎯 Problem & Solution
- **Pain Points**: Traditional SLAM splits frontend (heuristic association) & backend; frontend errors (e.g., low-light misassociation) cause convergence failure; no map cardinality/association uncertainty.
- **Solution**: JVS-SLAM – unify frontend-backend via Bayesian RFS + batch optimization.

### 🚀 Core Design
- **Joint State**: Trajectory (fixed vector) + map (RFS, random cardinality) → co-estimated.
- **RFS**: Handles ambiguous association/false alarms without heuristics.
- **Batch Integration**: Combines RFS with g2o-like solvers for global consistency.
- **No Separate Frontend**: Association/map management = part of joint Bayesian estimation.

---

# 📄 August 2025 Paper Review

*Coming Soon - ICRA & IROS Selections*

Stay tuned for comprehensive reviews of papers from:
- **ICRA 2025**: International Conference on Robotics and Automation
- **IROS 2025**: IEEE/RSJ International Conference on Intelligent Robots and Systems
- **Selected Journal Papers**: Latest publications in top robotics journals

Expected coverage areas:
- 🦾 **Manipulation & Grasping**
- 🚗 **Autonomous Navigation** 
- 🤖 **Human-Robot Interaction**
- 🔬 **Learning & AI in Robotics**

---

# 📄 September 2025 Paper Review  

*Planned - CoRL & RSS Highlights*

Upcoming comprehensive reviews from:
- **CoRL 2025**: Conference on Robot Learning
- **RSS 2025**: Robotics: Science and Systems
- **Specialized Workshops**: Focus on emerging robotics research

Anticipated research themes:
- 🧠 **Deep Learning for Robotics**
- 🌐 **Multi-Robot Systems**
- 🏗️ **Field & Service Robotics**
- ⚡ **Real-time Robotics Applications**

---