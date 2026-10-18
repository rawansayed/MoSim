# Software Requirements Specification (SRS)
## MoSim: Next-Generation Web Musculoskeletal Biomechanics Simulator & Modeling Engine

**Document Identifier:** `SRS-MOSIM-2026-V1.0`  
**Standard:** IEEE Std 830-1998 / ISO/IEC/IEEE 29148:2018  
**Author / Organization:** MoSim Biomechanics Systems Engineering & Computational Modeling Group  
**Target Repository:** [`rawansayed/MoSim`](https://github.com/rawansayed/MoSim)  
**Status:** Approved for Implementation  
**Date:** October 2026  

---

## 1. Introduction

### 1.1 Purpose
This Software Requirements Specification (SRS) details the complete functional, mathematical, architectural, and user interface requirements for **MoSim** (Motion & Musculoskeletal Simulator). MoSim is a high-performance, web-native virtual musculoskeletal simulator engineered to match and surpass legacy desktop tools (such as MoSim 4.5) by providing instant browser-based 3D modeling, physically grounded bone-tendon-muscle simulation, interactive biomechanical coordinate inspection, and modern glassmorphic visualization.

### 1.2 Scope of the Software
MoSim provides a web-based computational environment for:
1. **Multibody Skeletal Kinematics & Dynamics:** Representing anatomical segments as rigid bodies connected via anatomical joint primitives (hinge, ball-and-socket, planar, custom 6-DOF).
2. **Musculotendon Unit (MTU) Mechanics:** Modeling skeletal muscles and elastic tendons using physiologically validated Hill-type 3-element formulation (Thelen 2003, Millard 2012).
3. **Muscle Routing & Wrapping Geometries:** Defining muscle paths via origin, insertion, via-points, and geometric wrapping surfaces (cylinders, ellipsoids) to compute realistic moment arms $r(q) = -\partial l_{mt}/\partial q$.
4. **Real-Time Interactive Biomechanical Analytics:** Providing live operating-point plots on active/passive force-length and force-velocity curves, moment arm profiles, and joint torque equilibrium.
5. **Numerical Solvers:** Executing Forward Dynamics (numerical integration of $\ddot{q}$), Inverse Kinematics, and Static Optimization (muscle force sharing quadratic programming).
6. **MoSim Interoperability:** Importing and exporting model specifications (`.osim` XML) and motion capture data (`.mot`, `.trc`, `.csv`).

### 1.3 Definitions, Acronyms, and Abbreviations
| Term | Definition |
| :--- | :--- |
| **DOF** | Degree of Freedom: An independent coordinate describing spatial orientation or translation of a body segment. |
| **MTU** | Musculotendon Unit: A combined functional unit comprising muscle fibers and serial tendon. |
| **Hill-Type Model** | Lumped-parameter biomechanical representation of muscle active contractile element, passive elastic element, and tendon elasticity. |
| **$l_0^m$** | Optimal muscle fiber length at which active isometric tension is maximal ($F_0^{\max}$). |
| **$l_s^t$** | Tendon slack length beyond which tendon produces elastic restoring tension. |
| **$\alpha$** | Pennation angle of muscle fibers relative to the line of tendon action. |
| **IK** | Inverse Kinematics: Computing joint angles $q(t)$ matching spatial experimental marker trajectories. |
| **ID** | Inverse Dynamics: Computing net joint moments $\tau_{ID}(t)$ from motion and ground reaction forces. |
| **SO** | Static Optimization: Resolving the muscle force distribution indeterminacy problem by quadratic cost minimization. |
| **PBR** | Physically Based Rendering: Modern computer graphics shading simulating accurate light transport and material micro-facets. |

---

## 2. Overall Description

### 2.1 Product Perspective & Legacy MoSim Modernization
Legacy desktop simulation platforms (such as MoSim 4.5 based on Java NetBeans / OpenGL 1.2) require complex C++ toolchains, heavy desktop installations, and separate plotting windows with non-interactive batch pipelines.

**MoSim** re-engineers this experience into a modern web ecosystem:
- **Zero Installation:** Runs immediately in modern WebGL 2.0 / WebGPU web browsers.
- **Unified Reactive Interface:** Moving a joint coordinate immediately updates the 3D skeleton, stretches the musculotendon path, recalculates the moment arm, and animates the live operating point on the Hill force-length curve without needing modal dialogs.
- **Physical Realism:** 3D muscle belly tubes expand radially during contraction to preserve muscle volume, with anatomical color graduation (crimson contractile belly to silver aponeurosis and pearlescent tendon).

### 2.2 User Classes and Characteristics
1. **Biomechanical Researchers & Scientists:** Require validated mathematical accuracy, realistic Hill model parameterization, moment arm computation, and gait motion playback.
2. **Clinical & Orthopedic Specialists:** Require visualization of muscle contractures, tendon lengthening, joint contact force distribution (e.g. knee varus/valgus), and surgical alterations.
3. **Biomedical Engineering Students:** Require intuitive visual feedback showing how changing muscle activation, tendon slack length, or joint angles changes muscle force generation.

### 2.3 Operating Environment
- Modern desktop and tablet web browsers supporting WebGL 2.0 (Google Chrome 110+, Mozilla Firefox 110+, Apple Safari 16+, Microsoft Edge 110+).
- Hardware: Dual-core x64 / ARM CPU minimum, dedicated or integrated GPU supporting WebGL hardware acceleration.

---

## 3. Mathematical & Biomechanical Formulation

### 3.1 Multibody Rigid-Body Kinematics & Dynamics

#### 3.1.1 Spatial Coordinate Frames & Kinematic Chains
Each anatomical bone segment $i$ is assigned a body-fixed reference frame $\{B_i\}$ relative to the ground inertial frame $\{N\}$. The spatial position of any anatomical landmark $P$ on body $i$ is:
$$^N p^P = \,^N p^{B_i} + \,^N R^{B_i}(q) \cdot \,^{B_i} p^P$$

#### 3.1.2 Equations of Motion (Simbody/Kane's Formulation)
The equations of motion for an $n$-DOF constrained musculoskeletal system are:
$$M(q)\ddot{q} + C(q, \dot{q})\dot{q} + G(q) = \tau_{act} + \sum_{m=1}^{M} J_m^T(q) F_{mt, m} + \sum_{k} J_k^T(q) F_{ext, k}$$

---

### 3.2 Hill-Type Musculotendon Unit (MTU) Model

#### 3.2.1 Musculotendon Kinematic Relation & Pennation
$$l_{mt} = l_t + l_m \cos\alpha$$
Constant muscle belly volume implies constant muscle thickness $w$:
$$w = l_0^m \sin\alpha_0 = l_m \sin\alpha = \text{const}$$
$$\cos\alpha = \sqrt{1 - \left(\frac{w}{l_m}\right)^2}$$

#### 3.2.2 Muscle-Tendon Equilibrium
$$F_t = F_m \cos\alpha$$
Normalized by maximum isometric force $F_0^{\max}$:
$$\tilde{F}_m = \frac{F_m}{F_0^{\max}} = \left[ a(t) \cdot f_L(\tilde{l}_m) \cdot f_V(\tilde{v}_m) + f_{PE}(\tilde{l}_m) + \beta \tilde{v}_m \right]$$

#### 3.2.3 Active Force-Length Relation $f_L(\tilde{l}_m)$
$$f_L(\tilde{l}_m) = \exp\left( -\left(\frac{\tilde{l}_m - 1}{\gamma}\right)^2 \right), \quad \gamma = 0.45$$

#### 3.2.4 Force-Velocity Relation $f_V(\tilde{v}_m)$
For concentric shortening ($\tilde{v}_m \le 0$):
$$f_V(\tilde{v}_m) = \frac{1 + \tilde{v}_m}{1 - \frac{\tilde{v}_m}{A_{\text{rel}}}}, \quad A_{\text{rel}} \approx 0.25$$

#### 3.2.5 Passive Elastic Force-Length Relation $f_{PE}(\tilde{l}_m)$
$$f_{PE}(\tilde{l}_m) = \frac{\exp\left( \frac{k^{pe} (\tilde{l}_m - 1)}{\epsilon_0^m} \right) - 1}{\exp(k^{pe}) - 1} \quad (\tilde{l}_m > 1)$$

#### 3.2.6 Series Tendon Elastic Relation $f_T(\epsilon_t)$
$$\epsilon_t = \frac{l_t - l_s^t}{l_s^t}$$

#### 3.2.7 Neural Excitation-Activation Dynamics
$$\frac{da}{dt} = \frac{u - a}{\tau (0.5 + 1.5 a)}$$

---

### 3.3 Muscle Routing & Generalized Moment Arms
$$r_j(q) = -\frac{\partial l_{mt}(q)}{\partial q_j}$$
The effective torque contributed by muscle $m$ to joint coordinate $q_j$ is:
$$\tau_{m, j} = r_{m, j}(q) \cdot F_{mt, m}$$

---

### 3.4 Static Optimization (Force Sharing Formulation)
$$\min_{F_m} J = \sum_{m=1}^{M} \left( \frac{F_m}{F_{0,m}^{\max}} \right)^2 \quad \text{s.t.} \quad \sum_{m=1}^{M} r_{m, j}(q) \cdot F_m = \tau_{target, j}$$

---

## 4. Functional Requirements (FR-01 to FR-15)

- **FR-01 (3D Viewport):** 60 FPS Three.js rendering with ground grid and orbit camera navigation.
- **FR-02 (Rigid Bodies):** Pelvis, Femur, Tibia, Patella, Calcaneus, Toes with anatomical pivots.
- **FR-03 (Joint Coordinates):** Sliders for Hip, Knee, and Ankle DOFs.
- **FR-04 (Hill MTU Model):** 3-element contractile, passive elastic, and tendon dynamics.
- **FR-05 (Dynamic Tubes):** 3D Catmull-Rom tubes with dynamic volume-preserving radius dilation.
- **FR-06 (Moment Arms):** Real-time numerical gradient derivation of $r = -\partial l/\partial q$.
- **FR-07 (Live Curve Plotter):** Active/passive curve plotting with dynamic operating-point tracker.
- **FR-08 (Gait Simulation):** Playback of normal walking kinematics from Rajagopal et al. 2016.
- **FR-09 (Excitation Control):** Individual muscle stimulation sliders ($u \in [0, 1]$).
- **FR-10 (Posture Presets):** 1-click clinical postures (Neutral, Mid-Stance, Toe-Off, Deep Squat).
- **FR-11 (Model Hierarchy):** Interactive tree inspector with live force badges.
- **FR-12 (Static Optimization):** Quadratic programming muscle load-sharing solver.
- **FR-13 (Interoperability):** MoSim `.osim` XML and `.mot` motion compatibility.
- **FR-14 (Responsive Layout):** Dark glassmorphic workspace docking.
- **FR-15 (Telemetry HUD):** Floating viewport stats for current muscle tension and lengths.
