# MoSim UI/UX Design System & Figma Specification
## High-Fidelity Design Architecture for Next-Generation Musculoskeletal Simulation

**Figma Design Identifier:** `FIGMA-MOSIM-V1.0`  
**Target Repository:** [`rawansayed/MoSim`](https://github.com/rawansayed/MoSim)  
**Resolution Canvas:** `1920 × 1080 px` (Full HD Desktop Desktop Frame)  
**Layout Grid:** 12-Column Responsive Fluid Grid with 16px Gutters and 20px Outer Margins  

---

## 🖼️ Visual Preview

### 1. Main Desktop Application Interface (Figma Canvas View)
![MoSim Desktop Figma Mockup](assets/mosim_figma_desktop_mockup.jpg)

### 2. Design System Tokens & Component UI Kit
![MoSim Figma Design System](assets/mosim_figma_design_system.jpg)

---

## 🎨 Design Tokens & Color Palette

| Token Name | Hex / RGBA Value | Usage & Biomechanical Context |
| :--- | :--- | :--- |
| `color.bg.base` | `#060913` | Deep space midnight backdrop for high-contrast 3D WebGL rendering |
| `color.bg.surface` | `rgba(15, 23, 42, 0.75)` | Frosted glass panels (`backdrop-filter: blur(16px)`) |
| `color.bg.elevated` | `rgba(30, 41, 59, 0.70)` | Elevated button containers, cards, and preset pills |
| `color.border.glass` | `rgba(255, 255, 255, 0.08)` | Subtle crisp border outline for UI glass panels |
| `color.border.glow` | `rgba(56, 189, 248, 0.35)` | Glowing cyan border on active elements |
| `color.accent.cyan` | `#38BDF8` | Primary interactive elements, slider thumbs, joint angle metrics |
| `color.accent.blue` | `#3B82F6` | Secondary accent, topbar logo gradient, brand elements |
| `color.accent.rose` | `#F43F5E` | Passive elastic Hill force curve, muscle contracture alerts |
| `color.accent.gold` | `#FACC15` | Live operating point cursor, selected muscle highlight |
| `color.accent.green` | `#22C55E` | Optimal muscle force range, gait stance phase indicator |
| `color.accent.purple`| `#A855F7` | Total musculotendon equilibrium tension curve ($F_t$) |
| `color.text.primary` | `#F8FAFC` | Headings, readouts, high-contrast labels |
| `color.text.secondary`| `#94A3B8` | Subheaders, coordinate descriptions, secondary stats |
| `color.text.muted`   | `#64748B` | Metric units, disabled states, grid lines |

---

## 📐 Layout Frame Specifications (Auto Layout)

### 1. Root Container Frame: `MoSim_Desktop_1920x1080`
- **Dimensions:** `1920 × 1080 px`
- **Background:** `#060913`
- **Layout Direction:** Vertical Auto Layout, Spacing `0px`, Padding `0px`

### 2. Top Navigation Bar: `Frame_Topbar`
- **Dimensions:** Width `Fill Container`, Height `54 px`
- **Auto Layout:** Horizontal, Align items center, Justify Space Between
- **Padding:** Top/Bottom `0px`, Left/Right `20px`
- **Fill:** `rgba(10, 15, 29, 0.85)` with `Background Blur: 14px`
- **Border:** Bottom stroke `1px` inside `rgba(255, 255, 255, 0.08)`
- **Children:**
  - `Brand_Group`: Logo icon (`28 × 28 px`, `Border Radius: 8px`, Gradient `#38BDF8` to `#3B82F6`), Title "MoSim", Version Pill `v1.0.0-PRO`.
  - `Controls_Group`: "▶ Play Gait" button, Phase badge (`0% Stance`), "↺ Reset Pose", Camera View segmented pills (Perspective, Sagittal, Anterior), and GitHub external link.

### 3. Main Workspace Grid: `Frame_Workspace`
- **Dimensions:** Width `Fill Container`, Height `Fill Container` (calculated `1026 px`)
- **Grid Layout:** 3-Column / 2-Row Grid
  - Column 1: `280 px` (Model Explorer)
  - Column 2: `1fr` (3D Viewport + Bottom HUD)
  - Column 3: `340 px` (Coordinate Sliders & Solvers)
  - Row 1: `1fr` (Center 3D Canvas)
  - Row 2: `230 px` (Biomechanical HUD Plotter)
- **Gap:** `8 px`
- **Padding:** `8 px` all around

---

## 🧩 Component Library & Auto Layout Specs

### Component: `Joint_Slider`
- **Default State:** Track height `5px`, background `rgba(255, 255, 255, 0.12)`, Thumb radius `15px`, color `#38BDF8`, drop shadow `0 0 10px rgba(56, 189, 248, 0.8)`.
- **Hover State:** Thumb scales to `18px`, glow radius increases to `16px`.
- **Active State:** Value badge pulses in `#38BDF8` with bold mono font (`JetBrains Mono`).

### Component: `Model_Tree_Item`
- **Dimensions:** Width `Fill Container`, Height `32 px`
- **Auto Layout:** Horizontal, Align center, Space between, Padding `7px 10px`
- **States:**
  - *Default:* Transparent fill, white text `#F8FAFC`, right badge in crimson `#F43F5E`.
  - *Hover:* Background `rgba(255, 255, 255, 0.04)`.
  - *Selected:* Background `rgba(56, 189, 248, 0.12)`, border `1px solid rgba(56, 189, 248, 0.35)`, gold dot highlight.

### Component: `Biomechanical_HUD_Card`
- **Dimensions:** Height `230 px`
- **Canvas View:**
  - Left Section (60%): Interactive Cartesian plot of active force-length ($a \cdot f_L$), passive force-length ($f_{PE}$), and total equilibrium tension ($F_t$). Includes dynamic gold operating point tracker $(l_m/l_0, F/F_0)$.
  - Right Section (40%): Zero-centered moment arm bar chart ($r = -\partial l_{mt}/\partial q$) with negative values for flexion (red) and positive values for extension (green).

---

## 🚀 How to Import into Figma
1. Install the **Tokens Studio for Figma** plugin.
2. In the plugin, click **Load from file** and select [`design/figma_tokens.json`](figma_tokens.json).
3. All MoSim colors, typography scales, shadows, and radii will populate automatically into your Figma document!
