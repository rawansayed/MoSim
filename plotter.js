/**
 * MoSim Real-Time Biomechanical Plotter & Curve Inspector
 * Renders high-performance interactive 2D curves for:
 * 1. Active & Passive Force-Length Curves with Live Operating-Point Dot
 * 2. Tendon Force-Strain Curve
 * 3. Generalized Moment Arm Spectrum across all Musculotendon Units
 * Enhanced with clean non-overlapping headers, clear axis labels, and high-DPI scaling.
 */

class BiomechanicalPlotter {
    /**
     * @param {HTMLCanvasElement} canvas
     */
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext("2d");
        this.activeMuscle = null;
        this.system = null;
        this.selectedJoint = "knee"; // "knee", "hip", "ankle", "elbow", "shoulder"

        // Aesthetics tokens
        this.colors = {
            bg: "#080d1a",
            cardBg: "rgba(15, 23, 42, 0.65)",
            grid: "rgba(255, 255, 255, 0.07)",
            textDim: "#64748B",
            textMid: "#94A3B8",
            textBright: "#F8FAFC",
            activeCurve: "#38BDF8", // Cyan
            passiveCurve: "#F43F5E", // Rose / Red
            totalCurve: "#A855F7", // Purple
            tendonCurve: "#F59E0B", // Amber
            dot: "#FACC15", // Glow Gold
            dotRing: "rgba(250, 204, 21, 0.35)",
            extBar: "#22C55E", // Extension Green
            flexBar: "#EF4444" // Flexion Red
        };
    }

    /**
     * Set active muscle to inspect
     * @param {HillMuscleModel} muscle
     * @param {MusculoskeletalSystem|UpperLimbSystem} system
     */
    setTarget(muscle, system) {
        this.activeMuscle = muscle;
        this.system = system;
        this.render();
    }

    /**
     * Set selected joint for moment arm analysis
     * @param {string} jointName - "knee", "hip", "ankle", "elbow", "shoulder"
     */
    setSelectedJoint(jointName) {
        this.selectedJoint = jointName;
        this.render();
    }

    /**
     * Resize canvas to high-DPI resolution
     */
    resize() {
        if (!this.canvas) return;
        const rect = this.canvas.getBoundingClientRect();
        const dpr = Math.min(window.devicePixelRatio || 1, 2);
        const newW = Math.round(rect.width * dpr);
        const newH = Math.round(rect.height * dpr);
        if (this.canvas.width !== newW || this.canvas.height !== newH) {
            this.canvas.width = newW;
            this.canvas.height = newH;
            this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
            this.width = rect.width;
            this.height = rect.height;
        }
    }

    /**
     * Render the complete biomechanical telemetry panel
     */
    render() {
        if (!this.canvas || !this.ctx) return;
        this.resize();
        const ctx = this.ctx;
        const w = this.width;
        const h = this.height;

        // Clear canvas with deep dark backdrop
        ctx.fillStyle = this.colors.bg;
        ctx.fillRect(0, 0, w, h);

        if (!this.activeMuscle || !this.system) {
            ctx.fillStyle = this.colors.textDim;
            ctx.font = "13px 'Inter', sans-serif";
            ctx.textAlign = "center";
            ctx.fillText("Select a muscle in the Model Explorer to inspect Hill force-length equilibrium", w / 2, h / 2);
            return;
        }

        // Layout: Left 58% = Hill Force-Length Curves, Right 42% = Moment Arm Bar Chart
        const splitX = Math.floor(w * 0.56);

        // Vertical divider
        ctx.strokeStyle = "rgba(255, 255, 255, 0.08)";
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(splitX, 8);
        ctx.lineTo(splitX, h - 8);
        ctx.stroke();

        this.renderHillCurve(0, 0, splitX, h);
        this.renderMomentArmChart(splitX, 0, w - splitX, h);
    }

    /**
     * Render Active, Passive, and Total Force-Length curves with live operating point
     * Redesigned to strictly prevent text collisions and maximize clarity.
     */
    renderHillCurve(x0, y0, width, height) {
        const ctx = this.ctx;
        // Generous padding so labels never clip
        const pad = { left: 54, right: 28, top: 52, bottom: 38 };
        const plotW = width - pad.left - pad.right;
        const plotH = height - pad.top - pad.bottom;

        // ===== 1. Header Row 1: Section Title & Selected Muscle Name =====
        ctx.font = "700 12px 'Inter', sans-serif";
        ctx.fillStyle = this.colors.textBright;
        ctx.textAlign = "left";
        ctx.fillText("HILL MUSCULOTENDON EQUILIBRIUM", x0 + pad.left, y0 + 18);

        // Muscle name pill badge
        const muscleShort = this.activeMuscle.displayName.split("(")[0].trim();
        const muscleType = this.activeMuscle.displayName.includes("(") ?
            "(" + this.activeMuscle.displayName.split("(")[1].replace(")", "") + ")" : "";

        ctx.font = "600 11px 'JetBrains Mono', monospace";
        ctx.fillStyle = this.colors.dot;
        ctx.textAlign = "right";
        ctx.fillText(`${muscleShort} ${muscleType}`, x0 + width - pad.right, y0 + 18);

        // ===== 2. Header Row 2: Dedicated Interactive Legend Bar =====
        const legY = y0 + 35;
        ctx.font = "500 10px 'Inter', sans-serif";
        ctx.textAlign = "left";

        // Active Curve Pill: Cyan
        ctx.fillStyle = this.colors.activeCurve;
        ctx.beginPath();
        ctx.arc(x0 + pad.left + 5, legY - 3, 4, 0, 2 * Math.PI);
        ctx.fill();
        ctx.fillText("Active Force (a·fL)", x0 + pad.left + 14, legY);

        // Passive Curve Pill: Rose (dashed)
        const passX = x0 + pad.left + 130;
        ctx.fillStyle = this.colors.passiveCurve;
        ctx.beginPath();
        ctx.arc(passX + 5, legY - 3, 4, 0, 2 * Math.PI);
        ctx.fill();
        ctx.fillText("Passive Elastic (fPE)", passX + 14, legY);

        // Total Curve Pill: Purple
        const totX = x0 + pad.left + 260;
        if (totX + 120 < x0 + width) {
            ctx.fillStyle = this.colors.totalCurve;
            ctx.beginPath();
            ctx.arc(totX + 5, legY - 3, 4, 0, 2 * Math.PI);
            ctx.fill();
            ctx.fillText("Total Tension (Ft)", totX + 14, legY);
        }

        // ===== 3. Dynamic Axes Scaling =====
        const currentNormLm = this.activeMuscle.fiberLength / this.activeMuscle.optimalFiberLength;
        const currentNormForce = this.activeMuscle.totalForce / this.activeMuscle.maxIsometricForce;

        const minX = 0.4, maxX = 1.8;
        const minY = 0.0;
        // Dynamically accommodate high passive stretch forces up to 2.4 F0
        const maxY = Math.min(2.4, Math.max(1.6, Math.ceil((currentNormForce * 1.1) * 5) / 5));

        const toScreenX = (lm) => x0 + pad.left + ((lm - minX) / (maxX - minX)) * plotW;
        const toScreenY = (f) => y0 + pad.top + plotH - ((f - minY) / (maxY - minY)) * plotH;

        // ===== 4. Grid lines & Axis Numerics =====
        ctx.strokeStyle = this.colors.grid;
        ctx.lineWidth = 1;

        // Horizontal grid lines (Force)
        const yStep = maxY > 2.0 ? 0.6 : 0.4;
        for (let f = 0.0; f <= maxY + 0.05; f += yStep) {
            const sy = toScreenY(f);
            ctx.beginPath();
            ctx.moveTo(x0 + pad.left, sy);
            ctx.lineTo(x0 + pad.left + plotW, sy);
            ctx.stroke();

            ctx.fillStyle = this.colors.textDim;
            ctx.font = "10px 'JetBrains Mono', monospace";
            ctx.textAlign = "right";
            ctx.fillText(f.toFixed(1) + " F₀", x0 + pad.left - 6, sy + 3);
        }

        // Vertical grid lines (Fiber Length)
        for (let lm = 0.6; lm <= maxX; lm += 0.3) {
            const sx = toScreenX(lm);
            ctx.beginPath();
            ctx.moveTo(sx, y0 + pad.top);
            ctx.lineTo(sx, y0 + pad.top + plotH);
            ctx.stroke();

            ctx.fillStyle = this.colors.textDim;
            ctx.font = "10px 'JetBrains Mono', monospace";
            ctx.textAlign = "center";
            ctx.fillText(lm.toFixed(1) + " l₀", sx, y0 + pad.top + plotH + 15);
        }

        // Axis Titles
        ctx.font = "500 10px 'Inter', sans-serif";
        ctx.fillStyle = this.colors.textMid;
        ctx.textAlign = "center";
        ctx.fillText("Normalized Fiber Length (l_m / l_0^m)", x0 + pad.left + plotW / 2, y0 + height - 8);

        // ===== 5. Draw Active Force-Length Curve: a * fL(lm) =====
        ctx.strokeStyle = this.colors.activeCurve;
        ctx.lineWidth = 2.2;
        ctx.beginPath();
        const steps = 100;
        for (let i = 0; i <= steps; i++) {
            const normLm = minX + (i / steps) * (maxX - minX);
            const fL = this.activeMuscle.activation * this.activeMuscle.calcActiveForceLength(normLm);
            const sx = toScreenX(normLm);
            const sy = toScreenY(Math.min(maxY, fL));
            if (i === 0) ctx.moveTo(sx, sy);
            else ctx.lineTo(sx, sy);
        }
        ctx.stroke();

        // ===== 6. Draw Passive Elastic Curve: fPE(lm) =====
        ctx.strokeStyle = this.colors.passiveCurve;
        ctx.lineWidth = 1.8;
        ctx.setLineDash([4, 3]);
        ctx.beginPath();
        for (let i = 0; i <= steps; i++) {
            const normLm = minX + (i / steps) * (maxX - minX);
            const fPE = this.activeMuscle.calcPassiveForceLength(normLm);
            const sx = toScreenX(normLm);
            const sy = toScreenY(Math.min(maxY, fPE));
            if (i === 0) ctx.moveTo(sx, sy);
            else ctx.lineTo(sx, sy);
        }
        ctx.stroke();
        ctx.setLineDash([]);

        // ===== 7. Draw Total Musculotendon Tension: Ft = a*fL + fPE =====
        ctx.strokeStyle = this.colors.totalCurve;
        ctx.lineWidth = 2.4;
        ctx.beginPath();
        for (let i = 0; i <= steps; i++) {
            const normLm = minX + (i / steps) * (maxX - minX);
            const fTotal = this.activeMuscle.activation * this.activeMuscle.calcActiveForceLength(normLm) +
                           this.activeMuscle.calcPassiveForceLength(normLm);
            const sx = toScreenX(normLm);
            const sy = toScreenY(Math.min(maxY, fTotal));
            if (i === 0) ctx.moveTo(sx, sy);
            else ctx.lineTo(sx, sy);
        }
        ctx.stroke();

        // ===== 8. Operating Point Tracker (Dynamic Glowing Indicator) =====
        const dotX = toScreenX(Math.max(minX, Math.min(maxX, currentNormLm)));
        const dotY = toScreenY(Math.max(minY, Math.min(maxY, currentNormForce)));

        // Pulsing halo
        ctx.beginPath();
        ctx.arc(dotX, dotY, 9, 0, 2 * Math.PI);
        ctx.fillStyle = this.colors.dotRing;
        ctx.fill();

        // Solid gold center
        ctx.beginPath();
        ctx.arc(dotX, dotY, 5, 0, 2 * Math.PI);
        ctx.fillStyle = this.colors.dot;
        ctx.fill();
        ctx.strokeStyle = "#FFFFFF";
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // Formatted floating telemetry pill
        ctx.fillStyle = "rgba(10, 15, 29, 0.92)";
        ctx.strokeStyle = "rgba(250, 204, 21, 0.55)";
        ctx.lineWidth = 1;
        const tagText = `l_m=${currentNormLm.toFixed(2)} l₀ | F=${this.activeMuscle.totalForce.toFixed(0)} N`;
        ctx.font = "600 10px 'JetBrains Mono', monospace";
        const tagW = ctx.measureText(tagText).width + 14;
        const tagX = Math.min(x0 + width - tagW - 8, Math.max(x0 + pad.left, dotX - tagW / 2));
        const tagY = Math.max(y0 + pad.top + 4, dotY - 24);

        ctx.fillRect(tagX, tagY, tagW, 19);
        ctx.strokeRect(tagX, tagY, tagW, 19);
        ctx.fillStyle = this.colors.dot;
        ctx.textAlign = "left";
        ctx.fillText(tagText, tagX + 7, tagY + 13);
    }

    /**
     * Render Moment Arm spectrum chart for all muscles crossing the selected joint
     * Enhanced with wider margins for muscle names and clear extension/flexion indicators.
     */
    renderMomentArmChart(x0, y0, width, height) {
        const ctx = this.ctx;
        const pad = { left: 110, right: 35, top: 48, bottom: 35 };
        const chartW = width - pad.left - pad.right;
        const chartH = height - pad.top - pad.bottom;

        // Title Row 1
        ctx.font = "700 12px 'Inter', sans-serif";
        ctx.fillStyle = this.colors.textBright;
        ctx.textAlign = "left";
        const jointLabel = this.selectedJoint.toUpperCase();
        ctx.fillText(`MOMENT ARMS: ${jointLabel} JOINT (r = -∂l/∂q)`, x0 + pad.left - 90, y0 + 18);

        // Subtitle Row 2: Legend
        ctx.font = "500 10px 'Inter', sans-serif";
        ctx.fillStyle = this.colors.flexBar;
        ctx.textAlign = "left";
        ctx.fillText("◀ Flexion (-)", x0 + pad.left - 90, y0 + 35);
        ctx.fillStyle = this.colors.extBar;
        ctx.fillText("Extension (+) ▶", x0 + pad.left + 5, y0 + 35);

        // Get moment arms for the selected joint
        let coordKey = this.selectedJoint;
        if (!this.system.coordLimits || !this.system.coordLimits[coordKey]) {
            if (this.selectedJoint === "knee") {
                coordKey = (this.system.coordLimits && this.system.coordLimits["knee_angle_r"]) ? "knee_angle_r" : "knee_angle";
            } else if (this.selectedJoint === "hip") {
                coordKey = (this.system.coordLimits && this.system.coordLimits["hip_flexion_r"]) ? "hip_flexion_r" : "hip_flexion";
            } else if (this.selectedJoint === "ankle") {
                coordKey = (this.system.coordLimits && this.system.coordLimits["ankle_angle_r"]) ? "ankle_angle_r" : "ankle_angle";
            } else if (this.selectedJoint === "elbow") {
                coordKey = (this.system.coordLimits && this.system.coordLimits["elbow_flexion_r"]) ? "elbow_flexion_r" : "elbow_flexion";
            } else if (this.selectedJoint === "shoulder") {
                coordKey = (this.system.coordLimits && this.system.coordLimits["shoulder_flexion_r"]) ? "shoulder_flexion_r" : "shoulder_flexion";
            } else if (this.selectedJoint === "neck") {
                coordKey = "neck_flexion";
            } else {
                coordKey = this.system.coordLimits ? Object.keys(this.system.coordLimits)[0] : "knee_angle";
            }
        }
        const momentArms = this.system.computeMomentArms(coordKey);

        const muscleNames = Object.keys(momentArms);
        const barHeight = Math.max(13, Math.floor(chartH / muscleNames.length) - 5);
        const maxArm = 0.065; // 65mm max reference scale

        // Zero line in the center of the chart area
        const zeroX = x0 + pad.left + chartW / 2;

        ctx.strokeStyle = "rgba(255, 255, 255, 0.15)";
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(zeroX, y0 + pad.top);
        ctx.lineTo(zeroX, y0 + pad.top + chartH);
        ctx.stroke();

        // Ticks for +/- 50 mm
        ctx.font = "9px 'JetBrains Mono', monospace";
        ctx.fillStyle = this.colors.textDim;
        ctx.textAlign = "center";
        ctx.fillText("-50 mm", zeroX - (0.05 / maxArm) * (chartW / 2), y0 + height - 12);
        ctx.fillText("0", zeroX, y0 + height - 12);
        ctx.fillText("+50 mm", zeroX + (0.05 / maxArm) * (chartW / 2), y0 + height - 12);

        muscleNames.forEach((name, idx) => {
            const r = momentArms[name] || 0; // meters
            const r_mm = r * 1000;
            const y = y0 + pad.top + idx * (barHeight + 5);

            const isCurrent = this.activeMuscle && this.activeMuscle.name === name;

            // Muscle Label on the left
            ctx.fillStyle = isCurrent ? this.colors.dot : this.colors.textBright;
            ctx.font = isCurrent ? "700 11px 'Inter', sans-serif" : "11px 'Inter', sans-serif";
            ctx.textAlign = "right";
            ctx.fillText(name, x0 + pad.left - 10, y + barHeight - 2);

            // Bar length and position
            const barW = (r / maxArm) * (chartW / 2);
            ctx.fillStyle = r >= 0 ?
                (isCurrent ? "#22C55E" : "rgba(34, 197, 94, 0.80)") :
                (isCurrent ? "#EF4444" : "rgba(239, 68, 68, 0.80)");

            if (r >= 0) {
                ctx.fillRect(zeroX, y, barW, barHeight);
            } else {
                ctx.fillRect(zeroX + barW, y, -barW, barHeight);
            }

            // Value text
            ctx.fillStyle = this.colors.textMid;
            ctx.font = "10px 'JetBrains Mono', monospace";
            ctx.textAlign = r >= 0 ? "left" : "right";
            const textX = r >= 0 ? zeroX + barW + 5 : zeroX + barW - 5;
            ctx.fillText(`${r_mm > 0 ? "+" : ""}${r_mm.toFixed(1)} mm`, textX, y + barHeight - 2);
        });
    }
}

/**
 * OpenSim Multi-Curve Scientific Plotter Window
 * Matches the official OpenSim Plotter (Figure 1, Curve Add, Curves List, Y-Quantity vs X-Quantity)
 */
class OpenSimPlotterWindow {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas ? canvas.getContext("2d") : null;
        this.curves = [];
        this.title = "Figure 1";
        this.xLabel = "Time (s)";
        this.yLabel = "Quantity Value";
        this.hoverCoord = null;

        // Palette of distinct curve colors matching OpenSim
        this.palette = [
            "#38BDF8", // Cyan
            "#F43F5E", // Rose Red
            "#22C55E", // Emerald Green
            "#FACC15", // Amber Gold
            "#A855F7", // Purple
            "#FB923C", // Orange
            "#EC4899", // Pink
            "#60A5FA"  // Light Blue
        ];

        if (this.canvas) {
            this.canvas.addEventListener("mousemove", (e) => this.onMouseMove(e));
            this.canvas.addEventListener("mouseleave", () => {
                this.hoverCoord = null;
                this.render();
            });
        }
    }

    addCurve(name, xData, yData, xLabel = "Time (s)", yLabel = "Value", color = null) {
        const cColor = color || this.palette[this.curves.length % this.palette.length];
        const id = "curve_" + Date.now() + "_" + Math.floor(Math.random() * 1000);
        this.xLabel = xLabel;
        this.yLabel = yLabel;
        this.curves.push({
            id,
            name,
            xData,
            yData,
            color: cColor,
            visible: true
        });
        this.render();
        return id;
    }

    removeCurve(id) {
        this.curves = this.curves.filter(c => c.id !== id);
        this.render();
    }

    clearCurves() {
        this.curves = [];
        this.render();
    }

    toggleCurveVisibility(id) {
        const c = this.curves.find(x => x.id === id);
        if (c) c.visible = !c.visible;
        this.render();
    }

    onMouseMove(e) {
        const rect = this.canvas.getBoundingClientRect();
        this.hoverCoord = {
            px: e.clientX - rect.left,
            py: e.clientY - rect.top
        };
        this.render();
    }

    resize() {
        if (!this.canvas) return;
        const rect = this.canvas.getBoundingClientRect();
        const dpr = Math.min(window.devicePixelRatio || 1, 2);
        const newW = Math.round(rect.width * dpr);
        const newH = Math.round(rect.height * dpr);
        if (this.canvas.width !== newW || this.canvas.height !== newH) {
            this.canvas.width = newW;
            this.canvas.height = newH;
            this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
            this.width = rect.width;
            this.height = rect.height;
        }
    }

    render() {
        if (!this.canvas || !this.ctx) return;
        this.resize();
        const ctx = this.ctx;
        const w = this.width;
        const h = this.height;

        // Background (OpenSim Classic or Studio theme)
        ctx.fillStyle = "#0c1322";
        ctx.fillRect(0, 0, w, h);

        const pad = { top: 38, right: 35, bottom: 45, left: 65 };
        const plotW = w - pad.left - pad.right;
        const plotH = h - pad.top - pad.bottom;

        if (plotW <= 0 || plotH <= 0) return;

        // Title: Figure 1
        ctx.fillStyle = "#F8FAFC";
        ctx.font = "700 13px 'Inter', sans-serif";
        ctx.textAlign = "center";
        ctx.fillText(this.title, w / 2, 22);

        // Find data limits
        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;

        const visibleCurves = this.curves.filter(c => c.visible && c.xData.length > 0);

        if (visibleCurves.length === 0) {
            // Default axes when empty
            minX = 0; maxX = 1.0;
            minY = 0; maxY = 1.0;
        } else {
            visibleCurves.forEach(c => {
                c.xData.forEach(x => {
                    if (x < minX) minX = x;
                    if (x > maxX) maxX = x;
                });
                c.yData.forEach(y => {
                    if (y < minY) minY = y;
                    if (y > maxY) maxY = y;
                });
            });

            if (minX === maxX) { minX -= 0.5; maxX += 0.5; }
            if (minY === maxY) { minY -= 0.5; maxY += 0.5; }

            // Add 8% vertical margin
            const ySpan = maxY - minY;
            minY -= ySpan * 0.08;
            maxY += ySpan * 0.08;
        }

        // Plot Box Background
        ctx.fillStyle = "rgba(15, 23, 42, 0.85)";
        ctx.fillRect(pad.left, pad.top, plotW, plotH);
        ctx.strokeStyle = "rgba(255, 255, 255, 0.15)";
        ctx.lineWidth = 1;
        ctx.strokeRect(pad.left, pad.top, plotW, plotH);

        // Grid lines & Ticks
        const xTicks = 6;
        const yTicks = 5;

        // Vertical Grid lines
        for (let i = 0; i <= xTicks; i++) {
            const frac = i / xTicks;
            const x = pad.left + frac * plotW;
            const val = minX + frac * (maxX - minX);

            ctx.strokeStyle = "rgba(255, 255, 255, 0.06)";
            ctx.setLineDash([3, 3]);
            ctx.beginPath();
            ctx.moveTo(x, pad.top);
            ctx.lineTo(x, pad.top + plotH);
            ctx.stroke();
            ctx.setLineDash([]);

            // X tick label
            ctx.fillStyle = "#94A3B8";
            ctx.font = "10px 'JetBrains Mono', monospace";
            ctx.textAlign = "center";
            ctx.fillText(val.toFixed(2), x, pad.top + plotH + 16);
        }

        // Horizontal Grid lines
        for (let i = 0; i <= yTicks; i++) {
            const frac = i / yTicks;
            const y = pad.top + (1 - frac) * plotH;
            const val = minY + frac * (maxY - minY);

            ctx.strokeStyle = "rgba(255, 255, 255, 0.06)";
            ctx.setLineDash([3, 3]);
            ctx.beginPath();
            ctx.moveTo(pad.left, y);
            ctx.lineTo(pad.left + plotW, y);
            ctx.stroke();
            ctx.setLineDash([]);

            // Y tick label
            ctx.fillStyle = "#94A3B8";
            ctx.font = "10px 'JetBrains Mono', monospace";
            ctx.textAlign = "right";
            ctx.fillText(val.toFixed(1), pad.left - 8, y + 4);
        }

        // Axis Titles
        ctx.fillStyle = "#E2E8F0";
        ctx.font = "600 11px 'Inter', sans-serif";
        ctx.textAlign = "center";
        ctx.fillText(this.xLabel, pad.left + plotW / 2, h - 8);

        ctx.save();
        ctx.translate(16, pad.top + plotH / 2);
        ctx.rotate(-Math.PI / 2);
        ctx.fillText(this.yLabel, 0, 0);
        ctx.restore();

        // Draw Curves
        visibleCurves.forEach((curve) => {
            ctx.strokeStyle = curve.color;
            ctx.lineWidth = 2.0;
            ctx.beginPath();

            const n = curve.xData.length;
            for (let i = 0; i < n; i++) {
                const px = pad.left + ((curve.xData[i] - minX) / (maxX - minX)) * plotW;
                const py = pad.top + (1 - (curve.yData[i] - minY) / (maxY - minY)) * plotH;
                if (i === 0) ctx.moveTo(px, py);
                else ctx.lineTo(px, py);
            }
            ctx.stroke();
        });

        // Legend at top-right inside plot box
        if (visibleCurves.length > 0) {
            let legX = pad.left + plotW - 10;
            let legY = pad.top + 16;
            ctx.textAlign = "right";

            visibleCurves.forEach(c => {
                ctx.fillStyle = c.color;
                ctx.fillRect(legX - 10, legY - 8, 8, 8);
                ctx.fillStyle = "#F8FAFC";
                ctx.font = "10px 'Inter', sans-serif";
                ctx.fillText(c.name, legX - 16, legY);
                legY += 15;
            });
        }

        // Crosshair tooltip if hover
        if (this.hoverCoord && visibleCurves.length > 0) {
            const { px, py } = this.hoverCoord;
            if (px >= pad.left && px <= pad.left + plotW && py >= pad.top && py <= pad.top + plotH) {
                // Draw crosshair lines
                ctx.strokeStyle = "rgba(250, 204, 21, 0.4)";
                ctx.lineWidth = 1;
                ctx.setLineDash([2, 2]);
                ctx.beginPath();
                ctx.moveTo(px, pad.top);
                ctx.lineTo(px, pad.top + plotH);
                ctx.moveTo(pad.left, py);
                ctx.lineTo(pad.left + plotW, py);
                ctx.stroke();
                ctx.setLineDash([]);

                const valX = minX + ((px - pad.left) / plotW) * (maxX - minX);
                const valY = minY + (1 - (py - pad.top) / plotH) * (maxY - minY);

                // Tooltip badge
                const tipText = `(${valX.toFixed(3)}, ${valY.toFixed(2)})`;
                ctx.font = "10px 'JetBrains Mono', monospace";
                const tw = ctx.measureText(tipText).width;
                const tx = Math.min(px + 8, pad.left + plotW - tw - 12);
                const ty = Math.max(py - 12, pad.top + 14);

                ctx.fillStyle = "rgba(0, 0, 0, 0.85)";
                ctx.fillRect(tx - 4, ty - 12, tw + 8, 16);
                ctx.strokeStyle = "#FACC15";
                ctx.strokeRect(tx - 4, ty - 12, tw + 8, 16);
                ctx.fillStyle = "#FACC15";
                ctx.textAlign = "left";
                ctx.fillText(tipText, tx, ty);
            }
        }
    }

    exportToCSV() {
        if (this.curves.length === 0) return null;
        let csv = "time_s";
        this.curves.forEach(c => { csv += `,"${c.name}"`; });
        csv += "\n";

        const maxPoints = Math.max(...this.curves.map(c => c.xData.length));
        const refCurve = this.curves[0];

        for (let i = 0; i < maxPoints; i++) {
            const t = refCurve.xData[i] !== undefined ? refCurve.xData[i] : i;
            let line = `${t}`;
            this.curves.forEach(c => {
                line += `,${c.yData[i] !== undefined ? c.yData[i] : ""}`;
            });
            csv += line + "\n";
        }
        return csv;
    }
}

// Export for browser
if (typeof window !== "undefined") {
    window.BiomechanicalPlotter = BiomechanicalPlotter;
    window.OpenSimPlotterWindow = OpenSimPlotterWindow;
}
if (typeof module !== "undefined" && module.exports) {
    module.exports = {
        BiomechanicalPlotter,
        OpenSimPlotterWindow
    };
}

