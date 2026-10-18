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
        const rect = this.canvas.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        this.canvas.width = rect.width * dpr;
        this.canvas.height = rect.height * dpr;
        this.ctx.scale(dpr, dpr);
        this.width = rect.width;
        this.height = rect.height;
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

// Export for browser
if (typeof window !== "undefined") {
    window.BiomechanicalPlotter = BiomechanicalPlotter;
}
