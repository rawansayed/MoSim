/**
 * MoSim Real-Time Biomechanical Plotter & Curve Inspector
 * Renders high-performance interactive 2D curves for:
 * 1. Active & Passive Force-Length Curves with Live Operating-Point Dot
 * 2. Tendon Force-Strain Curve
 * 3. Generalized Moment Arm Spectrum across all Musculotendon Units
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
        this.selectedJoint = "knee"; // "knee", "hip", "ankle"

        // Aesthetics tokens
        this.colors = {
            bg: "#0B101D",
            grid: "rgba(255, 255, 255, 0.08)",
            textDim: "#64748B",
            textBright: "#F8FAFC",
            activeCurve: "#38BDF8", // Cyan
            passiveCurve: "#F43F5E", // Rose / Red
            totalCurve: "#A855F7", // Purple
            tendonCurve: "#F59E0B", // Amber
            dot: "#FACC15", // Glow Gold
            dotRing: "rgba(250, 204, 21, 0.4)"
        };
    }

    /**
     * Set active muscle to inspect
     * @param {HillMuscleModel} muscle
     * @param {MusculoskeletalSystem} system
     */
    setTarget(muscle, system) {
        this.activeMuscle = muscle;
        this.system = system;
        this.render();
    }

    /**
     * Set selected joint for moment arm analysis
     * @param {string} jointName - "knee", "hip", "ankle"
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
            ctx.fillText("Select a muscle in the hierarchy to inspect Hill force-length curves", w / 2, h / 2);
            return;
        }

        // Layout: Left 60% = Hill Force-Length Curves, Right 40% = Moment Arm Bar Chart
        const splitX = Math.floor(w * 0.58);
        this.renderHillCurve(0, 0, splitX, h);
        this.renderMomentArmChart(splitX, 0, w - splitX, h);
    }

    /**
     * Render Active, Passive, and Total Force-Length curves with live operating point
     */
    renderHillCurve(x0, y0, width, height) {
        const ctx = this.ctx;
        const pad = { left: 45, right: 25, top: 35, bottom: 35 };
        const plotW = width - pad.left - pad.right;
        const plotH = height - pad.top - pad.bottom;

        // Title and Legend
        ctx.font = "600 13px 'Inter', sans-serif";
        ctx.fillStyle = this.colors.textBright;
        ctx.textAlign = "left";
        ctx.fillText(`Hill Force-Length: ${this.activeMuscle.displayName}`, x0 + pad.left, y0 + 20);

        // Subtitle legend
        ctx.font = "11px 'Inter', sans-serif";
        let legX = x0 + pad.left + 220;
        if (legX + 240 < x0 + width) {
            // Active legend
            ctx.fillStyle = this.colors.activeCurve;
            ctx.fillText("— Active (a·fL)", legX, y0 + 20);
            // Passive legend
            ctx.fillStyle = this.colors.passiveCurve;
            ctx.fillText("— Passive (fPE)", legX + 90, y0 + 20);
            // Total legend
            ctx.fillStyle = this.colors.totalCurve;
            ctx.fillText("— Total (fT)", legX + 180, y0 + 20);
        }

        // Domain: norm_lm in [0.4, 1.8], Range: norm_force in [0.0, 1.6]
        const minX = 0.4, maxX = 1.8;
        const minY = 0.0, maxY = 1.6;

        const toScreenX = (lm) => x0 + pad.left + ((lm - minX) / (maxX - minX)) * plotW;
        const toScreenY = (f) => y0 + pad.top + plotH - ((f - minY) / (maxY - minY)) * plotH;

        // Grid lines
        ctx.strokeStyle = this.colors.grid;
        ctx.lineWidth = 1;

        // Horizontal grid
        for (let f = 0.0; f <= maxY; f += 0.4) {
            const sy = toScreenY(f);
            ctx.beginPath();
            ctx.moveTo(x0 + pad.left, sy);
            ctx.lineTo(x0 + pad.left + plotW, sy);
            ctx.stroke();

            ctx.fillStyle = this.colors.textDim;
            ctx.font = "10px 'JetBrains Mono', monospace";
            ctx.textAlign = "right";
            ctx.fillText(f.toFixed(1) + " F0", x0 + pad.left - 6, sy + 3);
        }

        // Vertical grid
        for (let lm = 0.6; lm <= maxX; lm += 0.3) {
            const sx = toScreenX(lm);
            ctx.beginPath();
            ctx.moveTo(sx, y0 + pad.top);
            ctx.lineTo(sx, y0 + pad.top + plotH);
            ctx.stroke();

            ctx.fillStyle = this.colors.textDim;
            ctx.font = "10px 'JetBrains Mono', monospace";
            ctx.textAlign = "center";
            ctx.fillText(lm.toFixed(1) + " l0", sx, y0 + pad.top + plotH + 16);
        }

        // Axis labels
        ctx.font = "11px 'Inter', sans-serif";
        ctx.fillStyle = this.colors.textDim;
        ctx.textAlign = "center";
        ctx.fillText("Normalized Fiber Length (l_m / l_0^m)", x0 + pad.left + plotW / 2, y0 + height - 8);

        // 1. Draw Active Curve: a * fL(lm)
        ctx.strokeStyle = this.colors.activeCurve;
        ctx.lineWidth = 2.0;
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

        // 2. Draw Passive Curve: fPE(lm)
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

        // 3. Draw Total Curve: a * fL + fPE
        ctx.strokeStyle = this.colors.totalCurve;
        ctx.lineWidth = 2.2;
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

        // 4. Current Operating Point Dot
        const currentNormLm = this.activeMuscle.fiberLength / this.activeMuscle.optimalFiberLength;
        const currentNormForce = this.activeMuscle.totalForce / this.activeMuscle.maxIsometricForce;

        const dotX = toScreenX(Math.max(minX, Math.min(maxX, currentNormLm)));
        const dotY = toScreenY(Math.max(minY, Math.min(maxY, currentNormForce)));

        // Glowing outer pulse ring
        ctx.beginPath();
        ctx.arc(dotX, dotY, 9, 0, 2 * Math.PI);
        ctx.fillStyle = this.colors.dotRing;
        ctx.fill();

        // Solid gold center dot
        ctx.beginPath();
        ctx.arc(dotX, dotY, 5, 0, 2 * Math.PI);
        ctx.fillStyle = this.colors.dot;
        ctx.fill();
        ctx.strokeStyle = "#FFFFFF";
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // Operating point HUD tag
        ctx.fillStyle = "rgba(15, 23, 42, 0.9)";
        ctx.strokeStyle = "rgba(250, 204, 21, 0.5)";
        ctx.lineWidth = 1;
        const tagText = `l_m=${currentNormLm.toFixed(2)} l0 | F=${this.activeMuscle.totalForce.toFixed(0)} N`;
        ctx.font = "600 11px 'JetBrains Mono', monospace";
        const tagW = ctx.measureText(tagText).width + 16;
        const tagX = Math.min(x0 + width - tagW - 10, Math.max(x0 + pad.left, dotX - tagW / 2));
        const tagY = Math.max(y0 + pad.top + 10, dotY - 24);

        ctx.fillRect(tagX, tagY, tagW, 20);
        ctx.strokeRect(tagX, tagY, tagW, 20);
        ctx.fillStyle = this.colors.dot;
        ctx.textAlign = "left";
        ctx.fillText(tagText, tagX + 8, tagY + 14);
    }

    /**
     * Render Moment Arm spectrum chart for all muscles crossing the selected joint
     */
    renderMomentArmChart(x0, y0, width, height) {
        const ctx = this.ctx;
        const pad = { left: 80, right: 30, top: 35, bottom: 35 };
        const chartW = width - pad.left - pad.right;
        const chartH = height - pad.top - pad.bottom;

        // Title
        ctx.font = "600 13px 'Inter', sans-serif";
        ctx.fillStyle = this.colors.textBright;
        ctx.textAlign = "left";
        const jointLabel = this.selectedJoint.toUpperCase();
        ctx.fillText(`Moment Arms: ${jointLabel} Joint (r = -∂l/∂q)`, x0 + pad.left - 60, y0 + 20);

        // Get moment arms for the selected joint
        let coordKey = "knee_angle";
        if (this.system.modelType === "upper_limb") {
            coordKey = this.selectedJoint === "shoulder" ? "shoulder_flexion" : "elbow_flexion";
        } else {
            coordKey = this.selectedJoint === "hip" ? "hip_flexion" :
                       this.selectedJoint === "ankle" ? "ankle_angle" : "knee_angle";
        }
        const momentArms = this.system.computeMomentArms(coordKey);

        const muscleNames = Object.keys(momentArms);
        const barHeight = Math.max(14, Math.floor(chartH / muscleNames.length) - 4);
        const maxArm = 0.065; // 65mm max reference scale

        // Zero line in the center
        const zeroX = x0 + pad.left + chartW / 2;

        ctx.strokeStyle = this.colors.grid;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(zeroX, y0 + pad.top);
        ctx.lineTo(zeroX, y0 + pad.top + chartH);
        ctx.stroke();

        // Ticks for +/- 50 mm
        ctx.font = "9px 'JetBrains Mono', monospace";
        ctx.fillStyle = this.colors.textDim;
        ctx.textAlign = "center";
        ctx.fillText("-50 mm (Flex)", zeroX - (0.05 / maxArm) * (chartW / 2), y0 + height - 12);
        ctx.fillText("0", zeroX, y0 + height - 12);
        ctx.fillText("+50 mm (Ext)", zeroX + (0.05 / maxArm) * (chartW / 2), y0 + height - 12);

        muscleNames.forEach((name, idx) => {
            const r = momentArms[name] || 0; // meters
            const r_mm = r * 1000;
            const y = y0 + pad.top + idx * (barHeight + 4);

            const isCurrent = this.activeMuscle && this.activeMuscle.name === name;

            // Label
            ctx.fillStyle = isCurrent ? this.colors.dot : this.colors.textBright;
            ctx.font = isCurrent ? "700 11px 'Inter', sans-serif" : "11px 'Inter', sans-serif";
            ctx.textAlign = "right";
            ctx.fillText(name, x0 + pad.left - 8, y + barHeight - 2);

            // Bar length
            const barW = (r / maxArm) * (chartW / 2);
            ctx.fillStyle = r >= 0 ?
                (isCurrent ? "#22C55E" : "rgba(34, 197, 94, 0.75)") :
                (isCurrent ? "#EF4444" : "rgba(239, 68, 68, 0.75)");

            if (r >= 0) {
                ctx.fillRect(zeroX, y, barW, barHeight);
            } else {
                ctx.fillRect(zeroX + barW, y, -barW, barHeight);
            }

            // Value text
            ctx.fillStyle = this.colors.textDim;
            ctx.font = "10px 'JetBrains Mono', monospace";
            ctx.textAlign = r >= 0 ? "left" : "right";
            const textX = r >= 0 ? zeroX + barW + 4 : zeroX + barW - 4;
            ctx.fillText(`${r_mm > 0 ? "+" : ""}${r_mm.toFixed(1)} mm`, textX, y + barHeight - 2);
        });
    }
}

// Export for browser
if (typeof window !== "undefined") {
    window.BiomechanicalPlotter = BiomechanicalPlotter;
}
