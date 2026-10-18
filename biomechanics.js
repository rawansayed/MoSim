/**
 * MoSim Biomechanics Computational Core
 * Implements:
 * 1. Multibody Spatial Kinematics (FK for Hip, Knee, Ankle DOFs)
 * 2. 3-Element Hill-Type Musculotendon Model (Thelen 2003 / Millard 2012)
 *    - Active Force-Length Curve fL(lm)
 *    - Force-Velocity Curve fV(vm)
 *    - Passive Elastic Force-Length Curve fPE(lm)
 *    - Series Elastic Tendon Curve fT(lt)
 *    - Pennation Angle Geometry with Constant Muscle Thickness
 *    - First-Order Activation Dynamics
 * 3. Generalized Moment Arm Computation via Virtual Work: r(q) = -d(l_mt)/dq
 * 4. Anatomical Model Definitions (Bones, Origin/Insertion/Via-points, Wrapping surfaces)
 */

class HillMuscleModel {
    /**
     * @param {Object} params
     * @param {string} params.name - Muscle identifier
     * @param {number} params.maxIsometricForce - F0_max in Newtons
     * @param {number} params.optimalFiberLength - lm0 in meters
     * @param {number} params.tendonSlackLength - lt0 in meters
     * @param {number} params.pennationAngle - alpha0 in radians
     * @param {number} [params.maxContractionVelocity] - vmax in fiber-lengths/s (default: 10)
     */
    constructor(params) {
        this.name = params.name;
        this.displayName = params.displayName || params.name;
        this.maxIsometricForce = params.maxIsometricForce; // N
        this.optimalFiberLength = params.optimalFiberLength; // m
        this.tendonSlackLength = params.tendonSlackLength; // m
        this.pennationAngleAtOptimal = params.pennationAngle || 0; // rad
        this.maxContractionVelocity = params.maxContractionVelocity || 10.0; // l0/s

        // Physiological Constants
        this.gamma = 0.45; // Active FL curve shape parameter
        this.kPE = 4.0; // Passive elastic exponential stiffness
        this.epsilon0M = 0.60; // Muscle fiber strain at maximum passive force
        this.tendonNominalStrain = 0.033; // 3.3% strain at F0_max
        this.tendonToeLimit = 0.020; // 2.0% toe strain limit
        this.damping = 0.05; // Muscle internal viscous damping coefficient

        // Constant muscle thickness (volume preservation)
        this.muscleThickness = this.optimalFiberLength * Math.sin(this.pennationAngleAtOptimal);

        // State variables
        this.activation = 0.05; // a in [0, 1]
        this.excitation = 0.05; // u in [0, 1]
        this.fiberLength = this.optimalFiberLength; // lm (m)
        this.fiberVelocity = 0.0; // vm (m/s)
        this.tendonLength = this.tendonSlackLength; // lt (m)
        this.mtuLength = this.optimalFiberLength + this.tendonSlackLength; // lmt (m)
        this.activeForce = 0.0; // N
        this.passiveForce = 0.0; // N
        this.tendonForce = 0.0; // N
        this.totalForce = 0.0; // N
        this.pennationAngle = this.pennationAngleAtOptimal; // rad

        // Time constants for activation dynamics
        this.tauAct = 0.015; // 15 ms
        this.tauDeact = 0.050; // 50 ms
    }

    /**
     * Active Force-Length Curve fL(norm_lm)
     * Normalized Gaussian representation of actin-myosin crossbridge overlap.
     * @param {number} normLm - Normalized fiber length lm / lm0
     * @returns {number} Normalized active isometric force multiplier
     */
    calcActiveForceLength(normLm) {
        const delta = (normLm - 1.0) / this.gamma;
        return Math.exp(-delta * delta);
    }

    /**
     * Passive Elastic Force-Length Curve fPE(norm_lm)
     * Exponential titin and connective tissue resistance upon stretch.
     * @param {number} normLm - Normalized fiber length lm / lm0
     * @returns {number} Normalized passive elastic force multiplier
     */
    calcPassiveForceLength(normLm) {
        if (normLm <= 1.0) {
            return 0.0;
        }
        const num = Math.exp((this.kPE * (normLm - 1.0)) / this.epsilon0M) - 1.0;
        const den = Math.exp(this.kPE) - 1.0;
        return Math.max(0.0, num / den);
    }

    /**
     * Force-Velocity Curve fV(norm_vm)
     * Hill hyperbolic relation for concentric shortening and eccentric lengthening.
     * @param {number} normVm - Normalized fiber velocity vm / (vmax * lm0)
     * @returns {number} Force multiplier
     */
    calcForceVelocity(normVm) {
        const aRel = 0.25;
        const fLen = 1.40; // Max eccentric force
        const de = 0.30;

        if (normVm <= 0) {
            // Concentric (shortening)
            return (1.0 + normVm) / (1.0 - normVm / aRel);
        } else {
            // Eccentric (lengthening)
            const c = (fLen - 1.0) * (aRel / de);
            return (fLen * normVm + c) / (normVm + (aRel / de));
        }
    }

    /**
     * Series Elastic Tendon Force-Strain Curve fT(strain)
     * Quadratic toe region followed by linear collagen response.
     * @param {number} strain - (lt - lt0) / lt0
     * @returns {number} Normalized tendon tension F_t / F0_max
     */
    calcTendonForceStrain(strain) {
        if (strain <= 0) {
            return 0.0;
        }
        const e0 = this.tendonNominalStrain;
        const eToe = this.tendonToeLimit;
        const kToe = 1.0 / (e0 * eToe);
        const kLin = 1.0 / (e0 - 0.5 * eToe);

        if (strain <= eToe) {
            return kToe * strain * strain;
        } else {
            return 1.0 + kLin * (strain - e0);
        }
    }

    /**
     * Calculate pennation angle preserving muscle volume
     * @param {number} lm - Muscle fiber length
     * @returns {number} Pennation angle alpha in radians
     */
    calcPennationAngle(lm) {
        if (lm <= 0.001) return Math.PI / 2;
        const sinAlpha = this.muscleThickness / lm;
        if (sinAlpha >= 0.99) {
            return Math.asin(0.99);
        }
        return Math.asin(Math.max(0, sinAlpha));
    }

    /**
     * Solve musculotendon equilibrium for a given total MTU length and activation
     * @param {number} totalMtuLength - Total path length from origin to insertion (m)
     * @param {number} activation - Current muscular activation a in [0, 1]
     */
    solveEquilibrium(totalMtuLength, activation) {
        this.mtuLength = totalMtuLength;
        this.activation = Math.max(0.01, Math.min(1.0, activation));

        // Equilibrium condition: F_tendon(lt) = F_muscle(lm) * cos(alpha)
        // With totalMtuLength = lt + lm * cos(alpha)
        // We find lm in [0.25*lm0, 2.2*lm0] using root-finding / bisection
        let lowLm = 0.25 * this.optimalFiberLength;
        let highLm = 2.20 * this.optimalFiberLength;
        let bestLm = this.optimalFiberLength;
        let minResidual = Infinity;

        for (let iter = 0; iter < 18; iter++) {
            const midLm = 0.5 * (lowLm + highLm);
            const alpha = this.calcPennationAngle(midLm);
            const cosAlpha = Math.cos(alpha);
            const lt = totalMtuLength - midLm * cosAlpha;
            const tendonStrain = (lt - this.tendonSlackLength) / this.tendonSlackLength;

            const normLm = midLm / this.optimalFiberLength;
            const fL = this.calcActiveForceLength(normLm);
            const fPE = this.calcPassiveForceLength(normLm);

            const fMuscleNorm = this.activation * fL + fPE;
            const fTendonNorm = this.calcTendonForceStrain(tendonStrain);

            const residual = fTendonNorm - fMuscleNorm * cosAlpha;

            if (Math.abs(residual) < minResidual) {
                minResidual = Math.abs(residual);
                bestLm = midLm;
            }

            if (residual > 0) {
                // Tendon is too stiff / fiber is too short
                lowLm = midLm;
            } else {
                highLm = midLm;
            }
        }

        this.fiberLength = bestLm;
        this.pennationAngle = this.calcPennationAngle(this.fiberLength);
        const cosAlpha = Math.cos(this.pennationAngle);
        this.tendonLength = Math.max(0.001, totalMtuLength - this.fiberLength * cosAlpha);

        const normLm = this.fiberLength / this.optimalFiberLength;
        const tendonStrain = (this.tendonLength - this.tendonSlackLength) / this.tendonSlackLength;

        const fL = this.calcActiveForceLength(normLm);
        const fPE = this.calcPassiveForceLength(normLm);
        const fT = this.calcTendonForceStrain(tendonStrain);

        this.activeForce = this.activation * fL * this.maxIsometricForce;
        this.passiveForce = fPE * this.maxIsometricForce;
        this.tendonForce = fT * this.maxIsometricForce;
        this.totalForce = (this.activeForce + this.passiveForce) * cosAlpha;

        return {
            mtuLength: this.mtuLength,
            fiberLength: this.fiberLength,
            tendonLength: this.tendonLength,
            normFiberLength: normLm,
            tendonStrain: tendonStrain,
            activeForce: this.activeForce,
            passiveForce: this.passiveForce,
            tendonForce: this.tendonForce,
            totalForce: this.totalForce,
            pennationAngleDeg: (this.pennationAngle * 180) / Math.PI
        };
    }

    /**
     * First-order neural activation dynamics: da/dt
     * @param {number} excitation - Target excitation u in [0, 1]
     * @param {number} dt - Time increment (s)
     */
    updateActivation(excitation, dt) {
        this.excitation = Math.max(0.0, Math.min(1.0, excitation));
        const u = this.excitation;
        const a = this.activation;
        let da_dt = 0;

        if (u >= a) {
            da_dt = (u - a) / (this.tauAct * (0.5 + 1.5 * a));
        } else {
            da_dt = (u - a) / (this.tauDeact / (0.5 + 1.5 * a));
        }

        this.activation = Math.max(0.01, Math.min(1.0, a + da_dt * dt));
        return this.activation;
    }
}

/**
 * MoSim Musculoskeletal System Definition & Spatial Kinematics
 */
class MusculoskeletalSystem {
    constructor() {
        // Generalized coordinates (in degrees)
        this.coordinates = {
            hip_flexion: 0.0,       // [-30, 120] deg
            hip_adduction: 0.0,     // [-30, 45] deg
            hip_rotation: 0.0,      // [-40, 40] deg
            knee_angle: 0.0,        // [0, 140] deg (0 = full extension)
            ankle_angle: 0.0        // [-30, 50] deg (negative = dorsiflexion, positive = plantarflexion)
        };

        // Coordinate limits (min, max, default)
        this.coordLimits = {
            hip_flexion: { min: -30, max: 120, default: 0, label: "Hip Flexion / Extension", unit: "°" },
            hip_adduction: { min: -30, max: 45, default: 0, label: "Hip Adduction / Abduction", unit: "°" },
            hip_rotation: { min: -40, max: 40, default: 0, label: "Hip Internal / External Rotation", unit: "°" },
            knee_angle: { min: 0, max: 140, default: 0, label: "Knee Angle (Flexion)", unit: "°" },
            ankle_angle: { min: -30, max: 50, default: 0, label: "Ankle Angle (Plantar/Dorsi)", unit: "°" }
        };

        // Skeletal Segment Dimensions (approximate 50th percentile adult male: height 1.76m, mass 75kg)
        this.segmentDimensions = {
            pelvis: { width: 0.28, height: 0.20, depth: 0.16 },
            femur: { length: 0.44, radius: 0.038 },
            tibia: { length: 0.41, radius: 0.032 },
            patella: { width: 0.05, height: 0.05, depth: 0.03 },
            foot: { length: 0.24, height: 0.08, width: 0.09 }
        };

        // Initialize Musculotendon Units with physiological parameters from Rajagopal 2016 / Delp 1990
        this.muscles = [
            new HillMuscleModel({
                name: "rect_fem",
                displayName: "Rectus Femoris (Quad Bi-articular)",
                maxIsometricForce: 1169.0,
                optimalFiberLength: 0.084,
                tendonSlackLength: 0.346,
                pennationAngle: 0.087 // 5.0 deg
            }),
            new HillMuscleModel({
                name: "vas_lat",
                displayName: "Vastus Lateralis (Quad Mono-articular)",
                maxIsometricForce: 1871.0,
                optimalFiberLength: 0.084,
                tendonSlackLength: 0.157,
                pennationAngle: 0.087 // 5.0 deg
            }),
            new HillMuscleModel({
                name: "vas_med",
                displayName: "Vastus Medialis (Quad Mono-articular)",
                maxIsometricForce: 1294.0,
                optimalFiberLength: 0.089,
                tendonSlackLength: 0.126,
                pennationAngle: 0.087
            }),
            new HillMuscleModel({
                name: "bifemlh",
                displayName: "Biceps Femoris Long Head (Hamstring)",
                maxIsometricForce: 896.0,
                optimalFiberLength: 0.109,
                tendonSlackLength: 0.326,
                pennationAngle: 0.0 // parallel
            }),
            new HillMuscleModel({
                name: "semiten",
                displayName: "Semitendinosus (Hamstring Medial)",
                maxIsometricForce: 410.0,
                optimalFiberLength: 0.201,
                tendonSlackLength: 0.262,
                pennationAngle: 0.087
            }),
            new HillMuscleModel({
                name: "iliopsoas",
                displayName: "Iliopsoas (Primary Hip Flexor)",
                maxIsometricForce: 1042.0,
                optimalFiberLength: 0.102,
                tendonSlackLength: 0.142,
                pennationAngle: 0.140 // 8.0 deg
            }),
            new HillMuscleModel({
                name: "glut_max",
                displayName: "Gluteus Maximus (Hip Extensor)",
                maxIsometricForce: 1944.0,
                optimalFiberLength: 0.157,
                tendonSlackLength: 0.127,
                pennationAngle: 0.0
            }),
            new HillMuscleModel({
                name: "gas_med",
                displayName: "Gastrocnemius Medialis (Calf Bi-articular)",
                maxIsometricForce: 1558.0,
                optimalFiberLength: 0.051,
                tendonSlackLength: 0.398,
                pennationAngle: 0.297 // 17.0 deg
            }),
            new HillMuscleModel({
                name: "soleus",
                displayName: "Soleus (Deep Calf Mono-articular)",
                maxIsometricForce: 3549.0,
                optimalFiberLength: 0.044,
                tendonSlackLength: 0.268,
                pennationAngle: 0.436 // 25.0 deg
            }),
            new HillMuscleModel({
                name: "tib_ant",
                displayName: "Tibialis Anterior (Ankle Dorsiflexor)",
                maxIsometricForce: 603.0,
                optimalFiberLength: 0.098,
                tendonSlackLength: 0.223,
                pennationAngle: 0.087 // 5.0 deg
            })
        ];

        this.muscleMap = new Map();
        this.muscles.forEach(m => this.muscleMap.set(m.name, m));
    }

    /**
     * Compute spatial joint attachment transformations and bone positions
     * @param {Object} coords - Joint coordinates in degrees
     */
    computeKinematics(coords = this.coordinates) {
        const rad = Math.PI / 180.0;
        const qHipFlex = coords.hip_flexion * rad;
        const qHipAdd = coords.hip_adduction * rad;
        const qHipRot = coords.hip_rotation * rad;
        const qKnee = coords.knee_angle * rad;
        const qAnkle = coords.ankle_angle * rad;

        // Hip joint center located in pelvis frame
        const hipCenter = { x: 0.09, y: -0.07, z: 0.0 };

        // Femur vector (length = 0.44m)
        const femurLength = this.segmentDimensions.femur.length;
        const cosFlex = Math.cos(qHipFlex), sinFlex = Math.sin(qHipFlex);
        const cosAdd = Math.cos(qHipAdd), sinAdd = Math.sin(qHipAdd);

        // Knee joint center
        const kneeCenter = {
            x: hipCenter.x + femurLength * sinFlex * cosAdd,
            y: hipCenter.y - femurLength * cosFlex * cosAdd,
            z: hipCenter.z + femurLength * sinAdd
        };

        // Tibia vector (length = 0.41m)
        const tibiaLength = this.segmentDimensions.tibia.length;
        const totalKneeFlex = qHipFlex - qKnee;
        const ankleCenter = {
            x: kneeCenter.x + tibiaLength * Math.sin(totalKneeFlex) * cosAdd,
            y: kneeCenter.y - tibiaLength * Math.cos(totalKneeFlex) * cosAdd,
            z: kneeCenter.z + tibiaLength * sinAdd
        };

        // Foot center / Calcaneus / Toes
        const totalFootAngle = totalKneeFlex - qAnkle;
        const calcaneus = {
            x: ankleCenter.x - 0.05 * Math.cos(totalFootAngle),
            y: ankleCenter.y - 0.04,
            z: ankleCenter.z
        };
        const toeCenter = {
            x: ankleCenter.x + 0.17 * Math.cos(totalFootAngle),
            y: ankleCenter.y - 0.04,
            z: ankleCenter.z
        };

        // Patella follows knee flexion with anatomically modeled quadriceps wrapping
        const patellaOffset = 0.045; // 45mm anterior lever arm
        const patellaAngle = qHipFlex - 0.5 * qKnee;
        const patellaCenter = {
            x: kneeCenter.x + patellaOffset * Math.cos(patellaAngle),
            y: kneeCenter.y + patellaOffset * Math.sin(patellaAngle),
            z: kneeCenter.z
        };

        return {
            pelvisOrigin: { x: 0, y: 0, z: 0 },
            hipCenter,
            kneeCenter,
            ankleCenter,
            patellaCenter,
            calcaneus,
            toeCenter,
            anglesRad: { qHipFlex, qHipAdd, qHipRot, qKnee, qAnkle }
        };
    }

    /**
     * Compute 3D muscle pathways (origin, via-points, insertion) based on kinematics
     * @param {Object} kin - Kinematics object from computeKinematics()
     * @returns {Object} Map of muscle names to 3D point arrays
     */
    computeMusclePaths(kin) {
        const p = kin;
        const paths = {};

        // 1. Rectus Femoris: Anterior Inferior Iliac Spine -> over Patella -> Tibial Tuberosity
        paths["rect_fem"] = [
            { x: p.hipCenter.x + 0.03, y: p.hipCenter.y + 0.08, z: p.hipCenter.z + 0.03 },
            { x: p.patellaCenter.x + 0.005, y: p.patellaCenter.y + 0.015, z: p.patellaCenter.z },
            { x: p.patellaCenter.x + 0.012, y: p.patellaCenter.y - 0.015, z: p.patellaCenter.z },
            { x: p.kneeCenter.x + 0.025, y: p.kneeCenter.y - 0.055, z: p.kneeCenter.z + 0.01 }
        ];

        // 2. Vastus Lateralis: Lateral femur -> Lateral patella -> Tibial tuberosity
        const femurMid = {
            x: 0.5 * (p.hipCenter.x + p.kneeCenter.x),
            y: 0.5 * (p.hipCenter.y + p.kneeCenter.y),
            z: 0.5 * (p.hipCenter.z + p.kneeCenter.z)
        };
        paths["vas_lat"] = [
            { x: p.hipCenter.x - 0.01, y: p.hipCenter.y - 0.06, z: p.hipCenter.z + 0.045 },
            { x: femurMid.x + 0.015, y: femurMid.y, z: femurMid.z + 0.04 },
            { x: p.patellaCenter.x + 0.005, y: p.patellaCenter.y, z: p.patellaCenter.z + 0.02 },
            { x: p.kneeCenter.x + 0.025, y: p.kneeCenter.y - 0.055, z: p.kneeCenter.z + 0.01 }
        ];

        // 3. Vastus Medialis: Medial femur -> Medial patella -> Tibial tuberosity
        paths["vas_med"] = [
            { x: p.hipCenter.x + 0.01, y: p.hipCenter.y - 0.08, z: p.hipCenter.z - 0.035 },
            { x: femurMid.x + 0.01, y: femurMid.y - 0.04, z: femurMid.z - 0.035 },
            { x: p.patellaCenter.x + 0.005, y: p.patellaCenter.y, z: p.patellaCenter.z - 0.015 },
            { x: p.kneeCenter.x + 0.025, y: p.kneeCenter.y - 0.055, z: p.kneeCenter.z + 0.005 }
        ];

        // 4. Biceps Femoris Long Head: Ischial tuberosity -> Head of fibula
        paths["bifemlh"] = [
            { x: p.hipCenter.x - 0.05, y: p.hipCenter.y - 0.03, z: p.hipCenter.z + 0.01 },
            { x: femurMid.x - 0.035, y: femurMid.y - 0.03, z: femurMid.z + 0.035 },
            { x: p.kneeCenter.x - 0.025, y: p.kneeCenter.y - 0.04, z: p.kneeCenter.z + 0.035 }
        ];

        // 5. Semitendinosus: Ischial tuberosity -> Medial proximal tibia (Pes anserinus)
        paths["semiten"] = [
            { x: p.hipCenter.x - 0.05, y: p.hipCenter.y - 0.03, z: p.hipCenter.z - 0.01 },
            { x: femurMid.x - 0.03, y: femurMid.y - 0.04, z: femurMid.z - 0.025 },
            { x: p.kneeCenter.x - 0.015, y: p.kneeCenter.y - 0.06, z: p.kneeCenter.z - 0.025 }
        ];

        // 6. Iliopsoas: Iliac fossa -> Lesser trochanter
        paths["iliopsoas"] = [
            { x: p.hipCenter.x - 0.02, y: p.hipCenter.y + 0.12, z: p.hipCenter.z - 0.02 },
            { x: p.hipCenter.x + 0.025, y: p.hipCenter.y + 0.02, z: p.hipCenter.z - 0.01 },
            { x: p.hipCenter.x + 0.005, y: p.hipCenter.y - 0.065, z: p.hipCenter.z - 0.025 }
        ];

        // 7. Gluteus Maximus: Posterior ilium & sacrum -> Gluteal tuberosity
        paths["glut_max"] = [
            { x: p.hipCenter.x - 0.07, y: p.hipCenter.y + 0.06, z: p.hipCenter.z + 0.03 },
            { x: p.hipCenter.x - 0.055, y: p.hipCenter.y - 0.01, z: p.hipCenter.z + 0.05 },
            { x: p.hipCenter.x - 0.015, y: p.hipCenter.y - 0.11, z: p.hipCenter.z + 0.04 }
        ];

        // 8. Gastrocnemius Medialis: Medial femoral condyle -> Achilles tendon -> Calcaneus
        const tibiaMid = {
            x: 0.5 * (p.kneeCenter.x + p.ankleCenter.x),
            y: 0.5 * (p.kneeCenter.y + p.ankleCenter.y),
            z: 0.5 * (p.kneeCenter.z + p.ankleCenter.z)
        };
        paths["gas_med"] = [
            { x: p.kneeCenter.x - 0.02, y: p.kneeCenter.y + 0.025, z: p.kneeCenter.z - 0.02 },
            { x: tibiaMid.x - 0.035, y: tibiaMid.y + 0.05, z: tibiaMid.z - 0.015 },
            { x: p.ankleCenter.x - 0.035, y: p.ankleCenter.y + 0.04, z: p.ankleCenter.z },
            { x: p.calcaneus.x, y: p.calcaneus.y + 0.015, z: p.calcaneus.z }
        ];

        // 9. Soleus: Posterior proximal tibia/fibula -> Achilles tendon -> Calcaneus
        paths["soleus"] = [
            { x: p.kneeCenter.x - 0.02, y: p.kneeCenter.y - 0.06, z: p.kneeCenter.z },
            { x: tibiaMid.x - 0.03, y: tibiaMid.y - 0.02, z: tibiaMid.z },
            { x: p.ankleCenter.x - 0.035, y: p.ankleCenter.y + 0.04, z: p.ankleCenter.z },
            { x: p.calcaneus.x, y: p.calcaneus.y + 0.015, z: p.calcaneus.z }
        ];

        // 10. Tibialis Anterior: Lateral tibia -> 1st metatarsal
        paths["tib_ant"] = [
            { x: p.kneeCenter.x + 0.015, y: p.kneeCenter.y - 0.06, z: p.kneeCenter.z + 0.02 },
            { x: tibiaMid.x + 0.025, y: tibiaMid.y - 0.02, z: tibiaMid.z + 0.015 },
            { x: p.ankleCenter.x + 0.03, y: p.ankleCenter.y + 0.01, z: p.ankleCenter.z },
            { x: p.toeCenter.x - 0.04, y: p.toeCenter.y + 0.01, z: p.toeCenter.z - 0.02 }
        ];

        return paths;
    }

    /**
     * Compute Euclidean length of a 3D piecewise path
     * @param {Array<{x:number, y:number, z:number}>} pointArray
     * @returns {number} Path length in meters
     */
    computePathLength(pointArray) {
        let len = 0;
        for (let i = 0; i < pointArray.length - 1; i++) {
            const dx = pointArray[i + 1].x - pointArray[i].x;
            const dy = pointArray[i + 1].y - pointArray[i].y;
            const dz = pointArray[i + 1].z - pointArray[i].z;
            len += Math.sqrt(dx * dx + dy * dy + dz * dz);
        }
        return len;
    }

    /**
     * Compute generalized moment arms r(q) = -d(l_mt)/dq using central difference
     * @param {string} coordName - Degree of freedom coordinate name
     * @param {number} deltaDeg - Perturbation in degrees (default: 0.5)
     */
    computeMomentArms(coordName = "knee_angle", deltaDeg = 0.5) {
        const currentCoord = this.coordinates[coordName];
        const radDelta = (deltaDeg * Math.PI) / 180.0;

        // Positive perturbation
        const coordsPlus = { ...this.coordinates, [coordName]: currentCoord + deltaDeg };
        const kinPlus = this.computeKinematics(coordsPlus);
        const pathsPlus = this.computeMusclePaths(kinPlus);

        // Negative perturbation
        const coordsMinus = { ...this.coordinates, [coordName]: currentCoord - deltaDeg };
        const kinMinus = this.computeKinematics(coordsMinus);
        const pathsMinus = this.computeMusclePaths(kinMinus);

        const momentArms = {};
        for (const muscle of this.muscles) {
            const lenPlus = this.computePathLength(pathsPlus[muscle.name]);
            const lenMinus = this.computePathLength(pathsMinus[muscle.name]);

            // r(q) = - d(l_mt) / dq  (in meters per radian)
            const r_rad = -(lenPlus - lenMinus) / (2.0 * radDelta);
            momentArms[muscle.name] = r_rad; // meters
        }

        return momentArms;
    }

    /**
     * Update entire musculoskeletal system state: kinematics, MTU lengths, Hill forces, and moment arms
     */
    update() {
        const kin = this.computeKinematics(this.coordinates);
        const paths = this.computeMusclePaths(kin);
        const results = {};

        for (const muscle of this.muscles) {
            const path = paths[muscle.name];
            const lmt = this.computePathLength(path);
            const state = muscle.solveEquilibrium(lmt, muscle.activation);
            results[muscle.name] = {
                state,
                path
            };
        }

        const kneeMomentArms = this.computeMomentArms("knee_angle");
        const hipMomentArms = this.computeMomentArms("hip_flexion");
        const ankleMomentArms = this.computeMomentArms("ankle_angle");

        return {
            kinematics: kin,
            muscles: results,
            momentArms: {
                knee: kneeMomentArms,
                hip: hipMomentArms,
                ankle: ankleMomentArms
            }
        };
    }
}

// Export for browser window and Node modules
if (typeof window !== "undefined") {
    window.HillMuscleModel = HillMuscleModel;
    window.MusculoskeletalSystem = MusculoskeletalSystem;
}
if (typeof module !== "undefined" && module.exports) {
    module.exports = { HillMuscleModel, MusculoskeletalSystem };
}
