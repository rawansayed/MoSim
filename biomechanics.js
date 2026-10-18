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

/**
 * Upper Limb Musculoskeletal System (Holzbaur et al. 2005)
 * Models Arm, Elbow Flexion/Extension, and Forearm with 6 Musculotendon Units
 */
class UpperLimbSystem {
    constructor() {
        this.modelType = "upper_limb";
        this.displayName = "Upper Limb (Arm & Elbow 3-DOF)";

        this.coordinates = {
            shoulder_flexion: 0.0, // [-20, 100] deg
            elbow_flexion: 20.0,    // [0, 140] deg
            pro_sup: 0.0           // [-90, 90] deg (pronation/supination)
        };

        this.coordLimits = {
            elbow_flexion: { min: 0, max: 140, default: 20, label: "Elbow Flexion / Extension", unit: "°" },
            shoulder_flexion: { min: -20, max: 100, default: 0, label: "Shoulder Flexion", unit: "°" },
            pro_sup: { min: -90, max: 90, default: 0, label: "Forearm Pronation / Supination", unit: "°" }
        };

        this.segmentDimensions = {
            scapula: { width: 0.16, height: 0.18 },
            humerus: { length: 0.32, radius: 0.022 },
            forearm: { length: 0.28, radius: 0.018 },
            hand: { length: 0.18, width: 0.08 }
        };

        this.muscles = [
            new HillMuscleModel({
                name: "biceps_lh",
                displayName: "Biceps Brachii Long Head (Flexor)",
                maxIsometricForce: 624.0,
                optimalFiberLength: 0.116,
                tendonSlackLength: 0.272,
                pennationAngle: 0.0
            }),
            new HillMuscleModel({
                name: "biceps_sh",
                displayName: "Biceps Brachii Short Head (Flexor)",
                maxIsometricForce: 435.0,
                optimalFiberLength: 0.132,
                tendonSlackLength: 0.192,
                pennationAngle: 0.0
            }),
            new HillMuscleModel({
                name: "brachialis",
                displayName: "Brachialis (Deep Pure Elbow Flexor)",
                maxIsometricForce: 987.0,
                optimalFiberLength: 0.086,
                tendonSlackLength: 0.054,
                pennationAngle: 0.0
            }),
            new HillMuscleModel({
                name: "triceps_lh",
                displayName: "Triceps Brachii Long Head (Extensor)",
                maxIsometricForce: 798.0,
                optimalFiberLength: 0.134,
                tendonSlackLength: 0.188,
                pennationAngle: 0.12
            }),
            new HillMuscleModel({
                name: "triceps_lat",
                displayName: "Triceps Brachii Lateral Head (Extensor)",
                maxIsometricForce: 624.0,
                optimalFiberLength: 0.114,
                tendonSlackLength: 0.137,
                pennationAngle: 0.15
            }),
            new HillMuscleModel({
                name: "triceps_med",
                displayName: "Triceps Brachii Medial Head (Extensor)",
                maxIsometricForce: 624.0,
                optimalFiberLength: 0.068,
                tendonSlackLength: 0.121,
                pennationAngle: 0.15
            })
        ];

        this.muscleMap = new Map();
        this.muscles.forEach(m => this.muscleMap.set(m.name, m));
    }

    computeKinematics(coords = this.coordinates) {
        const rad = Math.PI / 180.0;
        const qShoulder = coords.shoulder_flexion * rad;
        const qElbow = coords.elbow_flexion * rad;
        const qProSup = coords.pro_sup * rad;

        const shoulderCenter = { x: 0.0, y: 0.0, z: 0.0 };
        const humerusLen = this.segmentDimensions.humerus.length;

        // Elbow joint center
        const elbowCenter = {
            x: shoulderCenter.x + humerusLen * Math.sin(qShoulder),
            y: shoulderCenter.y - humerusLen * Math.cos(qShoulder),
            z: shoulderCenter.z
        };

        // Forearm & Wrist center
        const forearmLen = this.segmentDimensions.forearm.length;
        const totalElbowAngle = qShoulder - qElbow;
        const wristCenter = {
            x: elbowCenter.x + forearmLen * Math.sin(totalElbowAngle),
            y: elbowCenter.y - forearmLen * Math.cos(totalElbowAngle),
            z: elbowCenter.z
        };

        // Olecranon process (triceps extensor lever arm behind elbow)
        const olecranon = {
            x: elbowCenter.x - 0.025 * Math.sin(totalElbowAngle),
            y: elbowCenter.y + 0.025 * Math.cos(totalElbowAngle),
            z: elbowCenter.z
        };

        return {
            shoulderCenter,
            elbowCenter,
            wristCenter,
            olecranon,
            anglesRad: { qShoulder, qElbow, qProSup }
        };
    }

    computeMusclePaths(kin) {
        const p = kin;
        const paths = {};

        // 1. Biceps Long Head: Supraglenoid tubercle -> Radial tuberosity
        paths["biceps_lh"] = [
            { x: p.shoulderCenter.x + 0.015, y: p.shoulderCenter.y + 0.03, z: p.shoulderCenter.z + 0.02 },
            { x: 0.5 * (p.shoulderCenter.x + p.elbowCenter.x) + 0.025, y: 0.5 * (p.shoulderCenter.y + p.elbowCenter.y), z: p.shoulderCenter.z + 0.02 },
            { x: p.elbowCenter.x + 0.02, y: p.elbowCenter.y + 0.01, z: p.elbowCenter.z },
            { x: p.elbowCenter.x + 0.025, y: p.elbowCenter.y - 0.035, z: p.elbowCenter.z }
        ];

        // 2. Biceps Short Head: Coracoid process -> Radial tuberosity
        paths["biceps_sh"] = [
            { x: p.shoulderCenter.x + 0.03, y: p.shoulderCenter.y + 0.015, z: p.shoulderCenter.z - 0.015 },
            { x: 0.5 * (p.shoulderCenter.x + p.elbowCenter.x) + 0.02, y: 0.5 * (p.shoulderCenter.y + p.elbowCenter.y) - 0.02, z: p.shoulderCenter.z - 0.01 },
            { x: p.elbowCenter.x + 0.025, y: p.elbowCenter.y - 0.035, z: p.elbowCenter.z }
        ];

        // 3. Brachialis: Anterior lower humerus -> Coronoid process of ulna
        paths["brachialis"] = [
            { x: 0.5 * (p.shoulderCenter.x + p.elbowCenter.x) + 0.01, y: 0.5 * (p.shoulderCenter.y + p.elbowCenter.y) - 0.04, z: p.shoulderCenter.z },
            { x: p.elbowCenter.x + 0.02, y: p.elbowCenter.y, z: p.elbowCenter.z },
            { x: p.elbowCenter.x + 0.015, y: p.elbowCenter.y - 0.03, z: p.elbowCenter.z }
        ];

        // 4. Triceps Long Head: Infraglenoid tubercle -> Olecranon process
        paths["triceps_lh"] = [
            { x: p.shoulderCenter.x - 0.02, y: p.shoulderCenter.y - 0.015, z: p.shoulderCenter.z },
            { x: 0.5 * (p.shoulderCenter.x + p.elbowCenter.x) - 0.025, y: 0.5 * (p.shoulderCenter.y + p.elbowCenter.y), z: p.shoulderCenter.z },
            { x: p.olecranon.x, y: p.olecranon.y, z: p.olecranon.z }
        ];

        // 5. Triceps Lateral Head: Posterior proximal humerus -> Olecranon process
        paths["triceps_lat"] = [
            { x: p.shoulderCenter.x - 0.01, y: p.shoulderCenter.y - 0.06, z: p.shoulderCenter.z + 0.02 },
            { x: 0.5 * (p.shoulderCenter.x + p.elbowCenter.x) - 0.02, y: 0.5 * (p.shoulderCenter.y + p.elbowCenter.y) - 0.03, z: p.shoulderCenter.z + 0.015 },
            { x: p.olecranon.x, y: p.olecranon.y, z: p.olecranon.z }
        ];

        // 6. Triceps Medial Head: Posterior distal humerus -> Olecranon process
        paths["triceps_med"] = [
            { x: 0.5 * (p.shoulderCenter.x + p.elbowCenter.x) - 0.01, y: 0.5 * (p.shoulderCenter.y + p.elbowCenter.y) - 0.07, z: p.shoulderCenter.z - 0.015 },
            { x: p.olecranon.x, y: p.olecranon.y, z: p.olecranon.z }
        ];

        return paths;
    }

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

    computeMomentArms(coordName = "elbow_flexion", deltaDeg = 0.5) {
        const currentCoord = this.coordinates[coordName];
        const radDelta = (deltaDeg * Math.PI) / 180.0;

        const coordsPlus = { ...this.coordinates, [coordName]: currentCoord + deltaDeg };
        const kinPlus = this.computeKinematics(coordsPlus);
        const pathsPlus = this.computeMusclePaths(kinPlus);

        const coordsMinus = { ...this.coordinates, [coordName]: currentCoord - deltaDeg };
        const kinMinus = this.computeKinematics(coordsMinus);
        const pathsMinus = this.computeMusclePaths(kinMinus);

        const momentArms = {};
        for (const muscle of this.muscles) {
            const lenPlus = this.computePathLength(pathsPlus[muscle.name]);
            const lenMinus = this.computePathLength(pathsMinus[muscle.name]);
            const r_rad = -(lenPlus - lenMinus) / (2.0 * radDelta);
            momentArms[muscle.name] = r_rad;
        }

        return momentArms;
    }

    update() {
        const kin = this.computeKinematics(this.coordinates);
        const paths = this.computeMusclePaths(kin);
        const results = {};

        for (const muscle of this.muscles) {
            const path = paths[muscle.name];
            const lmt = this.computePathLength(path);
            const state = muscle.solveEquilibrium(lmt, muscle.activation);
            results[muscle.name] = { state, path };
        }

        const elbowMomentArms = this.computeMomentArms("elbow_flexion");
        const shoulderMomentArms = this.computeMomentArms("shoulder_flexion");

        return {
            kinematics: kin,
            muscles: results,
            momentArms: {
                elbow: elbowMomentArms,
                shoulder: shoulderMomentArms
            }
        };
    }
}

/**
 * Whole Body & Modular Regional Musculoskeletal System
 * Supports:
 * - "whole_body": Complete human (Skull, Neck, Thorax, Bilateral Arms & Hands, Pelvis, Bilateral Legs & Feet)
 * - "lower_body": Pelvis, Right Leg & Foot, Left Leg & Foot
 * - "upper_body": Skull, Neck, Thorax, Right Arm & Hand, Left Arm & Hand
 * - "right_arm": Right Arm & Hand (Humerus, Forearm, Hand & 6 MTUs)
 * - "left_arm": Left Arm & Hand (Humerus, Forearm, Hand & 6 MTUs)
 * - "right_leg": Right Leg (Pelvis, Femur, Tibia, Foot & 10 MTUs)
 * - "left_leg": Left Leg (Pelvis, Femur, Tibia, Foot & 10 MTUs)
 * - "head_neck": Head & Neck (Cranium, Mandible, C1-C7 Column & Cervical MTUs)
 */
class WholeBodySystem {
    constructor(scope = "whole_body") {
        this.scope = scope;
        this.modelType = scope;
        this.initScope();
    }

    initScope() {
        const s = this.scope;
        if (s === "whole_body") {
            this.displayName = "Whole Body (Upper & Lower Musculoskeletal Human)";
            this.coordinates = {
                neck_flexion: 0.0,
                neck_rotation: 0.0,
                shoulder_flexion_r: 0.0,
                elbow_flexion_r: 15.0,
                shoulder_flexion_l: 0.0,
                elbow_flexion_l: 15.0,
                hip_flexion_r: 0.0,
                knee_angle_r: 0.0,
                ankle_angle_r: 0.0,
                hip_flexion_l: 0.0,
                knee_angle_l: 0.0,
                ankle_angle_l: 0.0
            };
            this.coordLimits = {
                neck_flexion: { min: -20, max: 40, default: 0, label: "Neck Flexion / Extension", unit: "°" },
                neck_rotation: { min: -40, max: 40, default: 0, label: "Neck Axial Rotation", unit: "°" },
                shoulder_flexion_r: { min: -20, max: 100, default: 0, label: "R. Shoulder Flexion", unit: "°" },
                elbow_flexion_r: { min: 0, max: 140, default: 15, label: "R. Elbow Flexion", unit: "°" },
                shoulder_flexion_l: { min: -20, max: 100, default: 0, label: "L. Shoulder Flexion", unit: "°" },
                elbow_flexion_l: { min: 0, max: 140, default: 15, label: "L. Elbow Flexion", unit: "°" },
                hip_flexion_r: { min: -30, max: 120, default: 0, label: "R. Hip Flexion", unit: "°" },
                knee_angle_r: { min: 0, max: 140, default: 0, label: "R. Knee Angle", unit: "°" },
                ankle_angle_r: { min: -30, max: 50, default: 0, label: "R. Ankle Angle", unit: "°" },
                hip_flexion_l: { min: -30, max: 120, default: 0, label: "L. Hip Flexion", unit: "°" },
                knee_angle_l: { min: 0, max: 140, default: 0, label: "L. Knee Angle", unit: "°" },
                ankle_angle_l: { min: -30, max: 50, default: 0, label: "L. Ankle Angle", unit: "°" }
            };
            this.muscles = this.createWholeBodyMuscles();
        } else if (s === "lower_body") {
            this.displayName = "Lower Part (Pelvis, Right Leg, Left Leg)";
            this.coordinates = {
                hip_flexion_r: 0.0,
                knee_angle_r: 0.0,
                ankle_angle_r: 0.0,
                hip_flexion_l: 0.0,
                knee_angle_l: 0.0,
                ankle_angle_l: 0.0
            };
            this.coordLimits = {
                hip_flexion_r: { min: -30, max: 120, default: 0, label: "R. Hip Flexion", unit: "°" },
                knee_angle_r: { min: 0, max: 140, default: 0, label: "R. Knee Angle", unit: "°" },
                ankle_angle_r: { min: -30, max: 50, default: 0, label: "R. Ankle Angle", unit: "°" },
                hip_flexion_l: { min: -30, max: 120, default: 0, label: "L. Hip Flexion", unit: "°" },
                knee_angle_l: { min: 0, max: 140, default: 0, label: "L. Knee Angle", unit: "°" },
                ankle_angle_l: { min: -30, max: 50, default: 0, label: "L. Ankle Angle", unit: "°" }
            };
            this.muscles = this.createBilateralLegMuscles();
        } else if (s === "upper_body") {
            this.displayName = "Upper Part (Neck, Skull, Right & Left Hand / Arm)";
            this.coordinates = {
                neck_flexion: 0.0,
                neck_rotation: 0.0,
                shoulder_flexion_r: 0.0,
                elbow_flexion_r: 15.0,
                shoulder_flexion_l: 0.0,
                elbow_flexion_l: 15.0
            };
            this.coordLimits = {
                neck_flexion: { min: -20, max: 40, default: 0, label: "Neck Flexion / Extension", unit: "°" },
                neck_rotation: { min: -40, max: 40, default: 0, label: "Neck Axial Rotation", unit: "°" },
                shoulder_flexion_r: { min: -20, max: 100, default: 0, label: "R. Shoulder Flexion", unit: "°" },
                elbow_flexion_r: { min: 0, max: 140, default: 15, label: "R. Elbow Flexion", unit: "°" },
                shoulder_flexion_l: { min: -20, max: 100, default: 0, label: "L. Shoulder Flexion", unit: "°" },
                elbow_flexion_l: { min: 0, max: 140, default: 15, label: "L. Elbow Flexion", unit: "°" }
            };
            this.muscles = this.createUpperBodyMuscles();
        } else if (s === "right_arm") {
            this.displayName = "Right Arm & Hand Only (Humerus, Forearm, Hand & 6 MTUs)";
            this.coordinates = {
                shoulder_flexion: 0.0,
                elbow_flexion: 20.0,
                pro_sup: 0.0
            };
            this.coordLimits = {
                elbow_flexion: { min: 0, max: 140, default: 20, label: "Elbow Flexion / Extension", unit: "°" },
                shoulder_flexion: { min: -20, max: 100, default: 0, label: "Shoulder Flexion", unit: "°" },
                pro_sup: { min: -90, max: 90, default: 0, label: "Forearm Pronation / Supination", unit: "°" }
            };
            this.muscles = this.createSingleArmMuscles("r");
        } else if (s === "left_arm") {
            this.displayName = "Left Arm & Hand Only (Humerus, Forearm, Hand & 6 MTUs)";
            this.coordinates = {
                shoulder_flexion: 0.0,
                elbow_flexion: 20.0,
                pro_sup: 0.0
            };
            this.coordLimits = {
                elbow_flexion: { min: 0, max: 140, default: 20, label: "Elbow Flexion / Extension", unit: "°" },
                shoulder_flexion: { min: -20, max: 100, default: 0, label: "Shoulder Flexion", unit: "°" },
                pro_sup: { min: -90, max: 90, default: 0, label: "Forearm Pronation / Supination", unit: "°" }
            };
            this.muscles = this.createSingleArmMuscles("l");
        } else if (s === "right_leg") {
            this.displayName = "Right Leg Only (Pelvis, Femur, Tibia, Foot & 10 MTUs)";
            this.coordinates = {
                hip_flexion: 0.0,
                hip_adduction: 0.0,
                hip_rotation: 0.0,
                knee_angle: 0.0,
                ankle_angle: 0.0
            };
            this.coordLimits = {
                hip_flexion: { min: -30, max: 120, default: 0, label: "Hip Flexion / Extension", unit: "°" },
                hip_adduction: { min: -30, max: 45, default: 0, label: "Hip Adduction / Abduction", unit: "°" },
                hip_rotation: { min: -40, max: 40, default: 0, label: "Hip Rotation", unit: "°" },
                knee_angle: { min: 0, max: 140, default: 0, label: "Knee Angle", unit: "°" },
                ankle_angle: { min: -30, max: 50, default: 0, label: "Ankle Angle", unit: "°" }
            };
            this.muscles = this.createSingleLegMuscles("r");
        } else if (s === "left_leg") {
            this.displayName = "Left Leg Only (Pelvis, Femur, Tibia, Foot & 10 MTUs)";
            this.coordinates = {
                hip_flexion: 0.0,
                hip_adduction: 0.0,
                hip_rotation: 0.0,
                knee_angle: 0.0,
                ankle_angle: 0.0
            };
            this.coordLimits = {
                hip_flexion: { min: -30, max: 120, default: 0, label: "Hip Flexion / Extension", unit: "°" },
                hip_adduction: { min: -30, max: 45, default: 0, label: "Hip Adduction / Abduction", unit: "°" },
                hip_rotation: { min: -40, max: 40, default: 0, label: "Hip Rotation", unit: "°" },
                knee_angle: { min: 0, max: 140, default: 0, label: "Knee Angle", unit: "°" },
                ankle_angle: { min: -30, max: 50, default: 0, label: "Ankle Angle", unit: "°" }
            };
            this.muscles = this.createSingleLegMuscles("l");
        } else if (s === "head_neck") {
            this.displayName = "Head & Neck Only (Skull, Mandible, C1-C7 Column & MTUs)";
            this.coordinates = {
                neck_flexion: 0.0,
                neck_rotation: 0.0,
                jaw_opening: 0.0
            };
            this.coordLimits = {
                neck_flexion: { min: -25, max: 45, default: 0, label: "Neck Flexion / Extension", unit: "°" },
                neck_rotation: { min: -45, max: 45, default: 0, label: "Cervical Axial Rotation", unit: "°" },
                jaw_opening: { min: 0, max: 35, default: 0, label: "Mandible Depression (Jaw)", unit: "°" }
            };
            this.muscles = this.createHeadNeckMuscles();
        }

        this.muscleMap = new Map();
        this.muscles.forEach(m => this.muscleMap.set(m.name, m));
    }

    createWholeBodyMuscles() {
        return [
            // Neck
            new HillMuscleModel({ name: "scm_r", displayName: "R. Sternocleidomastoid (Neck Flexor)", maxIsometricForce: 160.0, optimalFiberLength: 0.09, tendonSlackLength: 0.07, pennationAngle: 0.05 }),
            new HillMuscleModel({ name: "scm_l", displayName: "L. Sternocleidomastoid (Neck Flexor)", maxIsometricForce: 160.0, optimalFiberLength: 0.09, tendonSlackLength: 0.07, pennationAngle: 0.05 }),
            new HillMuscleModel({ name: "splenius", displayName: "Splenius Capitis (Cervical Extensor)", maxIsometricForce: 220.0, optimalFiberLength: 0.10, tendonSlackLength: 0.10, pennationAngle: 0.08 }),

            // Right Arm
            new HillMuscleModel({ name: "biceps_r", displayName: "R. Biceps Brachii (Arm Flexor)", maxIsometricForce: 624.0, optimalFiberLength: 0.116, tendonSlackLength: 0.23, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "brachialis_r", displayName: "R. Brachialis (Elbow Flexor)", maxIsometricForce: 987.0, optimalFiberLength: 0.086, tendonSlackLength: 0.064, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "triceps_r", displayName: "R. Triceps Brachii (Elbow Extensor)", maxIsometricForce: 798.0, optimalFiberLength: 0.134, tendonSlackLength: 0.188, pennationAngle: 0.12 }),

            // Left Arm
            new HillMuscleModel({ name: "biceps_l", displayName: "L. Biceps Brachii (Arm Flexor)", maxIsometricForce: 624.0, optimalFiberLength: 0.116, tendonSlackLength: 0.23, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "brachialis_l", displayName: "L. Brachialis (Elbow Flexor)", maxIsometricForce: 987.0, optimalFiberLength: 0.086, tendonSlackLength: 0.064, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "triceps_l", displayName: "L. Triceps Brachii (Elbow Extensor)", maxIsometricForce: 798.0, optimalFiberLength: 0.134, tendonSlackLength: 0.188, pennationAngle: 0.12 }),

            // Right Leg
            new HillMuscleModel({ name: "rect_fem_r", displayName: "R. Rectus Femoris (Quad Bi-articular)", maxIsometricForce: 1169.0, optimalFiberLength: 0.084, tendonSlackLength: 0.346, pennationAngle: 0.087 }),
            new HillMuscleModel({ name: "vasti_r", displayName: "R. Vastus Complex (Knee Extensor)", maxIsometricForce: 2500.0, optimalFiberLength: 0.089, tendonSlackLength: 0.136, pennationAngle: 0.087 }),
            new HillMuscleModel({ name: "bifemlh_r", displayName: "R. Biceps Femoris (Hamstring)", maxIsometricForce: 896.0, optimalFiberLength: 0.109, tendonSlackLength: 0.326, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "glut_max_r", displayName: "R. Gluteus Maximus (Hip Extensor)", maxIsometricForce: 1944.0, optimalFiberLength: 0.157, tendonSlackLength: 0.127, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "gas_med_r", displayName: "R. Gastrocnemius (Calf Plantarflexor)", maxIsometricForce: 1558.0, optimalFiberLength: 0.051, tendonSlackLength: 0.398, pennationAngle: 0.297 }),
            new HillMuscleModel({ name: "tib_ant_r", displayName: "R. Tibialis Anterior (Dorsiflexor)", maxIsometricForce: 603.0, optimalFiberLength: 0.098, tendonSlackLength: 0.223, pennationAngle: 0.087 }),

            // Left Leg
            new HillMuscleModel({ name: "rect_fem_l", displayName: "L. Rectus Femoris (Quad Bi-articular)", maxIsometricForce: 1169.0, optimalFiberLength: 0.084, tendonSlackLength: 0.346, pennationAngle: 0.087 }),
            new HillMuscleModel({ name: "vasti_l", displayName: "L. Vastus Complex (Knee Extensor)", maxIsometricForce: 2500.0, optimalFiberLength: 0.089, tendonSlackLength: 0.136, pennationAngle: 0.087 }),
            new HillMuscleModel({ name: "bifemlh_l", displayName: "L. Biceps Femoris (Hamstring)", maxIsometricForce: 896.0, optimalFiberLength: 0.109, tendonSlackLength: 0.326, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "glut_max_l", displayName: "L. Gluteus Maximus (Hip Extensor)", maxIsometricForce: 1944.0, optimalFiberLength: 0.157, tendonSlackLength: 0.127, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "gas_med_l", displayName: "L. Gastrocnemius (Calf Plantarflexor)", maxIsometricForce: 1558.0, optimalFiberLength: 0.051, tendonSlackLength: 0.398, pennationAngle: 0.297 }),
            new HillMuscleModel({ name: "tib_ant_l", displayName: "L. Tibialis Anterior (Dorsiflexor)", maxIsometricForce: 603.0, optimalFiberLength: 0.098, tendonSlackLength: 0.223, pennationAngle: 0.087 })
        ];
    }

    createBilateralLegMuscles() {
        return [
            new HillMuscleModel({ name: "rect_fem_r", displayName: "R. Rectus Femoris (Quad)", maxIsometricForce: 1169.0, optimalFiberLength: 0.084, tendonSlackLength: 0.346, pennationAngle: 0.087 }),
            new HillMuscleModel({ name: "vasti_r", displayName: "R. Vastus Complex (Extensor)", maxIsometricForce: 2500.0, optimalFiberLength: 0.089, tendonSlackLength: 0.136, pennationAngle: 0.087 }),
            new HillMuscleModel({ name: "bifemlh_r", displayName: "R. Biceps Femoris (Hamstring)", maxIsometricForce: 896.0, optimalFiberLength: 0.109, tendonSlackLength: 0.326, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "glut_max_r", displayName: "R. Gluteus Maximus (Extensor)", maxIsometricForce: 1944.0, optimalFiberLength: 0.157, tendonSlackLength: 0.127, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "gas_med_r", displayName: "R. Gastrocnemius (Calf)", maxIsometricForce: 1558.0, optimalFiberLength: 0.051, tendonSlackLength: 0.398, pennationAngle: 0.297 }),
            new HillMuscleModel({ name: "tib_ant_r", displayName: "R. Tibialis Anterior (Dorsiflexor)", maxIsometricForce: 603.0, optimalFiberLength: 0.098, tendonSlackLength: 0.223, pennationAngle: 0.087 }),

            new HillMuscleModel({ name: "rect_fem_l", displayName: "L. Rectus Femoris (Quad)", maxIsometricForce: 1169.0, optimalFiberLength: 0.084, tendonSlackLength: 0.346, pennationAngle: 0.087 }),
            new HillMuscleModel({ name: "vasti_l", displayName: "L. Vastus Complex (Extensor)", maxIsometricForce: 2500.0, optimalFiberLength: 0.089, tendonSlackLength: 0.136, pennationAngle: 0.087 }),
            new HillMuscleModel({ name: "bifemlh_l", displayName: "L. Biceps Femoris (Hamstring)", maxIsometricForce: 896.0, optimalFiberLength: 0.109, tendonSlackLength: 0.326, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "glut_max_l", displayName: "L. Gluteus Maximus (Extensor)", maxIsometricForce: 1944.0, optimalFiberLength: 0.157, tendonSlackLength: 0.127, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "gas_med_l", displayName: "L. Gastrocnemius (Calf)", maxIsometricForce: 1558.0, optimalFiberLength: 0.051, tendonSlackLength: 0.398, pennationAngle: 0.297 }),
            new HillMuscleModel({ name: "tib_ant_l", displayName: "L. Tibialis Anterior (Dorsiflexor)", maxIsometricForce: 603.0, optimalFiberLength: 0.098, tendonSlackLength: 0.223, pennationAngle: 0.087 })
        ];
    }

    createUpperBodyMuscles() {
        return [
            new HillMuscleModel({ name: "scm_r", displayName: "R. Sternocleidomastoid", maxIsometricForce: 160.0, optimalFiberLength: 0.09, tendonSlackLength: 0.07, pennationAngle: 0.05 }),
            new HillMuscleModel({ name: "scm_l", displayName: "L. Sternocleidomastoid", maxIsometricForce: 160.0, optimalFiberLength: 0.09, tendonSlackLength: 0.07, pennationAngle: 0.05 }),
            new HillMuscleModel({ name: "splenius", displayName: "Splenius Capitis", maxIsometricForce: 220.0, optimalFiberLength: 0.10, tendonSlackLength: 0.10, pennationAngle: 0.08 }),

            new HillMuscleModel({ name: "biceps_r", displayName: "R. Biceps Brachii", maxIsometricForce: 624.0, optimalFiberLength: 0.116, tendonSlackLength: 0.23, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "brachialis_r", displayName: "R. Brachialis", maxIsometricForce: 987.0, optimalFiberLength: 0.086, tendonSlackLength: 0.064, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "triceps_r", displayName: "R. Triceps Brachii", maxIsometricForce: 798.0, optimalFiberLength: 0.134, tendonSlackLength: 0.188, pennationAngle: 0.12 }),

            new HillMuscleModel({ name: "biceps_l", displayName: "L. Biceps Brachii", maxIsometricForce: 624.0, optimalFiberLength: 0.116, tendonSlackLength: 0.23, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "brachialis_l", displayName: "L. Brachialis", maxIsometricForce: 987.0, optimalFiberLength: 0.086, tendonSlackLength: 0.064, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "triceps_l", displayName: "L. Triceps Brachii", maxIsometricForce: 798.0, optimalFiberLength: 0.134, tendonSlackLength: 0.188, pennationAngle: 0.12 })
        ];
    }

    createSingleArmMuscles(side = "r") {
        const prefix = side === "r" ? "R. " : "L. ";
        return [
            new HillMuscleModel({ name: "biceps_lh", displayName: `${prefix}Biceps Brachii Long Head (Flexor)`, maxIsometricForce: 624.0, optimalFiberLength: 0.116, tendonSlackLength: 0.272, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "biceps_sh", displayName: `${prefix}Biceps Brachii Short Head (Flexor)`, maxIsometricForce: 435.0, optimalFiberLength: 0.132, tendonSlackLength: 0.192, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "brachialis", displayName: `${prefix}Brachialis (Deep Elbow Flexor)`, maxIsometricForce: 987.0, optimalFiberLength: 0.086, tendonSlackLength: 0.054, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "triceps_lh", displayName: `${prefix}Triceps Long Head (Extensor)`, maxIsometricForce: 798.0, optimalFiberLength: 0.134, tendonSlackLength: 0.188, pennationAngle: 0.12 }),
            new HillMuscleModel({ name: "triceps_lat", displayName: `${prefix}Triceps Lateral Head (Extensor)`, maxIsometricForce: 624.0, optimalFiberLength: 0.114, tendonSlackLength: 0.137, pennationAngle: 0.15 }),
            new HillMuscleModel({ name: "triceps_med", displayName: `${prefix}Triceps Medial Head (Extensor)`, maxIsometricForce: 624.0, optimalFiberLength: 0.068, tendonSlackLength: 0.121, pennationAngle: 0.15 })
        ];
    }

    createSingleLegMuscles(side = "r") {
        const prefix = side === "r" ? "R. " : "L. ";
        return [
            new HillMuscleModel({ name: "rect_fem", displayName: `${prefix}Rectus Femoris (Quad Bi-articular)`, maxIsometricForce: 1169.0, optimalFiberLength: 0.084, tendonSlackLength: 0.346, pennationAngle: 0.087 }),
            new HillMuscleModel({ name: "vas_int", displayName: `${prefix}Vastus Intermedius (Mono-articular)`, maxIsometricForce: 1365.0, optimalFiberLength: 0.087, tendonSlackLength: 0.136, pennationAngle: 0.052 }),
            new HillMuscleModel({ name: "vas_lat", displayName: `${prefix}Vastus Lateralis (Mono-articular)`, maxIsometricForce: 1871.0, optimalFiberLength: 0.084, tendonSlackLength: 0.157, pennationAngle: 0.087 }),
            new HillMuscleModel({ name: "vas_med", displayName: `${prefix}Vastus Medialis (Mono-articular)`, maxIsometricForce: 1294.0, optimalFiberLength: 0.089, tendonSlackLength: 0.126, pennationAngle: 0.087 }),
            new HillMuscleModel({ name: "bifemlh", displayName: `${prefix}Biceps Femoris Long Head (Hamstring)`, maxIsometricForce: 896.0, optimalFiberLength: 0.109, tendonSlackLength: 0.326, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "semiten", displayName: `${prefix}Semitendinosus (Hamstring Medial)`, maxIsometricForce: 410.0, optimalFiberLength: 0.201, tendonSlackLength: 0.262, pennationAngle: 0.087 }),
            new HillMuscleModel({ name: "iliopsoas", displayName: `${prefix}Iliopsoas (Primary Hip Flexor)`, maxIsometricForce: 1042.0, optimalFiberLength: 0.102, tendonSlackLength: 0.142, pennationAngle: 0.140 }),
            new HillMuscleModel({ name: "glut_max", displayName: `${prefix}Gluteus Maximus (Hip Extensor)`, maxIsometricForce: 1944.0, optimalFiberLength: 0.157, tendonSlackLength: 0.127, pennationAngle: 0.0 }),
            new HillMuscleModel({ name: "gas_med", displayName: `${prefix}Gastrocnemius Medialis (Calf)`, maxIsometricForce: 1558.0, optimalFiberLength: 0.051, tendonSlackLength: 0.398, pennationAngle: 0.297 }),
            new HillMuscleModel({ name: "soleus", displayName: `${prefix}Soleus (Deep Calf Mono-articular)`, maxIsometricForce: 3549.0, optimalFiberLength: 0.044, tendonSlackLength: 0.268, pennationAngle: 0.436 }),
            new HillMuscleModel({ name: "tib_ant", displayName: `${prefix}Tibialis Anterior (Dorsiflexor)`, maxIsometricForce: 603.0, optimalFiberLength: 0.098, tendonSlackLength: 0.223, pennationAngle: 0.087 })
        ];
    }

    createHeadNeckMuscles() {
        return [
            new HillMuscleModel({ name: "scm_r", displayName: "R. Sternocleidomastoid (Flexor/Rotator)", maxIsometricForce: 160.0, optimalFiberLength: 0.09, tendonSlackLength: 0.07, pennationAngle: 0.05 }),
            new HillMuscleModel({ name: "scm_l", displayName: "L. Sternocleidomastoid (Flexor/Rotator)", maxIsometricForce: 160.0, optimalFiberLength: 0.09, tendonSlackLength: 0.07, pennationAngle: 0.05 }),
            new HillMuscleModel({ name: "splenius", displayName: "Splenius Capitis (Cervical Extensor)", maxIsometricForce: 220.0, optimalFiberLength: 0.10, tendonSlackLength: 0.10, pennationAngle: 0.08 }),
            new HillMuscleModel({ name: "masseter", displayName: "Masseter (Mandible Elevator/Jaw)", maxIsometricForce: 350.0, optimalFiberLength: 0.035, tendonSlackLength: 0.025, pennationAngle: 0.10 })
        ];
    }

    /**
     * Multibody Spatial Forward Kinematics for Full Body or Active Sub-System
     */
    computeKinematics(coords = this.coordinates) {
        const rad = Math.PI / 180.0;
        const c = coords;

        // Neck coordinates
        const qNeckFlex = (c.neck_flexion !== undefined ? c.neck_flexion : 0.0) * rad;
        const qNeckRot = (c.neck_rotation !== undefined ? c.neck_rotation : 0.0) * rad;
        const qJaw = (c.jaw_opening !== undefined ? c.jaw_opening : 0.0) * rad;

        // Arm coordinates (bilateral or single)
        const qShoulderR = (c.shoulder_flexion_r !== undefined ? c.shoulder_flexion_r : (c.shoulder_flexion !== undefined ? c.shoulder_flexion : 0.0)) * rad;
        const qElbowR = (c.elbow_flexion_r !== undefined ? c.elbow_flexion_r : (c.elbow_flexion !== undefined ? c.elbow_flexion : 20.0)) * rad;
        const qProSupR = (c.pro_sup !== undefined ? c.pro_sup : 0.0) * rad;

        const qShoulderL = (c.shoulder_flexion_l !== undefined ? c.shoulder_flexion_l : (c.shoulder_flexion !== undefined ? c.shoulder_flexion : 0.0)) * rad;
        const qElbowL = (c.elbow_flexion_l !== undefined ? c.elbow_flexion_l : (c.elbow_flexion !== undefined ? c.elbow_flexion : 20.0)) * rad;

        // Leg coordinates (bilateral or single)
        const qHipFlexR = (c.hip_flexion_r !== undefined ? c.hip_flexion_r : (c.hip_flexion !== undefined ? c.hip_flexion : 0.0)) * rad;
        const qHipAddR = (c.hip_adduction !== undefined ? c.hip_adduction : 0.0) * rad;
        const qKneeR = (c.knee_angle_r !== undefined ? c.knee_angle_r : (c.knee_angle !== undefined ? c.knee_angle : 0.0)) * rad;
        const qAnkleR = (c.ankle_angle_r !== undefined ? c.ankle_angle_r : (c.ankle_angle !== undefined ? c.ankle_angle : 0.0)) * rad;

        const qHipFlexL = (c.hip_flexion_l !== undefined ? c.hip_flexion_l : (c.hip_flexion !== undefined ? c.hip_flexion : 0.0)) * rad;
        const qKneeL = (c.knee_angle_l !== undefined ? c.knee_angle_l : (c.knee_angle !== undefined ? c.knee_angle : 0.0)) * rad;
        const qAnkleL = (c.ankle_angle_l !== undefined ? c.ankle_angle_l : (c.ankle_angle !== undefined ? c.ankle_angle : 0.0)) * rad;

        const isSingleLimb = (this.scope === "right_arm" || this.scope === "left_arm" || this.scope === "right_leg" || this.scope === "left_leg");
        const zOffsetLeg = isSingleLimb ? 0.0 : 0.10;
        const zOffsetArm = isSingleLimb ? 0.0 : 0.22;

        // 1. Pelvis Frame
        const pelvisCenter = { x: 0.0, y: 0.0, z: 0.0 };

        // 2. Thorax, Neck & Skull
        const thoraxCenter = { x: 0.0, y: 0.28, z: 0.0 };
        const neckBase = { x: 0.0, y: 0.44, z: 0.0 };
        const neckLen = 0.12;
        const neckTop = {
            x: neckBase.x + neckLen * Math.sin(qNeckFlex),
            y: neckBase.y + neckLen * Math.cos(qNeckFlex),
            z: neckBase.z + neckLen * Math.sin(qNeckRot)
        };
        const skullCenter = {
            x: neckTop.x + 0.02 * Math.cos(qNeckFlex),
            y: neckTop.y + 0.08,
            z: neckTop.z
        };
        const chin = {
            x: skullCenter.x + 0.08 + 0.02 * Math.sin(qJaw),
            y: skullCenter.y - 0.07 - 0.03 * Math.sin(qJaw),
            z: skullCenter.z
        };

        // 3. Right Leg Spatial Kinematics
        const hipCenterR = { x: 0.06, y: -0.06, z: zOffsetLeg };
        const femurLen = 0.42;
        const kneeCenterR = {
            x: hipCenterR.x + femurLen * Math.sin(qHipFlexR),
            y: hipCenterR.y - femurLen * Math.cos(qHipFlexR),
            z: hipCenterR.z + femurLen * Math.sin(qHipAddR)
        };
        const tibiaLen = 0.40;
        const totalKneeFlexR = qHipFlexR - qKneeR;
        const ankleCenterR = {
            x: kneeCenterR.x + tibiaLen * Math.sin(totalKneeFlexR),
            y: kneeCenterR.y - tibiaLen * Math.cos(totalKneeFlexR),
            z: kneeCenterR.z
        };
        const totalFootAngleR = totalKneeFlexR - qAnkleR;
        const calcaneusR = {
            x: ankleCenterR.x - 0.05 * Math.cos(totalFootAngleR),
            y: ankleCenterR.y - 0.04,
            z: ankleCenterR.z
        };
        const toeCenterR = {
            x: ankleCenterR.x + 0.17 * Math.cos(totalFootAngleR),
            y: ankleCenterR.y - 0.04,
            z: ankleCenterR.z
        };
        const patellaCenterR = {
            x: kneeCenterR.x + 0.045 * Math.cos(qHipFlexR - 0.5 * qKneeR),
            y: kneeCenterR.y + 0.045 * Math.sin(qHipFlexR - 0.5 * qKneeR),
            z: kneeCenterR.z
        };

        // 4. Left Leg Spatial Kinematics
        const hipCenterL = { x: 0.06, y: -0.06, z: -zOffsetLeg };
        const kneeCenterL = {
            x: hipCenterL.x + femurLen * Math.sin(qHipFlexL),
            y: hipCenterL.y - femurLen * Math.cos(qHipFlexL),
            z: hipCenterL.z
        };
        const totalKneeFlexL = qHipFlexL - qKneeL;
        const ankleCenterL = {
            x: kneeCenterL.x + tibiaLen * Math.sin(totalKneeFlexL),
            y: kneeCenterL.y - tibiaLen * Math.cos(totalKneeFlexL),
            z: kneeCenterL.z
        };
        const totalFootAngleL = totalKneeFlexL - qAnkleL;
        const calcaneusL = {
            x: ankleCenterL.x - 0.05 * Math.cos(totalFootAngleL),
            y: ankleCenterL.y - 0.04,
            z: ankleCenterL.z
        };
        const toeCenterL = {
            x: ankleCenterL.x + 0.17 * Math.cos(totalFootAngleL),
            y: ankleCenterL.y - 0.04,
            z: ankleCenterL.z
        };
        const patellaCenterL = {
            x: kneeCenterL.x + 0.045 * Math.cos(qHipFlexL - 0.5 * qKneeL),
            y: kneeCenterL.y + 0.045 * Math.sin(qHipFlexL - 0.5 * qKneeL),
            z: kneeCenterL.z
        };

        // 5. Right Arm Spatial Kinematics
        const shoulderCenterR = { x: 0.0, y: 0.42, z: zOffsetArm };
        const humerusLen = 0.30;
        const elbowCenterR = {
            x: shoulderCenterR.x + humerusLen * Math.sin(qShoulderR),
            y: shoulderCenterR.y - humerusLen * Math.cos(qShoulderR),
            z: shoulderCenterR.z
        };
        const forearmLen = 0.26;
        const totalElbowR = qShoulderR - qElbowR;
        const wristCenterR = {
            x: elbowCenterR.x + forearmLen * Math.sin(totalElbowR),
            y: elbowCenterR.y - forearmLen * Math.cos(totalElbowR),
            z: elbowCenterR.z
        };
        const olecranonR = {
            x: elbowCenterR.x - 0.025 * Math.sin(totalElbowR),
            y: elbowCenterR.y + 0.025 * Math.cos(totalElbowR),
            z: elbowCenterR.z
        };
        const handCenterR = {
            x: wristCenterR.x + 0.12 * Math.sin(totalElbowR),
            y: wristCenterR.y - 0.12 * Math.cos(totalElbowR),
            z: wristCenterR.z
        };

        // 6. Left Arm Spatial Kinematics
        const shoulderCenterL = { x: 0.0, y: 0.42, z: -zOffsetArm };
        const elbowCenterL = {
            x: shoulderCenterL.x + humerusLen * Math.sin(qShoulderL),
            y: shoulderCenterL.y - humerusLen * Math.cos(qShoulderL),
            z: shoulderCenterL.z
        };
        const totalElbowL = qShoulderL - qElbowL;
        const wristCenterL = {
            x: elbowCenterL.x + forearmLen * Math.sin(totalElbowL),
            y: elbowCenterL.y - forearmLen * Math.cos(totalElbowL),
            z: elbowCenterL.z
        };
        const olecranonL = {
            x: elbowCenterL.x - 0.025 * Math.sin(totalElbowL),
            y: elbowCenterL.y + 0.025 * Math.cos(totalElbowL),
            z: elbowCenterL.z
        };
        const handCenterL = {
            x: wristCenterL.x + 0.12 * Math.sin(totalElbowL),
            y: wristCenterL.y - 0.12 * Math.cos(totalElbowL),
            z: wristCenterL.z
        };

        return {
            pelvisCenter,
            thoraxCenter,
            neckBase,
            neckTop,
            skullCenter,
            chin,

            // Right Leg
            hipCenterR, kneeCenterR, ankleCenterR, patellaCenterR, calcaneusR, toeCenterR,
            // Left Leg
            hipCenterL, kneeCenterL, ankleCenterL, patellaCenterL, calcaneusL, toeCenterL,

            // Right Arm
            shoulderCenterR, elbowCenterR, wristCenterR, olecranonR, handCenterR,
            // Left Arm
            shoulderCenterL, elbowCenterL, wristCenterL, olecranonL, handCenterL,

            // Legacy fallbacks for single limb modes
            hipCenter: hipCenterR,
            kneeCenter: kneeCenterR,
            ankleCenter: ankleCenterR,
            patellaCenter: patellaCenterR,
            calcaneus: calcaneusR,
            toeCenter: toeCenterR,
            shoulderCenter: shoulderCenterR,
            elbowCenter: elbowCenterR,
            wristCenter: wristCenterR,
            olecranon: olecranonR,

            anglesRad: {
                qNeckFlex, qNeckRot, qJaw,
                qShoulderR, qElbowR, qProSupR,
                qShoulderL, qElbowL,
                qHipFlexR, qHipAddR, qKneeR, qAnkleR,
                qHipFlexL, qKneeL, qAnkleL,
                // fallbacks
                qHipFlex: qHipFlexR, qHipAdd: qHipAddR, qKnee: qKneeR, qAnkle: qAnkleR,
                qShoulder: qShoulderR, qElbow: qElbowR, qProSup: qProSupR
            }
        };
    }

    /**
     * Compute 3D muscle pathways based on kinematics
     */
    computeMusclePaths(kin) {
        const p = kin;
        const paths = {};

        // --- Neck & Head MTUs ---
        paths["scm_r"] = [
            { x: p.thoraxCenter.x + 0.03, y: p.neckBase.y - 0.01, z: 0.04 },
            { x: p.skullCenter.x - 0.02, y: p.skullCenter.y - 0.03, z: p.skullCenter.z + 0.055 }
        ];
        paths["scm_l"] = [
            { x: p.thoraxCenter.x + 0.03, y: p.neckBase.y - 0.01, z: -0.04 },
            { x: p.skullCenter.x - 0.02, y: p.skullCenter.y - 0.03, z: p.skullCenter.z - 0.055 }
        ];
        paths["splenius"] = [
            { x: p.thoraxCenter.x - 0.04, y: p.neckBase.y - 0.06, z: 0.0 },
            { x: p.skullCenter.x - 0.05, y: p.skullCenter.y + 0.03, z: 0.0 }
        ];
        paths["masseter"] = [
            { x: p.skullCenter.x + 0.04, y: p.skullCenter.y - 0.02, z: p.skullCenter.z + 0.045 },
            { x: p.chin.x - 0.02, y: p.chin.y + 0.01, z: p.skullCenter.z + 0.045 }
        ];

        // --- Arm MTUs (Whole Body / Upper Body sets) ---
        // Right Arm
        paths["biceps_r"] = [
            { x: p.shoulderCenterR.x + 0.02, y: p.shoulderCenterR.y + 0.02, z: p.shoulderCenterR.z + 0.01 },
            { x: 0.5 * (p.shoulderCenterR.x + p.elbowCenterR.x) + 0.02, y: 0.5 * (p.shoulderCenterR.y + p.elbowCenterR.y), z: p.shoulderCenterR.z + 0.01 },
            { x: p.elbowCenterR.x + 0.02, y: p.elbowCenterR.y - 0.03, z: p.elbowCenterR.z }
        ];
        paths["brachialis_r"] = [
            { x: 0.5 * (p.shoulderCenterR.x + p.elbowCenterR.x) + 0.015, y: 0.5 * (p.shoulderCenterR.y + p.elbowCenterR.y) - 0.03, z: p.shoulderCenterR.z },
            { x: p.elbowCenterR.x + 0.015, y: p.elbowCenterR.y - 0.025, z: p.elbowCenterR.z }
        ];
        paths["triceps_r"] = [
            { x: p.shoulderCenterR.x - 0.02, y: p.shoulderCenterR.y - 0.02, z: p.shoulderCenterR.z },
            { x: 0.5 * (p.shoulderCenterR.x + p.elbowCenterR.x) - 0.02, y: 0.5 * (p.shoulderCenterR.y + p.elbowCenterR.y), z: p.shoulderCenterR.z },
            { x: p.olecranonR.x, y: p.olecranonR.y, z: p.olecranonR.z }
        ];

        // Left Arm
        paths["biceps_l"] = [
            { x: p.shoulderCenterL.x + 0.02, y: p.shoulderCenterL.y + 0.02, z: p.shoulderCenterL.z - 0.01 },
            { x: 0.5 * (p.shoulderCenterL.x + p.elbowCenterL.x) + 0.02, y: 0.5 * (p.shoulderCenterL.y + p.elbowCenterL.y), z: p.shoulderCenterL.z - 0.01 },
            { x: p.elbowCenterL.x + 0.02, y: p.elbowCenterL.y - 0.03, z: p.elbowCenterL.z }
        ];
        paths["brachialis_l"] = [
            { x: 0.5 * (p.shoulderCenterL.x + p.elbowCenterL.x) + 0.015, y: 0.5 * (p.shoulderCenterL.y + p.elbowCenterL.y) - 0.03, z: p.shoulderCenterL.z },
            { x: p.elbowCenterL.x + 0.015, y: p.elbowCenterL.y - 0.025, z: p.elbowCenterL.z }
        ];
        paths["triceps_l"] = [
            { x: p.shoulderCenterL.x - 0.02, y: p.shoulderCenterL.y - 0.02, z: p.shoulderCenterL.z },
            { x: 0.5 * (p.shoulderCenterL.x + p.elbowCenterL.x) - 0.02, y: 0.5 * (p.shoulderCenterL.y + p.elbowCenterL.y), z: p.shoulderCenterL.z },
            { x: p.olecranonL.x, y: p.olecranonL.y, z: p.olecranonL.z }
        ];

        // --- Single Arm Detailed 6 MTUs ---
        const sArm = (this.scope === "left_arm") ? {
            shoulder: p.shoulderCenterL, elbow: p.elbowCenterL, olecranon: p.olecranonL, zSign: -1
        } : {
            shoulder: p.shoulderCenterR, elbow: p.elbowCenterR, olecranon: p.olecranonR, zSign: 1
        };
        paths["biceps_lh"] = [
            { x: sArm.shoulder.x + 0.015, y: sArm.shoulder.y + 0.03, z: sArm.shoulder.z + 0.02 * sArm.zSign },
            { x: 0.5 * (sArm.shoulder.x + sArm.elbow.x) + 0.025, y: 0.5 * (sArm.shoulder.y + sArm.elbow.y), z: sArm.shoulder.z + 0.02 * sArm.zSign },
            { x: sArm.elbow.x + 0.02, y: sArm.elbow.y + 0.01, z: sArm.elbow.z },
            { x: sArm.elbow.x + 0.025, y: sArm.elbow.y - 0.035, z: sArm.elbow.z }
        ];
        paths["biceps_sh"] = [
            { x: sArm.shoulder.x + 0.03, y: sArm.shoulder.y + 0.015, z: sArm.shoulder.z - 0.015 * sArm.zSign },
            { x: 0.5 * (sArm.shoulder.x + sArm.elbow.x) + 0.02, y: 0.5 * (sArm.shoulder.y + sArm.elbow.y) - 0.02, z: sArm.shoulder.z - 0.01 * sArm.zSign },
            { x: sArm.elbow.x + 0.025, y: sArm.elbow.y - 0.035, z: sArm.elbow.z }
        ];
        paths["brachialis"] = [
            { x: 0.5 * (sArm.shoulder.x + sArm.elbow.x) + 0.01, y: 0.5 * (sArm.shoulder.y + sArm.elbow.y) - 0.04, z: sArm.shoulder.z },
            { x: sArm.elbow.x + 0.02, y: sArm.elbow.y, z: sArm.elbow.z },
            { x: sArm.elbow.x + 0.015, y: sArm.elbow.y - 0.03, z: sArm.elbow.z }
        ];
        paths["triceps_lh"] = [
            { x: sArm.shoulder.x - 0.02, y: sArm.shoulder.y - 0.015, z: sArm.shoulder.z },
            { x: 0.5 * (sArm.shoulder.x + sArm.elbow.x) - 0.025, y: 0.5 * (sArm.shoulder.y + sArm.elbow.y), z: sArm.shoulder.z },
            { x: sArm.olecranon.x, y: sArm.olecranon.y, z: sArm.olecranon.z }
        ];
        paths["triceps_lat"] = [
            { x: sArm.shoulder.x - 0.01, y: sArm.shoulder.y - 0.06, z: sArm.shoulder.z + 0.02 * sArm.zSign },
            { x: 0.5 * (sArm.shoulder.x + sArm.elbow.x) - 0.02, y: 0.5 * (sArm.shoulder.y + sArm.elbow.y) - 0.03, z: sArm.shoulder.z + 0.015 * sArm.zSign },
            { x: sArm.olecranon.x, y: sArm.olecranon.y, z: sArm.olecranon.z }
        ];
        paths["triceps_med"] = [
            { x: 0.5 * (sArm.shoulder.x + sArm.elbow.x) - 0.01, y: 0.5 * (sArm.shoulder.y + sArm.elbow.y) - 0.07, z: sArm.shoulder.z - 0.015 * sArm.zSign },
            { x: sArm.olecranon.x, y: sArm.olecranon.y, z: sArm.olecranon.z }
        ];

        // --- Leg MTUs (Whole Body / Lower Body Bilateral sets) ---
        // Right Leg
        paths["rect_fem_r"] = [
            { x: p.hipCenterR.x + 0.03, y: p.hipCenterR.y + 0.08, z: p.hipCenterR.z + 0.03 },
            { x: p.patellaCenterR.x + 0.005, y: p.patellaCenterR.y + 0.015, z: p.patellaCenterR.z },
            { x: p.kneeCenterR.x + 0.025, y: p.kneeCenterR.y - 0.055, z: p.kneeCenterR.z + 0.01 }
        ];
        paths["vasti_r"] = [
            { x: 0.5 * (p.hipCenterR.x + p.kneeCenterR.x) + 0.02, y: 0.5 * (p.hipCenterR.y + p.kneeCenterR.y) + 0.04, z: p.hipCenterR.z + 0.02 },
            { x: p.patellaCenterR.x + 0.01, y: p.patellaCenterR.y, z: p.patellaCenterR.z },
            { x: p.kneeCenterR.x + 0.025, y: p.kneeCenterR.y - 0.055, z: p.kneeCenterR.z }
        ];
        paths["bifemlh_r"] = [
            { x: p.hipCenterR.x - 0.05, y: p.hipCenterR.y - 0.04, z: p.hipCenterR.z },
            { x: p.kneeCenterR.x - 0.03, y: p.kneeCenterR.y - 0.02, z: p.kneeCenterR.z + 0.03 },
            { x: p.kneeCenterR.x - 0.02, y: p.kneeCenterR.y - 0.07, z: p.kneeCenterR.z + 0.035 }
        ];
        paths["glut_max_r"] = [
            { x: p.hipCenterR.x - 0.08, y: p.hipCenterR.y + 0.06, z: p.hipCenterR.z + 0.02 },
            { x: p.hipCenterR.x + 0.01, y: p.hipCenterR.y - 0.08, z: p.hipCenterR.z + 0.04 }
        ];
        paths["gas_med_r"] = [
            { x: p.kneeCenterR.x - 0.025, y: p.kneeCenterR.y + 0.025, z: p.kneeCenterR.z - 0.015 },
            { x: p.ankleCenterR.x - 0.025, y: p.ankleCenterR.y, z: p.ankleCenterR.z },
            { x: p.calcaneusR.x, y: p.calcaneusR.y, z: p.calcaneusR.z }
        ];
        paths["tib_ant_r"] = [
            { x: p.kneeCenterR.x + 0.025, y: p.kneeCenterR.y - 0.08, z: p.kneeCenterR.z + 0.015 },
            { x: p.ankleCenterR.x + 0.03, y: p.ankleCenterR.y + 0.01, z: p.ankleCenterR.z },
            { x: p.toeCenterR.x - 0.04, y: p.toeCenterR.y, z: p.toeCenterR.z }
        ];

        // Left Leg
        paths["rect_fem_l"] = [
            { x: p.hipCenterL.x + 0.03, y: p.hipCenterL.y + 0.08, z: p.hipCenterL.z - 0.03 },
            { x: p.patellaCenterL.x + 0.005, y: p.patellaCenterL.y + 0.015, z: p.patellaCenterL.z },
            { x: p.kneeCenterL.x + 0.025, y: p.kneeCenterL.y - 0.055, z: p.kneeCenterL.z - 0.01 }
        ];
        paths["vasti_l"] = [
            { x: 0.5 * (p.hipCenterL.x + p.kneeCenterL.x) + 0.02, y: 0.5 * (p.hipCenterL.y + p.kneeCenterL.y) + 0.04, z: p.hipCenterL.z - 0.02 },
            { x: p.patellaCenterL.x + 0.01, y: p.patellaCenterL.y, z: p.patellaCenterL.z },
            { x: p.kneeCenterL.x + 0.025, y: p.kneeCenterL.y - 0.055, z: p.kneeCenterL.z }
        ];
        paths["bifemlh_l"] = [
            { x: p.hipCenterL.x - 0.05, y: p.hipCenterL.y - 0.04, z: p.hipCenterL.z },
            { x: p.kneeCenterL.x - 0.03, y: p.kneeCenterL.y - 0.02, z: p.kneeCenterL.z - 0.03 },
            { x: p.kneeCenterL.x - 0.02, y: p.kneeCenterL.y - 0.07, z: p.kneeCenterL.z - 0.035 }
        ];
        paths["glut_max_l"] = [
            { x: p.hipCenterL.x - 0.08, y: p.hipCenterL.y + 0.06, z: p.hipCenterL.z - 0.02 },
            { x: p.hipCenterL.x + 0.01, y: p.hipCenterL.y - 0.08, z: p.hipCenterL.z - 0.04 }
        ];
        paths["gas_med_l"] = [
            { x: p.kneeCenterL.x - 0.025, y: p.kneeCenterL.y + 0.025, z: p.kneeCenterL.z + 0.015 },
            { x: p.ankleCenterL.x - 0.025, y: p.ankleCenterL.y, z: p.ankleCenterL.z },
            { x: p.calcaneusL.x, y: p.calcaneusL.y, z: p.calcaneusL.z }
        ];
        paths["tib_ant_l"] = [
            { x: p.kneeCenterL.x + 0.025, y: p.kneeCenterL.y - 0.08, z: p.kneeCenterL.z - 0.015 },
            { x: p.ankleCenterL.x + 0.03, y: p.ankleCenterL.y + 0.01, z: p.ankleCenterL.z },
            { x: p.toeCenterL.x - 0.04, y: p.toeCenterL.y, z: p.toeCenterL.z }
        ];

        // --- Single Leg Detailed 10 MTUs ---
        const sLeg = (this.scope === "left_leg") ? {
            hip: p.hipCenterL, knee: p.kneeCenterL, ankle: p.ankleCenterL, patella: p.patellaCenterL, calc: p.calcaneusL, toe: p.toeCenterL, zSign: -1
        } : {
            hip: p.hipCenterR, knee: p.kneeCenterR, ankle: p.ankleCenterR, patella: p.patellaCenterR, calc: p.calcaneusR, toe: p.toeCenterR, zSign: 1
        };
        paths["rect_fem"] = [
            { x: sLeg.hip.x + 0.03, y: sLeg.hip.y + 0.08, z: sLeg.hip.z + 0.03 * sLeg.zSign },
            { x: sLeg.patella.x + 0.005, y: sLeg.patella.y + 0.015, z: sLeg.patella.z },
            { x: sLeg.patella.x + 0.012, y: sLeg.patella.y - 0.015, z: sLeg.patella.z },
            { x: sLeg.knee.x + 0.025, y: sLeg.knee.y - 0.055, z: sLeg.knee.z + 0.01 * sLeg.zSign }
        ];
        paths["vas_int"] = [
            { x: 0.5 * (sLeg.hip.x + sLeg.knee.x) + 0.018, y: 0.5 * (sLeg.hip.y + sLeg.knee.y) + 0.05, z: sLeg.hip.z },
            { x: sLeg.patella.x + 0.008, y: sLeg.patella.y + 0.01, z: sLeg.patella.z },
            { x: sLeg.knee.x + 0.025, y: sLeg.knee.y - 0.055, z: sLeg.knee.z }
        ];
        paths["vas_lat"] = [
            { x: 0.5 * (sLeg.hip.x + sLeg.knee.x) + 0.01, y: 0.5 * (sLeg.hip.y + sLeg.knee.y) + 0.08, z: sLeg.hip.z + 0.035 * sLeg.zSign },
            { x: sLeg.patella.x + 0.01, y: sLeg.patella.y + 0.01, z: sLeg.patella.z + 0.015 * sLeg.zSign },
            { x: sLeg.knee.x + 0.025, y: sLeg.knee.y - 0.055, z: sLeg.knee.z + 0.01 * sLeg.zSign }
        ];
        paths["vas_med"] = [
            { x: 0.5 * (sLeg.hip.x + sLeg.knee.x) + 0.01, y: 0.5 * (sLeg.hip.y + sLeg.knee.y) + 0.03, z: sLeg.hip.z - 0.032 * sLeg.zSign },
            { x: sLeg.patella.x + 0.01, y: sLeg.patella.y - 0.005, z: sLeg.patella.z - 0.015 * sLeg.zSign },
            { x: sLeg.knee.x + 0.025, y: sLeg.knee.y - 0.055, z: sLeg.knee.z }
        ];
        paths["bifemlh"] = [
            { x: sLeg.hip.x - 0.05, y: sLeg.hip.y - 0.04, z: sLeg.hip.z },
            { x: sLeg.knee.x - 0.03, y: sLeg.knee.y - 0.02, z: sLeg.knee.z + 0.035 * sLeg.zSign },
            { x: sLeg.knee.x - 0.02, y: sLeg.knee.y - 0.07, z: sLeg.knee.z + 0.04 * sLeg.zSign }
        ];
        paths["semiten"] = [
            { x: sLeg.hip.x - 0.05, y: sLeg.hip.y - 0.04, z: sLeg.hip.z },
            { x: sLeg.knee.x - 0.025, y: sLeg.knee.y - 0.02, z: sLeg.knee.z - 0.03 * sLeg.zSign },
            { x: sLeg.knee.x + 0.01, y: sLeg.knee.y - 0.08, z: sLeg.knee.z - 0.025 * sLeg.zSign }
        ];
        paths["iliopsoas"] = [
            { x: sLeg.hip.x + 0.02, y: sLeg.hip.y + 0.12, z: sLeg.hip.z - 0.02 * sLeg.zSign },
            { x: sLeg.hip.x + 0.03, y: sLeg.hip.y + 0.02, z: sLeg.hip.z - 0.01 * sLeg.zSign },
            { x: sLeg.hip.x - 0.01, y: sLeg.hip.y - 0.07, z: sLeg.hip.z - 0.025 * sLeg.zSign }
        ];
        paths["glut_max"] = [
            { x: sLeg.hip.x - 0.08, y: sLeg.hip.y + 0.06, z: sLeg.hip.z + 0.02 * sLeg.zSign },
            { x: sLeg.hip.x - 0.06, y: sLeg.hip.y - 0.02, z: sLeg.hip.z + 0.045 * sLeg.zSign },
            { x: sLeg.hip.x + 0.01, y: sLeg.hip.y - 0.09, z: sLeg.hip.z + 0.03 * sLeg.zSign }
        ];
        paths["gas_med"] = [
            { x: sLeg.knee.x - 0.025, y: sLeg.knee.y + 0.025, z: sLeg.knee.z - 0.02 * sLeg.zSign },
            { x: sLeg.ankle.x - 0.025, y: sLeg.ankle.y + 0.04, z: sLeg.ankle.z },
            { x: sLeg.calc.x, y: sLeg.calc.y, z: sLeg.calc.z }
        ];
        paths["soleus"] = [
            { x: sLeg.knee.x - 0.015, y: sLeg.knee.y - 0.08, z: sLeg.knee.z },
            { x: sLeg.ankle.x - 0.025, y: sLeg.ankle.y + 0.03, z: sLeg.ankle.z },
            { x: sLeg.calc.x, y: sLeg.calc.y, z: sLeg.calc.z }
        ];
        paths["tib_ant"] = [
            { x: sLeg.knee.x + 0.025, y: sLeg.knee.y - 0.08, z: sLeg.knee.z + 0.015 * sLeg.zSign },
            { x: sLeg.ankle.x + 0.03, y: sLeg.ankle.y + 0.01, z: sLeg.ankle.z },
            { x: sLeg.toe.x - 0.04, y: sLeg.toe.y, z: sLeg.toe.z }
        ];

        return paths;
    }

    computePathLength(pointArray) {
        if (!pointArray || pointArray.length < 2) return 0.05;
        let len = 0;
        for (let i = 0; i < pointArray.length - 1; i++) {
            const dx = pointArray[i + 1].x - pointArray[i].x;
            const dy = pointArray[i + 1].y - pointArray[i].y;
            const dz = pointArray[i + 1].z - pointArray[i].z;
            len += Math.sqrt(dx * dx + dy * dy + dz * dz);
        }
        return len;
    }

    computeMomentArms(coordName, deltaDeg = 0.5) {
        if (!coordName || this.coordinates[coordName] === undefined) {
            coordName = Object.keys(this.coordLimits)[0];
        }
        const currentCoord = this.coordinates[coordName];
        const radDelta = (deltaDeg * Math.PI) / 180.0;

        const coordsPlus = { ...this.coordinates, [coordName]: currentCoord + deltaDeg };
        const kinPlus = this.computeKinematics(coordsPlus);
        const pathsPlus = this.computeMusclePaths(kinPlus);

        const coordsMinus = { ...this.coordinates, [coordName]: currentCoord - deltaDeg };
        const kinMinus = this.computeKinematics(coordsMinus);
        const pathsMinus = this.computeMusclePaths(kinMinus);

        const momentArms = {};
        for (const muscle of this.muscles) {
            if (pathsPlus[muscle.name] && pathsMinus[muscle.name]) {
                const lenPlus = this.computePathLength(pathsPlus[muscle.name]);
                const lenMinus = this.computePathLength(pathsMinus[muscle.name]);
                const r_rad = -(lenPlus - lenMinus) / (2.0 * radDelta);
                momentArms[muscle.name] = r_rad;
            } else {
                momentArms[muscle.name] = 0.0;
            }
        }
        return momentArms;
    }

    update() {
        const kin = this.computeKinematics(this.coordinates);
        const paths = this.computeMusclePaths(kin);
        const results = {};

        for (const muscle of this.muscles) {
            const path = paths[muscle.name];
            if (path) {
                const lmt = this.computePathLength(path);
                const state = muscle.solveEquilibrium(lmt, muscle.activation);
                results[muscle.name] = { state, path };
            }
        }

        const defaultJoint = Object.keys(this.coordLimits)[0];
        const momentArms = this.computeMomentArms(defaultJoint);

        return {
            kinematics: kin,
            muscles: results,
            momentArms: {
                [defaultJoint]: momentArms
            }
        };
    }
}

/**
 * OpenSim Interoperability Engine
 * Exports and Imports OpenSim .osim XML models and .mot motion trajectory data
 */
class OpenSimIO {
    /**
     * Export Musculoskeletal System to standard OpenSim 4.x .osim XML format
     */
    static exportToOsimXml(system, modelName = "MoSim_Musculoskeletal_Model") {

        let xml = `<?xml version="1.0" encoding="UTF-8" ?>\n`;
        xml += `<OpenSimDocument Version="40000">\n`;
        xml += `  <Model name="${modelName}">\n`;
        xml += `    <credits>Built with MoSim Next-Generation Web Musculoskeletal Simulator</credits>\n`;
        xml += `    <publications>Rajagopal et al. 2016 / Holzbaur et al. 2005</publications>\n`;
        xml += `    <length_units>meters</length_units>\n`;
        xml += `    <force_units>N</force_units>\n\n`;

        // CoordinateSet
        xml += `    <CoordinateSet>\n      <objects>\n`;
        for (const [coordName, cfg] of Object.entries(system.coordLimits)) {
            xml += `        <Coordinate name="${coordName}">\n`;
            xml += `          <default_value>${((cfg.default * Math.PI) / 180).toFixed(4)}</default_value>\n`;
            xml += `          <range>${((cfg.min * Math.PI) / 180).toFixed(4)} ${((cfg.max * Math.PI) / 180).toFixed(4)}</range>\n`;
            xml += `          <clamped>true</clamped>\n`;
            xml += `          <locked>false</locked>\n`;
            xml += `        </Coordinate>\n`;
        }
        xml += `      </objects>\n    </CoordinateSet>\n\n`;

        // ForceSet / Millard2012EquilibriumMuscle
        xml += `    <ForceSet>\n      <objects>\n`;
        for (const m of system.muscles) {
            xml += `        <Millard2012EquilibriumMuscle name="${m.name}">\n`;
            xml += `          <max_isometric_force>${m.maxIsometricForce.toFixed(1)}</max_isometric_force>\n`;
            xml += `          <optimal_fiber_length>${m.optimalFiberLength.toFixed(4)}</optimal_fiber_length>\n`;
            xml += `          <tendon_slack_length>${m.tendonSlackLength.toFixed(4)}</tendon_slack_length>\n`;
            xml += `          <pennation_angle_at_optimal>${m.pennationAngleAtOptimal.toFixed(4)}</pennation_angle_at_optimal>\n`;
            xml += `          <max_contraction_velocity>${m.maxContractionVelocity.toFixed(1)}</max_contraction_velocity>\n`;
            xml += `        </Millard2012EquilibriumMuscle>\n`;
        }
        xml += `      </objects>\n    </ForceSet>\n`;
        xml += `  </Model>\n`;
        xml += `</OpenSimDocument>\n`;

        return xml;
    }

    /**
     * Export time series kinematic motion data to OpenSim .mot format
     */
    static exportMotionFile(timeArray, coordNames, rowData) {
        let mot = `Coordinates\n`;
        mot += `version=1\n`;
        mot += `nRows=${rowData.length}\n`;
        mot += `nColumns=${coordNames.length + 1}\n`;
        mot += `inDegrees=yes\n\n`;
        mot += `Units are in degrees.\n\n`;
        mot += `endheader\n`;
        mot += `time\t${coordNames.join("\t")}\n`;

        for (let i = 0; i < rowData.length; i++) {
            const t = (timeArray[i] !== undefined ? timeArray[i] : i * 0.02).toFixed(4);
            const vals = coordNames.map(name => (rowData[i][name] || 0).toFixed(3)).join("\t");
            mot += `${t}\t${vals}\n`;
        }

        return mot;
    }
}

/**
 * Forward Dynamics Numerical Integrator (Runge-Kutta 4th Order)
 * Simulates joint rotational dynamics: I * ddq = tau_muscle - tau_gravity - b * dq
 */
class ForwardDynamicsEngine {
    constructor(inertia = 0.35, damping = 0.15, mass = 4.2, comDist = 0.22) {
        this.I = inertia; // kg*m^2
        this.b = damping; // N*m*s/rad
        this.m = mass; // kg
        this.d = comDist; // m
        this.g = 9.81; // m/s^2

        this.q = 0.0; // rad
        this.dq = 0.0; // rad/s
    }

    reset(initialAngleDeg = 0) {
        this.q = (initialAngleDeg * Math.PI) / 180.0;
        this.dq = 0.0;
    }

    calcAcceleration(q, dq, muscleTorque) {
        // Gravity torque opposing flexion
        const tauGrav = this.m * this.g * this.d * Math.sin(q);
        const tauDamp = this.b * dq;
        const ddq = (muscleTorque - tauGrav - tauDamp) / this.I;
        return ddq;
    }

    stepRK4(muscleTorque, dt) {
        const q0 = this.q;
        const dq0 = this.dq;

        // k1
        const k1_dq = dq0;
        const k1_ddq = this.calcAcceleration(q0, dq0, muscleTorque);

        // k2
        const q_mid1 = q0 + 0.5 * dt * k1_dq;
        const dq_mid1 = dq0 + 0.5 * dt * k1_ddq;
        const k2_dq = dq_mid1;
        const k2_ddq = this.calcAcceleration(q_mid1, dq_mid1, muscleTorque);

        // k3
        const q_mid2 = q0 + 0.5 * dt * k2_dq;
        const dq_mid2 = dq0 + 0.5 * dt * k2_ddq;
        const k3_dq = dq_mid2;
        const k3_ddq = this.calcAcceleration(q_mid2, dq_mid2, muscleTorque);

        // k4
        const q_end = q0 + dt * k3_dq;
        const dq_end = dq0 + dt * k3_ddq;
        const k4_dq = dq_end;
        const k4_ddq = this.calcAcceleration(q_end, dq_end, muscleTorque);

        // Update state
        this.q += (dt / 6.0) * (k1_dq + 2 * k2_dq + 2 * k3_dq + k4_dq);
        this.dq += (dt / 6.0) * (k1_ddq + 2 * k2_ddq + 2 * k3_ddq + k4_ddq);

        // Joint anatomical limits clamp [0, 140] deg
        const minQ = 0.0;
        const maxQ = (140.0 * Math.PI) / 180.0;
        if (this.q <= minQ) {
            this.q = minQ;
            if (this.dq < 0) this.dq = -0.2 * this.dq; // slight bounce
        } else if (this.q >= maxQ) {
            this.q = maxQ;
            if (this.dq > 0) this.dq = -0.2 * this.dq;
        }

        return (this.q * 180.0) / Math.PI; // degrees
    }
}

// Export for browser window and Node modules
if (typeof window !== "undefined") {
    window.HillMuscleModel = HillMuscleModel;
    window.MusculoskeletalSystem = MusculoskeletalSystem;
    window.UpperLimbSystem = UpperLimbSystem;
    window.WholeBodySystem = WholeBodySystem;
    window.OpenSimIO = OpenSimIO;
    window.ForwardDynamicsEngine = ForwardDynamicsEngine;
}
if (typeof module !== "undefined" && module.exports) {
    module.exports = {
        HillMuscleModel,
        MusculoskeletalSystem,
        UpperLimbSystem,
        WholeBodySystem,
        OpenSimIO,
        ForwardDynamicsEngine
    };
}

