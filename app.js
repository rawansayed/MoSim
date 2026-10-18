/**
 * MoSim - Next-Generation Virtual Musculoskeletal Simulator
 * Main Application Controller & Three.js 3D Viewport Engine
 * Features:
 * 1. Multi-Model Architecture (Lower Extremity Rajagopal 2016 & Upper Limb Holzbaur 2005)
 * 2. Real-Time 3D WebGL PBR Viewport with Catmull-Rom Dynamic Muscle Bulking Tubes
 * 3. 3-Element Hill Musculotendon Model & Live Biomechanical Curve Tracking
 * 4. OpenSim 4.x Interoperability (.osim XML Exporter/Importer & .mot Motion Exporter)
 * 5. Runge-Kutta 4 Forward Dynamics Physics Engine
 * 6. Gait Cycle & Kinematic Replay
 * 7. High-DPI Snapshot Camera Capture
 */

class MoSimApp {
    constructor() {
        this.currentModelType = "lower_limb"; // "lower_limb" or "upper_limb"
        this.system = new MusculoskeletalSystem();
        this.plotter = null;
        this.selectedMuscle = this.system.muscles[0];

        // Simulation & Motion state
        this.isPlaying = false;
        this.isPhysicsMode = false;
        this.simTime = 0.0;
        this.gaitSpeed = 1.0;
        this.gaitProgress = 0.0;

        // Forward Dynamics Engine
        this.physicsEngine = new ForwardDynamicsEngine();

        // Three.js instances
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.boneMeshes = {};
        this.muscleMeshes = {};
        this.jointMarkers = {};

        // Recorded Motion Trajectory for OpenSim .mot export
        this.recordedTrajectory = [];

        // Lower Limb Gait Kinematics (Rajagopal et al. 2016)
        this.gaitDataLower = [
            { t: 0.00, hip_flexion: 28.0, knee_angle: 5.0,  ankle_angle: 0.0 },
            { t: 0.15, hip_flexion: 20.0, knee_angle: 18.0, ankle_angle: -5.0 },
            { t: 0.30, hip_flexion: 5.0,  knee_angle: 10.0, ankle_angle: 8.0 },
            { t: 0.50, hip_flexion: -10.0, knee_angle: 8.0, ankle_angle: 12.0 },
            { t: 0.60, hip_flexion: -8.0,  knee_angle: 35.0, ankle_angle: -15.0 },
            { t: 0.75, hip_flexion: 15.0, knee_angle: 60.0, ankle_angle: -5.0 },
            { t: 0.85, hip_flexion: 25.0, knee_angle: 30.0, ankle_angle: 0.0 },
            { t: 1.00, hip_flexion: 28.0, knee_angle: 5.0,  ankle_angle: 0.0 }
        ];

        // Upper Limb Arm Curl / Flexion Cycle
        this.gaitDataUpper = [
            { t: 0.00, shoulder_flexion: 0.0,  elbow_flexion: 10.0,  pro_sup: 0.0 },
            { t: 0.25, shoulder_flexion: 15.0, elbow_flexion: 55.0,  pro_sup: 20.0 },
            { t: 0.50, shoulder_flexion: 25.0, elbow_flexion: 115.0, pro_sup: 45.0 },
            { t: 0.75, shoulder_flexion: 15.0, elbow_flexion: 65.0,  pro_sup: 20.0 },
            { t: 1.00, shoulder_flexion: 0.0,  elbow_flexion: 10.0,  pro_sup: 0.0 }
        ];

        this.init();
    }

    init() {
        this.initThreeJS();
        this.buildCurrentModel();
        this.initUI();
        this.initPlotter();
        this.updateSimulation();
        this.animate();
    }

    /**
     * Initialize Three.js WebGL Scene with PBR Shading and Lighting
     */
    initThreeJS() {
        const container = document.getElementById("canvas-container");
        const width = container.clientWidth;
        const height = container.clientHeight;

        // Scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0a0f1d);
        this.scene.fog = new THREE.FogExp2(0x0a0f1d, 0.22);

        // Camera
        this.camera = new THREE.PerspectiveCamera(45, width / height, 0.05, 50);
        this.camera.position.set(1.4, 0.1, 1.6);

        // Renderer with High Dynamic Range & Shadows
        this.renderer = new THREE.WebGLRenderer({
            antialias: true,
            preserveDrawingBuffer: true, // Allows snapshot PNG export
            powerPreference: "high-performance"
        });
        this.renderer.setSize(width, height);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 1.15;
        container.appendChild(this.renderer.domElement);

        // OrbitControls
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.08;
        this.controls.target.set(0.0, -0.4, 0.0);
        this.controls.maxPolarAngle = Math.PI / 2 + 0.05;

        // Studio Lighting
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.85);
        this.scene.add(ambientLight);

        const keyLight = new THREE.DirectionalLight(0xffffff, 1.4);
        keyLight.position.set(3, 4, 3);
        keyLight.castShadow = true;
        keyLight.shadow.mapSize.width = 1024;
        keyLight.shadow.mapSize.height = 1024;
        this.scene.add(keyLight);

        const rimLight = new THREE.DirectionalLight(0x38bdf8, 0.95);
        rimLight.position.set(-3, 2, -3);
        this.scene.add(rimLight);

        const fillLight = new THREE.DirectionalLight(0xf43f5e, 0.35);
        fillLight.position.set(0, -2, 2);
        this.scene.add(fillLight);

        // Biomechanical Ground Plane
        const gridHelper = new THREE.GridHelper(3.0, 30, 0x38bdf8, 0x1e293b);
        gridHelper.position.y = -0.98;
        this.scene.add(gridHelper);

        const shadowGeo = new THREE.PlaneGeometry(3.0, 3.0);
        const shadowMat = new THREE.ShadowMaterial({ opacity: 0.35 });
        const groundShadow = new THREE.Mesh(shadowGeo, shadowMat);
        groundShadow.rotation.x = -Math.PI / 2;
        groundShadow.position.y = -0.981;
        groundShadow.receiveShadow = true;
        this.scene.add(groundShadow);

        // Resize handler
        window.addEventListener("resize", () => {
            const w = container.clientWidth;
            const h = container.clientHeight;
            this.camera.aspect = w / h;
            this.camera.updateProjectionMatrix();
            this.renderer.setSize(w, h);
            if (this.plotter) this.plotter.resize();
        });
    }

    /**
     * Clear existing 3D meshes when switching models
     */
    clearModelMeshes() {
        for (const [key, mesh] of Object.entries(this.boneMeshes)) {
            this.scene.remove(mesh);
        }
        for (const [key, mesh] of Object.entries(this.muscleMeshes)) {
            this.scene.remove(mesh);
        }
        for (const [key, marker] of Object.entries(this.jointMarkers)) {
            this.scene.remove(marker);
        }
        this.boneMeshes = {};
        this.muscleMeshes = {};
        this.jointMarkers = {};
    }

    /**
     * Build the currently active musculoskeletal model (Lower Extremity or Upper Limb)
     */
    buildCurrentModel() {
        this.clearModelMeshes();

        if (this.currentModelType === "lower_limb") {
            this.system = new MusculoskeletalSystem();
            this.createLowerLimbBones();
            this.controls.target.set(0.0, -0.4, 0.0);
            this.camera.position.set(1.4, 0.1, 1.6);
        } else {
            this.system = new UpperLimbSystem();
            this.createUpperLimbBones();
            this.controls.target.set(0.0, -0.2, 0.0);
            this.camera.position.set(1.2, 0.1, 1.2);
        }

        this.controls.update();
        this.selectedMuscle = this.system.muscles[0];
        this.createMuscleTubes();
        this.physicsEngine.reset(this.system.coordinates[Object.keys(this.system.coordinates)[0]]);
    }

    /**
     * Create Anatomical Lower Limb Bones (Pelvis, Femur, Patella, Tibia, Foot)
     */
    createLowerLimbBones() {
        const boneMaterial = new THREE.MeshStandardMaterial({
            color: 0xf1f5f9,
            roughness: 0.35,
            metalness: 0.05
        });
        const jointMaterial = new THREE.MeshStandardMaterial({
            color: 0x38bdf8,
            roughness: 0.2,
            metalness: 0.6,
            emissive: 0x0284c7,
            emissiveIntensity: 0.3
        });

        // 1. Pelvis
        const pelvisGroup = new THREE.Group();
        const sacrumGeo = new THREE.CylinderGeometry(0.04, 0.02, 0.12, 16);
        const sacrum = new THREE.Mesh(sacrumGeo, boneMaterial);
        sacrum.rotation.x = 0.2;
        pelvisGroup.add(sacrum);

        const iliumGeo = new THREE.TorusGeometry(0.08, 0.025, 12, 24, Math.PI * 0.9);
        const iliumRight = new THREE.Mesh(iliumGeo, boneMaterial);
        iliumRight.position.set(0.08, 0.04, 0);
        iliumRight.rotation.z = -0.4;
        pelvisGroup.add(iliumRight);

        const pubisGeo = new THREE.TorusGeometry(0.06, 0.02, 8, 16, Math.PI * 0.7);
        const pubis = new THREE.Mesh(pubisGeo, boneMaterial);
        pubis.position.set(0.04, -0.06, 0.03);
        pubis.rotation.x = Math.PI / 2;
        pelvisGroup.add(pubis);

        this.scene.add(pelvisGroup);
        this.boneMeshes.pelvis = pelvisGroup;

        // 2. Femur
        const femurGroup = new THREE.Group();
        const femurShaftGeo = new THREE.CylinderGeometry(0.024, 0.020, 0.40, 16);
        const femurShaft = new THREE.Mesh(femurShaftGeo, boneMaterial);
        femurShaft.position.y = -0.20;
        femurShaft.castShadow = true;
        femurGroup.add(femurShaft);

        const headGeo = new THREE.SphereGeometry(0.032, 24, 24);
        const femoralHead = new THREE.Mesh(headGeo, boneMaterial);
        femoralHead.position.set(-0.02, 0.01, 0.0);
        femurGroup.add(femoralHead);

        const trochGeo = new THREE.SphereGeometry(0.026, 16, 16);
        const trochanter = new THREE.Mesh(trochGeo, boneMaterial);
        trochanter.position.set(0.035, -0.03, 0.03);
        femurGroup.add(trochanter);

        const condyleGeo = new THREE.CylinderGeometry(0.036, 0.036, 0.07, 16);
        const condyles = new THREE.Mesh(condyleGeo, boneMaterial);
        condyles.position.set(0, -0.40, 0);
        condyles.rotation.z = Math.PI / 2;
        femurGroup.add(condyles);

        this.scene.add(femurGroup);
        this.boneMeshes.femur = femurGroup;

        // 3. Patella
        const patellaGeo = new THREE.SphereGeometry(0.025, 16, 16);
        patellaGeo.scale(1.0, 1.2, 0.6);
        const patella = new THREE.Mesh(patellaGeo, boneMaterial);
        this.scene.add(patella);
        this.boneMeshes.patella = patella;

        // 4. Tibia & Fibula
        const tibiaGroup = new THREE.Group();
        const plateauGeo = new THREE.CylinderGeometry(0.038, 0.030, 0.04, 16);
        const plateau = new THREE.Mesh(plateauGeo, boneMaterial);
        tibiaGroup.add(plateau);

        const tibiaShaftGeo = new THREE.CylinderGeometry(0.022, 0.016, 0.38, 16);
        const tibiaShaft = new THREE.Mesh(tibiaShaftGeo, boneMaterial);
        tibiaShaft.position.y = -0.19;
        tibiaShaft.castShadow = true;
        tibiaGroup.add(tibiaShaft);

        const fibulaGeo = new THREE.CylinderGeometry(0.008, 0.007, 0.37, 12);
        const fibula = new THREE.Mesh(fibulaGeo, boneMaterial);
        fibula.position.set(0, -0.19, 0.038);
        tibiaGroup.add(fibula);

        const malleolusGeo = new THREE.CylinderGeometry(0.025, 0.025, 0.065, 16);
        const malleoli = new THREE.Mesh(malleolusGeo, boneMaterial);
        malleoli.position.set(0, -0.38, 0);
        malleoli.rotation.z = Math.PI / 2;
        tibiaGroup.add(malleoli);

        this.scene.add(tibiaGroup);
        this.boneMeshes.tibia = tibiaGroup;

        // 5. Foot
        const footGroup = new THREE.Group();
        const heelGeo = new THREE.BoxGeometry(0.08, 0.05, 0.06);
        const heel = new THREE.Mesh(heelGeo, boneMaterial);
        heel.position.set(-0.04, -0.025, 0);
        footGroup.add(heel);

        const archGeo = new THREE.CylinderGeometry(0.025, 0.035, 0.14, 12);
        const arch = new THREE.Mesh(archGeo, boneMaterial);
        arch.position.set(0.06, -0.025, 0);
        arch.rotation.z = Math.PI / 2;
        footGroup.add(arch);

        const toesGeo = new THREE.BoxGeometry(0.06, 0.025, 0.07);
        const toes = new THREE.Mesh(toesGeo, boneMaterial);
        toes.position.set(0.15, -0.035, 0);
        footGroup.add(toes);

        this.scene.add(footGroup);
        this.boneMeshes.foot = footGroup;

        // Joint Markers
        const jointSphereGeo = new THREE.SphereGeometry(0.015, 16, 16);
        const hipPivot = new THREE.Mesh(jointSphereGeo, jointMaterial);
        const kneePivot = new THREE.Mesh(jointSphereGeo, jointMaterial);
        const anklePivot = new THREE.Mesh(jointSphereGeo, jointMaterial);
        this.scene.add(hipPivot);
        this.scene.add(kneePivot);
        this.scene.add(anklePivot);
        this.jointMarkers = { hip: hipPivot, knee: kneePivot, ankle: anklePivot };
    }

    /**
     * Create Anatomical Upper Limb Bones (Scapula, Humerus, Radius, Ulna, Hand)
     */
    createUpperLimbBones() {
        const boneMaterial = new THREE.MeshStandardMaterial({
            color: 0xf1f5f9,
            roughness: 0.35,
            metalness: 0.05
        });
        const jointMaterial = new THREE.MeshStandardMaterial({
            color: 0x38bdf8,
            roughness: 0.2,
            metalness: 0.6,
            emissive: 0x0284c7,
            emissiveIntensity: 0.3
        });

        // 1. Scapula / Clavicle Root
        const scapulaGroup = new THREE.Group();
        const glenoidGeo = new THREE.SphereGeometry(0.035, 16, 16);
        const glenoid = new THREE.Mesh(glenoidGeo, boneMaterial);
        scapulaGroup.add(glenoid);

        const bladeGeo = new THREE.BoxGeometry(0.12, 0.14, 0.02);
        const blade = new THREE.Mesh(bladeGeo, boneMaterial);
        blade.position.set(-0.06, 0.03, -0.04);
        scapulaGroup.add(blade);

        this.scene.add(scapulaGroup);
        this.boneMeshes.scapula = scapulaGroup;

        // 2. Humerus
        const humerusGroup = new THREE.Group();
        const humerusShaftGeo = new THREE.CylinderGeometry(0.018, 0.016, 0.30, 16);
        const humerusShaft = new THREE.Mesh(humerusShaftGeo, boneMaterial);
        humerusShaft.position.y = -0.15;
        humerusShaft.castShadow = true;
        humerusGroup.add(humerusShaft);

        const humeralHeadGeo = new THREE.SphereGeometry(0.028, 16, 16);
        const humeralHead = new THREE.Mesh(humeralHeadGeo, boneMaterial);
        humerusGroup.add(humeralHead);

        const condylesGeo = new THREE.CylinderGeometry(0.024, 0.024, 0.05, 16);
        const condyles = new THREE.Mesh(condylesGeo, boneMaterial);
        condyles.position.set(0, -0.30, 0);
        condyles.rotation.z = Math.PI / 2;
        humerusGroup.add(condyles);

        this.scene.add(humerusGroup);
        this.boneMeshes.humerus = humerusGroup;

        // 3. Forearm (Radius & Ulna)
        const forearmGroup = new THREE.Group();
        const ulnaGeo = new THREE.CylinderGeometry(0.014, 0.011, 0.26, 16);
        const ulna = new THREE.Mesh(ulnaGeo, boneMaterial);
        ulna.position.set(-0.012, -0.13, 0);
        ulna.castShadow = true;
        forearmGroup.add(ulna);

        const radiusGeo = new THREE.CylinderGeometry(0.013, 0.010, 0.25, 16);
        const radius = new THREE.Mesh(radiusGeo, boneMaterial);
        radius.position.set(0.014, -0.13, 0);
        radius.castShadow = true;
        forearmGroup.add(radius);

        // Olecranon process
        const olecranonGeo = new THREE.BoxGeometry(0.025, 0.035, 0.025);
        const olecranon = new THREE.Mesh(olecranonGeo, boneMaterial);
        olecranon.position.set(-0.012, 0.01, 0);
        forearmGroup.add(olecranon);

        this.scene.add(forearmGroup);
        this.boneMeshes.forearm = forearmGroup;

        // 4. Hand
        const handGroup = new THREE.Group();
        const palmGeo = new THREE.BoxGeometry(0.04, 0.08, 0.02);
        const palm = new THREE.Mesh(palmGeo, boneMaterial);
        palm.position.y = -0.04;
        handGroup.add(palm);

        this.scene.add(handGroup);
        this.boneMeshes.hand = handGroup;

        // Joint Pivots
        const jointSphereGeo = new THREE.SphereGeometry(0.014, 16, 16);
        const shoulderPivot = new THREE.Mesh(jointSphereGeo, jointMaterial);
        const elbowPivot = new THREE.Mesh(jointSphereGeo, jointMaterial);
        this.scene.add(shoulderPivot);
        this.scene.add(elbowPivot);
        this.jointMarkers = { shoulder: shoulderPivot, elbow: elbowPivot };
    }

    /**
     * Create 3D Smooth Tubular Muscle Meshes
     */
    createMuscleTubes() {
        for (const muscle of this.system.muscles) {
            const material = new THREE.MeshStandardMaterial({
                color: 0xef4444,
                roughness: 0.35,
                metalness: 0.15,
                emissive: 0x881337,
                emissiveIntensity: 0.2
            });

            const mesh = new THREE.Mesh(new THREE.BufferGeometry(), material);
            mesh.castShadow = true;
            this.scene.add(mesh);
            this.muscleMeshes[muscle.name] = mesh;
        }
    }

    /**
     * Update 3D Geometry and Kinematics
     */
    updateSimulation() {
        const updateData = this.system.update();
        const kin = updateData.kinematics;
        const muscleResults = updateData.muscles;

        if (this.currentModelType === "lower_limb") {
            // Lower limb updates
            this.boneMeshes.pelvis.position.set(0, 0, 0);

            const femur = this.boneMeshes.femur;
            femur.position.set(kin.hipCenter.x, kin.hipCenter.y, kin.hipCenter.z);
            femur.rotation.set(0, 0, 0);
            femur.rotateZ(kin.anglesRad.qHipFlex);
            femur.rotateX(kin.anglesRad.qHipAdd);
            femur.rotateY(kin.anglesRad.qHipRot);

            const tibia = this.boneMeshes.tibia;
            tibia.position.set(kin.kneeCenter.x, kin.kneeCenter.y, kin.kneeCenter.z);
            const totalKneeFlex = kin.anglesRad.qHipFlex - kin.anglesRad.qKnee;
            tibia.rotation.set(0, 0, 0);
            tibia.rotateZ(totalKneeFlex);
            tibia.rotateX(kin.anglesRad.qHipAdd);

            const patella = this.boneMeshes.patella;
            patella.position.set(kin.patellaCenter.x, kin.patellaCenter.y, kin.patellaCenter.z);
            patella.rotation.set(0, 0, totalKneeFlex + 0.2);

            const foot = this.boneMeshes.foot;
            foot.position.set(kin.ankleCenter.x, kin.ankleCenter.y, kin.ankleCenter.z);
            const totalFootAngle = totalKneeFlex - kin.anglesRad.qAnkle;
            foot.rotation.set(0, 0, totalFootAngle);
            foot.rotateX(kin.anglesRad.qHipAdd);

            this.jointMarkers.hip.position.set(kin.hipCenter.x, kin.hipCenter.y, kin.hipCenter.z);
            this.jointMarkers.knee.position.set(kin.kneeCenter.x, kin.kneeCenter.y, kin.kneeCenter.z);
            this.jointMarkers.ankle.position.set(kin.ankleCenter.x, kin.ankleCenter.y, kin.ankleCenter.z);
        } else {
            // Upper limb updates
            this.boneMeshes.scapula.position.set(0, 0, 0);

            const humerus = this.boneMeshes.humerus;
            humerus.position.set(kin.shoulderCenter.x, kin.shoulderCenter.y, kin.shoulderCenter.z);
            humerus.rotation.set(0, 0, 0);
            humerus.rotateZ(kin.anglesRad.qShoulder);

            const forearm = this.boneMeshes.forearm;
            forearm.position.set(kin.elbowCenter.x, kin.elbowCenter.y, kin.elbowCenter.z);
            const totalElbow = kin.anglesRad.qShoulder - kin.anglesRad.qElbow;
            forearm.rotation.set(0, 0, 0);
            forearm.rotateZ(totalElbow);
            forearm.rotateY(kin.anglesRad.qProSup);

            const hand = this.boneMeshes.hand;
            hand.position.set(kin.wristCenter.x, kin.wristCenter.y, kin.wristCenter.z);
            hand.rotation.set(0, 0, totalElbow);

            this.jointMarkers.shoulder.position.set(kin.shoulderCenter.x, kin.shoulderCenter.y, kin.shoulderCenter.z);
            this.jointMarkers.elbow.position.set(kin.elbowCenter.x, kin.elbowCenter.y, kin.elbowCenter.z);
        }

        // Update Muscle Tubes (Catmull-Rom Curves & Dynamic Bulking)
        for (const muscle of this.system.muscles) {
            const mData = muscleResults[muscle.name];
            const mesh = this.muscleMeshes[muscle.name];
            if (!mData || !mesh) continue;

            const pathPoints = mData.path.map(p => new THREE.Vector3(p.x, p.y, p.z));
            const curve = new THREE.CatmullRomCurve3(pathPoints);

            const normLm = mData.state.normFiberLength || 1.0;
            const baseRadius = 0.012;
            const bulkingFactor = Math.sqrt(1.0 / Math.max(0.5, normLm)) * (1.0 + 0.35 * muscle.activation);
            const tubeRadius = baseRadius * bulkingFactor;

            if (mesh.geometry) mesh.geometry.dispose();
            mesh.geometry = new THREE.TubeGeometry(curve, 26, tubeRadius, 10, false);

            const act = muscle.activation;
            const isSelected = this.selectedMuscle && this.selectedMuscle.name === muscle.name;

            if (isSelected) {
                mesh.material.color.setHex(0xfacc15);
                mesh.material.emissive.setHex(0xeab308);
                mesh.material.emissiveIntensity = 0.45 + 0.4 * act;
            } else if (act > 0.4) {
                mesh.material.color.setHex(0xf97316);
                mesh.material.emissive.setHex(0xe11d48);
                mesh.material.emissiveIntensity = 0.35 + 0.4 * act;
            } else {
                mesh.material.color.setHex(0xef4444);
                mesh.material.emissive.setHex(0x881337);
                mesh.material.emissiveIntensity = 0.18;
            }
        }

        this.updateTelemetryReadouts(updateData);

        if (this.plotter) {
            this.plotter.render();
        }

        // Record trajectory row if playing
        if (this.isPlaying) {
            this.recordedTrajectory.push({ ...this.system.coordinates });
        }
    }

    /**
     * Update Telemetry Panels
     */
    updateTelemetryReadouts(updateData) {
        for (const [key, val] of Object.entries(this.system.coordinates)) {
            const badge = document.getElementById(`val-${key}`);
            if (badge) badge.innerText = `${val.toFixed(1)}°`;
        }

        if (this.selectedMuscle) {
            const mData = updateData.muscles[this.selectedMuscle.name];
            if (mData) {
                const s = mData.state;
                document.getElementById("hud-muscle-name").innerText = this.selectedMuscle.displayName;
                document.getElementById("hud-force").innerText = `${s.totalForce.toFixed(0)} N`;
                document.getElementById("hud-active").innerText = `${s.activeForce.toFixed(0)} N`;
                document.getElementById("hud-passive").innerText = `${s.passiveForce.toFixed(0)} N`;
                document.getElementById("hud-length").innerText = `${(s.fiberLength * 1000).toFixed(1)} mm`;
                document.getElementById("hud-tendon").innerText = `${(s.tendonLength * 1000).toFixed(1)} mm`;
                document.getElementById("hud-pennation").innerText = `${s.pennationAngleDeg.toFixed(1)}°`;
                document.getElementById("hud-activation").innerText = `${(this.selectedMuscle.activation * 100).toFixed(0)}%`;
            }
        }

        for (const muscle of this.system.muscles) {
            const badge = document.getElementById(`tree-force-${muscle.name}`);
            if (badge) badge.innerText = `${muscle.totalForce.toFixed(0)} N`;
        }
    }

    /**
     * Switch Active Model between Lower Limb and Upper Limb
     */
    switchModel(modelKey) {
        if (this.currentModelType === modelKey) return;
        this.currentModelType = modelKey;
        this.buildCurrentModel();
        this.rebindUI();
        if (this.plotter) {
            const defaultJoint = modelKey === "lower_limb" ? "knee" : "elbow";
            this.plotter.setSelectedJoint(defaultJoint);
            this.plotter.setTarget(this.selectedMuscle, this.system);
        }
        this.updateSimulation();
    }

    /**
     * Rebind UI Elements when switching models
     */
    rebindUI() {
        // 1. Rebuild coordinate sliders container
        const coordContainer = document.getElementById("coords-sliders-container");
        if (coordContainer) {
            coordContainer.innerHTML = "";
            for (const [key, cfg] of Object.entries(this.system.coordLimits)) {
                const div = document.createElement("div");
                div.className = "slider-item";
                div.innerHTML = `
                    <div class="slider-header">
                        <span class="slider-name">${cfg.label}</span>
                        <span class="slider-val" id="val-${key}">${this.system.coordinates[key].toFixed(1)}°</span>
                    </div>
                    <input type="range" class="coord-slider" id="slider-${key}" min="${cfg.min}" max="${cfg.max}" value="${this.system.coordinates[key]}" step="0.5">
                `;
                coordContainer.appendChild(div);

                const slider = div.querySelector(`#slider-${key}`);
                slider.addEventListener("input", (e) => {
                    this.system.coordinates[key] = parseFloat(e.target.value);
                    this.updateSimulation();
                });
            }
        }

        // 2. Rebuild muscle excitation sliders
        const stimContainer = document.getElementById("stim-sliders-container");
        if (stimContainer) {
            stimContainer.innerHTML = "";
            for (const muscle of this.system.muscles) {
                const div = document.createElement("div");
                div.className = "slider-item";
                div.innerHTML = `
                    <div class="slider-header">
                        <span class="slider-name">${muscle.displayName.split("(")[0]}</span>
                        <span class="slider-val" id="stim-val-${muscle.name}">${(muscle.activation * 100).toFixed(0)}%</span>
                    </div>
                    <input type="range" class="coord-slider" id="stim-${muscle.name}" min="0" max="100" value="${(muscle.activation * 100).toFixed(0)}">
                `;
                stimContainer.appendChild(div);

                const stimInput = div.querySelector(`#stim-${muscle.name}`);
                stimInput.addEventListener("input", (e) => {
                    const u = parseFloat(e.target.value) / 100.0;
                    muscle.updateActivation(u, 0.05);
                    document.getElementById(`stim-val-${muscle.name}`).innerText = `${(u * 100).toFixed(0)}%`;
                    this.updateSimulation();
                });
            }
        }

        // 3. Rebuild Model Tree Hierarchy
        const treeList = document.getElementById("muscle-tree-list");
        if (treeList) {
            treeList.innerHTML = "";
            for (const muscle of this.system.muscles) {
                const li = document.createElement("li");
                li.className = `tree-item ${this.selectedMuscle.name === muscle.name ? "active" : ""}`;
                li.id = `tree-item-${muscle.name}`;
                li.innerHTML = `
                    <div class="tree-item-left">
                        <span class="tree-dot" style="background:#ef4444;"></span>
                        <span class="tree-title">${muscle.displayName.split("(")[0]}</span>
                    </div>
                    <span class="tree-badge" id="tree-force-${muscle.name}">${muscle.totalForce.toFixed(0)} N</span>
                `;
                li.addEventListener("click", () => this.selectMuscle(muscle));
                treeList.appendChild(li);
            }
        }

        // 4. Update Joint filter buttons in bottom HUD
        const jointFilterBar = document.querySelector(".joint-filter-bar");
        if (jointFilterBar) {
            if (this.currentModelType === "lower_limb") {
                jointFilterBar.innerHTML = `
                    <span style="font-size: 10px; color: var(--text-muted); align-self: center; margin-right: 4px;">Moment Arm Joint:</span>
                    <button class="btn-joint-select active" data-joint="knee">Knee</button>
                    <button class="btn-joint-select" data-joint="hip">Hip</button>
                    <button class="btn-joint-select" data-joint="ankle">Ankle</button>
                `;
            } else {
                jointFilterBar.innerHTML = `
                    <span style="font-size: 10px; color: var(--text-muted); align-self: center; margin-right: 4px;">Moment Arm Joint:</span>
                    <button class="btn-joint-select active" data-joint="elbow">Elbow</button>
                    <button class="btn-joint-select" data-joint="shoulder">Shoulder</button>
                `;
            }

            jointFilterBar.querySelectorAll(".btn-joint-select").forEach(btn => {
                btn.addEventListener("click", (e) => {
                    jointFilterBar.querySelectorAll(".btn-joint-select").forEach(b => b.classList.remove("active"));
                    e.target.classList.add("active");
                    if (this.plotter) this.plotter.setSelectedJoint(e.target.dataset.joint);
                });
            });
        }
    }

    /**
     * Initialize UI Sliders, Buttons, and Tree Event Listeners
     */
    initUI() {
        this.rebindUI();

        // Model Switcher Selector
        const modelSelect = document.getElementById("select-model");
        if (modelSelect) {
            modelSelect.addEventListener("change", (e) => {
                this.switchModel(e.target.value);
            });
        }

        // Biomechanical Preset Postures
        document.querySelectorAll(".btn-preset").forEach(btn => {
            btn.addEventListener("click", (e) => {
                const preset = e.target.dataset.preset;
                this.applyPreset(preset);
            });
        });

        // Motion Simulation Controls
        const playBtn = document.getElementById("btn-play-gait");
        if (playBtn) {
            playBtn.addEventListener("click", () => {
                this.isPlaying = !this.isPlaying;
                playBtn.innerHTML = this.isPlaying ? "❚❚ Pause Motion" : "▶ Play Motion";
                playBtn.classList.toggle("btn-active", this.isPlaying);
            });
        }

        const resetBtn = document.getElementById("btn-reset-pose");
        if (resetBtn) {
            resetBtn.addEventListener("click", () => {
                this.applyPreset("neutral");
            });
        }

        // Camera Presets
        document.querySelectorAll(".btn-cam").forEach(btn => {
            btn.addEventListener("click", (e) => {
                this.setCameraPreset(e.target.dataset.view);
            });
        });

        // Static Optimization Trigger
        const btnSolveSO = document.getElementById("btn-solve-so");
        if (btnSolveSO) {
            btnSolveSO.addEventListener("click", () => {
                this.solveStaticOptimization();
            });
        }

        // Forward Dynamics Physics Toggle
        const btnPhysics = document.getElementById("btn-toggle-physics");
        if (btnPhysics) {
            btnPhysics.addEventListener("click", () => {
                this.isPhysicsMode = !this.isPhysicsMode;
                btnPhysics.classList.toggle("btn-active", this.isPhysicsMode);
                btnPhysics.innerHTML = this.isPhysicsMode ? "⚡ Physics: Active (RK4)" : "⚡ Forward Dynamics";
            });
        }

        // OpenSim .osim XML Export Trigger
        const btnExportOsim = document.getElementById("btn-export-osim");
        if (btnExportOsim) {
            btnExportOsim.addEventListener("click", () => {
                this.exportOpenSimModel();
            });
        }

        // OpenSim .mot Motion Export Trigger
        const btnExportMot = document.getElementById("btn-export-mot");
        if (btnExportMot) {
            btnExportMot.addEventListener("click", () => {
                this.exportMotionTrajectory();
            });
        }

        // 3D Canvas Snapshot Download Trigger
        const btnSnapshot = document.getElementById("btn-snapshot");
        if (btnSnapshot) {
            btnSnapshot.addEventListener("click", () => {
                this.captureSnapshot();
            });
        }
    }

    /**
     * Select active muscle for telemetry and Hill curve inspector
     */
    selectMuscle(muscle) {
        this.selectedMuscle = muscle;
        document.querySelectorAll(".tree-item").forEach(el => el.classList.remove("active"));
        const activeLi = document.getElementById(`tree-item-${muscle.name}`);
        if (activeLi) activeLi.classList.add("active");

        if (this.plotter) {
            this.plotter.setTarget(muscle, this.system);
        }
        this.updateSimulation();
    }

    /**
     * Initialize Biomechanical Plotter on bottom canvas
     */
    initPlotter() {
        const plotCanvas = document.getElementById("curve-plot-canvas");
        if (plotCanvas) {
            this.plotter = new BiomechanicalPlotter(plotCanvas);
            this.plotter.setTarget(this.selectedMuscle, this.system);
        }
    }

    /**
     * Apply Clinical / Biomechanical Posture Presets
     */
    applyPreset(name) {
        if (this.currentModelType === "lower_limb") {
            const targets = {
                neutral: { hip_flexion: 0, hip_adduction: 0, hip_rotation: 0, knee_angle: 0, ankle_angle: 0 },
                mid_stance: { hip_flexion: 5, hip_adduction: 2, hip_rotation: 0, knee_angle: 12, ankle_angle: 8 },
                toe_off: { hip_flexion: -10, hip_adduction: -2, hip_rotation: 0, knee_angle: 35, ankle_angle: -15 },
                deep_squat: { hip_flexion: 85, hip_adduction: 8, hip_rotation: 5, knee_angle: 90, ankle_angle: 25 },
                hamstring_stretch: { hip_flexion: 75, hip_adduction: 0, hip_rotation: 0, knee_angle: 5, ankle_angle: 0 }
            };
            const t = targets[name] || targets.neutral;
            for (const [k, v] of Object.entries(t)) {
                this.system.coordinates[k] = v;
                const slider = document.getElementById(`slider-${k}`);
                if (slider) slider.value = v;
            }
        } else {
            const targets = {
                neutral: { shoulder_flexion: 0, elbow_flexion: 20, pro_sup: 0 },
                mid_stance: { shoulder_flexion: 15, elbow_flexion: 50, pro_sup: 10 },
                toe_off: { shoulder_flexion: 30, elbow_flexion: 90, pro_sup: 30 },
                deep_squat: { shoulder_flexion: 20, elbow_flexion: 120, pro_sup: 45 }
            };
            const t = targets[name] || targets.neutral;
            for (const [k, v] of Object.entries(t)) {
                this.system.coordinates[k] = v;
                const slider = document.getElementById(`slider-${k}`);
                if (slider) slider.value = v;
            }
        }
        this.updateSimulation();
    }

    /**
     * Set Camera Preset Angle
     */
    setCameraPreset(view) {
        switch (view) {
            case "sagittal":
                this.camera.position.set(2.0, -0.3, 0.0);
                break;
            case "anterior":
                this.camera.position.set(0.0, -0.3, 2.0);
                break;
            case "oblique":
                this.camera.position.set(1.4, 0.1, 1.6);
                break;
            case "superior":
                this.camera.position.set(0.0, 1.8, 0.1);
                break;
        }
        this.controls.update();
    }

    /**
     * Static Optimization Solver: Min sum(Fm / F0)^2 subject to target joint moment
     */
    solveStaticOptimization() {
        const targetTorque = parseFloat(document.getElementById("input-target-torque").value) || 35.0;

        if (this.currentModelType === "lower_limb") {
            const momentArms = this.system.computeMomentArms("knee_angle");
            const candidateMuscles = ["rect_fem", "vas_lat", "vas_med"];

            let sumDenom = 0;
            for (const name of candidateMuscles) {
                const m = this.system.muscleMap.get(name);
                const r = momentArms[name] || 0.04;
                sumDenom += (r * r) * (m.maxIsometricForce * m.maxIsometricForce);
            }
            const lambda = targetTorque / Math.max(1e-5, sumDenom);

            for (const name of candidateMuscles) {
                const m = this.system.muscleMap.get(name);
                const r = momentArms[name] || 0.04;
                const optimalF = lambda * r * (m.maxIsometricForce * m.maxIsometricForce);
                const optimalAct = Math.max(0.05, Math.min(1.0, optimalF / m.maxIsometricForce));
                m.activation = optimalAct;

                const stimSlider = document.getElementById(`stim-${name}`);
                if (stimSlider) stimSlider.value = (optimalAct * 100).toFixed(0);
                const stimVal = document.getElementById(`stim-val-${name}`);
                if (stimVal) stimVal.innerText = `${(optimalAct * 100).toFixed(0)}%`;
            }
        } else {
            const momentArms = this.system.computeMomentArms("elbow_flexion");
            const candidateMuscles = ["biceps_lh", "biceps_sh", "brachialis"];

            let sumDenom = 0;
            for (const name of candidateMuscles) {
                const m = this.system.muscleMap.get(name);
                const r = momentArms[name] || 0.03;
                sumDenom += (r * r) * (m.maxIsometricForce * m.maxIsometricForce);
            }
            const lambda = targetTorque / Math.max(1e-5, sumDenom);

            for (const name of candidateMuscles) {
                const m = this.system.muscleMap.get(name);
                const r = momentArms[name] || 0.03;
                const optimalF = lambda * r * (m.maxIsometricForce * m.maxIsometricForce);
                const optimalAct = Math.max(0.05, Math.min(1.0, optimalF / m.maxIsometricForce));
                m.activation = optimalAct;

                const stimSlider = document.getElementById(`stim-${name}`);
                if (stimSlider) stimSlider.value = (optimalAct * 100).toFixed(0);
                const stimVal = document.getElementById(`stim-val-${name}`);
                if (stimVal) stimVal.innerText = `${(optimalAct * 100).toFixed(0)}%`;
            }
        }

        this.updateSimulation();
        alert(`Static Optimization Solved for Target Torque: ${targetTorque} N·m!\nOptimal muscle recruitment has been calculated and applied to the simulation.`);
    }

    /**
     * Export current model to standard OpenSim 4.x .osim XML format
     */
    exportOpenSimModel() {
        const modelName = this.currentModelType === "lower_limb" ? "MoSim_Lower_Limb_Rajagopal2016" : "MoSim_Upper_Limb_Holzbaur2005";
        const xml = OpenSimIO.exportToOsimXml(this.system, modelName);

        const blob = new Blob([xml], { type: "application/xml" });
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = `${modelName}.osim`;
        a.click();
        URL.revokeObjectURL(url);
    }

    /**
     * Export motion trajectory to standard OpenSim .mot format
     */
    exportMotionTrajectory() {
        const coordNames = Object.keys(this.system.coordLimits);
        const rows = this.recordedTrajectory.length > 5 ? this.recordedTrajectory : (this.currentModelType === "lower_limb" ? this.gaitDataLower : this.gaitDataUpper);
        const timeArray = rows.map((r, i) => i * 0.02);

        const motText = OpenSimIO.exportMotionFile(timeArray, coordNames, rows);
        const blob = new Blob([motText], { type: "text/plain" });
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = `MoSim_Simulation_${this.currentModelType}.mot`;
        a.click();
        URL.revokeObjectURL(url);
    }

    /**
     * Capture high-resolution PNG snapshot from the 3D WebGL canvas
     */
    captureSnapshot() {
        this.renderer.render(this.scene, this.camera);
        const dataUrl = this.renderer.domElement.toDataURL("image/png");
        const a = document.createElement("a");
        a.href = dataUrl;
        a.download = `MoSim_3D_Snapshot_${Date.now()}.png`;
        a.click();
    }

    /**
     * Animation Loop (60 FPS) with Gait Cycle Interpolation & RK4 Physics
     */
    animate() {
        requestAnimationFrame(() => this.animate());

        // 1. Forward Dynamics Physics Integration Mode
        if (this.isPhysicsMode) {
            const activeCoordKey = this.currentModelType === "lower_limb" ? "knee_angle" : "elbow_flexion";
            const momentArms = this.system.computeMomentArms(activeCoordKey);

            let netTorque = 0.0;
            for (const muscle of this.system.muscles) {
                const r = momentArms[muscle.name] || 0.0;
                netTorque += r * muscle.totalForce;
            }

            // Step Runge-Kutta 4th order (dt = 0.016s)
            const newAngleDeg = this.physicsEngine.stepRK4(netTorque, 0.016);
            this.system.coordinates[activeCoordKey] = newAngleDeg;

            const slider = document.getElementById(`slider-${activeCoordKey}`);
            if (slider) slider.value = newAngleDeg;

            this.updateSimulation();
        }
        // 2. Kinematic Gait / Flexion Replay Mode
        else if (this.isPlaying) {
            this.gaitProgress = (this.gaitProgress + 0.006 * this.gaitSpeed) % 1.0;
            const data = this.currentModelType === "lower_limb" ? this.gaitDataLower : this.gaitDataUpper;

            const cycleCount = data.length;
            const sampleIdx = this.gaitProgress * (cycleCount - 1);
            const i0 = Math.floor(sampleIdx);
            const i1 = Math.min(cycleCount - 1, i0 + 1);
            const alpha = sampleIdx - i0;

            const p0 = data[i0];
            const p1 = data[i1];

            for (const key of Object.keys(this.system.coordLimits)) {
                if (p0[key] !== undefined && p1[key] !== undefined) {
                    const interpolated = p0[key] + alpha * (p1[key] - p0[key]);
                    this.system.coordinates[key] = interpolated;
                    const slider = document.getElementById(`slider-${key}`);
                    if (slider) slider.value = interpolated;
                }
            }

            const gaitBadge = document.getElementById("gait-phase-badge");
            if (gaitBadge) {
                const pct = (this.gaitProgress * 100).toFixed(0);
                const phase = this.currentModelType === "lower_limb" ?
                    (this.gaitProgress < 0.6 ? "Stance" : "Swing") :
                    (this.gaitProgress < 0.5 ? "Flexing" : "Extending");
                gaitBadge.innerText = `${pct}% (${phase})`;
            }

            this.updateSimulation();
        }

        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
}

// Global start
window.addEventListener("DOMContentLoaded", () => {
    window.app = new MoSimApp();
});
