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
        this.currentModelType = "whole_body"; // "whole_body", "lower_body", "upper_body", "right_arm", "left_arm", "right_leg", "left_leg", "head_neck"
        this.system = new WholeBodySystem("whole_body");
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
     * Build the currently active musculoskeletal model
     * Supports Whole Body, Lower Body, Upper Body, and Single-Part Models
     */
    buildCurrentModel() {
        this.clearModelMeshes();

        const m = this.currentModelType;
        if (m === "whole_body") {
            this.system = new WholeBodySystem("whole_body");
            this.createPelvisBones(true);
            this.createLegBones("_r", 0.10);
            this.createLegBones("_l", -0.10);
            this.createTorsoBones();
            this.createNeckBones();
            this.createSkullBones();
            this.createArmBones("_r", 0.22);
            this.createArmBones("_l", -0.22);

            this.controls.target.set(0.0, -0.05, 0.0);
            this.camera.position.set(1.9, 0.2, 2.2);
        } else if (m === "lower_body") {
            this.system = new WholeBodySystem("lower_body");
            this.createPelvisBones(true);
            this.createLegBones("_r", 0.10);
            this.createLegBones("_l", -0.10);

            this.controls.target.set(0.0, -0.45, 0.0);
            this.camera.position.set(1.5, -0.3, 1.6);
        } else if (m === "upper_body") {
            this.system = new WholeBodySystem("upper_body");
            this.createTorsoBones();
            this.createNeckBones();
            this.createSkullBones();
            this.createArmBones("_r", 0.22);
            this.createArmBones("_l", -0.22);

            this.controls.target.set(0.0, 0.35, 0.0);
            this.camera.position.set(1.4, 0.45, 1.4);
        } else if (m === "right_arm" || m === "upper_limb") {
            this.system = new WholeBodySystem("right_arm");
            this.createScapulaRoot();
            this.createArmBones("", 0);

            this.controls.target.set(0.0, 0.25, 0.0);
            this.camera.position.set(1.1, 0.3, 1.2);
        } else if (m === "left_arm") {
            this.system = new WholeBodySystem("left_arm");
            this.createScapulaRoot();
            this.createArmBones("", 0);

            this.controls.target.set(0.0, 0.25, 0.0);
            this.camera.position.set(1.1, 0.3, -1.2);
        } else if (m === "right_leg" || m === "lower_limb") {
            this.system = new WholeBodySystem("right_leg");
            this.createPelvisBones(false, "right");
            this.createLegBones("", 0);

            this.controls.target.set(0.0, -0.45, 0.0);
            this.camera.position.set(1.4, -0.3, 1.5);
        } else if (m === "left_leg") {
            this.system = new WholeBodySystem("left_leg");
            this.createPelvisBones(false, "left");
            this.createLegBones("", 0);

            this.controls.target.set(0.0, -0.45, 0.0);
            this.camera.position.set(1.4, -0.3, -1.5);
        } else if (m === "head_neck") {
            this.system = new WholeBodySystem("head_neck");
            this.createTorsoUpperBase();
            this.createNeckBones();
            this.createSkullBones();

            this.controls.target.set(0.0, 0.58, 0.0);
            this.camera.position.set(0.85, 0.62, 0.85);
        }

        this.controls.update();
        this.selectedMuscle = this.system.muscles[0];
        this.createMuscleTubes();
        this.physicsEngine.reset(this.system.coordinates[Object.keys(this.system.coordinates)[0]]);
    }

    /**
     * Standard PBR Bone & Joint Materials
     */
    getBoneMaterial() {
        return new THREE.MeshStandardMaterial({
            color: 0xf1f5f9,
            roughness: 0.35,
            metalness: 0.05
        });
    }

    getJointMaterial() {
        return new THREE.MeshStandardMaterial({
            color: 0x38bdf8,
            roughness: 0.2,
            metalness: 0.6,
            emissive: 0x0284c7,
            emissiveIntensity: 0.3
        });
    }

    /**
     * Create Pelvis & Sacrum Bone Geometry
     */
    createPelvisBones(isBilateral = true, side = "both") {
        const mat = this.getBoneMaterial();
        const pelvisGroup = new THREE.Group();

        // 1. Sacrum
        const sacrumGeo = new THREE.CylinderGeometry(0.045, 0.02, 0.12, 16);
        const sacrum = new THREE.Mesh(sacrumGeo, mat);
        sacrum.rotation.x = 0.2;
        pelvisGroup.add(sacrum);

        // 2. Right Hemi-Pelvis
        if (isBilateral || side === "right" || side === "both") {
            const iliumGeoR = new THREE.TorusGeometry(0.08, 0.025, 12, 24, Math.PI * 0.9);
            const iliumR = new THREE.Mesh(iliumGeoR, mat);
            iliumR.position.set(0.06, 0.04, 0.08);
            iliumR.rotation.z = -0.4;
            pelvisGroup.add(iliumR);

            const pubisGeoR = new THREE.TorusGeometry(0.05, 0.018, 8, 16, Math.PI * 0.7);
            const pubisR = new THREE.Mesh(pubisGeoR, mat);
            pubisR.position.set(0.04, -0.06, 0.035);
            pubisR.rotation.x = Math.PI / 2;
            pelvisGroup.add(pubisR);
        }

        // 3. Left Hemi-Pelvis
        if (isBilateral || side === "left" || side === "both") {
            const iliumGeoL = new THREE.TorusGeometry(0.08, 0.025, 12, 24, Math.PI * 0.9);
            const iliumL = new THREE.Mesh(iliumGeoL, mat);
            iliumL.position.set(0.06, 0.04, -0.08);
            iliumL.rotation.z = -0.4;
            pelvisGroup.add(iliumL);

            const pubisGeoL = new THREE.TorusGeometry(0.05, 0.018, 8, 16, Math.PI * 0.7);
            const pubisL = new THREE.Mesh(pubisGeoL, mat);
            pubisL.position.set(0.04, -0.06, -0.035);
            pubisL.rotation.x = -Math.PI / 2;
            pelvisGroup.add(pubisL);
        }

        this.scene.add(pelvisGroup);
        this.boneMeshes.pelvis = pelvisGroup;
    }

    /**
     * Create Leg Bones (Femur, Patella, Tibia, Fibula, Foot)
     */
    createLegBones(suffix = "", zOffset = 0) {
        const mat = this.getBoneMaterial();
        const jointMat = this.getJointMaterial();
        const sign = zOffset >= 0 ? 1 : -1;

        // 1. Femur
        const femurGroup = new THREE.Group();
        const femurShaftGeo = new THREE.CylinderGeometry(0.022, 0.018, 0.40, 16);
        const femurShaft = new THREE.Mesh(femurShaftGeo, mat);
        femurShaft.position.y = -0.20;
        femurShaft.castShadow = true;
        femurGroup.add(femurShaft);

        const headGeo = new THREE.SphereGeometry(0.030, 20, 20);
        const femoralHead = new THREE.Mesh(headGeo, mat);
        femoralHead.position.set(-0.02, 0.01, -0.01 * sign);
        femurGroup.add(femoralHead);

        const trochGeo = new THREE.SphereGeometry(0.024, 16, 16);
        const trochanter = new THREE.Mesh(trochGeo, mat);
        trochanter.position.set(0.032, -0.03, 0.028 * sign);
        femurGroup.add(trochanter);

        const condyleGeo = new THREE.CylinderGeometry(0.034, 0.034, 0.065, 16);
        const condyles = new THREE.Mesh(condyleGeo, mat);
        condyles.position.set(0, -0.40, 0);
        condyles.rotation.z = Math.PI / 2;
        femurGroup.add(condyles);

        this.scene.add(femurGroup);
        this.boneMeshes["femur" + suffix] = femurGroup;

        // 2. Patella
        const patellaGeo = new THREE.SphereGeometry(0.024, 16, 16);
        patellaGeo.scale(1.0, 1.2, 0.6);
        const patella = new THREE.Mesh(patellaGeo, mat);
        this.scene.add(patella);
        this.boneMeshes["patella" + suffix] = patella;

        // 3. Tibia & Fibula
        const tibiaGroup = new THREE.Group();
        const plateauGeo = new THREE.CylinderGeometry(0.036, 0.028, 0.04, 16);
        const plateau = new THREE.Mesh(plateauGeo, mat);
        tibiaGroup.add(plateau);

        const tibiaShaftGeo = new THREE.CylinderGeometry(0.021, 0.015, 0.38, 16);
        const tibiaShaft = new THREE.Mesh(tibiaShaftGeo, mat);
        tibiaShaft.position.y = -0.19;
        tibiaShaft.castShadow = true;
        tibiaGroup.add(tibiaShaft);

        const fibulaGeo = new THREE.CylinderGeometry(0.007, 0.006, 0.37, 12);
        const fibula = new THREE.Mesh(fibulaGeo, mat);
        fibula.position.set(0, -0.19, 0.036 * sign);
        tibiaGroup.add(fibula);

        const malleolusGeo = new THREE.CylinderGeometry(0.024, 0.024, 0.062, 16);
        const malleoli = new THREE.Mesh(malleolusGeo, mat);
        malleoli.position.set(0, -0.38, 0);
        malleoli.rotation.z = Math.PI / 2;
        tibiaGroup.add(malleoli);

        this.scene.add(tibiaGroup);
        this.boneMeshes["tibia" + suffix] = tibiaGroup;

        // 4. Foot
        const footGroup = new THREE.Group();
        const heelGeo = new THREE.BoxGeometry(0.08, 0.045, 0.055);
        const heel = new THREE.Mesh(heelGeo, mat);
        heel.position.set(-0.04, -0.025, 0);
        footGroup.add(heel);

        const archGeo = new THREE.CylinderGeometry(0.022, 0.032, 0.13, 12);
        const arch = new THREE.Mesh(archGeo, mat);
        arch.position.set(0.06, -0.025, 0);
        arch.rotation.z = Math.PI / 2;
        footGroup.add(arch);

        const toesGeo = new THREE.BoxGeometry(0.06, 0.022, 0.065);
        const toes = new THREE.Mesh(toesGeo, mat);
        toes.position.set(0.15, -0.035, 0);
        footGroup.add(toes);

        this.scene.add(footGroup);
        this.boneMeshes["foot" + suffix] = footGroup;

        // Joint Pivots
        const jointSphereGeo = new THREE.SphereGeometry(0.014, 16, 16);
        const hipPivot = new THREE.Mesh(jointSphereGeo, jointMat);
        const kneePivot = new THREE.Mesh(jointSphereGeo, jointMat);
        const anklePivot = new THREE.Mesh(jointSphereGeo, jointMat);
        this.scene.add(hipPivot);
        this.scene.add(kneePivot);
        this.scene.add(anklePivot);
        this.jointMarkers["hip" + suffix] = hipPivot;
        this.jointMarkers["knee" + suffix] = kneePivot;
        this.jointMarkers["ankle" + suffix] = anklePivot;
    }

    /**
     * Create Torso Bones (Spine, Ribcage barrel, Sternum, Clavicles, Scapulae)
     */
    createTorsoBones() {
        const mat = this.getBoneMaterial();
        const torsoGroup = new THREE.Group();

        // 1. Thoracic Spine Column (T1-T12 vertebrae rings)
        for (let i = 0; i < 9; i++) {
            const vGeo = new THREE.CylinderGeometry(0.022, 0.022, 0.03, 12);
            const vMesh = new THREE.Mesh(vGeo, mat);
            vMesh.position.set(-0.01, 0.06 + i * 0.04, 0);
            torsoGroup.add(vMesh);
        }

        // 2. Ribcage Barrel (Stacked Anatomical Rib Rings)
        const ribRadii = [0.08, 0.10, 0.12, 0.13, 0.12, 0.10];
        const ribY = [0.10, 0.16, 0.22, 0.28, 0.34, 0.39];
        for (let i = 0; i < ribRadii.length; i++) {
            const rGeo = new THREE.TorusGeometry(ribRadii[i], 0.010, 8, 20, Math.PI * 1.85);
            const rMesh = new THREE.Mesh(rGeo, mat);
            rMesh.rotation.x = Math.PI / 2;
            rMesh.rotation.z = Math.PI * 0.075;
            rMesh.position.set(0.02, ribY[i], 0);
            torsoGroup.add(rMesh);
        }

        // 3. Sternum (Anterior Breastbone)
        const sternumGeo = new THREE.BoxGeometry(0.030, 0.18, 0.014);
        const sternum = new THREE.Mesh(sternumGeo, mat);
        sternum.position.set(0.12, 0.28, 0);
        torsoGroup.add(sternum);

        // 4. Clavicles (Collar Bones reaching to shoulders)
        const clavicleGeoR = new THREE.CylinderGeometry(0.009, 0.008, 0.20, 12);
        const clavicleR = new THREE.Mesh(clavicleGeoR, mat);
        clavicleR.position.set(0.05, 0.40, 0.10);
        clavicleR.rotation.x = 1.35;
        clavicleR.rotation.z = -0.2;
        torsoGroup.add(clavicleR);

        const clavicleGeoL = new THREE.CylinderGeometry(0.009, 0.008, 0.20, 12);
        const clavicleL = new THREE.Mesh(clavicleGeoL, mat);
        clavicleL.position.set(0.05, 0.40, -0.10);
        clavicleL.rotation.x = -1.35;
        clavicleL.rotation.z = -0.2;
        torsoGroup.add(clavicleL);

        // 5. Scapular Blades (Posterior shoulder blades)
        const scapGeoR = new THREE.BoxGeometry(0.08, 0.11, 0.012);
        const scapR = new THREE.Mesh(scapGeoR, mat);
        scapR.position.set(-0.08, 0.32, 0.12);
        scapR.rotation.y = -0.25;
        torsoGroup.add(scapR);

        const scapGeoL = new THREE.BoxGeometry(0.08, 0.11, 0.012);
        const scapL = new THREE.Mesh(scapGeoL, mat);
        scapL.position.set(-0.08, 0.32, -0.12);
        scapL.rotation.y = 0.25;
        torsoGroup.add(scapL);

        this.scene.add(torsoGroup);
        this.boneMeshes.torso = torsoGroup;
    }

    /**
     * Create Neck Bones (Cervical Spine C1-C7)
     */
    createNeckBones() {
        const mat = this.getBoneMaterial();
        const jointMat = this.getJointMaterial();
        const neckGroup = new THREE.Group();

        for (let i = 0; i < 5; i++) {
            const cGeo = new THREE.CylinderGeometry(0.018, 0.018, 0.022, 12);
            const cMesh = new THREE.Mesh(cGeo, mat);
            cMesh.position.set(0, 0.015 + i * 0.025, 0);
            neckGroup.add(cMesh);
        }

        this.scene.add(neckGroup);
        this.boneMeshes.neck = neckGroup;

        const jointSphereGeo = new THREE.SphereGeometry(0.014, 16, 16);
        const neckPivot = new THREE.Mesh(jointSphereGeo, jointMat);
        this.scene.add(neckPivot);
        this.jointMarkers.neck = neckPivot;
    }

    /**
     * Create Skull (Cranium Dome, Facial Skeleton, Mandible)
     */
    createSkullBones() {
        const mat = this.getBoneMaterial();
        const skullGroup = new THREE.Group();

        // 1. Cranium Dome
        const craniumGeo = new THREE.SphereGeometry(0.075, 24, 24);
        craniumGeo.scale(1.05, 1.20, 1.22);
        const cranium = new THREE.Mesh(craniumGeo, mat);
        cranium.castShadow = true;
        skullGroup.add(cranium);

        // 2. Facial Skeleton & Maxilla
        const faceGeo = new THREE.BoxGeometry(0.045, 0.055, 0.075);
        const face = new THREE.Mesh(faceGeo, mat);
        face.position.set(0.05, -0.04, 0);
        skullGroup.add(face);

        // 3. Zygomatic Arches (Cheekbones)
        const cheekRGeo = new THREE.TorusGeometry(0.035, 0.008, 8, 16, Math.PI * 0.8);
        const cheekR = new THREE.Mesh(cheekRGeo, mat);
        cheekR.position.set(0.03, -0.03, 0.055);
        cheekR.rotation.y = Math.PI / 2;
        skullGroup.add(cheekR);

        const cheekLGeo = new THREE.TorusGeometry(0.035, 0.008, 8, 16, Math.PI * 0.8);
        const cheekL = new THREE.Mesh(cheekLGeo, mat);
        cheekL.position.set(0.03, -0.03, -0.055);
        cheekL.rotation.y = -Math.PI / 2;
        skullGroup.add(cheekL);

        // 4. Mandible (Jawbone)
        const jawGroup = new THREE.Group();
        const jawGeo = new THREE.TorusGeometry(0.045, 0.012, 8, 16, Math.PI * 0.75);
        const jaw = new THREE.Mesh(jawGeo, mat);
        jaw.rotation.x = Math.PI / 2;
        jaw.position.set(0.04, -0.075, 0);
        jawGroup.add(jaw);

        const chinGeo = new THREE.BoxGeometry(0.025, 0.025, 0.035);
        const chin = new THREE.Mesh(chinGeo, mat);
        chin.position.set(0.08, -0.08, 0);
        jawGroup.add(chin);

        skullGroup.add(jawGroup);

        this.scene.add(skullGroup);
        this.boneMeshes.skull = skullGroup;
    }

    /**
     * Create Arm & Hand Bones (Humerus, Radius, Ulna, Hand & Fingers)
     */
    createArmBones(suffix = "", zOffset = 0) {
        const mat = this.getBoneMaterial();
        const jointMat = this.getJointMaterial();

        // 1. Humerus
        const humerusGroup = new THREE.Group();
        const humerusShaftGeo = new THREE.CylinderGeometry(0.016, 0.014, 0.30, 16);
        const humerusShaft = new THREE.Mesh(humerusShaftGeo, mat);
        humerusShaft.position.y = -0.15;
        humerusShaft.castShadow = true;
        humerusGroup.add(humerusShaft);

        const humeralHeadGeo = new THREE.SphereGeometry(0.026, 16, 16);
        const humeralHead = new THREE.Mesh(humeralHeadGeo, mat);
        humerusGroup.add(humeralHead);

        const condylesGeo = new THREE.CylinderGeometry(0.022, 0.022, 0.048, 16);
        const condyles = new THREE.Mesh(condylesGeo, mat);
        condyles.position.set(0, -0.30, 0);
        condyles.rotation.z = Math.PI / 2;
        humerusGroup.add(condyles);

        this.scene.add(humerusGroup);
        this.boneMeshes["humerus" + suffix] = humerusGroup;

        // 2. Forearm (Radius & Ulna)
        const forearmGroup = new THREE.Group();
        const ulnaGeo = new THREE.CylinderGeometry(0.012, 0.009, 0.26, 16);
        const ulna = new THREE.Mesh(ulnaGeo, mat);
        ulna.position.set(-0.010, -0.13, 0);
        ulna.castShadow = true;
        forearmGroup.add(ulna);

        const radiusGeo = new THREE.CylinderGeometry(0.011, 0.009, 0.25, 16);
        const radius = new THREE.Mesh(radiusGeo, mat);
        radius.position.set(0.012, -0.13, 0);
        radius.castShadow = true;
        forearmGroup.add(radius);

        const olecranonGeo = new THREE.BoxGeometry(0.022, 0.030, 0.022);
        const olecranon = new THREE.Mesh(olecranonGeo, mat);
        olecranon.position.set(-0.010, 0.01, 0);
        forearmGroup.add(olecranon);

        this.scene.add(forearmGroup);
        this.boneMeshes["forearm" + suffix] = forearmGroup;

        // 3. Hand & Fingers
        const handGroup = new THREE.Group();
        const palmGeo = new THREE.BoxGeometry(0.035, 0.07, 0.018);
        const palm = new THREE.Mesh(palmGeo, mat);
        palm.position.y = -0.035;
        handGroup.add(palm);

        // 4 fingers
        for (let f = 0; f < 4; f++) {
            const fGeo = new THREE.CylinderGeometry(0.004, 0.003, 0.045, 8);
            const finger = new THREE.Mesh(fGeo, mat);
            finger.position.set(0, -0.085, -0.012 + f * 0.008);
            handGroup.add(finger);
        }
        // thumb
        const thumbGeo = new THREE.CylinderGeometry(0.005, 0.004, 0.035, 8);
        const thumb = new THREE.Mesh(thumbGeo, mat);
        thumb.position.set(0.018, -0.045, 0.015);
        thumb.rotation.z = -0.5;
        handGroup.add(thumb);

        this.scene.add(handGroup);
        this.boneMeshes["hand" + suffix] = handGroup;

        // Joint Pivots
        const jointSphereGeo = new THREE.SphereGeometry(0.014, 16, 16);
        const shoulderPivot = new THREE.Mesh(jointSphereGeo, jointMat);
        const elbowPivot = new THREE.Mesh(jointSphereGeo, jointMat);
        this.scene.add(shoulderPivot);
        this.scene.add(elbowPivot);
        this.jointMarkers["shoulder" + suffix] = shoulderPivot;
        this.jointMarkers["elbow" + suffix] = elbowPivot;
    }

    /**
     * Isolated Single Arm Scapula Mount
     */
    createScapulaRoot() {
        const mat = this.getBoneMaterial();
        const scapulaGroup = new THREE.Group();
        const glenoidGeo = new THREE.SphereGeometry(0.032, 16, 16);
        const glenoid = new THREE.Mesh(glenoidGeo, mat);
        scapulaGroup.add(glenoid);

        const bladeGeo = new THREE.BoxGeometry(0.12, 0.14, 0.02);
        const blade = new THREE.Mesh(bladeGeo, mat);
        blade.position.set(-0.06, 0.03, -0.04);
        scapulaGroup.add(blade);

        this.scene.add(scapulaGroup);
        this.boneMeshes.scapula = scapulaGroup;
    }

    /**
     * Isolated Head & Neck Base Mount
     */
    createTorsoUpperBase() {
        const mat = this.getBoneMaterial();
        const baseGeo = new THREE.BoxGeometry(0.24, 0.06, 0.14);
        const base = new THREE.Mesh(baseGeo, mat);
        base.position.set(0, 0.40, 0);
        this.scene.add(base);
        this.boneMeshes.torso_base = base;
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
     * Update 3D Geometry and Kinematics across Active Models
     */
    updateSimulation() {
        const updateData = this.system.update();
        const kin = updateData.kinematics;
        const muscleResults = updateData.muscles;

        // 1. Torso
        if (this.boneMeshes.torso) {
            this.boneMeshes.torso.position.set(0, 0, 0);
        }

        // 2. Neck
        if (this.boneMeshes.neck) {
            this.boneMeshes.neck.position.set(kin.neckBase.x, kin.neckBase.y, kin.neckBase.z);
            this.boneMeshes.neck.rotation.set(0, 0, 0);
            this.boneMeshes.neck.rotateZ(kin.anglesRad.qNeckFlex);
            this.boneMeshes.neck.rotateY(kin.anglesRad.qNeckRot);
            if (this.jointMarkers.neck) {
                this.jointMarkers.neck.position.set(kin.neckTop.x, kin.neckTop.y, kin.neckTop.z);
            }
        }

        // 3. Skull
        if (this.boneMeshes.skull) {
            this.boneMeshes.skull.position.set(kin.skullCenter.x, kin.skullCenter.y, kin.skullCenter.z);
            this.boneMeshes.skull.rotation.set(0, 0, 0);
            this.boneMeshes.skull.rotateZ(kin.anglesRad.qNeckFlex);
            this.boneMeshes.skull.rotateY(kin.anglesRad.qNeckRot);
        }

        // 4. Pelvis
        if (this.boneMeshes.pelvis) {
            this.boneMeshes.pelvis.position.set(0, 0, 0);
        }

        // 5. Bilateral Legs (_r and _l)
        if (this.boneMeshes.femur_r) {
            this.updateLegMesh(kin.hipCenterR, kin.kneeCenterR, kin.ankleCenterR, kin.patellaCenterR,
                              kin.anglesRad.qHipFlexR, kin.anglesRad.qHipAddR, kin.anglesRad.qKneeR, kin.anglesRad.qAnkleR, "_r");
        }
        if (this.boneMeshes.femur_l) {
            this.updateLegMesh(kin.hipCenterL, kin.kneeCenterL, kin.ankleCenterL, kin.patellaCenterL,
                              kin.anglesRad.qHipFlexL, 0, kin.anglesRad.qKneeL, kin.anglesRad.qAnkleL, "_l");
        }

        // 6. Single Leg (femur, tibia, patella, foot)
        if (this.boneMeshes.femur && !this.boneMeshes.femur_r) {
            this.updateLegMesh(kin.hipCenter, kin.kneeCenter, kin.ankleCenter, kin.patellaCenter,
                              kin.anglesRad.qHipFlex, kin.anglesRad.qHipAdd, kin.anglesRad.qKnee, kin.anglesRad.qAnkle, "");
        }

        // 7. Bilateral Arms (_r and _l)
        if (this.boneMeshes.humerus_r) {
            this.updateArmMesh(kin.shoulderCenterR, kin.elbowCenterR, kin.wristCenterR,
                              kin.anglesRad.qShoulderR, kin.anglesRad.qElbowR, kin.anglesRad.qProSupR, "_r");
        }
        if (this.boneMeshes.humerus_l) {
            this.updateArmMesh(kin.shoulderCenterL, kin.elbowCenterL, kin.wristCenterL,
                              kin.anglesRad.qShoulderL, kin.anglesRad.qElbowL, 0, "_l");
        }

        // 8. Single Arm (humerus, forearm, hand)
        if (this.boneMeshes.humerus && !this.boneMeshes.humerus_r) {
            this.updateArmMesh(kin.shoulderCenter, kin.elbowCenter, kin.wristCenter,
                              kin.anglesRad.qShoulder, kin.anglesRad.qElbow, kin.anglesRad.qProSup, "");
        }

        // 9. Scapula in isolated arm mode
        if (this.boneMeshes.scapula) {
            this.boneMeshes.scapula.position.set(0, 0, 0);
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
     * Helper to update leg bone meshes and joint pivots
     */
    updateLegMesh(hip, knee, ankle, patella, qHipFlex, qHipAdd, qKnee, qAnkle, suffix) {
        const femur = this.boneMeshes["femur" + suffix];
        if (femur) {
            femur.position.set(hip.x, hip.y, hip.z);
            femur.rotation.set(0, 0, 0);
            femur.rotateZ(qHipFlex);
            femur.rotateX(qHipAdd || 0);
        }
        const totalKnee = qHipFlex - qKnee;
        const tibia = this.boneMeshes["tibia" + suffix];
        if (tibia) {
            tibia.position.set(knee.x, knee.y, knee.z);
            tibia.rotation.set(0, 0, 0);
            tibia.rotateZ(totalKnee);
            tibia.rotateX(qHipAdd || 0);
        }
        const pat = this.boneMeshes["patella" + suffix];
        if (pat) {
            pat.position.set(patella.x, patella.y, patella.z);
            pat.rotation.set(0, 0, totalKnee + 0.2);
        }
        const foot = this.boneMeshes["foot" + suffix];
        const totalFoot = totalKnee - qAnkle;
        if (foot) {
            foot.position.set(ankle.x, ankle.y, ankle.z);
            foot.rotation.set(0, 0, totalFoot);
            foot.rotateX(qHipAdd || 0);
        }
        if (this.jointMarkers["hip" + suffix]) this.jointMarkers["hip" + suffix].position.set(hip.x, hip.y, hip.z);
        if (this.jointMarkers["knee" + suffix]) this.jointMarkers["knee" + suffix].position.set(knee.x, knee.y, knee.z);
        if (this.jointMarkers["ankle" + suffix]) this.jointMarkers["ankle" + suffix].position.set(ankle.x, ankle.y, ankle.z);
    }

    /**
     * Helper to update arm bone meshes and joint pivots
     */
    updateArmMesh(shoulder, elbow, wrist, qShoulder, qElbow, qProSup, suffix) {
        const humerus = this.boneMeshes["humerus" + suffix];
        if (humerus) {
            humerus.position.set(shoulder.x, shoulder.y, shoulder.z);
            humerus.rotation.set(0, 0, 0);
            humerus.rotateZ(qShoulder);
        }
        const totalElbow = qShoulder - qElbow;
        const forearm = this.boneMeshes["forearm" + suffix];
        if (forearm) {
            forearm.position.set(elbow.x, elbow.y, elbow.z);
            forearm.rotation.set(0, 0, 0);
            forearm.rotateZ(totalElbow);
            forearm.rotateY(qProSup || 0);
        }
        const hand = this.boneMeshes["hand" + suffix];
        if (hand) {
            hand.position.set(wrist.x, wrist.y, wrist.z);
            hand.rotation.set(0, 0, totalElbow);
        }
        if (this.jointMarkers["shoulder" + suffix]) this.jointMarkers["shoulder" + suffix].position.set(shoulder.x, shoulder.y, shoulder.z);
        if (this.jointMarkers["elbow" + suffix]) this.jointMarkers["elbow" + suffix].position.set(elbow.x, elbow.y, elbow.z);
    }

    /**
     * Get anatomical skeletal body list for the explorer tree
     */
    getSkeletalBodiesForCurrentModel() {
        const m = this.currentModelType;
        if (m === "whole_body") {
            return [
                { name: "Pelvis & Sacrum Frame", type: "Root Ground" },
                { name: "Thoracic Ribcage & Sternum", type: "Torso" },
                { name: "Cervical Column (C1-C7)", type: "Neck" },
                { name: "Cranium & Facial Skeleton", type: "Skull" },
                { name: "Mandible (Jaw)", type: "Articulated" },
                { name: "R. & L. Clavicle", type: "Pectoral" },
                { name: "R. Humerus & L. Humerus", type: "Rigid Lever" },
                { name: "R. Forearm & L. Forearm", type: "Articulated" },
                { name: "R. Hand & L. Hand", type: "End-Effector" },
                { name: "R. Femur & L. Femur", type: "Rigid Lever" },
                { name: "R. Patella & L. Patella", type: "Sesamoid" },
                { name: "R. Tibia & L. Tibia", type: "Rigid Lever" },
                { name: "R. Foot & L. Foot", type: "End-Effector" }
            ];
        } else if (m === "lower_body") {
            return [
                { name: "Pelvis & Sacrum", type: "Root Frame" },
                { name: "Right Femur", type: "Rigid Lever" },
                { name: "Left Femur", type: "Rigid Lever" },
                { name: "Right Patella & Left Patella", type: "Sesamoid" },
                { name: "Right Tibia & Fibula", type: "Articulated" },
                { name: "Left Tibia & Fibula", type: "Articulated" },
                { name: "Right Foot (Calcaneus/Toes)", type: "End-Effector" },
                { name: "Left Foot (Calcaneus/Toes)", type: "End-Effector" }
            ];
        } else if (m === "upper_body") {
            return [
                { name: "Thorax & Ribcage Barrel", type: "Trunk" },
                { name: "Sternum & Clavicles", type: "Pectoral" },
                { name: "Cervical Spine C1-C7", type: "Neck" },
                { name: "Skull (Cranium & Mandible)", type: "Head" },
                { name: "Right Humerus & Left Humerus", type: "Rigid Lever" },
                { name: "Right Forearm & Left Forearm", type: "Articulated" },
                { name: "Right Hand & Left Hand", type: "End-Effector" }
            ];
        } else if (m === "head_neck") {
            return [
                { name: "Thoracic Upper Base", type: "Root Frame" },
                { name: "Cervical Spine Column (C1-C7)", type: "Articulated" },
                { name: "Cranium & Facial Skeleton", type: "Rigid Dome" },
                { name: "Mandible (Jawbone)", type: "Articulated" }
            ];
        } else if (m === "right_arm" || m === "left_arm" || m === "upper_limb") {
            const side = m === "left_arm" ? "Left" : "Right";
            return [
                { name: `${side} Scapula & Clavicle`, type: "Root Frame" },
                { name: `${side} Humerus`, type: "Rigid Lever" },
                { name: `${side} Radius & Ulna`, type: "Articulated" },
                { name: `${side} Hand & Digits`, type: "End-Effector" }
            ];
        } else {
            const side = m === "left_leg" ? "Left" : "Right";
            return [
                { name: `${side} Hemi-Pelvis`, type: "Root Frame" },
                { name: `${side} Femur`, type: "Rigid Lever" },
                { name: `${side} Patella`, type: "Sesamoid" },
                { name: `${side} Tibia & Fibula`, type: "Articulated" },
                { name: `${side} Foot Complex`, type: "End-Effector" }
            ];
        }
    }

    /**
     * Switch Active Model between Whole Body, Regional, and Isolated Single-Part Models
     */
    switchModel(modelKey) {
        if (this.currentModelType === modelKey) return;
        this.currentModelType = modelKey;
        this.buildCurrentModel();
        this.rebindUI();
        if (this.plotter) {
            let defaultJoint = "knee";
            if (modelKey === "upper_body" || modelKey === "right_arm" || modelKey === "left_arm" || modelKey === "upper_limb") {
                defaultJoint = "elbow";
            } else if (modelKey === "head_neck") {
                defaultJoint = "neck";
            }
            this.plotter.setSelectedJoint(defaultJoint);
            this.plotter.setTarget(this.selectedMuscle, this.system);
        }
        this.updateSimulation();
    }

    /**
     * Rebind UI Elements when switching models
     */
    rebindUI() {
        // 0. Update Model Badge
        const badge = document.getElementById("model-type-badge");
        if (badge) {
            badge.innerText = this.system.displayName ? this.system.displayName.split("(")[0].trim() : "Whole Body";
        }

        // 1. Rebuild Skeletal Bodies tree list
        const bodiesTreeList = document.getElementById("bodies-tree-list");
        if (bodiesTreeList) {
            bodiesTreeList.innerHTML = "";
            const bodies = this.getSkeletalBodiesForCurrentModel();
            for (const b of bodies) {
                const li = document.createElement("li");
                li.className = "tree-item";
                li.innerHTML = `
                    <div class="tree-item-left">
                        <span class="tree-dot" style="background:#cbd5e1;"></span>
                        <span class="tree-title">${b.name}</span>
                    </div>
                    <span style="font-size:10px; color:var(--text-muted);">${b.type}</span>
                `;
                bodiesTreeList.appendChild(li);
            }
        }

        // 2. Rebuild coordinate sliders container
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

        // 3. Rebuild muscle excitation sliders
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

        // 4. Rebuild Model Tree Hierarchy
        const treeList = document.getElementById("muscle-tree-list");
        if (treeList) {
            treeList.innerHTML = "";
            for (const muscle of this.system.muscles) {
                const li = document.createElement("li");
                li.className = `tree-item ${this.selectedMuscle && this.selectedMuscle.name === muscle.name ? "active" : ""}`;
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

        // 5. Update Joint filter buttons in bottom HUD
        const jointFilterBar = document.querySelector(".joint-filter-bar");
        if (jointFilterBar) {
            let jointButtons = "";
            if (this.currentModelType === "whole_body") {
                jointButtons = `
                    <button class="btn-joint-select active" data-joint="knee">Knee</button>
                    <button class="btn-joint-select" data-joint="hip">Hip</button>
                    <button class="btn-joint-select" data-joint="ankle">Ankle</button>
                    <button class="btn-joint-select" data-joint="elbow">Elbow</button>
                    <button class="btn-joint-select" data-joint="shoulder">Shoulder</button>
                    <button class="btn-joint-select" data-joint="neck">Neck</button>
                `;
            } else if (this.currentModelType === "lower_body" || this.currentModelType === "right_leg" || this.currentModelType === "left_leg" || this.currentModelType === "lower_limb") {
                jointButtons = `
                    <button class="btn-joint-select active" data-joint="knee">Knee</button>
                    <button class="btn-joint-select" data-joint="hip">Hip</button>
                    <button class="btn-joint-select" data-joint="ankle">Ankle</button>
                `;
            } else if (this.currentModelType === "upper_body") {
                jointButtons = `
                    <button class="btn-joint-select active" data-joint="elbow">Elbow</button>
                    <button class="btn-joint-select" data-joint="shoulder">Shoulder</button>
                    <button class="btn-joint-select" data-joint="neck">Neck</button>
                `;
            } else if (this.currentModelType === "head_neck") {
                jointButtons = `
                    <button class="btn-joint-select active" data-joint="neck">Neck</button>
                `;
            } else {
                jointButtons = `
                    <button class="btn-joint-select active" data-joint="elbow">Elbow</button>
                    <button class="btn-joint-select" data-joint="shoulder">Shoulder</button>
                `;
            }

            jointFilterBar.innerHTML = `
                <span style="font-size: 10px; color: var(--text-muted); align-self: center; margin-right: 4px;">Moment Arm Joint:</span>
                ${jointButtons}
            `;

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

        // Action Info Explainer System (Provides brief info whenever mouse presses any button)
        this.setupButtonInfoExplainer();
    }

    /**
     * Wire up interactive info banners and inspector for all buttons and controls
     */
    setupButtonInfoExplainer() {
        const bannerClose = document.getElementById("action-banner-close");
        if (bannerClose) {
            bannerClose.addEventListener("click", () => {
                const banner = document.getElementById("action-info-banner");
                if (banner) banner.classList.remove("show");
            });
        }

        // 1. Explicitly mapped interactive elements
        const buttonIds = [
            "select-model",
            "btn-play-gait",
            "btn-toggle-physics",
            "btn-export-osim",
            "btn-export-mot",
            "btn-snapshot",
            "btn-reset-pose",
            "btn-solve-so",
            "tab-btn-coords",
            "tab-btn-stim",
            "tab-btn-solver"
        ];

        for (const id of buttonIds) {
            const el = document.getElementById(id);
            if (el) {
                el.addEventListener("click", () => this.showActionInfo(id));
                el.addEventListener("mouseenter", () => {
                    const inspectorMsg = document.getElementById("inspector-message");
                    const inspectorTag = document.getElementById("inspector-tag");
                    if (inspectorMsg) {
                        const info = this.getActionInfoData(id);
                        if (info) {
                            inspectorTag.innerText = info.icon || "INFO";
                            inspectorMsg.innerHTML = `<strong>${info.title}:</strong> ${info.desc}`;
                        }
                    }
                });
            }
        }

        // 2. Camera View buttons
        document.querySelectorAll(".btn-cam").forEach(btn => {
            const view = btn.dataset.view || "custom";
            const info = {
                title: `Camera View: ${view.toUpperCase()}`,
                icon: "📷",
                desc: `Smoothly repositions the 3D camera to the ${view} anatomical viewpoint for clear visualization of joint kinematics.`,
                tip: "You can also use left-click to orbit, right-click to pan, and scroll to zoom."
            };
            btn.addEventListener("click", () => this.showActionInfo("btn-cam", info));
            btn.addEventListener("mouseenter", () => {
                const msg = document.getElementById("inspector-message");
                if (msg) msg.innerHTML = `<strong>${info.title}:</strong> ${info.desc}`;
            });
        });

        // 3. Posture Preset buttons
        document.querySelectorAll(".btn-preset").forEach(btn => {
            const preset = btn.dataset.preset || "neutral";
            const info = {
                title: `Posture Preset: ${preset.replace("_", " ").toUpperCase()}`,
                icon: "🧍",
                desc: `Coordinates all musculoskeletal joint angles simultaneously to simulate the ${preset.replace("_", " ")} biomechanical posture.`,
                tip: "Watch the operating points on the Hill force-length curve glide in real time."
            };
            btn.addEventListener("click", () => this.showActionInfo("btn-preset", info));
            btn.addEventListener("mouseenter", () => {
                const msg = document.getElementById("inspector-message");
                if (msg) msg.innerHTML = `<strong>${info.title}:</strong> ${info.desc}`;
            });
        });

        // 4. Moment Arm Joint Filter buttons
        document.querySelectorAll(".btn-joint-select").forEach(btn => {
            const joint = btn.dataset.joint || "knee";
            const info = {
                title: `Moment Arm Filter: ${joint.toUpperCase()}`,
                icon: "📊",
                desc: `Recalculates and graphs generalized moment arms (r = -∂l/∂q) for all muscles crossing the ${joint} joint.`,
                tip: "Green bars (+) denote extension capability; Red bars (-) denote flexion capability."
            };
            btn.addEventListener("click", () => this.showActionInfo("btn-joint-select", info));
        });
    }

    /**
     * Dictionary of descriptions for all buttons and interactive elements
     */
    getActionInfoData(key) {
        const infoMap = {
            "select-model": {
                title: "Musculoskeletal Model Selector",
                icon: "🧬",
                desc: "Selects active model: Whole Body (Upper & Lower parts), Lower Part (Pelvis, Right Leg, Left Leg), Upper Part (Neck, Skull, Right & Left Hand/Arm), or isolated single-segment models (Right Arm Only, Left Arm Only, Right Leg Only, Left Leg Only, Head & Neck Only). Rebuilds the 3D skeleton and muscles instantly.",
                tip: "Select 'Whole Body' for the complete human skeleton with bilateral reciprocal walking gait, or isolate a single part like 'Right Arm Only' for focused joint analysis!"
            },
            "btn-play-gait": {
                title: "Kinematic Motion Simulation",
                icon: "▶",
                desc: "Replays dynamic experimental human motion cycles across all joints simultaneously (normal walking gait cycle 0%-100% or dynamic elbow curl).",
                tip: "Observe how agonist/antagonist muscles stretch and contract across stance and swing phases."
            },
            "btn-toggle-physics": {
                title: "Runge-Kutta 4 Forward Dynamics",
                icon: "⚡",
                desc: "Toggles physical forward dynamics integration. Net muscle moments dynamically accelerate joint segments against gravity and joint damping (I·ddq = τ_muscle - m·g·d·sin(q) - b·dq).",
                tip: "Switch to the 'Excitation' tab and stimulate muscles to see the joint pull and swing under active forces!"
            },
            "btn-export-osim": {
                title: "Export OpenSim 4.x Model (.osim)",
                icon: "📁",
                desc: "Generates and downloads a complete OpenSim 4.x XML model file (.osim) including BodySet, JointSet, CoordinateSet, and Millard2012EquilibriumMuscle definitions.",
                tip: "Open this file in desktop OpenSim 4.5 to run Inverse Kinematics (IK), RRA, CMC, or Moco."
            },
            "btn-export-mot": {
                title: "Export Motion Trajectory (.mot)",
                icon: "📊",
                desc: "Exports time-series simulated joint coordinate angles as a standard OpenSim motion file (.mot) with degree units and timestamps.",
                tip: "You can load this motion file into OpenSim's Motion Player or Plotter tool."
            },
            "btn-snapshot": {
                title: "3D Viewport Snapshot Capture",
                icon: "📷",
                desc: "Renders the 3D WebGL scene at native device resolution and triggers an automatic download of a high-quality PNG image.",
                tip: "Use this to save high-resolution diagrams for research reports or presentation slides."
            },
            "btn-reset-pose": {
                title: "Reset Posture & Controls",
                icon: "↺",
                desc: "Restores all joint coordinates to 0.0° anatomical neutral stance, stops playback, and resets muscle excitations to baseline resting tone.",
                tip: "Quickly restores default alignment after testing extreme joint angles."
            },
            "btn-solve-so": {
                title: "Static Optimization Solver (Force Sharing)",
                icon: "⚡",
                desc: "Solves the quadratic programming muscle redundancy problem: minimizes sum(Fm / F0_max)² subject to balancing the user-defined joint torque.",
                tip: "Optimal activations are calculated and immediately applied to the 3D model and sliders."
            },
            "tab-btn-coords": {
                title: "Joint Coordinates Tab",
                icon: "📐",
                desc: "Displays real-time sliders for all anatomical degrees of freedom (Hip, Knee, Ankle, Elbow, Shoulder) with range limits and angle readouts.",
                tip: "Drag any slider to see the 3D skeleton and muscle paths adapt at 60 FPS."
            },
            "tab-btn-stim": {
                title: "Muscle Neural Excitation Tab",
                icon: "⚡",
                desc: "Allows direct stimulation of individual muscles (u ∈ [0, 100%]). Excitation builds active muscle tension via first-order activation kinetics.",
                tip: "Stimulating muscles expands their 3D belly tube to simulate physiological volume preservation."
            },
            "tab-btn-solver": {
                title: "Optimization & Moments Tab",
                icon: "🎯",
                desc: "Configures target joint torques and solves optimal muscle recruitment across agonist and antagonist muscle groups.",
                tip: "Enter a target torque in N·m and click 'Solve Muscle Recruitment'."
            }
        };

        return infoMap[key] || null;
    }

    /**
     * Show brief info and biomechanical rationale whenever any button is pressed
     */
    showActionInfo(key, customData = null) {
        const item = customData || this.getActionInfoData(key);
        if (!item || !item.title) return;

        // 1. Update persistent inspector bar
        const inspectorBar = document.getElementById("action-inspector-bar");
        const inspectorTag = document.getElementById("inspector-tag");
        const inspectorMsg = document.getElementById("inspector-message");
        const inspectorStatus = document.getElementById("inspector-status-badge");

        if (inspectorBar && inspectorMsg) {
            if (inspectorTag) inspectorTag.innerText = item.icon || "ACTION";
            inspectorMsg.innerHTML = `<strong>${item.title}:</strong> ${item.desc}`;
            if (inspectorStatus) inspectorStatus.innerText = "Active";
            inspectorBar.classList.add("highlight");
            setTimeout(() => inspectorBar.classList.remove("highlight"), 1000);
        }

        // 2. Show floating action info banner
        const banner = document.getElementById("action-info-banner");
        const bannerTitle = document.getElementById("action-banner-title");
        const bannerDesc = document.getElementById("action-banner-desc");
        const bannerTip = document.getElementById("action-banner-tip-text");

        if (banner) {
            bannerTitle.innerHTML = `<span>${item.icon || "ℹ️"}</span> <span>${item.title}</span>`;
            bannerDesc.innerText = item.desc;
            if (bannerTip) bannerTip.innerText = item.tip || "Interactive biomechanics feature active.";

            banner.classList.add("show");

            if (this.bannerTimeout) clearTimeout(this.bannerTimeout);
            this.bannerTimeout = setTimeout(() => {
                banner.classList.remove("show");
            }, 4500);
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

        // Show interactive action info for selected muscle
        this.showActionInfo("tree-muscle", {
            title: `Muscle: ${muscle.displayName}`,
            icon: "🧬",
            desc: `F0_max = ${muscle.maxIsometricForce} N | Optimal Fiber Length (lm0) = ${(muscle.optimalFiberLength * 1000).toFixed(1)} mm | Tendon Slack (lt0) = ${(muscle.tendonSlackLength * 1000).toFixed(1)} mm. Live Hill curves now tracking this muscle.`,
            tip: "Move joint sliders to watch its operating point glide along the active and passive curves."
        });
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
    /**
     * Apply Clinical / Biomechanical Posture Presets across Active Models
     */
    applyPreset(name) {
        const coords = this.system.coordinates;
        if (name === "neutral") {
            for (const key of Object.keys(coords)) {
                coords[key] = (this.system.coordLimits[key] && this.system.coordLimits[key].default !== undefined) ?
                    this.system.coordLimits[key].default : 0.0;
            }
        } else if (name === "mid_stance") {
            if (coords.hip_flexion_r !== undefined) coords.hip_flexion_r = 20.0;
            if (coords.knee_angle_r !== undefined) coords.knee_angle_r = 18.0;
            if (coords.ankle_angle_r !== undefined) coords.ankle_angle_r = -5.0;
            if (coords.hip_flexion_l !== undefined) coords.hip_flexion_l = -8.0;
            if (coords.knee_angle_l !== undefined) coords.knee_angle_l = 35.0;
            if (coords.ankle_angle_l !== undefined) coords.ankle_angle_l = -15.0;
            if (coords.hip_flexion !== undefined) coords.hip_flexion = 20.0;
            if (coords.knee_angle !== undefined) coords.knee_angle = 18.0;
            if (coords.ankle_angle !== undefined) coords.ankle_angle = -5.0;
            if (coords.shoulder_flexion_r !== undefined) coords.shoulder_flexion_r = -10.0;
            if (coords.elbow_flexion_r !== undefined) coords.elbow_flexion_r = 25.0;
            if (coords.shoulder_flexion_l !== undefined) coords.shoulder_flexion_l = 20.0;
            if (coords.elbow_flexion_l !== undefined) coords.elbow_flexion_l = 35.0;
            if (coords.shoulder_flexion !== undefined) coords.shoulder_flexion = 15.0;
            if (coords.elbow_flexion !== undefined) coords.elbow_flexion = 45.0;
            if (coords.neck_flexion !== undefined) coords.neck_flexion = 5.0;
        } else if (name === "toe_off") {
            if (coords.hip_flexion_r !== undefined) coords.hip_flexion_r = -10.0;
            if (coords.knee_angle_r !== undefined) coords.knee_angle_r = 8.0;
            if (coords.ankle_angle_r !== undefined) coords.ankle_angle_r = 12.0;
            if (coords.hip_flexion_l !== undefined) coords.hip_flexion_l = 25.0;
            if (coords.knee_angle_l !== undefined) coords.knee_angle_l = 30.0;
            if (coords.ankle_angle_l !== undefined) coords.ankle_angle_l = 0.0;
            if (coords.hip_flexion !== undefined) coords.hip_flexion = -10.0;
            if (coords.knee_angle !== undefined) coords.knee_angle = 8.0;
            if (coords.ankle_angle !== undefined) coords.ankle_angle = 12.0;
            if (coords.shoulder_flexion_r !== undefined) coords.shoulder_flexion_r = 25.0;
            if (coords.elbow_flexion_r !== undefined) coords.elbow_flexion_r = 30.0;
            if (coords.shoulder_flexion_l !== undefined) coords.shoulder_flexion_l = -12.0;
            if (coords.elbow_flexion_l !== undefined) coords.elbow_flexion_l = 20.0;
            if (coords.shoulder_flexion !== undefined) coords.shoulder_flexion = 30.0;
            if (coords.elbow_flexion !== undefined) coords.elbow_flexion = 85.0;
            if (coords.neck_flexion !== undefined) coords.neck_flexion = -5.0;
        } else if (name === "deep_squat") {
            if (coords.hip_flexion_r !== undefined) coords.hip_flexion_r = 75.0;
            if (coords.knee_angle_r !== undefined) coords.knee_angle_r = 85.0;
            if (coords.ankle_angle_r !== undefined) coords.ankle_angle_r = 20.0;
            if (coords.hip_flexion_l !== undefined) coords.hip_flexion_l = 75.0;
            if (coords.knee_angle_l !== undefined) coords.knee_angle_l = 85.0;
            if (coords.ankle_angle_l !== undefined) coords.ankle_angle_l = 20.0;
            if (coords.hip_flexion !== undefined) coords.hip_flexion = 85.0;
            if (coords.knee_angle !== undefined) coords.knee_angle = 90.0;
            if (coords.ankle_angle !== undefined) coords.ankle_angle = 25.0;
            if (coords.shoulder_flexion_r !== undefined) coords.shoulder_flexion_r = 45.0;
            if (coords.elbow_flexion_r !== undefined) coords.elbow_flexion_r = 20.0;
            if (coords.shoulder_flexion_l !== undefined) coords.shoulder_flexion_l = 45.0;
            if (coords.elbow_flexion_l !== undefined) coords.elbow_flexion_l = 20.0;
            if (coords.shoulder_flexion !== undefined) coords.shoulder_flexion = 45.0;
            if (coords.elbow_flexion !== undefined) coords.elbow_flexion = 20.0;
        }

        for (const [k, v] of Object.entries(coords)) {
            const slider = document.getElementById(`slider-${k}`);
            if (slider) slider.value = v;
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
        const modelName = `MoSim_${this.currentModelType}_Model`;
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
     * Interpolate time-series motion frames with cyclic wrap
     */
    sampleGaitData(data, progress) {
        const cycleCount = data.length;
        const normT = ((progress % 1.0) + 1.0) % 1.0;
        const sampleIdx = normT * (cycleCount - 1);
        const i0 = Math.floor(sampleIdx);
        const i1 = Math.min(cycleCount - 1, i0 + 1);
        const alpha = sampleIdx - i0;
        const p0 = data[i0];
        const p1 = data[i1];

        const res = {};
        for (const key of Object.keys(p0)) {
            if (key !== "t") {
                res[key] = p0[key] + alpha * (p1[key] - p0[key]);
            }
        }
        return res;
    }

    /**
     * Animation Loop (60 FPS) with Bilateral Gait Cycle & Multi-Segment Replay
     */
    animate() {
        requestAnimationFrame(() => this.animate());

        // 1. Forward Dynamics Physics Integration Mode
        if (this.isPhysicsMode) {
            let activeCoordKey = Object.keys(this.system.coordLimits)[0];
            if (this.system.coordLimits["knee_angle_r"]) activeCoordKey = "knee_angle_r";
            else if (this.system.coordLimits["knee_angle"]) activeCoordKey = "knee_angle";
            else if (this.system.coordLimits["elbow_flexion_r"]) activeCoordKey = "elbow_flexion_r";
            else if (this.system.coordLimits["elbow_flexion"]) activeCoordKey = "elbow_flexion";
            else if (this.system.coordLimits["neck_flexion"]) activeCoordKey = "neck_flexion";

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
        // 2. Kinematic Gait / Motion Replay Mode
        else if (this.isPlaying) {
            this.gaitProgress = (this.gaitProgress + 0.006 * this.gaitSpeed) % 1.0;
            const t = this.gaitProgress;

            if (this.currentModelType === "whole_body" || this.currentModelType === "lower_body") {
                // Bilateral leg gait: Right leg at t, Left leg at (t + 0.5) % 1.0
                const sampleR = this.sampleGaitData(this.gaitDataLower, t);
                const sampleL = this.sampleGaitData(this.gaitDataLower, (t + 0.5) % 1.0);

                if (this.system.coordinates.hip_flexion_r !== undefined) {
                    this.system.coordinates.hip_flexion_r = sampleR.hip_flexion;
                    this.system.coordinates.knee_angle_r = sampleR.knee_angle;
                    this.system.coordinates.ankle_angle_r = sampleR.ankle_angle;

                    this.system.coordinates.hip_flexion_l = sampleL.hip_flexion;
                    this.system.coordinates.knee_angle_l = sampleL.knee_angle;
                    this.system.coordinates.ankle_angle_l = sampleL.ankle_angle;
                }

                if (this.system.coordinates.shoulder_flexion_r !== undefined) {
                    // Reciprocal arm swing: Right arm swings forward with left leg (t + 0.5), Left arm with right leg (t)
                    const armPhaseR = (t + 0.5) % 1.0;
                    const armPhaseL = t;
                    const swingR = -8.0 + 30.0 * Math.sin(armPhaseR * 2 * Math.PI);
                    const swingL = -8.0 + 30.0 * Math.sin(armPhaseL * 2 * Math.PI);
                    this.system.coordinates.shoulder_flexion_r = swingR;
                    this.system.coordinates.elbow_flexion_r = 15.0 + 15.0 * Math.max(0, Math.sin(armPhaseR * 2 * Math.PI));
                    this.system.coordinates.shoulder_flexion_l = swingL;
                    this.system.coordinates.elbow_flexion_l = 15.0 + 15.0 * Math.max(0, Math.sin(armPhaseL * 2 * Math.PI));
                }

                if (this.system.coordinates.neck_flexion !== undefined) {
                    this.system.coordinates.neck_flexion = 2.0 * Math.sin(t * 4 * Math.PI);
                }
            } else if (this.currentModelType === "upper_body") {
                const sample = this.sampleGaitData(this.gaitDataUpper, t);
                if (this.system.coordinates.shoulder_flexion_r !== undefined) {
                    this.system.coordinates.shoulder_flexion_r = sample.shoulder_flexion;
                    this.system.coordinates.elbow_flexion_r = sample.elbow_flexion;
                    this.system.coordinates.shoulder_flexion_l = sample.shoulder_flexion;
                    this.system.coordinates.elbow_flexion_l = sample.elbow_flexion;
                }
            } else if (this.currentModelType === "head_neck") {
                this.system.coordinates.neck_flexion = 15.0 * Math.sin(t * 2 * Math.PI);
                this.system.coordinates.neck_rotation = 20.0 * Math.cos(t * 2 * Math.PI);
                this.system.coordinates.jaw_opening = 10.0 * Math.max(0, Math.sin(t * 4 * Math.PI));
            } else if (this.currentModelType === "right_arm" || this.currentModelType === "left_arm" || this.currentModelType === "upper_limb") {
                const sample = this.sampleGaitData(this.gaitDataUpper, t);
                this.system.coordinates.shoulder_flexion = sample.shoulder_flexion;
                this.system.coordinates.elbow_flexion = sample.elbow_flexion;
                this.system.coordinates.pro_sup = sample.pro_sup;
            } else {
                const sample = this.sampleGaitData(this.gaitDataLower, t);
                this.system.coordinates.hip_flexion = sample.hip_flexion;
                this.system.coordinates.knee_angle = sample.knee_angle;
                this.system.coordinates.ankle_angle = sample.ankle_angle;
            }

            for (const key of Object.keys(this.system.coordinates)) {
                const slider = document.getElementById(`slider-${key}`);
                if (slider) slider.value = this.system.coordinates[key];
            }

            const gaitBadge = document.getElementById("gait-phase-badge");
            if (gaitBadge) {
                const pct = (this.gaitProgress * 100).toFixed(0);
                const isWalking = (this.currentModelType === "whole_body" || this.currentModelType === "lower_body" || this.currentModelType === "right_leg" || this.currentModelType === "left_leg" || this.currentModelType === "lower_limb");
                const phase = isWalking ?
                    (this.gaitProgress < 0.6 ? "R. Stance / L. Swing" : "R. Swing / L. Stance") :
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
