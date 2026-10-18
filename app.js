/**
 * MoSim - Next-Generation Virtual Musculoskeletal Simulator
 * Main Application Controller & Three.js 3D Viewport Engine
 */

class MoSimApp {
    constructor() {
        this.system = new MusculoskeletalSystem();
        this.plotter = null;
        this.selectedMuscle = this.system.muscles[0]; // Default: Rectus Femoris

        // Animation & Gait Cycle
        this.isPlaying = false;
        this.simTime = 0.0;
        this.gaitSpeed = 1.0;
        this.gaitProgress = 0.0; // 0.0 to 1.0 (0% to 100% gait cycle)

        // Three.js instances
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.boneMeshes = {};
        this.muscleMeshes = {};
        this.jointMarkers = {};

        // Normal Gait Kinematics (Rajagopal et al. 2016 normalized walking cycle 0% to 100%)
        this.gaitData = [
            { t: 0.00, hip: 28.0, knee: 5.0,  ankle: 0.0 },   // Heel Strike (0%)
            { t: 0.15, hip: 20.0, knee: 18.0, ankle: -5.0 },  // Loading Response (15%)
            { t: 0.30, hip: 5.0,  knee: 10.0, ankle: 8.0 },   // Mid-Stance (30%)
            { t: 0.50, hip: -10.0, knee: 8.0, ankle: 12.0 },  // Terminal Stance (50%)
            { t: 0.60, hip: -8.0,  knee: 35.0, ankle: -15.0 }, // Toe-Off / Pre-Swing (60%)
            { t: 0.75, hip: 15.0, knee: 60.0, ankle: -5.0 },  // Mid-Swing (75%)
            { t: 0.85, hip: 25.0, knee: 30.0, ankle: 0.0 },   // Terminal Swing (85%)
            { t: 1.00, hip: 28.0, knee: 5.0,  ankle: 0.0 }    // Heel Strike (100%)
        ];

        this.init();
    }

    init() {
        this.initThreeJS();
        this.createSkeletalBones();
        this.createMuscleTubes();
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
        this.scene.fog = new THREE.FogExp2(0x0a0f1d, 0.25);

        // Camera
        this.camera = new THREE.PerspectiveCamera(45, width / height, 0.05, 50);
        this.camera.position.set(1.4, 0.1, 1.6);

        // Renderer
        this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
        this.renderer.setSize(width, height);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 1.1;
        container.appendChild(this.renderer.domElement);

        // OrbitControls
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.08;
        this.controls.target.set(0.0, -0.4, 0.0);
        this.controls.maxPolarAngle = Math.PI / 2 + 0.05; // Don't clip under ground

        // Lighting
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.8);
        this.scene.add(ambientLight);

        const keyLight = new THREE.DirectionalLight(0xffffff, 1.4);
        keyLight.position.set(3, 4, 3);
        keyLight.castShadow = true;
        keyLight.shadow.mapSize.width = 1024;
        keyLight.shadow.mapSize.height = 1024;
        this.scene.add(keyLight);

        const rimLight = new THREE.DirectionalLight(0x38bdf8, 0.9); // Cool rim light
        rimLight.position.set(-3, 2, -3);
        this.scene.add(rimLight);

        const fillLight = new THREE.DirectionalLight(0xf43f5e, 0.3); // Warm fill
        fillLight.position.set(0, -2, 2);
        this.scene.add(fillLight);

        // Biomechanical Ground Plane with Grid
        const gridHelper = new THREE.GridHelper(3.0, 30, 0x38bdf8, 0x1e293b);
        gridHelper.position.y = -0.98;
        this.scene.add(gridHelper);

        // Ground shadow receiver
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
     * Create Anatomical Bone Meshes (Pelvis, Femur, Tibia, Patella, Foot)
     */
    createSkeletalBones() {
        const boneMaterial = new THREE.MeshStandardMaterial({
            color: 0xf1f5f9,
            roughness: 0.35,
            metalness: 0.05,
            flatShading: false
        });

        const jointMaterial = new THREE.MeshStandardMaterial({
            color: 0x38bdf8,
            roughness: 0.2,
            metalness: 0.6,
            emissive: 0x0284c7,
            emissiveIntensity: 0.3
        });

        // 1. Pelvis Mesh (Composite realistic anatomical shape)
        const pelvisGroup = new THREE.Group();
        const sacrumGeo = new THREE.CylinderGeometry(0.04, 0.02, 0.12, 16);
        const sacrum = new THREE.Mesh(sacrumGeo, boneMaterial);
        sacrum.rotation.x = 0.2;
        pelvisGroup.add(sacrum);

        // Iliac wings (left & right)
        const iliumGeo = new THREE.TorusGeometry(0.08, 0.025, 12, 24, Math.PI * 0.9);
        const iliumRight = new THREE.Mesh(iliumGeo, boneMaterial);
        iliumRight.position.set(0.08, 0.04, 0);
        iliumRight.rotation.z = -0.4;
        pelvisGroup.add(iliumRight);

        // Pubic arch
        const pubisGeo = new THREE.TorusGeometry(0.06, 0.02, 8, 16, Math.PI * 0.7);
        const pubis = new THREE.Mesh(pubisGeo, boneMaterial);
        pubis.position.set(0.04, -0.06, 0.03);
        pubis.rotation.x = Math.PI / 2;
        pelvisGroup.add(pubis);

        this.scene.add(pelvisGroup);
        this.boneMeshes.pelvis = pelvisGroup;

        // 2. Femur (Thigh bone)
        const femurGroup = new THREE.Group();
        const femurShaftGeo = new THREE.CylinderGeometry(0.024, 0.020, 0.40, 16);
        const femurShaft = new THREE.Mesh(femurShaftGeo, boneMaterial);
        femurShaft.position.y = -0.20;
        femurShaft.castShadow = true;
        femurGroup.add(femurShaft);

        // Femoral Head (Ball of hip joint)
        const headGeo = new THREE.SphereGeometry(0.032, 24, 24);
        const femoralHead = new THREE.Mesh(headGeo, boneMaterial);
        femoralHead.position.set(-0.02, 0.01, 0.0);
        femurGroup.add(femoralHead);

        // Greater trochanter
        const trochGeo = new THREE.SphereGeometry(0.026, 16, 16);
        const trochanter = new THREE.Mesh(trochGeo, boneMaterial);
        trochanter.position.set(0.035, -0.03, 0.03);
        femurGroup.add(trochanter);

        // Femoral Condyles (Knee joint surface)
        const condyleGeo = new THREE.CylinderGeometry(0.036, 0.036, 0.07, 16);
        const condyles = new THREE.Mesh(condyleGeo, boneMaterial);
        condyles.position.set(0, -0.40, 0);
        condyles.rotation.z = Math.PI / 2;
        femurGroup.add(condyles);

        this.scene.add(femurGroup);
        this.boneMeshes.femur = femurGroup;

        // 3. Patella (Kneecap)
        const patellaGeo = new THREE.SphereGeometry(0.025, 16, 16);
        patellaGeo.scale(1.0, 1.2, 0.6);
        const patella = new THREE.Mesh(patellaGeo, boneMaterial);
        this.scene.add(patella);
        this.boneMeshes.patella = patella;

        // 4. Tibia & Fibula (Shin bones)
        const tibiaGroup = new THREE.Group();
        const plateauGeo = new THREE.CylinderGeometry(0.038, 0.030, 0.04, 16);
        const plateau = new THREE.Mesh(plateauGeo, boneMaterial);
        tibiaGroup.add(plateau);

        const tibiaShaftGeo = new THREE.CylinderGeometry(0.022, 0.016, 0.38, 16);
        const tibiaShaft = new THREE.Mesh(tibiaShaftGeo, boneMaterial);
        tibiaShaft.position.y = -0.19;
        tibiaShaft.castShadow = true;
        tibiaGroup.add(tibiaShaft);

        // Fibula (lateral slender bone)
        const fibulaGeo = new THREE.CylinderGeometry(0.008, 0.007, 0.37, 12);
        const fibula = new THREE.Mesh(fibulaGeo, boneMaterial);
        fibula.position.set(0, -0.19, 0.038);
        tibiaGroup.add(fibula);

        // Malleoli (ankle joint)
        const malleolusGeo = new THREE.CylinderGeometry(0.025, 0.025, 0.065, 16);
        const malleoli = new THREE.Mesh(malleolusGeo, boneMaterial);
        malleoli.position.set(0, -0.38, 0);
        malleoli.rotation.z = Math.PI / 2;
        tibiaGroup.add(malleoli);

        this.scene.add(tibiaGroup);
        this.boneMeshes.tibia = tibiaGroup;

        // 5. Foot (Calcaneus, Metatarsals, Toes)
        const footGroup = new THREE.Group();
        // Calcaneus (Heel)
        const heelGeo = new THREE.BoxGeometry(0.08, 0.05, 0.06);
        const heel = new THREE.Mesh(heelGeo, boneMaterial);
        heel.position.set(-0.04, -0.025, 0);
        heel.castShadow = true;
        footGroup.add(heel);

        // Midfoot / Arch
        const archGeo = new THREE.CylinderGeometry(0.025, 0.035, 0.14, 12);
        const arch = new THREE.Mesh(archGeo, boneMaterial);
        arch.position.set(0.06, -0.025, 0);
        arch.rotation.z = Math.PI / 2;
        arch.castShadow = true;
        footGroup.add(arch);

        // Toes
        const toesGeo = new THREE.BoxGeometry(0.06, 0.025, 0.07);
        const toes = new THREE.Mesh(toesGeo, boneMaterial);
        toes.position.set(0.15, -0.035, 0);
        footGroup.add(toes);

        this.scene.add(footGroup);
        this.boneMeshes.foot = footGroup;

        // Visual joint pivot markers
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
     * Create 3D Smooth Tubular Muscle Meshes (Catmull-Rom Curves)
     */
    createMuscleTubes() {
        for (const muscle of this.system.muscles) {
            // Material with dynamic tension color
            const material = new THREE.MeshStandardMaterial({
                color: 0xef4444, // Crimson muscle red
                roughness: 0.35,
                metalness: 0.15,
                emissive: 0x991b1b,
                emissiveIntensity: 0.2
            });

            const mesh = new THREE.Mesh(new THREE.BufferGeometry(), material);
            mesh.castShadow = true;
            this.scene.add(mesh);
            this.muscleMeshes[muscle.name] = mesh;
        }
    }

    /**
     * Update 3D Skeletal and Musculotendon geometry from Kinematics Engine
     */
    updateSimulation() {
        const updateData = this.system.update();
        const kin = updateData.kinematics;
        const muscleResults = updateData.muscles;

        // 1. Position Pelvis
        this.boneMeshes.pelvis.position.set(0, 0, 0);

        // 2. Position & Orient Femur
        const femur = this.boneMeshes.femur;
        femur.position.set(kin.hipCenter.x, kin.hipCenter.y, kin.hipCenter.z);

        // Femur orientation from hip angles
        const qHipFlex = kin.anglesRad.qHipFlex;
        const qHipAdd = kin.anglesRad.qHipAdd;
        const qHipRot = kin.anglesRad.qHipRot;
        femur.rotation.set(0, 0, 0);
        femur.rotateZ(qHipFlex);
        femur.rotateX(qHipAdd);
        femur.rotateY(qHipRot);

        // 3. Position & Orient Tibia
        const tibia = this.boneMeshes.tibia;
        tibia.position.set(kin.kneeCenter.x, kin.kneeCenter.y, kin.kneeCenter.z);
        const totalKneeFlex = qHipFlex - kin.anglesRad.qKnee;
        tibia.rotation.set(0, 0, 0);
        tibia.rotateZ(totalKneeFlex);
        tibia.rotateX(qHipAdd);

        // 4. Position Patella
        const patella = this.boneMeshes.patella;
        patella.position.set(kin.patellaCenter.x, kin.patellaCenter.y, kin.patellaCenter.z);
        patella.rotation.set(0, 0, totalKneeFlex + 0.2);

        // 5. Position Foot
        const foot = this.boneMeshes.foot;
        foot.position.set(kin.ankleCenter.x, kin.ankleCenter.y, kin.ankleCenter.z);
        const totalFootAngle = totalKneeFlex - kin.anglesRad.qAnkle;
        foot.rotation.set(0, 0, totalFootAngle);
        foot.rotateX(qHipAdd);

        // 6. Update Joint Pivot Markers
        this.jointMarkers.hip.position.set(kin.hipCenter.x, kin.hipCenter.y, kin.hipCenter.z);
        this.jointMarkers.knee.position.set(kin.kneeCenter.x, kin.kneeCenter.y, kin.kneeCenter.z);
        this.jointMarkers.ankle.position.set(kin.ankleCenter.x, kin.ankleCenter.y, kin.ankleCenter.z);

        // 7. Update Muscle Tubes (Catmull-Rom Curves & Dynamic Bulking)
        for (const muscle of this.system.muscles) {
            const mData = muscleResults[muscle.name];
            const mesh = this.muscleMeshes[muscle.name];
            if (!mData || !mesh) continue;

            const pathPoints = mData.path.map(p => new THREE.Vector3(p.x, p.y, p.z));
            const curve = new THREE.CatmullRomCurve3(pathPoints);

            // Dynamic muscle bulking radius: preserves muscle belly volume
            // r = r0 * sqrt(1 / normLm) * (1 + 0.3 * activation)
            const normLm = mData.state.normFiberLength || 1.0;
            const baseRadius = 0.012;
            const bulkingFactor = Math.sqrt(1.0 / Math.max(0.5, normLm)) * (1.0 + 0.35 * muscle.activation);
            const tubeRadius = baseRadius * bulkingFactor;

            // Dispose old geometry
            if (mesh.geometry) mesh.geometry.dispose();
            mesh.geometry = new THREE.TubeGeometry(curve, 28, tubeRadius, 10, false);

            // Color gradient based on activation and tension
            const act = muscle.activation;
            const isSelected = this.selectedMuscle && this.selectedMuscle.name === muscle.name;

            if (isSelected) {
                mesh.material.color.setHex(0xfacc15); // Glowing gold for selected
                mesh.material.emissive.setHex(0xeab308);
                mesh.material.emissiveIntensity = 0.4 + 0.4 * act;
            } else if (act > 0.4) {
                mesh.material.color.setHex(0xf97316); // Hot orange for high contraction
                mesh.material.emissive.setHex(0xe11d48);
                mesh.material.emissiveIntensity = 0.3 + 0.4 * act;
            } else {
                mesh.material.color.setHex(0xef4444); // Crimson for resting
                mesh.material.emissive.setHex(0x881337);
                mesh.material.emissiveIntensity = 0.15;
            }
        }

        // Update UI Readouts
        this.updateTelemetryReadouts(updateData);

        // Update Biomechanical Plotter
        if (this.plotter) {
            this.plotter.render();
        }
    }

    /**
     * Update Telemetry Panels (Force readouts, moment arms, angle numbers)
     */
    updateTelemetryReadouts(updateData) {
        // Update angle display badges
        for (const [key, val] of Object.entries(this.system.coordinates)) {
            const badge = document.getElementById(`val-${key}`);
            if (badge) {
                badge.innerText = `${val.toFixed(1)}°`;
            }
        }

        // Update selected muscle telemetry HUD
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

        // Update Model Tree list force badges
        for (const muscle of this.system.muscles) {
            const badge = document.getElementById(`tree-force-${muscle.name}`);
            if (badge) {
                badge.innerText = `${muscle.totalForce.toFixed(0)} N`;
            }
        }
    }

    /**
     * Initialize UI Sliders, Buttons, and Tree Event Listeners
     */
    initUI() {
        // 1. Coordinate Sliders
        for (const [key, cfg] of Object.entries(this.system.coordLimits)) {
            const slider = document.getElementById(`slider-${key}`);
            if (slider) {
                slider.min = cfg.min;
                slider.max = cfg.max;
                slider.value = this.system.coordinates[key];
                slider.addEventListener("input", (e) => {
                    this.system.coordinates[key] = parseFloat(e.target.value);
                    this.updateSimulation();
                });
            }
        }

        // 2. Muscle Excitation Sliders
        const stimContainer = document.getElementById("stim-sliders-container");
        if (stimContainer) {
            stimContainer.innerHTML = "";
            for (const muscle of this.system.muscles) {
                const div = document.createElement("div");
                div.className = "slider-item";
                div.innerHTML = `
                    <div class="slider-header">
                        <span class="slider-name">${muscle.displayName.split("(")[0]}</span>
                        <span class="slider-val" id="stim-val-${muscle.name}">5%</span>
                    </div>
                    <input type="range" class="coord-slider" id="stim-${muscle.name}" min="0" max="100" value="5">
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

        // 3. Model Hierarchy Explorer Items
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
                li.addEventListener("click", () => {
                    this.selectMuscle(muscle);
                });
                treeList.appendChild(li);
            }
        }

        // 4. Biomechanical Preset Postures
        document.querySelectorAll(".btn-preset").forEach(btn => {
            btn.addEventListener("click", (e) => {
                const preset = e.target.dataset.preset;
                this.applyPreset(preset);
            });
        });

        // 5. Gait Playback Controls
        const playBtn = document.getElementById("btn-play-gait");
        if (playBtn) {
            playBtn.addEventListener("click", () => {
                this.isPlaying = !this.isPlaying;
                playBtn.innerHTML = this.isPlaying ? "❚❚ Pause Gait" : "▶ Play Gait";
                playBtn.classList.toggle("btn-active", this.isPlaying);
            });
        }

        const resetBtn = document.getElementById("btn-reset-pose");
        if (resetBtn) {
            resetBtn.addEventListener("click", () => {
                this.applyPreset("neutral");
            });
        }

        // 6. Camera Presets
        document.querySelectorAll(".btn-cam").forEach(btn => {
            btn.addEventListener("click", (e) => {
                const view = e.target.dataset.view;
                this.setCameraPreset(view);
            });
        });

        // 7. Static Optimization Trigger
        const btnSolveSO = document.getElementById("btn-solve-so");
        if (btnSolveSO) {
            btnSolveSO.addEventListener("click", () => {
                this.solveStaticOptimization();
            });
        }

        // 8. Joint selector for moment arm chart
        document.querySelectorAll(".btn-joint-select").forEach(btn => {
            btn.addEventListener("click", (e) => {
                document.querySelectorAll(".btn-joint-select").forEach(b => b.classList.remove("active"));
                e.target.classList.add("active");
                if (this.plotter) {
                    this.plotter.setSelectedJoint(e.target.dataset.joint);
                }
            });
        });
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
        const targets = {
            neutral: { hip_flexion: 0, hip_adduction: 0, hip_rotation: 0, knee_angle: 0, ankle_angle: 0 },
            mid_stance: { hip_flexion: 5, hip_adduction: 2, hip_rotation: 0, knee_angle: 12, ankle_angle: 8 },
            toe_off: { hip_flexion: -10, hip_adduction: -2, hip_rotation: 0, knee_angle: 35, ankle_angle: -15 },
            deep_squat: { hip_flexion: 85, hip_adduction: 8, hip_rotation: 5, knee_angle: 90, ankle_angle: 25 },
            hamstring_stretch: { hip_flexion: 75, hip_adduction: 0, hip_rotation: 0, knee_angle: 5, ankle_angle: 0 }
        };

        const t = targets[name] || targets.neutral;
        // Smoothly set coordinates
        for (const [k, v] of Object.entries(t)) {
            this.system.coordinates[k] = v;
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
                this.controls.target.set(0.0, -0.4, 0.0);
                break;
            case "anterior":
                this.camera.position.set(0.0, -0.3, 2.0);
                this.controls.target.set(0.0, -0.4, 0.0);
                break;
            case "oblique":
                this.camera.position.set(1.4, 0.1, 1.6);
                this.controls.target.set(0.0, -0.4, 0.0);
                break;
            case "superior":
                this.camera.position.set(0.0, 1.8, 0.1);
                this.controls.target.set(0.0, -0.4, 0.0);
                break;
        }
        this.controls.update();
    }

    /**
     * Static Optimization Solver: Min sum(Fm / F0)^2 subject to target joint moment
     */
    solveStaticOptimization() {
        const targetTorque = parseFloat(document.getElementById("input-target-torque").value) || 30.0; // N.m knee extension
        const momentArms = this.system.computeMomentArms("knee_angle");

        // Identify agonist muscles (quadriceps with positive knee extension moment arms)
        // Rectus femoris, Vastus lateralis, Vastus medialis
        let sumDenom = 0;
        const candidateMuscles = ["rect_fem", "vas_lat", "vas_med"];

        for (const name of candidateMuscles) {
            const m = this.system.muscleMap.get(name);
            const r = momentArms[name] || 0.04;
            sumDenom += (r * r) * (m.maxIsometricForce * m.maxIsometricForce);
        }

        const lambda = targetTorque / Math.max(1e-5, sumDenom);

        // Compute optimal activations
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

        this.updateSimulation();
        alert(`Static Optimization Solved!\nTarget Knee Torque: ${targetTorque} N·m\nOptimized Quadriceps Recruitment:\n- Rectus Femoris: ${(this.system.muscleMap.get("rect_fem").activation * 100).toFixed(1)}%\n- Vastus Lateralis: ${(this.system.muscleMap.get("vas_lat").activation * 100).toFixed(1)}%\n- Vastus Medialis: ${(this.system.muscleMap.get("vas_med").activation * 100).toFixed(1)}%`);
    }

    /**
     * Animation Loop (60 FPS) with Gait Cycle Interpolation
     */
    animate() {
        requestAnimationFrame(() => this.animate());

        if (this.isPlaying) {
            this.gaitProgress = (this.gaitProgress + 0.006 * this.gaitSpeed) % 1.0;

            // Interpolate gait data points
            const cycleCount = this.gaitData.length;
            const sampleIdx = this.gaitProgress * (cycleCount - 1);
            const i0 = Math.floor(sampleIdx);
            const i1 = Math.min(cycleCount - 1, i0 + 1);
            const alpha = sampleIdx - i0;

            const p0 = this.gaitData[i0];
            const p1 = this.gaitData[i1];

            this.system.coordinates.hip_flexion = p0.hip + alpha * (p1.hip - p0.hip);
            this.system.coordinates.knee_angle = p0.knee + alpha * (p1.knee - p0.knee);
            this.system.coordinates.ankle_angle = p0.ankle + alpha * (p1.ankle - p0.ankle);

            // Update sliders
            const hipSlider = document.getElementById("slider-hip_flexion");
            if (hipSlider) hipSlider.value = this.system.coordinates.hip_flexion;
            const kneeSlider = document.getElementById("slider-knee_angle");
            if (kneeSlider) kneeSlider.value = this.system.coordinates.knee_angle;
            const ankleSlider = document.getElementById("slider-ankle_angle");
            if (ankleSlider) ankleSlider.value = this.system.coordinates.ankle_angle;

            const gaitBadge = document.getElementById("gait-phase-badge");
            if (gaitBadge) {
                const pct = (this.gaitProgress * 100).toFixed(0);
                const phase = this.gaitProgress < 0.6 ? "Stance Phase" : "Swing Phase";
                gaitBadge.innerText = `${pct}% (${phase})`;
            }

            this.updateSimulation();
        }

        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
}

// Instantiate upon DOM load
window.addEventListener("DOMContentLoaded", () => {
    window.app = new MoSimApp();
});
