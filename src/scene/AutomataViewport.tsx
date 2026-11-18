import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import {
  CIRCULAR_CAM_MAX_OFFSET,
  clampCircularCamOffset,
  clampFourBarPivot,
  clampModuleSlotX,
  findNearestFreeSlotX,
  solveCircularCam,
  solveFourBarLinkage,
  solveSemiCircularCam,
} from '../engine/kinematics';
import type {
  AutomataBlueprint,
  CameraPresetId,
  ModuleType,
} from '../engine/types';
import { createAutomataMaterials } from './materials';
import {
  buildAutomataBoxGroup,
  buildBellCrankModuleRig,
  buildBevelGearModuleRig,
  buildCamModuleRig,
  buildCircularCamModuleRig,
  buildDigitalNumbersModuleRig,
  buildDualEccentricModuleRig,
  buildFourBarModuleRig,
  buildPinLanternModuleRig,
  buildRatchetModuleRig,
  buildScotchYokeModuleRig,
  buildSemiCircularCamModuleRig,
  buildWritingCamModuleRig,
  type AutomataBoxRig,
  type ModuleRig,
} from './meshBuilders';
import { createProceduralTextures } from './textures';

export interface AutomataViewportProps {
  blueprint: AutomataBlueprint;
  selectedModuleId: string | null;
  isPlaying: boolean;
  rpm: number;
  manualAngleDeg: number;
  cutawayAmount: number;
  explodeAmount: number;
  cameraPreset: CameraPresetId;
  cameraPresetTrigger: number;
  onSelectModule: (id: string | null) => void;
  onMoveModuleSlotX: (id: string, slotX: number) => void;
  onMoveFourBarPivot: (id: string, pivotZ: number, pivotY: number) => void;
  onMoveCircularCamOffset: (id: string, camOffsetRatio: number) => void;
  onDropNewModule: (type: ModuleType, slotX: number) => void;
  onAngleSync: (angleDeg: number) => void;
}

const CAMERA_PRESETS: Record<
  CameraPresetId,
  { pos: [number, number, number]; target: [number, number, number] }
> = {
  'three-quarter': { pos: [5.5, 4.9, 9.8], target: [0, 1.55, 0] },
  front: { pos: [0, 2.4, 10.6], target: [0, 1.55, 0] },
  side: { pos: [10.6, 3.0, 0.4], target: [0, 1.85, 0] },
  top: { pos: [0.1, 8.8, 3.8], target: [0, 2.4, 0] },
};

// Scratch objects pre-allocated outside animation loop (zero GC)
const _mouse = new THREE.Vector2();
const _hitPoint = new THREE.Vector3();
const _dragPlaneZ0 = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
const _dragPlaneYZ = new THREE.Plane(new THREE.Vector3(1, 0, 0), 0);

export const AutomataViewport: React.FC<AutomataViewportProps> = ({
  blueprint,
  selectedModuleId,
  isPlaying,
  rpm,
  manualAngleDeg,
  cutawayAmount,
  explodeAmount,
  cameraPreset,
  cameraPresetTrigger,
  onSelectModule,
  onMoveModuleSlotX,
  onMoveFourBarPivot,
  onMoveCircularCamOffset,
  onDropNewModule,
  onAngleSync,
}) => {
  const mountRef = useRef<HTMLDivElement | null>(null);
  const [contextLost, setContextLost] = useState(false);
  const [dragHoverHint, setDragHoverHint] = useState<string | null>(null);
  const [isDraggingOver, setIsDraggingOver] = useState(false);

  // Store latest React props & callbacks in refs so WebGL mounts only once
  const propsRef = useRef({
    blueprint,
    selectedModuleId,
    isPlaying,
    rpm,
    manualAngleDeg,
    cutawayAmount,
    explodeAmount,
    onSelectModule,
    onMoveModuleSlotX,
    onMoveFourBarPivot,
    onMoveCircularCamOffset,
    onDropNewModule,
    onAngleSync,
  });
  propsRef.current = {
    blueprint,
    selectedModuleId,
    isPlaying,
    rpm,
    manualAngleDeg,
    cutawayAmount,
    explodeAmount,
    onSelectModule,
    onMoveModuleSlotX,
    onMoveFourBarPivot,
    onMoveCircularCamOffset,
    onDropNewModule,
    onAngleSync,
  };

  // Engine control handles populated on mount
  const engineRef = useRef<{
    rebuildModules: () => void;
    applyFinish: () => void;
    requestRender: () => void;
    triggerCameraPreset: (preset: CameraPresetId) => void;
    syncManualAngle: (deg: number) => void;
    updateSelectionHighlight: () => void;
    setGhostRingX: (x: number | null) => void;
  } | null>(null);

  // Mount Three.js scene once
  useEffect(() => {
    const container = mountRef.current;
    if (!container) return;

    const isSoftwareGl = (() => {
      try {
        const probe = document.createElement('canvas').getContext('webgl2');
        if (!probe) return true;
        const ext = probe.getExtension('WEBGL_debug_renderer_info');
        const param = ext ? ext.UNMASKED_RENDERER_WEBGL : probe.RENDERER;
        const gpu = String(probe.getParameter(param));
        probe.getExtension('WEBGL_lose_context')?.loseContext();
        return /SwiftShader|llvmpipe|softpipe|Software|Basic Render/i.test(gpu);
      } catch {
        return true;
      }
    })();

    const width = container.clientWidth || window.innerWidth;
    const height = container.clientHeight || window.innerHeight;

    const renderer = new THREE.WebGLRenderer({
      antialias: !isSoftwareGl,
      powerPreference: 'high-performance',
      preserveDrawingBuffer: isSoftwareGl,
    });
    renderer.setSize(width, height);
    renderer.setPixelRatio(isSoftwareGl ? 1 : Math.min(window.devicePixelRatio, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.08;
    renderer.shadowMap.enabled = !isSoftwareGl;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.domElement.style.display = 'block';
    renderer.domElement.style.width = '100%';
    renderer.domElement.style.height = '100%';
    container.appendChild(renderer.domElement);

    const onContextLost = (e: Event) => {
      e.preventDefault();
      setContextLost(true);
    };
    const onContextRestored = () => {
      setContextLost(false);
    };
    renderer.domElement.addEventListener('webglcontextlost', onContextLost);
    renderer.domElement.addEventListener('webglcontextrestored', onContextRestored);

    const scene = new THREE.Scene();
    const bgHex = 0xede9e1;
    scene.background = new THREE.Color(bgHex);
    scene.fog = new THREE.FogExp2(bgHex, 0.022);

    const pmremGenerator = new THREE.PMREMGenerator(renderer);
    const envTex = pmremGenerator.fromScene(new RoomEnvironment(), 0.04).texture;
    scene.environment = envTex;
    pmremGenerator.dispose();

    const camera = new THREE.PerspectiveCamera(38, width / height, 0.1, 120);
    const initCam = CAMERA_PRESETS['three-quarter'];
    camera.position.set(...initCam.pos);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(...initCam.target);
    controls.enableDamping = !isSoftwareGl;
    controls.dampingFactor = 0.07;
    controls.minDistance = 4.0;
    controls.maxDistance = 18.0;
    controls.maxPolarAngle = Math.PI * 0.495;
    controls.update();

    // 3-Point Warm/Cool Studio Lighting
    const hemiLight = new THREE.HemisphereLight(0xe4edf7, 0x8f7d68, 0.72);
    scene.add(hemiLight);

    const keyLight = new THREE.DirectionalLight(0xfff1d9, 1.85);
    keyLight.position.set(6.5, 11.5, 7.5);
    keyLight.castShadow = !isSoftwareGl;
    keyLight.shadow.mapSize.set(1024, 1024);
    keyLight.shadow.camera.near = 1;
    keyLight.shadow.camera.far = 28;
    const d = 6.5;
    keyLight.shadow.camera.left = -d;
    keyLight.shadow.camera.right = d;
    keyLight.shadow.camera.top = d;
    keyLight.shadow.camera.bottom = -d;
    keyLight.shadow.bias = -0.0005;
    keyLight.shadow.normalBias = 0.02;
    scene.add(keyLight);

    const rimLight = new THREE.DirectionalLight(0xffe7c7, 0.75);
    rimLight.position.set(-7.0, 5.5, -6.5);
    scene.add(rimLight);

    // Procedural Textures & Materials
    const textures = createProceduralTextures(
      isSoftwareGl ? 1 : renderer.capabilities.getMaxAnisotropy()
    );
    const mats = createAutomataMaterials(textures, isSoftwareGl);

    // Workbench Ground & Drafting Mat + Contact Shadow
    const benchGeo = new THREE.PlaneGeometry(48, 48);
    benchGeo.rotateX(-Math.PI / 2);
    const benchMat = new THREE.MeshStandardMaterial({
      color: 0xffffff,
      map: textures.workbenchMatMap,
      roughness: 0.88,
      metalness: 0.02,
    });
    const benchMesh = new THREE.Mesh(benchGeo, benchMat);
    benchMesh.position.y = 0;
    benchMesh.receiveShadow = true;
    scene.add(benchMesh);

    const shadowDiscGeo = new THREE.PlaneGeometry(10.8, 5.6);
    shadowDiscGeo.rotateX(-Math.PI / 2);
    const shadowDiscMat = new THREE.MeshBasicMaterial({
      map: textures.contactShadowMap,
      transparent: true,
      depthWrite: false,
      opacity: 0.72,
    });
    const shadowDisc = new THREE.Mesh(shadowDiscGeo, shadowDiscMat);
    shadowDisc.position.set(0, 0.003, 0);
    scene.add(shadowDisc);

    // Drop ghost snap ring along shaft
    const ghostRingGeo = new THREE.TorusGeometry(0.5, 0.032, 10, 28);
    ghostRingGeo.rotateY(Math.PI / 2);
    const ghostRingMat = new THREE.MeshBasicMaterial({
      color: 0xb45309,
      transparent: true,
      opacity: 0.85,
    });
    const ghostRing = new THREE.Mesh(ghostRingGeo, ghostRingMat);
    ghostRing.position.set(0, propsRef.current.blueprint.shaftY, 0);
    ghostRing.visible = false;
    scene.add(ghostRing);

    // Build Chassis Box
    let boxRig: AutomataBoxRig = buildAutomataBoxGroup(propsRef.current.blueprint, mats);
    scene.add(boxRig.group);
    mats.applyFinish(propsRef.current.blueprint.finish);

    // Simulation, damping, and render-loop state (declared before rebuildModules)
    let shaftAngleRad = (propsRef.current.manualAngleDeg * Math.PI) / 180;
    let currentCutaway = propsRef.current.cutawayAmount;
    let currentExplode = propsRef.current.explodeAmount;
    let lastAngleUiSync = 0;
    let camTransition: {
      pos: THREE.Vector3;
      target: THREE.Vector3;
    } | null = null;
    let frameId = 0;
    let disposed = false;
    let renderBusy = false;
    let renderDirty = false;
    let lastTickTime = performance.now();

    // Build Module Rigs
    const moduleRigs: ModuleRig[] = [];
    const moduleRoot = new THREE.Group();
    scene.add(moduleRoot);
    const circularCamAccum = new Map<
      string,
      { lastKey: string; lastSolved: number; bias: number }
    >();
    const semiCircularCamAccum = new Map<
      string,
      { lastOffsetRatio: number; lastGearRatio: number; lastSolved: number; bias: number }
    >();

    function rebuildModules() {
      while (moduleRigs.length > 0) {
        const old = moduleRigs.pop()!;
        moduleRoot.remove(old.rootGroup);
        old.dispose();
      }
      const bp = propsRef.current.blueprint;
      const activeIds = new Set(bp.modules.map((m) => m.id));
      for (const key of circularCamAccum.keys()) {
        if (!activeIds.has(key)) circularCamAccum.delete(key);
      }
      for (const key of semiCircularCamAccum.keys()) {
        if (!activeIds.has(key)) semiCircularCamAccum.delete(key);
      }

      for (const mod of bp.modules) {
        let rig: ModuleRig;
        if (mod.type === 'cam-follower') {
          rig = buildCamModuleRig(mod, bp, mats);
        } else if (mod.type === 'four-bar-linkage') {
          rig = buildFourBarModuleRig(mod, bp, mats, (ang, m) => solveFourBarLinkage(ang, m));
        } else if (mod.type === 'bevel-gear') {
          rig = buildBevelGearModuleRig(mod, bp, mats);
        } else if (mod.type === 'pin-lantern') {
          rig = buildPinLanternModuleRig(mod, bp, mats);
        } else if (mod.type === 'ratchet') {
          rig = buildRatchetModuleRig(mod, bp, mats);
        } else if (mod.type === 'dual-eccentric') {
          rig = buildDualEccentricModuleRig(mod, bp, mats);
        } else if (mod.type === 'scotch-yoke') {
          rig = buildScotchYokeModuleRig(mod, bp, mats);
        } else if (mod.type === 'bell-crank') {
          rig = buildBellCrankModuleRig(mod, bp, mats);
        } else if (mod.type === 'digital-numbers') {
          rig = buildDigitalNumbersModuleRig(mod, bp, mats);
        } else if (mod.type === 'writing-cam') {
          rig = buildWritingCamModuleRig(mod, bp, mats);
        } else if (mod.type === 'semi-circular-cam') {
          const modId = mod.id;
          rig = buildSemiCircularCamModuleRig(mod, bp, mats, (shaftAng, curMod) => {
            const sol = solveSemiCircularCam(shaftAng, curMod);
            const prev = semiCircularCamAccum.get(modId);
            if (!prev) {
              semiCircularCamAccum.set(modId, {
                lastOffsetRatio: curMod.camOffsetRatio,
                lastGearRatio: curMod.gearRatio,
                lastSolved: sol.drivenAngleRad,
                bias: 0,
              });
              return sol.drivenAngleRad;
            }
            if (
              prev.lastOffsetRatio !== curMod.camOffsetRatio ||
              prev.lastGearRatio !== curMod.gearRatio
            ) {
              const prevDisplayed = prev.lastSolved + prev.bias;
              prev.bias = prevDisplayed - sol.drivenAngleRad;
              prev.lastOffsetRatio = curMod.camOffsetRatio;
              prev.lastGearRatio = curMod.gearRatio;
            }
            prev.lastSolved = sol.drivenAngleRad;
            return sol.drivenAngleRad + prev.bias;
          });
        } else {
          const modId = mod.id;
          rig = buildCircularCamModuleRig(mod, bp, mats, (shaftAng, curMod) => {
            const sol = solveCircularCam(shaftAng, curMod);
            const paramKey = `${curMod.camOffsetRatio}:${curMod.gearRatio}:${curMod.camShape ?? 'circle'}`;
            const prev = circularCamAccum.get(modId);
            if (!prev) {
              circularCamAccum.set(modId, {
                lastKey: paramKey,
                lastSolved: sol.drivenAngleRad,
                bias: 0,
              });
              return sol.drivenAngleRad;
            }
            if (prev.lastKey !== paramKey) {
              const prevDisplayed = prev.lastSolved + prev.bias;
              prev.bias = prevDisplayed - sol.drivenAngleRad;
              prev.lastKey = paramKey;
            }
            prev.lastSolved = sol.drivenAngleRad;
            return sol.drivenAngleRad + prev.bias;
          });
        }
        rig.setSelected(mod.id === propsRef.current.selectedModuleId);
        moduleRigs.push(rig);
        moduleRoot.add(rig.rootGroup);
      }
      requestRender();
    }

    function updateSelectionHighlight() {
      const selId = propsRef.current.selectedModuleId;
      for (const rig of moduleRigs) {
        rig.setSelected(rig.id === selId);
      }
      requestRender();
    }

    rebuildModules();

    if (!isSoftwareGl) {
      renderer.compile(scene, camera);
    }

    controls.addEventListener('start', () => {
      camTransition = null;
    });
    controls.addEventListener('change', () => {
      requestRender();
    });

    function applyKinematics(now: number, dt: number) {
      const state = propsRef.current;

      if (state.isPlaying) {
        const radPerSec = (state.rpm * Math.PI * 2) / 60;
        shaftAngleRad += radPerSec * dt;
        if (now - lastAngleUiSync > 110) {
          lastAngleUiSync = now;
          const deg = Math.round((((shaftAngleRad * 180) / Math.PI) % 360 + 360) % 360);
          state.onAngleSync(deg);
        }
      }

      // Smooth exponential damping for cutaway, explode, and camera presets
      currentCutaway = THREE.MathUtils.damp(currentCutaway, state.cutawayAmount, 10, dt);
      currentExplode = THREE.MathUtils.damp(currentExplode, state.explodeAmount, 10, dt);

      if (camTransition) {
        camera.position.x = THREE.MathUtils.damp(camera.position.x, camTransition.pos.x, 7, dt);
        camera.position.y = THREE.MathUtils.damp(camera.position.y, camTransition.pos.y, 7, dt);
        camera.position.z = THREE.MathUtils.damp(camera.position.z, camTransition.pos.z, 7, dt);
        controls.target.x = THREE.MathUtils.damp(controls.target.x, camTransition.target.x, 7, dt);
        controls.target.y = THREE.MathUtils.damp(controls.target.y, camTransition.target.y, 7, dt);
        controls.target.z = THREE.MathUtils.damp(controls.target.z, camTransition.target.z, 7, dt);

        if (
          camera.position.distanceToSquared(camTransition.pos) < 0.002 &&
          controls.target.distanceToSquared(camTransition.target) < 0.002
        ) {
          camera.position.copy(camTransition.pos);
          controls.target.copy(camTransition.target);
          camTransition = null;
        }
      }

      controls.update();

      // Drive 3D rigs
      boxRig.crankPivot.rotation.x = -shaftAngleRad;
      boxRig.updateLayout(currentExplode, currentCutaway);
      for (const rig of moduleRigs) {
        rig.update(shaftAngleRad, state.blueprint, currentExplode);
      }
    }

    function afterDraw(cb: () => void) {
      if (disposed) return;
      const gl = renderer.getContext() as WebGL2RenderingContext;
      const sync = gl.fenceSync(gl.SYNC_GPU_COMMANDS_COMPLETE, 0);
      gl.flush();
      const poll = () => {
        if (disposed) return;
        const status = sync && gl.getSyncParameter(sync, gl.SYNC_STATUS);
        if (status === gl.UNSIGNALED) {
          frameId = window.setTimeout(poll, 5);
        } else {
          if (sync) gl.deleteSync(sync);
          frameId = window.setTimeout(cb, 80);
        }
      };
      frameId = window.setTimeout(poll, 5);
    }

    function needsContinuousTick() {
      const state = propsRef.current;
      return (
        state.isPlaying ||
        camTransition !== null ||
        Math.abs(currentCutaway - state.cutawayAmount) > 0.003 ||
        Math.abs(currentExplode - state.explodeAmount) > 0.003
      );
    }

    function renderFrame() {
      if (disposed) return;
      const now = performance.now();
      const dt = Math.min(0.08, Math.max(0.001, (now - lastTickTime) / 1000));
      lastTickTime = now;

      applyKinematics(now, dt);
      renderer.render(scene, camera);

      const finish = () => {
        renderBusy = false;
        if (disposed) return;
        if (needsContinuousTick() || renderDirty) {
          renderDirty = false;
          scheduleNextRender();
        }
      };

      if (isSoftwareGl) afterDraw(finish);
      else finish();
    }

    function scheduleNextRender() {
      if (disposed || renderBusy) return;
      renderBusy = true;
      frameId = requestAnimationFrame(renderFrame);
    }

    function requestRender() {
      if (disposed) return;
      if (renderBusy) {
        renderDirty = true;
        return;
      }
      scheduleNextRender();
    }

    // Raycasting & Direct 3D Dragging (Slot X along shaft, 4-Bar Pivot YZ, & Circular Cam X Offset)
    const raycaster = new THREE.Raycaster();
    let dragState: {
      mode: 'module' | 'pivot' | 'cam-offset';
      moduleId: string;
      slotX: number;
      camOffsetRatio?: number;
    } | null = null;
    let lastHoverCheck = 0;

    function updateMouseFromPointer(clientX: number, clientY: number) {
      const rect = renderer.domElement.getBoundingClientRect();
      _mouse.x = ((clientX - rect.left) / rect.width) * 2 - 1;
      _mouse.y = -((clientY - rect.top) / rect.height) * 2 + 1;
    }

    function formatCamOffsetDragHint(offsetRatio: number): string {
      if (offsetRatio <= -0.99) return '拖拽中：圆形凸轮已对齐 托盘最左侧（正转输出）';
      if (Math.abs(offsetRatio) < 0.01) return '拖拽中：圆形凸轮已对齐 托盘正中心（停转空档）';
      if (offsetRatio >= 0.99) return '拖拽中：圆形凸轮已对齐 托盘最右侧（反转输出）';
      const pct = Math.round(Math.abs(offsetRatio) * 100);
      return `拖拽中：仅调整圆形凸轮位置（托盘${offsetRatio < 0 ? '偏左' : '偏右'} ${pct}%）`;
    }

    function onPointerDown(e: PointerEvent) {
      if (e.button !== 0) return;
      updateMouseFromPointer(e.clientX, e.clientY);
      raycaster.setFromCamera(_mouse, camera);

      // Priority 1: Draggable 4-bar fulcrum pivot thumbnuts
      const pivotTargets = moduleRigs.flatMap((r) => r.pivotHitMeshes);
      const pivotHits = raycaster.intersectObjects(pivotTargets, false);
      if (pivotHits.length > 0) {
        const modId = pivotHits[0].object.userData.moduleId as string;
        const mod = propsRef.current.blueprint.modules.find((m) => m.id === modId);
        if (mod) {
          controls.enabled = false;
          dragState = { mode: 'pivot', moduleId: modId, slotX: mod.slotX };
          propsRef.current.onSelectModule(modId);
          setDragHoverHint('拖拽中：正在调整四连杆固定支点 (Y / Z)');
          renderer.domElement.setPointerCapture(e.pointerId);
          return;
        }
      }

      // Priority 1b: Draggable vertical circular cam wheel under horizontal turntable tray
      const camOffsetTargets = moduleRigs.flatMap((r) => r.camOffsetHitMeshes ?? []);
      const camOffsetHits = raycaster.intersectObjects(camOffsetTargets, false);
      if (camOffsetHits.length > 0) {
        const modId = camOffsetHits[0].object.userData.moduleId as string;
        const mod = propsRef.current.blueprint.modules.find((m) => m.id === modId);
        if (mod && (mod.type === 'circular-cam' || mod.type === 'semi-circular-cam')) {
          controls.enabled = false;
          dragState = {
            mode: 'cam-offset',
            moduleId: modId,
            slotX: mod.slotX,
            camOffsetRatio: mod.camOffsetRatio,
          };
          propsRef.current.onSelectModule(modId);
          setDragHoverHint(formatCamOffsetDragHint(mod.camOffsetRatio));
          renderer.domElement.setPointerCapture(e.pointerId);
          return;
        }
      }

      // Priority 2: Modular blocks along camshaft (whole module move)
      const modTargets = moduleRigs.flatMap((r) => r.hitMeshes);
      const modHits = raycaster.intersectObjects(modTargets, false);
      if (modHits.length > 0) {
        const modId = modHits[0].object.userData.moduleId as string;
        const mod = propsRef.current.blueprint.modules.find((m) => m.id === modId);
        if (mod) {
          controls.enabled = false;
          dragState = { mode: 'module', moduleId: modId, slotX: mod.slotX };
          propsRef.current.onSelectModule(modId);
          setDragHoverHint('拖拽中：整体沿主轴左右滑动装配位置 (X)');
          renderer.domElement.setPointerCapture(e.pointerId);
          return;
        }
      }
    }

    function onPointerMove(e: PointerEvent) {
      if (dragState) {
        updateMouseFromPointer(e.clientX, e.clientY);
        raycaster.setFromCamera(_mouse, camera);

        if (dragState.mode === 'module') {
          if (raycaster.ray.intersectPlane(_dragPlaneZ0, _hitPoint)) {
            const clampedX = clampModuleSlotX(
              _hitPoint.x,
              dragState.moduleId,
              propsRef.current.blueprint.modules
            );
            ghostRing.visible = true;
            ghostRing.position.set(clampedX, propsRef.current.blueprint.shaftY, 0);
            if (Math.abs(clampedX - dragState.slotX) > 0.02) {
              dragState.slotX = clampedX;
              propsRef.current.onMoveModuleSlotX(dragState.moduleId, clampedX);
            }
          }
        } else if (dragState.mode === 'cam-offset') {
          if (raycaster.ray.intersectPlane(_dragPlaneZ0, _hitPoint)) {
            const rawRatio = (_hitPoint.x - dragState.slotX) / CIRCULAR_CAM_MAX_OFFSET;
            const nextOffset = clampCircularCamOffset(rawRatio, 0.09);
            setDragHoverHint(formatCamOffsetDragHint(nextOffset));
            if (Math.abs(nextOffset - (dragState.camOffsetRatio ?? -1)) > 0.005) {
              dragState.camOffsetRatio = nextOffset;
              propsRef.current.onMoveCircularCamOffset(dragState.moduleId, nextOffset);
            }
          }
        } else if (dragState.mode === 'pivot') {
          _dragPlaneYZ.constant = -dragState.slotX;
          if (raycaster.ray.intersectPlane(_dragPlaneYZ, _hitPoint)) {
            const localY = _hitPoint.y - propsRef.current.blueprint.shaftY;
            const localZ = _hitPoint.z;
            const clamped = clampFourBarPivot(localZ, localY);
            propsRef.current.onMoveFourBarPivot(
              dragState.moduleId,
              clamped.pivotZ,
              clamped.pivotY
            );
          }
        }
        return;
      }

      // Throttled hover affordance
      const now = performance.now();
      if (now - lastHoverCheck < 60) return;
      lastHoverCheck = now;

      updateMouseFromPointer(e.clientX, e.clientY);
      raycaster.setFromCamera(_mouse, camera);

      const pivotTargets = moduleRigs.flatMap((r) => r.pivotHitMeshes);
      if (raycaster.intersectObjects(pivotTargets, false).length > 0) {
        renderer.domElement.style.cursor = 'grab';
        return;
      }
      const camOffsetTargets = moduleRigs.flatMap((r) => r.camOffsetHitMeshes ?? []);
      if (raycaster.intersectObjects(camOffsetTargets, false).length > 0) {
        renderer.domElement.style.cursor = 'grab';
        return;
      }
      const modTargets = moduleRigs.flatMap((r) => r.hitMeshes);
      if (raycaster.intersectObjects(modTargets, false).length > 0) {
        renderer.domElement.style.cursor = 'ew-resize';
        return;
      }
      renderer.domElement.style.cursor = 'default';
    }

    function onPointerUp(e: PointerEvent) {
      if (dragState) {
        dragState = null;
        controls.enabled = true;
        ghostRing.visible = false;
        setDragHoverHint(null);
        try {
          renderer.domElement.releasePointerCapture(e.pointerId);
        } catch {
          // ignore if not captured
        }
        requestRender();
      }
    }

    renderer.domElement.addEventListener('pointerdown', onPointerDown);
    renderer.domElement.addEventListener('pointermove', onPointerMove);
    renderer.domElement.addEventListener('pointerup', onPointerUp);

    function onResize() {
      if (!container || disposed) return;
      const w = container.clientWidth || window.innerWidth;
      const h = container.clientHeight || window.innerHeight;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
      applyKinematics(performance.now(), 0);
      renderer.render(scene, camera);
      requestRender();
    }
    window.addEventListener('resize', onResize);

    engineRef.current = {
      rebuildModules,
      applyFinish: () => {
        mats.applyFinish(propsRef.current.blueprint.finish);
        requestRender();
      },
      requestRender,
      triggerCameraPreset: (presetId) => {
        const p = CAMERA_PRESETS[presetId];
        camTransition = {
          pos: new THREE.Vector3(...p.pos),
          target: new THREE.Vector3(...p.target),
        };
        requestRender();
      },
      syncManualAngle: (deg) => {
        const revs = Math.floor(shaftAngleRad / (Math.PI * 2));
        shaftAngleRad = revs * Math.PI * 2 + (deg * Math.PI) / 180;
        requestRender();
      },
      updateSelectionHighlight,
      setGhostRingX: (x) => {
        if (x === null) {
          ghostRing.visible = false;
        } else {
          ghostRing.visible = true;
          ghostRing.position.set(x, propsRef.current.blueprint.shaftY, 0);
        }
        requestRender();
      },
    };

    // Initial synchronous render so first paint is never blank
    applyKinematics(performance.now(), 0);
    renderer.render(scene, camera);
    requestRender();

    return () => {
      disposed = true;
      cancelAnimationFrame(frameId);
      clearTimeout(frameId);
      window.removeEventListener('resize', onResize);
      renderer.domElement.removeEventListener('pointerdown', onPointerDown);
      renderer.domElement.removeEventListener('pointermove', onPointerMove);
      renderer.domElement.removeEventListener('pointerup', onPointerUp);
      renderer.domElement.removeEventListener('webglcontextlost', onContextLost);
      renderer.domElement.removeEventListener('webglcontextrestored', onContextRestored);
      controls.dispose();
      while (moduleRigs.length > 0) {
        moduleRigs.pop()!.dispose();
      }
      boxRig.dispose();
      benchGeo.dispose();
      benchMat.dispose();
      shadowDiscGeo.dispose();
      shadowDiscMat.dispose();
      ghostRingGeo.dispose();
      ghostRingMat.dispose();
      mats.dispose();
      textures.dispose();
      envTex.dispose();
      renderer.dispose();
      if (renderer.domElement.parentNode === container) {
        container.removeChild(renderer.domElement);
      }
    };
  }, []);

  // Rebuild 3D module rigs when module list or structural parameters change
  useEffect(() => {
    engineRef.current?.rebuildModules();
  }, [blueprint.modules, blueprint.id]);

  // Apply material finish when switched
  useEffect(() => {
    engineRef.current?.applyFinish();
  }, [blueprint.finish]);

  // Update selection highlight ring
  useEffect(() => {
    engineRef.current?.updateSelectionHighlight();
  }, [selectedModuleId]);

  // Kick render loop when playback, cutaway, or explode changes
  useEffect(() => {
    engineRef.current?.requestRender();
  }, [isPlaying, rpm, cutawayAmount, explodeAmount]);

  // Sync manual crank scrubber when paused
  useEffect(() => {
    if (!isPlaying) {
      engineRef.current?.syncManualAngle(manualAngleDeg);
    }
  }, [manualAngleDeg, isPlaying]);

  // Trigger camera preset transition
  useEffect(() => {
    if (cameraPresetTrigger > 0) {
      engineRef.current?.triggerCameraPreset(cameraPreset);
    }
  }, [cameraPreset, cameraPresetTrigger]);

  // HTML5 Drag & Drop from Bottom Modular Kit onto 3D Shaft
  const computeDropSlotX = (clientX: number): number => {
    const el = mountRef.current;
    if (!el) return 0;
    const rect = el.getBoundingClientRect();
    const normX = ((clientX - rect.left) / rect.width) * 2 - 1;
    return findNearestFreeSlotX(normX * 2.6, blueprint.modules);
  };

  const handleDragOver = (e: React.DragEvent<HTMLDivElement>) => {
    e.preventDefault();
    e.dataTransfer.dropEffect = 'copy';
    setIsDraggingOver(true);
    const slotX = computeDropSlotX(e.clientX);
    engineRef.current?.setGhostRingX(slotX);
  };

  const handleDragLeave = () => {
    setIsDraggingOver(false);
    engineRef.current?.setGhostRingX(null);
  };

  const handleDrop = (e: React.DragEvent<HTMLDivElement>) => {
    e.preventDefault();
    setIsDraggingOver(false);
    engineRef.current?.setGhostRingX(null);
    const rawType = e.dataTransfer.getData('application/automata-module') as ModuleType;
    if (
      rawType === 'cam-follower' ||
      rawType === 'four-bar-linkage' ||
      rawType === 'bevel-gear' ||
      rawType === 'pin-lantern' ||
      rawType === 'ratchet' ||
      rawType === 'circular-cam' ||
      rawType === 'dual-eccentric' ||
      rawType === 'semi-circular-cam' ||
      rawType === 'scotch-yoke' ||
      rawType === 'bell-crank' ||
      rawType === 'digital-numbers' ||
      rawType === 'writing-cam'
    ) {
      const slotX = computeDropSlotX(e.clientX);
      onDropNewModule(rawType, slotX);
    }
  };

  return (
    <div
      className="relative w-full h-full select-none"
      onDragOver={handleDragOver}
      onDragLeave={handleDragLeave}
      onDrop={handleDrop}
    >
      {/* Dedicated empty Three.js WebGL container */}
      <div
        ref={mountRef}
        data-testid="automata-canvas-mount"
        className="w-full h-full"
      />

      {/* Graceful fallback if WebGL context is lost */}
      {contextLost && (
        <div className="absolute inset-0 flex items-center justify-center bg-[#EDE9E1] text-[#1C1917] p-6">
          <div className="max-w-md text-center space-y-2 border border-[#1C1917]/15 bg-[#FAF8F4] p-6 rounded-xl">
            <h2 className="font-display text-lg font-semibold">3D 渲染上下文暂不可用</h2>
            <p className="text-sm text-[#57534E]">
              浏览器正在恢复 WebGL 图形上下文，请稍候或刷新页面重新初始化 3D 工坊。
            </p>
          </div>
        </div>
      )}

      {/* Subtle contextual drag feedback banner (non-blocking, top-center below header) */}
      {(dragHoverHint || isDraggingOver) && (
        <div
          className="pointer-events-none absolute top-20 left-1/2 -translate-x-1/2 px-4 py-1.5 rounded-lg text-xs font-medium bg-[#1C1917] text-[#FAF8F4] shadow-md"
          style={{ zIndex: 15 }}
        >
          {isDraggingOver ? '松手即可将模块化积木卡入主轴对应槽位' : dragHoverHint}
        </div>
      )}
    </div>
  );
};
