import * as THREE from 'three';
import {
  BOATMAN_KINEMATICS,
  CAM_BEVEL_RADIAL,
  computeFourBarPushrodConfig,
  degToRad,
  sampleCamContour,
  sampleRatchetContour,
  solveBevelGearGeometry,
  solveCircularCam,
  solveCircularCamGeometry,
  solveDualEccentric,
  solveDualEccentricGeometry,
  solveFourBarPushrod,
  solvePinLanternGeometry,
  solveRatchet,
  solveRatchetGeometry,
  solveSemiCircularCam,
  solveSemiCircularCamGeometry,
  solveTopperBoatmanLinkage,
  solveTopperSeagullLinkage,
  solveTopperWoodpeckerLinkage,
} from '../engine/kinematics';
import type {
  AutomataBlueprint,
  AutomataModule,
  BevelGearModule,
  CamFollowerModule,
  CircularCamModule,
  DualEccentricModule,
  FourBarLinkageModule,
  PinLanternModule,
  RatchetModule,
  SemiCircularCamModule,
  TopperType,
} from '../engine/types';
import type { AutomataMaterialSet } from './materials';

export type TopperDriveMode = 'linear' | 'rotary';

export interface TopperRig {
  group: THREE.Group;
  update: (
    drive01: number,
    continuousAngleRad: number,
    pushrodTipAboveDeckWorld: number,
    axialLiftWorld?: number
  ) => void;
}

export interface ModuleRig {
  id: string;
  type: AutomataModule['type'];
  rootGroup: THREE.Group;
  hitMeshes: THREE.Mesh[];
  pivotHitMeshes: THREE.Mesh[];
  camOffsetHitMeshes?: THREE.Mesh[];
  highlightRing: THREE.Mesh;
  update: (
    shaftAngleRad: number,
    blueprint: AutomataBlueprint,
    explodeAmount: number
  ) => void;
  setSelected: (selected: boolean) => void;
  dispose: () => void;
}

export interface AutomataBoxRig {
  group: THREE.Group;
  crankPivot: THREE.Group;
  topDeckGroup: THREE.Group;
  frontWindowMesh: THREE.Mesh;
  leftWallGroup: THREE.Group;
  rightWallGroup: THREE.Group;
  backWallGroup: THREE.Group;
  updateLayout: (explodeAmount: number, cutawayAmount: number) => void;
  dispose: () => void;
}

/**
 * Helper to create a rounded-end linkage bar Shape in XY, extruded and rotated so length runs along +Z.
 */
function createLinkBarGeometry(length: number, width: number, thickness: number): THREE.ExtrudeGeometry {
  const r = width * 0.5;
  const shape = new THREE.Shape();
  shape.absarc(0, 0, r, Math.PI * 0.5, Math.PI * 1.5, false);
  shape.lineTo(length, -r);
  shape.absarc(length, 0, r, -Math.PI * 0.5, Math.PI * 0.5, false);
  shape.lineTo(0, r);

  const geo = new THREE.ExtrudeGeometry(shape, {
    depth: thickness,
    bevelEnabled: true,
    bevelThickness: 0.012,
    bevelSize: 0.012,
    bevelSegments: 2,
    curveSegments: 16,
  });
  geo.translate(0, 0, -thickness * 0.5);
  geo.rotateY(-Math.PI / 2);
  return geo;
}

/**
 * Builds a single 8-vertex trapezoidal bevel tooth prism whose heel and toe
 * both converge mathematically toward the pitch cone apex (0, apexDist, 0).
 */
function createTaperedBevelToothGeometry(
  pitchRadius: number,
  apexDist: number,
  coneAngle: number,
  teethCount: number,
  faceFrac: number
): THREE.BufferGeometry {
  const circularPitch = (2 * Math.PI * pitchRadius) / teethCount;
  const mod = circularPitch / Math.PI;
  const addendum = 0.62 * mod;
  const dedendum = 0.88 * mod;

  const sinC = Math.sin(coneAngle);
  const cosC = Math.cos(coneAngle);

  const rRoot = Math.max(0.035, pitchRadius - dedendum * cosC);
  const yRoot = -dedendum * sinC;
  const rTip = pitchRadius + addendum * cosC;
  const yTip = addendum * sinC;

  // Backlash margin keeps mating teeth cleanly inside valleys without flank clipping
  const halfWRoot = 0.19 * circularPitch;
  const halfWTip = 0.105 * circularPitch;

  const taper = (p: [number, number, number]): [number, number, number] => [
    p[0] * (1 - faceFrac),
    apexDist + (p[1] - apexDist) * (1 - faceFrac),
    p[2] * (1 - faceFrac),
  ];

  // 4 heel vertices (t = 0)
  const hRL: [number, number, number] = [-halfWRoot, yRoot, rRoot];
  const hRR: [number, number, number] = [halfWRoot, yRoot, rRoot];
  const hTR: [number, number, number] = [halfWTip, yTip, rTip];
  const hTL: [number, number, number] = [-halfWTip, yTip, rTip];

  // 4 toe vertices (t = faceFrac)
  const tRL = taper(hRL);
  const tRR = taper(hRR);
  const tTR = taper(hTR);
  const tTL = taper(hTL);

  const verts = [hRL, hRR, hTR, hTL, tRL, tRR, tTR, tTL];
  // Triangulate 6 quad faces with strictly outward-facing CCW winding
  const idx = [
    // Heel face (outward -Y, +Z)
    0, 1, 2, 0, 2, 3,
    // Toe face (outward +Y, -Z toward apex)
    4, 6, 5, 4, 7, 6,
    // Left flank (outward -X)
    0, 3, 7, 0, 7, 4,
    // Right flank (outward +X)
    1, 6, 2, 1, 5, 6,
    // Top land / tip (outward +Y, +Z)
    3, 2, 6, 3, 6, 7,
    // Bottom root (outward -Y, -Z)
    0, 5, 1, 0, 4, 5,
  ];

  const positions = new Float32Array(idx.length * 3);
  for (let i = 0; i < idx.length; i++) {
    const v = verts[idx[i]];
    positions[i * 3] = v[0];
    positions[i * 3 + 1] = v[1];
    positions[i * 3 + 2] = v[2];
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geo.computeVertexNormals();
  return geo;
}

/**
 * Builds a true 3D conical bevel gear whose solid root frustum and tapered involute-like
 * teeth both converge toward the exact shared pitch-cone apex (0, apexDist, 0).
 */
function buildConicalBevelGear(
  pitchRadius: number,
  apexDist: number,
  coneAngle: number,
  teethCount: number,
  faceFrac: number,
  baseAngleRad: number,
  gearMat: THREE.Material,
  hubMat: THREE.Material,
  track: <T extends THREE.BufferGeometry>(g: T) => T,
  moduleId: string
): { group: THREE.Group; hitMesh: THREE.Mesh } {
  const gearGroup = new THREE.Group();

  const circularPitch = (2 * Math.PI * pitchRadius) / teethCount;
  const mod = circularPitch / Math.PI;
  const dedendum = 0.88 * mod;
  const sinC = Math.sin(coneAngle);
  const cosC = Math.cos(coneAngle);

  // Solid root frustum strictly recessed inside pitch cone by dedendum so mating tips clear cleanly
  const coneBackR = Math.max(0.04, pitchRadius - dedendum * cosC);
  const y0 = -dedendum * sinC;
  const coneFrontR = Math.max(0.025, coneBackR * (1 - faceFrac));
  const y1 = y0 + (apexDist - y0) * faceFrac;
  const coneH = Math.max(0.03, y1 - y0);

  const coneGeo = track(
    new THREE.CylinderGeometry(coneFrontR, coneBackR, coneH, Math.max(20, teethCount * 2))
  );
  const coneMesh = new THREE.Mesh(coneGeo, gearMat);
  coneMesh.position.y = (y0 + y1) * 0.5;
  coneMesh.castShadow = true;
  coneMesh.receiveShadow = true;
  coneMesh.userData = { moduleId, dragRole: 'module' };
  gearGroup.add(coneMesh);

  // Brass center boss behind root cone
  const bossR = Math.min(coneBackR * 0.68, Math.max(0.04, pitchRadius * 0.34));
  const bossH = Math.max(0.04, coneH * 0.55);
  const bossGeo = track(new THREE.CylinderGeometry(bossR * 0.9, bossR, bossH, 16));
  const boss = new THREE.Mesh(bossGeo, hubMat);
  boss.position.y = y0 - bossH * 0.48;
  boss.castShadow = true;
  boss.userData = { moduleId, dragRole: 'module' };
  gearGroup.add(boss);

  const toothGeo = track(
    createTaperedBevelToothGeometry(pitchRadius, apexDist, coneAngle, teethCount, faceFrac)
  );
  const step = (Math.PI * 2) / teethCount;
  for (let i = 0; i < teethCount; i++) {
    const tooth = new THREE.Mesh(toothGeo, gearMat);
    tooth.rotation.y = baseAngleRad + i * step;
    tooth.castShadow = true;
    tooth.receiveShadow = true;
    tooth.userData = { moduleId, dragRole: 'module' };
    gearGroup.add(tooth);
  }

  return { group: gearGroup, hitMesh: coneMesh };
}

export function buildAutomataBoxGroup(
  blueprint: AutomataBlueprint,
  mats: AutomataMaterialSet
): AutomataBoxRig {
  const disposables: THREE.BufferGeometry[] = [];
  const track = <T extends THREE.BufferGeometry>(g: T): T => {
    disposables.push(g);
    return g;
  };

  const group = new THREE.Group();
  const { boxWidth: W, boxDepth: D, shaftY, stageDeckY } = blueprint;
  const wallThick = 0.24;
  const deckThickness = 0.18;
  const plinthTopY = 0.28;
  // Side & back walls seat flush from plinth top (0.28) to underside of top deck (stageDeckY - 0.18)
  const wallHeight = Math.max(2.0, stageDeckY - deckThickness - plinthTopY);

  // 1. Base Plinth + Dark Trim Skirt (strictly non-coplanar with workbench floor)
  const plinthGeo = track(new THREE.BoxGeometry(W + 0.55, 0.2, D + 0.55));
  const plinthMesh = new THREE.Mesh(plinthGeo, mats.woodPrimary);
  plinthMesh.position.set(0, 0.18, 0);
  plinthMesh.castShadow = true;
  plinthMesh.receiveShadow = true;
  group.add(plinthMesh);

  const skirtGeo = track(new THREE.BoxGeometry(W + 0.72, 0.08, D + 0.72));
  const skirtMesh = new THREE.Mesh(skirtGeo, mats.woodDarkTrim);
  skirtMesh.position.set(0, 0.045, 0);
  skirtMesh.castShadow = true;
  skirtMesh.receiveShadow = true;
  group.add(skirtMesh);

  // 2. Left & Right Side Uprights with Stepped Brass Bearing Pillow Bushings
  const buildSideWall = (side: -1 | 1) => {
    const wg = new THREE.Group();
    wg.position.set(side * (W * 0.5 - wallThick * 0.5), 0, 0);

    const panelGeo = track(new THREE.BoxGeometry(wallThick, wallHeight + 0.02, D - 0.004));
    const panel = new THREE.Mesh(panelGeo, mats.woodPrimary);
    panel.position.set(0, plinthTopY + wallHeight * 0.5, 0);
    panel.castShadow = true;
    panel.receiveShadow = true;
    wg.add(panel);

    // Outer stepped brass bearing housing around camshaft
    const housingGeo = track(new THREE.CylinderGeometry(0.28, 0.31, 0.07, 24));
    housingGeo.rotateZ(Math.PI / 2);
    const housing = new THREE.Mesh(housingGeo, mats.metalHardware);
    housing.position.set(side * (wallThick * 0.5 + 0.036), shaftY, 0);
    housing.castShadow = true;
    wg.add(housing);

    // Inner brass sleeve bushing
    const sleeveGeo = track(new THREE.CylinderGeometry(0.15, 0.15, 0.06, 20));
    sleeveGeo.rotateZ(Math.PI / 2);
    const sleeve = new THREE.Mesh(sleeveGeo, mats.metalHardware);
    sleeve.position.set(-side * (wallThick * 0.5 + 0.025), shaftY, 0);
    wg.add(sleeve);

    // Corner brass dowel pins
    const pinGeo = track(new THREE.CylinderGeometry(0.045, 0.045, 0.04, 12));
    pinGeo.rotateZ(Math.PI / 2);
    for (const py of [0.55, stageDeckY - 0.42]) {
      for (const pz of [-D * 0.35, D * 0.35]) {
        const pin = new THREE.Mesh(pinGeo, mats.metalHardware);
        pin.position.set(side * (wallThick * 0.5 + 0.021), py, pz);
        wg.add(pin);
      }
    }

    return wg;
  };

  const leftWallGroup = buildSideWall(-1);
  const rightWallGroup = buildSideWall(1);
  group.add(leftWallGroup, rightWallGroup);

  // 3. Back Pegboard Wall with InstancedMesh Brass Grommets & Recessed Bore Cylinders
  const backWallGroup = new THREE.Group();
  backWallGroup.position.set(0, 0, -D * 0.5 + 0.12);
  const backPanelGeo = track(
    new THREE.BoxGeometry(W - wallThick * 2 + 0.02, wallHeight + 0.02, 0.18)
  );
  const backPanel = new THREE.Mesh(backPanelGeo, mats.woodSecondary);
  backPanel.position.set(0, plinthTopY + wallHeight * 0.5, 0);
  backPanel.castShadow = true;
  backPanel.receiveShadow = true;
  backWallGroup.add(backPanel);

  const cols = 9;
  const rows = 4;
  const grommetGeo = track(new THREE.TorusGeometry(0.06, 0.018, 8, 16));
  const grommetInst = new THREE.InstancedMesh(grommetGeo, mats.metalHardware, cols * rows);
  const dummy = new THREE.Object3D();
  let idx = 0;
  for (let c = 0; c < cols; c++) {
    const gx = -2.4 + (c / (cols - 1)) * 4.8;
    for (let r = 0; r < rows; r++) {
      const gy = 0.8 + (r / (rows - 1)) * 2.3;
      dummy.position.set(gx, gy, 0.105);
      dummy.rotation.set(0, 0, 0);
      dummy.updateMatrix();
      grommetInst.setMatrixAt(idx++, dummy.matrix);
    }
  }
  grommetInst.instanceMatrix.needsUpdate = true;
  grommetInst.computeBoundingSphere();
  backWallGroup.add(grommetInst);

  // Use 3D cylinder plugs standing proud of z=0.09 (never coplanar CircleGeometry -> zero Z-fighting)
  const boreGeo = track(new THREE.CylinderGeometry(0.052, 0.052, 0.018, 12));
  boreGeo.rotateX(Math.PI / 2);
  const boreInst = new THREE.InstancedMesh(boreGeo, mats.pegHoleDark, cols * rows);
  idx = 0;
  for (let c = 0; c < cols; c++) {
    const gx = -2.4 + (c / (cols - 1)) * 4.8;
    for (let r = 0; r < rows; r++) {
      const gy = 0.8 + (r / (rows - 1)) * 2.3;
      dummy.position.set(gx, gy, 0.094);
      dummy.rotation.set(0, 0, 0);
      dummy.updateMatrix();
      boreInst.setMatrixAt(idx++, dummy.matrix);
    }
  }
  boreInst.instanceMatrix.needsUpdate = true;
  boreInst.computeBoundingSphere();
  backWallGroup.add(boreInst);
  group.add(backWallGroup);

  // 4. Lower Front Rail & Removable Translucent Display Glass
  const frontRailGeo = track(new THREE.BoxGeometry(W - wallThick * 2 + 0.02, 0.23, 0.2));
  const frontRail = new THREE.Mesh(frontRailGeo, mats.woodDarkTrim);
  frontRail.position.set(0, 0.385, D * 0.5 - 0.1);
  frontRail.castShadow = true;
  frontRail.receiveShadow = true;
  group.add(frontRail);

  const windowGeo = track(new THREE.PlaneGeometry(W - wallThick * 2.2, wallHeight - 0.35));
  const frontWindowMesh = new THREE.Mesh(windowGeo, mats.cutawayGlass);
  frontWindowMesh.position.set(0, 0.38 + (wallHeight - 0.1) * 0.5, D * 0.5 - 0.06);
  frontWindowMesh.renderOrder = 5;
  group.add(frontWindowMesh);

  // 5. Top Stage Deck with Stepped Molding (strictly separated Y levels -> ZERO Z-fighting)
  const topDeckGroup = new THREE.Group();
  topDeckGroup.position.set(0, stageDeckY, 0);

  // Upper main board spans [-0.18, 0.000]
  const deckBoardGeo = track(new THREE.BoxGeometry(W + 0.36, deckThickness, D + 0.36));
  const deckBoard = new THREE.Mesh(deckBoardGeo, mats.woodPrimary);
  deckBoard.position.set(0, -deckThickness * 0.5, 0);
  deckBoard.castShadow = true;
  deckBoard.receiveShadow = true;
  topDeckGroup.add(deckBoard);

  // Lower outer molding apron spans [-0.14, -0.06], cleanly clear of both top (0.00) and bottom (-0.18)!
  const deckLipGeo = track(new THREE.BoxGeometry(W + 0.52, 0.08, D + 0.52));
  const deckLip = new THREE.Mesh(deckLipGeo, mats.woodDarkTrim);
  deckLip.position.set(0, -0.1, 0);
  deckLip.castShadow = true;
  deckLip.receiveShadow = true;
  topDeckGroup.add(deckLip);

  // Raised guide rail strip spans [-0.004, +0.028]
  const slotStripGeo = track(new THREE.BoxGeometry(5.6, 0.032, 0.2));
  const slotStrip = new THREE.Mesh(slotStripGeo, mats.woodDarkTrim);
  slotStrip.position.set(0, 0.012, 0);
  slotStrip.receiveShadow = true;
  topDeckGroup.add(slotStrip);
  group.add(topDeckGroup);

  // 6. Main Steel Camshaft & Turned Hand Crank Assembly
  const crankPivot = new THREE.Group();
  crankPivot.position.set(0, shaftY, 0);

  const shaftLength = W + 1.0;
  const shaftGeo = track(new THREE.CylinderGeometry(0.085, 0.085, shaftLength, 20));
  shaftGeo.rotateZ(Math.PI / 2);
  const shaftMesh = new THREE.Mesh(shaftGeo, mats.steelShaft);
  shaftMesh.position.set(0.18, 0, 0);
  shaftMesh.castShadow = true;
  crankPivot.add(shaftMesh);

  const endCapGeo = track(new THREE.CylinderGeometry(0.12, 0.12, 0.12, 16));
  endCapGeo.rotateZ(Math.PI / 2);
  const endCap = new THREE.Mesh(endCapGeo, mats.metalHardware);
  endCap.position.set(-W * 0.5 - 0.18, 0, 0);
  crankPivot.add(endCap);

  const crankX = W * 0.5 + 0.52;
  const crankArmGeo = track(createLinkBarGeometry(0.95, 0.24, 0.1));
  const crankArm = new THREE.Mesh(crankArmGeo, mats.metalHardware);
  crankArm.position.set(crankX, 0, 0);
  crankArm.castShadow = true;
  crankPivot.add(crankArm);

  const handlePts = [
    new THREE.Vector2(0.0, 0.0),
    new THREE.Vector2(0.08, 0.0),
    new THREE.Vector2(0.11, 0.15),
    new THREE.Vector2(0.145, 0.42),
    new THREE.Vector2(0.13, 0.68),
    new THREE.Vector2(0.09, 0.78),
    new THREE.Vector2(0.0, 0.8),
  ];
  const handleGeo = track(new THREE.LatheGeometry(handlePts, 20));
  handleGeo.rotateZ(-Math.PI / 2);
  const handleMesh = new THREE.Mesh(handleGeo, mats.woodDarkTrim);
  handleMesh.position.set(crankX + 0.06, 0, 0.95);
  handleMesh.castShadow = true;
  crankPivot.add(handleMesh);

  const crankPinGeo = track(new THREE.CylinderGeometry(0.045, 0.045, 0.92, 12));
  crankPinGeo.rotateZ(Math.PI / 2);
  const crankPin = new THREE.Mesh(crankPinGeo, mats.metalHardware);
  crankPin.position.set(crankX + 0.45, 0, 0.95);
  crankPivot.add(crankPin);

  group.add(crankPivot);

  function updateLayout(explodeAmount: number, cutawayAmount: number) {
    topDeckGroup.position.y = stageDeckY + explodeAmount * 1.15;
    leftWallGroup.position.x = -(W * 0.5 - wallThick * 0.5) - explodeAmount * 0.85;
    rightWallGroup.position.x = (W * 0.5 - wallThick * 0.5) + explodeAmount * 0.85;
    backWallGroup.position.z = (-D * 0.5 + 0.12) - explodeAmount * 0.95;
    frontWindowMesh.position.z = (D * 0.5 - 0.06) + explodeAmount * 0.9;
    mats.setCutawayOpacity(cutawayAmount);
  }

  return {
    group,
    crankPivot,
    topDeckGroup,
    frontWindowMesh,
    leftWallGroup,
    rightWallGroup,
    backWallGroup,
    updateLayout,
    dispose: () => {
      for (const g of disposables) g.dispose();
    },
  };
}

/**
 * Helper to orient a cylinder/link bar between two 3D points pA and pB.
 */
function alignBarBetweenPoints(
  barMesh: THREE.Object3D,
  pA: THREE.Vector3,
  pB: THREE.Vector3,
  localAxis: 'y' | 'z' = 'y'
) {
  barMesh.position.copy(pA).lerp(pB, 0.5);
  const dir = new THREE.Vector3().subVectors(pB, pA);
  const len = dir.length();
  if (len > 1e-4) {
    dir.normalize();
    const baseVec = localAxis === 'y' ? new THREE.Vector3(0, 1, 0) : new THREE.Vector3(0, 0, 1);
    barMesh.quaternion.setFromUnitVectors(baseVec, dir);
  }
}

/**
 * Builds composite articulated wooden automata toppers with real mechanical linkages
 * physically driven by either a vertical pushrod tip ('linear') or spinning spindle ('rotary').
 */
export function buildTopperGroup(
  type: TopperType,
  scale: number,
  mats: AutomataMaterialSet,
  track: <T extends THREE.BufferGeometry>(g: T) => T,
  driveMode: TopperDriveMode = 'linear'
): TopperRig {
  const group = new THREE.Group();
  group.scale.setScalar(scale);

  // Optional rotary-to-linear brass crank converter when a reciprocating topper sits on a bevel gear
  let rotaryCrankWheel: THREE.Group | null = null;
  let rotaryLinkBar: THREE.Mesh | null = null;
  let rotaryTopPin: THREE.Mesh | null = null;
  const rotaryCrankCenterY = 0.16;
  const rotaryCrankR = 0.12;
  const rotaryRodLen = 0.24;

  const ensureRotaryConverter = () => {
    if (driveMode !== 'rotary') return;
    const baseColGeo = track(new THREE.CylinderGeometry(0.11, 0.13, 0.08, 16));
    const baseCol = new THREE.Mesh(baseColGeo, mats.woodDarkTrim);
    baseCol.position.set(0, 0.08, 0);
    group.add(baseCol);

    rotaryCrankWheel = new THREE.Group();
    rotaryCrankWheel.position.set(0, rotaryCrankCenterY, 0);
    const stemGeo = track(new THREE.CylinderGeometry(0.044, 0.044, 0.24, 12));
    const stem = new THREE.Mesh(stemGeo, mats.steelShaft);
    stem.position.set(0, -0.12, 0);
    rotaryCrankWheel.add(stem);

    const wheelGeo = track(new THREE.CylinderGeometry(0.16, 0.16, 0.05, 18));
    const wheel = new THREE.Mesh(wheelGeo, mats.metalHardware);
    wheel.castShadow = true;
    rotaryCrankWheel.add(wheel);

    const crankPinGeo = track(new THREE.CylinderGeometry(0.025, 0.025, 0.07, 10));
    const pin = new THREE.Mesh(crankPinGeo, mats.steelShaft);
    pin.position.set(rotaryCrankR, 0.04, 0);
    rotaryCrankWheel.add(pin);
    group.add(rotaryCrankWheel);

    const rodGeo = track(new THREE.CylinderGeometry(0.018, 0.018, rotaryRodLen, 10));
    rotaryLinkBar = new THREE.Mesh(rodGeo, mats.steelShaft);
    rotaryLinkBar.castShadow = true;
    group.add(rotaryLinkBar);

    const topPinGeo = track(new THREE.CylinderGeometry(0.032, 0.032, 0.07, 10));
    topPinGeo.rotateZ(Math.PI / 2);
    rotaryTopPin = new THREE.Mesh(topPinGeo, mats.metalHardware);
    group.add(rotaryTopPin);
  };

  let currentAxialLiftLocal = 0;

  const computeEffectivePinY = (
    drive01: number,
    continuousAngleRad: number,
    pushrodTipAboveDeckWorld: number
  ): number => {
    if (driveMode === 'rotary' && rotaryCrankWheel && rotaryLinkBar && rotaryTopPin) {
      const wheelLift = Math.min(0.2, Math.max(0, currentAxialLiftLocal));
      const wheelY = rotaryCrankCenterY + wheelLift;
      rotaryCrankWheel.position.y = wheelY;
      rotaryCrankWheel.rotation.y = -continuousAngleRad;
      const px = rotaryCrankR * Math.cos(continuousAngleRad);
      const pz = -rotaryCrankR * Math.sin(continuousAngleRad);
      const horiz = Math.hypot(px, pz);
      const dy = Math.sqrt(Math.max(0.01, rotaryRodLen * rotaryRodLen - horiz * horiz));
      const pinY = wheelY + 0.04 + dy;
      alignBarBetweenPoints(
        rotaryLinkBar,
        new THREE.Vector3(px, wheelY + 0.04, pz),
        new THREE.Vector3(0, pinY, 0),
        'y'
      );
      rotaryTopPin.position.set(0, pinY, 0);
      return pinY;
    }
    if (pushrodTipAboveDeckWorld > 0.01) {
      return Math.max(0.11, Math.min(0.75, pushrodTipAboveDeckWorld / Math.max(0.5, scale)));
    }
    return 0.12 + drive01 * 0.44;
  };

  if (type === 'none') {
    if (driveMode === 'rotary') {
      const hubGeo = track(new THREE.CylinderGeometry(0.14, 0.16, 0.08, 18));
      const hub = new THREE.Mesh(hubGeo, mats.metalHardware);
      hub.position.set(0, 0.08, 0);
      hub.castShadow = true;
      group.add(hub);

      const stemGeo = track(new THREE.CylinderGeometry(0.044, 0.044, 0.24, 12));
      const stem = new THREE.Mesh(stemGeo, mats.steelShaft);
      stem.position.set(0, -0.14, 0);
      hub.add(stem);

      const vaneGeo = track(new THREE.BoxGeometry(0.56, 0.05, 0.09));
      const vane = new THREE.Mesh(vaneGeo, mats.paintedAccent);
      vane.position.set(0, 0.13, 0);
      vane.castShadow = true;
      hub.add(vane);

      return {
        group,
        update: (_d, continuousAngleRad, _tipWorldY, axialLiftWorld = 0) => {
          const liftLocal = Math.max(0, axialLiftWorld) / Math.max(0.5, scale);
          hub.position.y = 0.08 + liftLocal;
          hub.rotation.y = -continuousAngleRad;
        },
      };
    }

    const capGroup = new THREE.Group();
    const capGeo = track(new THREE.CylinderGeometry(0.13, 0.11, 0.08, 18));
    const cap = new THREE.Mesh(capGeo, mats.metalHardware);
    cap.castShadow = true;
    capGroup.add(cap);

    const needleGeo = track(new THREE.BoxGeometry(0.28, 0.025, 0.04));
    const needle = new THREE.Mesh(needleGeo, mats.paintedAccent);
    needle.position.set(0.12, 0.045, 0);
    capGroup.add(needle);
    group.add(capGroup);

    return {
      group,
      update: (drive, _ang, tipWorldY) => {
        const pinY = tipWorldY > 0.01 ? tipWorldY / scale : 0.14 + drive * 0.45;
        capGroup.position.y = pinY;
      },
    };
  }

  if (type === 'propeller') {
    // Turned wooden rosette base & brass thrust bushing on stage deck
    const baseGeo = track(new THREE.CylinderGeometry(0.22, 0.26, 0.07, 20));
    const base = new THREE.Mesh(baseGeo, mats.woodDarkTrim);
    base.position.set(0, 0.095, 0);
    base.castShadow = true;
    base.receiveShadow = true;
    group.add(base);

    const bushingGeo = track(new THREE.CylinderGeometry(0.11, 0.135, 0.09, 16));
    const bushing = new THREE.Mesh(bushingGeo, mats.metalHardware);
    bushing.position.set(0, 0.17, 0);
    bushing.castShadow = true;
    group.add(bushing);

    // Vertical steel rotor mast rising from deck bushing to cross propeller hub
    const mastGeo = track(new THREE.CylinderGeometry(0.044, 0.044, 0.84, 14));
    const mast = new THREE.Mesh(mastGeo, mats.steelShaft);
    mast.position.set(0, 0.22, 0);
    mast.castShadow = true;
    group.add(mast);

    // Cross-Shaped 4-Blade Propeller Assembly (十字型螺旋桨)
    const propPivot = new THREE.Group();
    propPivot.position.set(0, 0.64, 0);
    group.add(propPivot);

    const hubGeo = track(new THREE.CylinderGeometry(0.092, 0.104, 0.115, 18));
    const hub = new THREE.Mesh(hubGeo, mats.metalHardware);
    hub.castShadow = true;
    propPivot.add(hub);

    const noseGeo = track(new THREE.SphereGeometry(0.092, 16, 12, 0, Math.PI * 2, 0, Math.PI * 0.5));
    const nose = new THREE.Mesh(noseGeo, mats.metalHardware);
    nose.position.set(0, 0.058, 0);
    nose.castShadow = true;
    propPivot.add(nose);

    const washerGeo = track(new THREE.CylinderGeometry(0.12, 0.12, 0.028, 18));
    const washer = new THREE.Mesh(washerGeo, mats.woodDarkTrim);
    washer.position.set(0, -0.062, 0);
    propPivot.add(washer);

    // Sculpted wide-chord airfoil blade profile in XZ extruded upward
    const bladeShape = new THREE.Shape();
    bladeShape.moveTo(0.11, -0.032);
    bladeShape.quadraticCurveTo(0.28, -0.088, 0.48, -0.076);
    bladeShape.quadraticCurveTo(0.56, -0.048, 0.56, 0);
    bladeShape.quadraticCurveTo(0.56, 0.048, 0.48, 0.076);
    bladeShape.quadraticCurveTo(0.28, 0.088, 0.11, 0.032);
    bladeShape.closePath();

    const bladeGeo = track(
      new THREE.ExtrudeGeometry(bladeShape, {
        depth: 0.026,
        bevelEnabled: true,
        bevelThickness: 0.005,
        bevelSize: 0.005,
        bevelSegments: 2,
      })
    );
    bladeGeo.translate(0, 0, -0.013);
    bladeGeo.rotateX(Math.PI / 2);

    const ferruleGeo = track(new THREE.CylinderGeometry(0.034, 0.04, 0.09, 12));
    ferruleGeo.rotateZ(Math.PI / 2);
    const sparGeo = track(new THREE.BoxGeometry(0.39, 0.034, 0.028));
    const tipStripeGeo = track(new THREE.BoxGeometry(0.08, 0.032, 0.136));

    for (let i = 0; i < 4; i++) {
      const armGroup = new THREE.Group();
      armGroup.rotation.y = (i * Math.PI) / 2;

      const ferrule = new THREE.Mesh(ferruleGeo, mats.metalHardware);
      ferrule.position.set(0.105, 0, 0);
      ferrule.castShadow = true;
      armGroup.add(ferrule);

      const pitchedBlade = new THREE.Group();
      pitchedBlade.rotation.x = 0.38; // ~22 deg aerodynamic pitch so wide blade face catches light from side/front

      const bladeMesh = new THREE.Mesh(bladeGeo, mats.woodDarkTrim);
      bladeMesh.castShadow = true;
      bladeMesh.receiveShadow = true;
      pitchedBlade.add(bladeMesh);

      const spar = new THREE.Mesh(sparGeo, mats.metalHardware);
      spar.position.set(0.305, 0, 0);
      spar.castShadow = true;
      pitchedBlade.add(spar);

      const tipStripe = new THREE.Mesh(tipStripeGeo, mats.paintedAccent);
      tipStripe.position.set(0.475, 0, 0);
      tipStripe.castShadow = true;
      pitchedBlade.add(tipStripe);

      armGroup.add(pitchedBlade);
      propPivot.add(armGroup);
    }

    return {
      group,
      update: (drive, continuousAngleRad, tipWorldY, axialLiftWorld = 0) => {
        if (driveMode === 'rotary') {
          const liftLocal = Math.max(0, axialLiftWorld) / Math.max(0.5, scale);
          mast.position.y = 0.22 + liftLocal;
          propPivot.position.y = 0.64 + liftLocal;
          mast.rotation.y = -continuousAngleRad;
          propPivot.rotation.y = -continuousAngleRad;
        } else {
          const pinY = computeEffectivePinY(drive, continuousAngleRad, tipWorldY);
          const lift = Math.max(0, pinY - 0.12);
          mast.position.y = 0.22 + lift * 0.35;
          propPivot.position.y = 0.64 + lift * 0.55;
          propPivot.rotation.y = drive * Math.PI * 1.5;
        }
      },
    };
  }

  if (type === 'arrow') {
    // Turned wooden rosette base & brass thrust bushing on stage deck
    const baseGeo = track(new THREE.CylinderGeometry(0.22, 0.26, 0.07, 20));
    const base = new THREE.Mesh(baseGeo, mats.woodDarkTrim);
    base.position.set(0, 0.095, 0);
    base.castShadow = true;
    base.receiveShadow = true;
    group.add(base);

    const bushingGeo = track(new THREE.CylinderGeometry(0.11, 0.135, 0.09, 16));
    const bushing = new THREE.Mesh(bushingGeo, mats.metalHardware);
    bushing.position.set(0, 0.17, 0);
    bushing.castShadow = true;
    group.add(bushing);

    // 4 brass compass index studs around stationary rosette dial base
    const dialStudGeo = track(new THREE.CylinderGeometry(0.018, 0.018, 0.02, 10));
    for (let i = 0; i < 4; i++) {
      const a = (i * Math.PI) / 2;
      const stud = new THREE.Mesh(dialStudGeo, i === 0 ? mats.paintedAccent : mats.metalHardware);
      stud.position.set(0.185 * Math.cos(a), 0.132, 0.185 * Math.sin(a));
      group.add(stud);
    }

    // Vertical steel spindle mast rising from deck bushing to directional arrow turret
    const mastGeo = track(new THREE.CylinderGeometry(0.044, 0.044, 0.82, 14));
    const mast = new THREE.Mesh(mastGeo, mats.steelShaft);
    mast.position.set(0, 0.21, 0);
    mast.castShadow = true;
    group.add(mast);

    // Sculpted Directional Wooden & Brass Arrow Assembly (指示箭头)
    const arrowPivot = new THREE.Group();
    arrowPivot.position.set(0, 0.62, 0);
    group.add(arrowPivot);

    const hubGeo = track(new THREE.CylinderGeometry(0.092, 0.104, 0.11, 18));
    const hub = new THREE.Mesh(hubGeo, mats.metalHardware);
    hub.castShadow = true;
    arrowPivot.add(hub);

    const noseGeo = track(new THREE.SphereGeometry(0.092, 16, 12, 0, Math.PI * 2, 0, Math.PI * 0.5));
    const nose = new THREE.Mesh(noseGeo, mats.metalHardware);
    nose.position.set(0, 0.055, 0);
    nose.castShadow = true;
    arrowPivot.add(nose);

    const washerGeo = track(new THREE.CylinderGeometry(0.118, 0.118, 0.026, 18));
    const washer = new THREE.Mesh(washerGeo, mats.woodDarkTrim);
    washer.position.set(0, -0.058, 0);
    arrowPivot.add(washer);

    // Main horizontal arrow shaft beam
    const shaftBeamGeo = track(new THREE.BoxGeometry(0.7, 0.048, 0.076));
    const shaftBeam = new THREE.Mesh(shaftBeamGeo, mats.woodPrimary);
    shaftBeam.position.set(-0.07, 0, 0);
    shaftBeam.castShadow = true;
    shaftBeam.receiveShadow = true;
    arrowPivot.add(shaftBeam);

    // Brass neck ferrule ahead of arrowhead socket
    const neckCollarGeo = track(new THREE.BoxGeometry(0.055, 0.058, 0.096));
    const neckCollar = new THREE.Mesh(neckCollarGeo, mats.metalHardware);
    neckCollar.position.set(0.23, 0, 0);
    neckCollar.castShadow = true;
    arrowPivot.add(neckCollar);

    // Broad sculpted barbed arrowhead pointing along +X
    const headShape = new THREE.Shape();
    headShape.moveTo(0.64, 0);
    headShape.lineTo(0.19, 0.195);
    headShape.lineTo(0.255, 0.055);
    headShape.lineTo(0.255, -0.055);
    headShape.lineTo(0.19, -0.195);
    headShape.closePath();

    const headGeo = track(
      new THREE.ExtrudeGeometry(headShape, {
        depth: 0.054,
        bevelEnabled: true,
        bevelThickness: 0.007,
        bevelSize: 0.007,
        bevelSegments: 2,
      })
    );
    headGeo.translate(0, 0, -0.027);
    headGeo.rotateX(Math.PI / 2);
    const headMesh = new THREE.Mesh(headGeo, mats.paintedAccent);
    headMesh.castShadow = true;
    headMesh.receiveShadow = true;
    arrowPivot.add(headMesh);

    // Notched fletching tail vane at -X
    const tailShape = new THREE.Shape();
    tailShape.moveTo(-0.22, 0.045);
    tailShape.lineTo(-0.46, 0.145);
    tailShape.lineTo(-0.54, 0.125);
    tailShape.lineTo(-0.42, 0);
    tailShape.lineTo(-0.54, -0.125);
    tailShape.lineTo(-0.46, -0.145);
    tailShape.lineTo(-0.22, -0.045);
    tailShape.closePath();

    const tailFinGeo = track(
      new THREE.ExtrudeGeometry(tailShape, {
        depth: 0.036,
        bevelEnabled: true,
        bevelThickness: 0.005,
        bevelSize: 0.005,
        bevelSegments: 2,
      })
    );
    tailFinGeo.translate(0, 0, -0.018);
    tailFinGeo.rotateX(Math.PI / 2);
    const tailFin = new THREE.Mesh(tailFinGeo, mats.woodDarkTrim);
    tailFin.castShadow = true;
    tailFin.receiveShadow = true;
    arrowPivot.add(tailFin);

    // Vertical dorsal tail crest so arrow reads crisply from side elevations too
    const dorsalCrestGeo = track(new THREE.BoxGeometry(0.22, 0.092, 0.026));
    const dorsalCrest = new THREE.Mesh(dorsalCrestGeo, mats.paintedSecondary);
    dorsalCrest.position.set(-0.35, 0.054, 0);
    dorsalCrest.castShadow = true;
    arrowPivot.add(dorsalCrest);

    return {
      group,
      update: (drive, continuousAngleRad, tipWorldY, axialLiftWorld = 0) => {
        if (driveMode === 'rotary') {
          const liftLocal = Math.max(0, axialLiftWorld) / Math.max(0.5, scale);
          mast.position.y = 0.21 + liftLocal;
          arrowPivot.position.y = 0.62 + liftLocal;
          mast.rotation.y = -continuousAngleRad;
          arrowPivot.rotation.y = -continuousAngleRad;
        } else {
          const pinY = computeEffectivePinY(drive, continuousAngleRad, tipWorldY);
          const lift = Math.max(0, pinY - 0.12);
          mast.position.y = 0.21 + lift * 0.35;
          arrowPivot.position.y = 0.62 + lift * 0.55;
          arrowPivot.rotation.y = drive * Math.PI * 0.75;
        }
      },
    };
  }

  if (type === 'seagull') {
    // Stationary Gull on Rear Perch; Pushrod -> T-Crosshead Yoke -> Twin Brass Wing Links
    const perchBaseGeo = track(new THREE.BoxGeometry(0.54, 0.06, 0.34));
    const perchBase = new THREE.Mesh(perchBaseGeo, mats.woodDarkTrim);
    perchBase.position.set(0, 0.09, -0.24);
    perchBase.castShadow = true;
    group.add(perchBase);

    const perchPostGeo = track(new THREE.CylinderGeometry(0.048, 0.058, 0.72, 12));
    const perchPost = new THREE.Mesh(perchPostGeo, mats.woodDarkTrim);
    perchPost.position.set(0, 0.45, -0.26);
    perchPost.castShadow = true;
    group.add(perchPost);

    const perchArmGeo = track(new THREE.BoxGeometry(0.08, 0.07, 0.3));
    const perchArm = new THREE.Mesh(perchArmGeo, mats.woodDarkTrim);
    perchArm.position.set(0, 0.8, -0.14);
    perchArm.castShadow = true;
    group.add(perchArm);

    const birdBodyGroup = new THREE.Group();
    birdBodyGroup.position.set(0, 0.9, 0.02);
    group.add(birdBodyGroup);

    const bodyGeo = track(new THREE.SphereGeometry(0.21, 18, 14));
    bodyGeo.scale(0.84, 0.72, 1.5);
    const body = new THREE.Mesh(bodyGeo, mats.paintedIvory);
    body.castShadow = true;
    birdBodyGroup.add(body);

    const headGeo = track(new THREE.SphereGeometry(0.125, 16, 12));
    const head = new THREE.Mesh(headGeo, mats.paintedIvory);
    head.position.set(0, 0.13, 0.28);
    head.castShadow = true;
    birdBodyGroup.add(head);

    const beakGeo = track(new THREE.ConeGeometry(0.045, 0.19, 10));
    beakGeo.rotateX(Math.PI / 2);
    const beak = new THREE.Mesh(beakGeo, mats.paintedAccent);
    beak.position.set(0, 0.12, 0.44);
    beak.castShadow = true;
    birdBodyGroup.add(beak);

    const tailGeo = track(new THREE.BoxGeometry(0.22, 0.04, 0.26));
    const tail = new THREE.Mesh(tailGeo, mats.paintedSecondary);
    tail.position.set(0, 0.04, -0.34);
    tail.rotation.x = 0.24;
    tail.castShadow = true;
    birdBodyGroup.add(tail);

    const wingShape = new THREE.Shape();
    wingShape.moveTo(0, 0);
    wingShape.lineTo(0.55, 0.08);
    wingShape.lineTo(0.86, -0.02);
    wingShape.lineTo(0.74, -0.22);
    wingShape.lineTo(0.22, -0.2);
    wingShape.closePath();

    const wingGeo = track(
      new THREE.ExtrudeGeometry(wingShape, {
        depth: 0.042,
        bevelEnabled: true,
        bevelThickness: 0.008,
        bevelSize: 0.008,
        bevelSegments: 1,
      })
    );
    wingGeo.rotateX(-Math.PI / 2);

    const wingPivotX = 0.14;
    const wingPivotY = 0.96;
    const hornRadius = 0.3;
    const wingLinkLen = 0.6;
    const crossheadX = 0.18;

    const buildWing = (side: -1 | 1) => {
      const wp = new THREE.Group();
      wp.position.set(side * wingPivotX, wingPivotY, 0.04);
      const wm = new THREE.Mesh(wingGeo, mats.paintedSecondary);
      if (side === -1) wm.scale.x = -1;
      wm.castShadow = true;
      wp.add(wm);

      // Visible brass hinge barrel at shoulder
      const hingeGeo = track(new THREE.CylinderGeometry(0.032, 0.032, 0.09, 10));
      hingeGeo.rotateX(Math.PI / 2);
      const hinge = new THREE.Mesh(hingeGeo, mats.metalHardware);
      wp.add(hinge);

      // Under-wing control horn bracket + pin at hornRadius
      const hornGeo = track(new THREE.BoxGeometry(hornRadius, 0.03, 0.03));
      const horn = new THREE.Mesh(hornGeo, mats.metalHardware);
      horn.position.set(side * hornRadius * 0.5, -0.02, 0);
      wp.add(horn);

      const hornPinGeo = track(new THREE.CylinderGeometry(0.025, 0.025, 0.06, 10));
      hornPinGeo.rotateX(Math.PI / 2);
      const hornPin = new THREE.Mesh(hornPinGeo, mats.metalHardware);
      hornPin.position.set(side * hornRadius, -0.02, 0);
      wp.add(hornPin);

      group.add(wp);
      return wp;
    };

    const leftWingPivot = buildWing(-1);
    const rightWingPivot = buildWing(1);

    // Sliding Brass T-Crosshead Yoke fixed to pushrod top
    const yokeGroup = new THREE.Group();
    const yokeBarGeo = track(new THREE.CylinderGeometry(0.026, 0.026, crossheadX * 2 + 0.04, 12));
    yokeBarGeo.rotateZ(Math.PI / 2);
    const yokeBar = new THREE.Mesh(yokeBarGeo, mats.metalHardware);
    yokeBar.castShadow = true;
    yokeGroup.add(yokeBar);

    const yokeHubGeo = track(new THREE.CylinderGeometry(0.055, 0.055, 0.08, 12));
    const yokeHub = new THREE.Mesh(yokeHubGeo, mats.metalHardware);
    yokeGroup.add(yokeHub);
    group.add(yokeGroup);

    // Twin Brass Wing Push-Links connecting Crosshead Pins <-> Wing Horn Pins
    const linkRodGeo = track(new THREE.CylinderGeometry(0.018, 0.018, wingLinkLen, 10));
    const leftWingLink = new THREE.Mesh(linkRodGeo, mats.metalHardware);
    const rightWingLink = new THREE.Mesh(linkRodGeo, mats.metalHardware);
    leftWingLink.castShadow = true;
    rightWingLink.castShadow = true;
    group.add(leftWingLink, rightWingLink);

    ensureRotaryConverter();

    return {
      group,
      update: (drive, continuousAngleRad, tipWorldY, axialLiftWorld = 0) => {
        currentAxialLiftLocal = Math.max(0, axialLiftWorld) / Math.max(0.5, scale);
        const pinY = computeEffectivePinY(drive, continuousAngleRad, tipWorldY);
        const sol = solveTopperSeagullLinkage(
          pinY,
          wingPivotX,
          wingPivotY,
          crossheadX,
          hornRadius,
          wingLinkLen
        );

        yokeGroup.position.set(0, pinY, 0.02);
        rightWingPivot.rotation.z = sol.wingAngleRad;
        leftWingPivot.rotation.z = -sol.wingAngleRad;

        alignBarBetweenPoints(
          rightWingLink,
          new THREE.Vector3(crossheadX, pinY, 0.04),
          new THREE.Vector3(sol.hornX, sol.hornY, 0.04),
          'y'
        );
        alignBarBetweenPoints(
          leftWingLink,
          new THREE.Vector3(-crossheadX, pinY, 0.04),
          new THREE.Vector3(-sol.hornX, sol.hornY, 0.04),
          'y'
        );
      },
    };
  }

  if (type === 'woodpecker') {
    // Side-profile Woodpecker (+X tree trunk, -X fulcrum) so entire belly-crank linkage & beak strike are clearly visible
    const baseGeo = track(new THREE.BoxGeometry(0.86, 0.06, 0.52));
    const base = new THREE.Mesh(baseGeo, mats.woodPrimary);
    base.position.set(0.06, 0.09, 0);
    base.castShadow = true;
    group.add(base);

    const wpSideGroup = new THREE.Group();
    wpSideGroup.rotation.y = Math.PI / 2; // local +Z maps to world +X
    group.add(wpSideGroup);

    const trunkGeo = track(new THREE.CylinderGeometry(0.11, 0.145, 1.15, 12));
    const trunk = new THREE.Mesh(trunkGeo, mats.woodDarkTrim);
    trunk.position.set(0, 0.68, 0.34);
    trunk.castShadow = true;
    trunk.receiveShadow = true;
    wpSideGroup.add(trunk);

    const fulcrumZ = -0.22;
    const fulcrumY = 0.72;
    const fulcrumPostGeo = track(new THREE.CylinderGeometry(0.045, 0.055, 0.62, 12));
    const fulcrumPost = new THREE.Mesh(fulcrumPostGeo, mats.metalHardware);
    fulcrumPost.position.set(0, 0.41, fulcrumZ);
    fulcrumPost.castShadow = true;
    wpSideGroup.add(fulcrumPost);

    const fulcrumPinGeo = track(new THREE.CylinderGeometry(0.036, 0.036, 0.16, 12));
    fulcrumPinGeo.rotateZ(Math.PI / 2);
    const fulcrumPin = new THREE.Mesh(fulcrumPinGeo, mats.metalHardware);
    fulcrumPin.position.set(0, fulcrumY, fulcrumZ);
    wpSideGroup.add(fulcrumPin);

    const birdPivot = new THREE.Group();
    birdPivot.position.set(0, fulcrumY, fulcrumZ);
    wpSideGroup.add(birdPivot);

    const torsoGeo = track(new THREE.CapsuleGeometry(0.115, 0.27, 6, 12));
    const torso = new THREE.Mesh(torsoGeo, mats.paintedSecondary);
    torso.position.set(0, 0.14, 0.05);
    torso.rotation.x = 0.35;
    torso.castShadow = true;
    birdPivot.add(torso);

    const chestGeo = track(new THREE.SphereGeometry(0.1, 12, 10));
    const chest = new THREE.Mesh(chestGeo, mats.paintedIvory);
    chest.position.set(0, 0.16, 0.13);
    birdPivot.add(chest);

    const headGeo = track(new THREE.SphereGeometry(0.098, 14, 12));
    const head = new THREE.Mesh(headGeo, mats.paintedIvory);
    head.position.set(0, 0.35, 0.16);
    head.castShadow = true;
    birdPivot.add(head);

    const crestGeo = track(new THREE.ConeGeometry(0.055, 0.15, 8));
    const crest = new THREE.Mesh(crestGeo, mats.paintedAccent);
    crest.position.set(0, 0.44, 0.09);
    crest.rotation.x = -0.5;
    birdPivot.add(crest);

    const beakGeo = track(new THREE.ConeGeometry(0.03, 0.21, 8));
    beakGeo.rotateX(Math.PI / 2);
    const beak = new THREE.Mesh(beakGeo, mats.metalHardware);
    beak.position.set(0, 0.35, 0.31);
    beak.castShadow = true;
    birdPivot.add(beak);

    // Rigid belly crank lever arm extending forward-downward to pin at radius 0.27
    const leverLen = 0.27;
    const linkLen = 0.39;
    const leverGeo = track(createLinkBarGeometry(leverLen, 0.075, 0.04));
    const leverMesh = new THREE.Mesh(leverGeo, mats.metalHardware);
    leverMesh.rotation.x = 0.62;
    leverMesh.castShadow = true;
    birdPivot.add(leverMesh);

    const pushLinkGeo = track(new THREE.CylinderGeometry(0.02, 0.02, linkLen, 10));
    const pushLink = new THREE.Mesh(pushLinkGeo, mats.steelShaft);
    pushLink.castShadow = true;
    wpSideGroup.add(pushLink);

    const clevisPinGeo = track(new THREE.CylinderGeometry(0.032, 0.032, 0.09, 10));
    clevisPinGeo.rotateZ(Math.PI / 2);
    const clevisPin = new THREE.Mesh(clevisPinGeo, mats.metalHardware);
    wpSideGroup.add(clevisPin);

    ensureRotaryConverter();

    return {
      group,
      update: (drive, continuousAngleRad, tipWorldY, axialLiftWorld = 0) => {
        currentAxialLiftLocal = Math.max(0, axialLiftWorld) / Math.max(0.5, scale);
        const pinY = computeEffectivePinY(drive, continuousAngleRad, tipWorldY);
        const sol = solveTopperWoodpeckerLinkage(pinY, fulcrumZ, fulcrumY, leverLen, linkLen);
        birdPivot.rotation.x = sol.birdPitchRad;
        clevisPin.position.set(0, pinY, 0);
        alignBarBetweenPoints(
          pushLink,
          new THREE.Vector3(0, pinY, 0),
          new THREE.Vector3(0, sol.leverPinY, sol.leverPinZ),
          'y'
        );
      },
    };
  }

  if (type === 'boatman') {
    const K = BOATMAN_KINEMATICS;

    // Open-Center Wave Frame & Side Crests (keeps central deck collar & vertical pushrod 100% visible)
    const railGeo = track(new THREE.BoxGeometry(0.075, 0.05, 1.04));
    for (const rx of [-0.26, 0.26]) {
      const rail = new THREE.Mesh(railGeo, mats.woodDarkTrim);
      rail.position.set(rx, 0.085, 0.06);
      rail.castShadow = true;
      rail.receiveShadow = true;
      group.add(rail);
    }
    const crossTieGeo = track(new THREE.BoxGeometry(0.595, 0.05, 0.08));
    for (const rz of [-0.42, 0.54]) {
      const tie = new THREE.Mesh(crossTieGeo, mats.woodDarkTrim);
      tie.position.set(0, 0.085, rz);
      tie.castShadow = true;
      group.add(tie);
    }

    const waveShape = new THREE.Shape();
    waveShape.moveTo(-0.46, 0.11);
    waveShape.lineTo(0.58, 0.11);
    waveShape.lineTo(0.58, 0.38);
    waveShape.quadraticCurveTo(0.42, 0.62, 0.24, 0.46);
    waveShape.quadraticCurveTo(0.04, 0.64, -0.14, 0.46);
    waveShape.quadraticCurveTo(-0.32, 0.60, -0.46, 0.40);
    waveShape.closePath();

    const waveGeo = track(
      new THREE.ExtrudeGeometry(waveShape, {
        depth: 0.055,
        bevelEnabled: true,
        bevelThickness: 0.008,
        bevelSize: 0.008,
        bevelSegments: 1,
      })
    );
    waveGeo.rotateY(-Math.PI / 2);
    for (const side of [-1, 1] as const) {
      const waveWall = new THREE.Mesh(waveGeo, mats.paintedSecondary);
      waveWall.position.set(side * 0.26 + 0.0275, 0, 0);
      waveWall.castShadow = true;
      waveWall.receiveShadow = true;
      group.add(waveWall);
    }

    // Rear Brass Boat-Pivot Stanchions & Fixed Dorsal-Link Anchor Crossbeam
    const stanchionH = K.boatFulcrumY - 0.11;
    const stanchionGeo = track(new THREE.CylinderGeometry(0.03, 0.036, stanchionH, 10));
    for (const sx of [-0.14, 0.14]) {
      const st = new THREE.Mesh(stanchionGeo, mats.metalHardware);
      st.position.set(sx, 0.11 + stanchionH * 0.5, K.boatFulcrumZ);
      st.castShadow = true;
      group.add(st);
    }
    const fulcrumAxleGeo = track(new THREE.CylinderGeometry(0.028, 0.028, 0.34, 10));
    fulcrumAxleGeo.rotateZ(Math.PI / 2);
    const fulcrumAxle = new THREE.Mesh(fulcrumAxleGeo, mats.metalHardware);
    fulcrumAxle.position.set(0, K.boatFulcrumY, K.boatFulcrumZ);
    group.add(fulcrumAxle);

    const anchorBarGeo = track(new THREE.CylinderGeometry(0.024, 0.024, 0.52, 10));
    anchorBarGeo.rotateZ(Math.PI / 2);
    const anchorBar = new THREE.Mesh(anchorBarGeo, mats.metalHardware);
    anchorBar.position.set(0, K.anchorY, K.anchorZ);
    group.add(anchorBar);

    const anchorBraceGeo = track(new THREE.CylinderGeometry(0.016, 0.016, 1.0, 8));
    for (const sx of [-0.14, 0.14]) {
      const brace = new THREE.Mesh(anchorBraceGeo, mats.metalHardware);
      const p1 = new THREE.Vector3(sx, 0.14, K.boatFulcrumZ);
      const p2 = new THREE.Vector3(sx, K.anchorY, K.anchorZ);
      brace.scale.set(1, p1.distanceTo(p2), 1);
      alignBarBetweenPoints(brace, p1, p2, 'y');
      group.add(brace);
    }

    // Boat Pivot Group elevated cleanly above maximum pushrod lift
    const boatPivot = new THREE.Group();
    boatPivot.position.set(0, K.boatFulcrumY, K.boatFulcrumZ);
    group.add(boatPivot);

    // Carved Wooden Skiff Hull (transom at z=0.12 leaves clear air behind stern for sailorDriveLink)
    const hullShape = new THREE.Shape();
    hullShape.moveTo(-0.14, 0.12);
    hullShape.lineTo(0.14, 0.12);
    hullShape.lineTo(0.2, 0.42);
    hullShape.lineTo(0.16, 0.7);
    hullShape.lineTo(0, 0.88);
    hullShape.lineTo(-0.16, 0.7);
    hullShape.lineTo(-0.2, 0.42);
    hullShape.closePath();

    const hullGeo = track(
      new THREE.ExtrudeGeometry(hullShape, {
        depth: 0.085,
        bevelEnabled: true,
        bevelThickness: 0.01,
        bevelSize: 0.01,
        bevelSegments: 2,
      })
    );
    hullGeo.rotateX(Math.PI / 2);
    const hull = new THREE.Mesh(hullGeo, mats.woodPrimary);
    hull.position.set(0, 0.1, 0);
    hull.castShadow = true;
    hull.receiveShadow = true;
    boatPivot.add(hull);

    // Twin brass stern hinge brackets connecting hull transom (z=0.13) to boatPivot axle (z=0)
    const sternArmGeo = track(new THREE.BoxGeometry(0.03, 0.05, 0.16));
    for (const sx of [-0.11, 0.11]) {
      const arm = new THREE.Mesh(sternArmGeo, mats.metalHardware);
      arm.position.set(sx, 0.02, 0.07);
      arm.castShadow = true;
      boatPivot.add(arm);
    }

    const gunwaleGeo = track(new THREE.BoxGeometry(0.032, 0.028, 0.54));
    for (const gx of [-0.18, 0.18]) {
      const gunwale = new THREE.Mesh(gunwaleGeo, mats.woodDarkTrim);
      gunwale.position.set(gx, 0.12, 0.42);
      gunwale.castShadow = true;
      boatPivot.add(gunwale);
    }

    const thwartGeo = track(new THREE.BoxGeometry(0.32, 0.026, 0.13));
    const thwart = new THREE.Mesh(thwartGeo, mats.woodSecondary);
    thwart.position.set(0, 0.125, K.seatLocalZ);
    boatPivot.add(thwart);

    // Brass Keel Crank Arm from Boat Pivot down to forward keel clevis pin
    const keelPinLocal = new THREE.Vector3(
      0,
      K.boatLeverLen * Math.sin(K.neutralLeverAngle),
      K.boatLeverLen * Math.cos(K.neutralLeverAngle)
    );
    const keelArmGeo = track(new THREE.BoxGeometry(0.045, 0.045, 1.0));
    const keelArm = new THREE.Mesh(keelArmGeo, mats.metalHardware);
    keelArm.scale.set(1, 1, keelPinLocal.length());
    alignBarBetweenPoints(keelArm, new THREE.Vector3(0, 0, 0), keelPinLocal, 'z');
    keelArm.castShadow = true;
    boatPivot.add(keelArm);

    const keelStrutGeo = track(new THREE.CylinderGeometry(0.02, 0.02, 1.0, 8));
    const keelStrut = new THREE.Mesh(keelStrutGeo, mats.metalHardware);
    const hullKeelAttach = new THREE.Vector3(0, 0.02, 0.64);
    keelStrut.scale.set(1, hullKeelAttach.distanceTo(keelPinLocal), 1);
    alignBarBetweenPoints(keelStrut, hullKeelAttach, keelPinLocal, 'y');
    boatPivot.add(keelStrut);

    const keelPinMeshGeo = track(new THREE.CylinderGeometry(0.03, 0.03, 0.09, 10));
    keelPinMeshGeo.rotateZ(Math.PI / 2);
    const keelPinMesh = new THREE.Mesh(keelPinMeshGeo, mats.metalHardware);
    keelPinMesh.position.copy(keelPinLocal);
    boatPivot.add(keelPinMesh);

    // Gunwale Brass Oarlocks
    const oarlockLocalPos = (side: -1 | 1) =>
      new THREE.Vector3(side * 0.24, K.oarlockLocalY, K.oarlockLocalZ);
    const oarlockGeo = track(new THREE.TorusGeometry(0.032, 0.01, 8, 14, Math.PI * 1.35));
    for (const side of [-1, 1] as const) {
      const ol = new THREE.Mesh(oarlockGeo, mats.metalHardware);
      ol.position.copy(oarlockLocalPos(side));
      ol.rotation.y = Math.PI / 2;
      ol.rotation.z = -Math.PI * 0.65;
      boatPivot.add(ol);
    }

    // Sailor Torso Hinge on Thwart Seat + Rigid Dorsal Crank Horn
    const sailorPivot = new THREE.Group();
    sailorPivot.position.set(0, K.seatLocalY, K.seatLocalZ);
    boatPivot.add(sailorPivot);

    const torsoGeo = track(new THREE.CylinderGeometry(0.085, 0.1, 0.27, 12));
    const torso = new THREE.Mesh(torsoGeo, mats.paintedAccent);
    torso.position.set(0, 0.14, 0);
    torso.castShadow = true;
    sailorPivot.add(torso);

    const headGeo = track(new THREE.SphereGeometry(0.082, 14, 12));
    const head = new THREE.Mesh(headGeo, mats.paintedIvory);
    head.position.set(0, 0.34, 0);
    head.castShadow = true;
    sailorPivot.add(head);

    const capGeo = track(new THREE.CylinderGeometry(0.088, 0.102, 0.052, 12));
    const cap = new THREE.Mesh(capGeo, mats.paintedSecondary);
    cap.position.set(0, 0.41, 0);
    sailorPivot.add(cap);

    const dorsalHornLocal = new THREE.Vector3(
      0,
      K.sailorHornLen * Math.sin(K.neutralHornAngle),
      K.sailorHornLen * Math.cos(K.neutralHornAngle)
    );
    const hornGeo = track(new THREE.BoxGeometry(0.036, 0.032, 1.0));
    const dorsalHorn = new THREE.Mesh(hornGeo, mats.metalHardware);
    dorsalHorn.scale.set(1, 1, dorsalHornLocal.length());
    alignBarBetweenPoints(dorsalHorn, new THREE.Vector3(0, 0, 0), dorsalHornLocal, 'z');
    sailorPivot.add(dorsalHorn);

    const hornPinGeo = track(new THREE.CylinderGeometry(0.025, 0.025, 0.08, 10));
    hornPinGeo.rotateZ(Math.PI / 2);
    const hornPin = new THREE.Mesh(hornPinGeo, mats.metalHardware);
    hornPin.position.copy(dorsalHornLocal);
    sailorPivot.add(hornPin);

    // Rigid Arms & Oars (exact triangle closure at hand grip)
    const armGeo = track(new THREE.CylinderGeometry(0.022, 0.02, 1.0, 8));
    const leftArm = new THREE.Mesh(armGeo, mats.paintedIvory);
    const rightArm = new THREE.Mesh(armGeo, mats.paintedIvory);
    leftArm.castShadow = true;
    rightArm.castShadow = true;
    boatPivot.add(leftArm, rightArm);

    const inboard3DLen = Math.hypot(0.24 - 0.11, K.inboardOarLenYZ);
    const buildOar = () => {
      const oarGroup = new THREE.Group();
      const totalShaftLen = inboard3DLen + 0.32;
      const shaftGeo = track(new THREE.CylinderGeometry(0.017, 0.017, totalShaftLen, 8));
      shaftGeo.rotateX(Math.PI / 2);
      const shaft = new THREE.Mesh(shaftGeo, mats.woodDarkTrim);
      shaft.position.set(0, 0, (-inboard3DLen + 0.32) * 0.5);
      shaft.castShadow = true;
      oarGroup.add(shaft);

      const bladeGeo = track(new THREE.BoxGeometry(0.08, 0.018, 0.16));
      const blade = new THREE.Mesh(bladeGeo, mats.paintedAccent);
      blade.position.set(0, 0, 0.36);
      blade.castShadow = true;
      oarGroup.add(blade);
      boatPivot.add(oarGroup);
      return oarGroup;
    };

    const leftOar = buildOar();
    const rightOar = buildOar();

    // Visible Rigid Connecting Links & Pushrod Crosshead Pin in Open Wave Bay
    const linkCylGeo = track(new THREE.CylinderGeometry(0.02, 0.02, 1.0, 10));
    const boatDriveLink = new THREE.Mesh(linkCylGeo, mats.steelShaft);
    boatDriveLink.castShadow = true;
    group.add(boatDriveLink);

    const sailorDriveLink = new THREE.Mesh(linkCylGeo, mats.metalHardware);
    sailorDriveLink.castShadow = true;
    group.add(sailorDriveLink);

    const crossPinGeo = track(new THREE.CylinderGeometry(0.036, 0.036, 0.12, 12));
    crossPinGeo.rotateZ(Math.PI / 2);
    const crossPin = new THREE.Mesh(crossPinGeo, mats.metalHardware);
    crossPin.castShadow = true;
    group.add(crossPin);

    ensureRotaryConverter();

    return {
      group,
      update: (drive, continuousAngleRad, tipWorldY, axialLiftWorld = 0) => {
        currentAxialLiftLocal = Math.max(0, axialLiftWorld) / Math.max(0.5, scale);
        const pinY = computeEffectivePinY(drive, continuousAngleRad, tipWorldY);
        const sol = solveTopperBoatmanLinkage(pinY);

        boatPivot.rotation.x = sol.boatPitchRad;
        sailorPivot.rotation.x = sol.sailorPitchRad;

        crossPin.position.set(0, pinY, 0);

        const pushPos = new THREE.Vector3(0, pinY, 0);
        const keelPinPos = new THREE.Vector3(0, sol.keelPinY, sol.keelPinZ);
        boatDriveLink.scale.set(1, pushPos.distanceTo(keelPinPos), 1);
        alignBarBetweenPoints(boatDriveLink, pushPos, keelPinPos, 'y');

        const anchorPos = new THREE.Vector3(0, K.anchorY, K.anchorZ);
        const dorsalPinPos = new THREE.Vector3(0, sol.dorsalHornY, sol.dorsalHornZ);
        sailorDriveLink.scale.set(1, anchorPos.distanceTo(dorsalPinPos), 1);
        alignBarBetweenPoints(sailorDriveLink, anchorPos, dorsalPinPos, 'y');

        for (const side of [-1, 1] as const) {
          const shoulder = new THREE.Vector3(side * 0.1, sol.shoulderLocalY, sol.shoulderLocalZ);
          const hand = new THREE.Vector3(side * 0.11, sol.handLocalY, sol.handLocalZ);
          const armMesh = side === -1 ? leftArm : rightArm;
          armMesh.scale.set(1, shoulder.distanceTo(hand), 1);
          alignBarBetweenPoints(armMesh, shoulder, hand, 'y');

          const oarlock = oarlockLocalPos(side);
          const oarGroup = side === -1 ? leftOar : rightOar;
          const dir = new THREE.Vector3().subVectors(oarlock, hand).normalize();
          oarGroup.position.copy(oarlock);
          oarGroup.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), dir);
        }
      },
    };
  }

  // 'windmill' — Open Timber Belfry revealing 90-deg Bevel Gears (rotary) or Crank Rocker (linear)
  const towerBaseGeo = track(new THREE.CylinderGeometry(0.24, 0.3, 0.1, 8));
  const towerBase = new THREE.Mesh(towerBaseGeo, mats.woodDarkTrim);
  towerBase.position.set(0, 0.11, 0);
  towerBase.castShadow = true;
  group.add(towerBase);

  const lowerTowerGeo = track(new THREE.CylinderGeometry(0.2, 0.25, 0.36, 8));
  const lowerTower = new THREE.Mesh(lowerTowerGeo, mats.paintedIvory);
  lowerTower.position.set(0, 0.34, 0);
  lowerTower.castShadow = true;
  group.add(lowerTower);

  // Open timber-frame gallery corner posts so internal 90-deg gear train is 100% visible
  const postGeo = track(new THREE.BoxGeometry(0.05, 0.44, 0.05));
  for (const px of [-0.15, 0.15]) {
    for (const pz of [-0.15, 0.15]) {
      const post = new THREE.Mesh(postGeo, mats.woodDarkTrim);
      post.position.set(px, 0.74, pz);
      post.castShadow = true;
      group.add(post);
    }
  }

  const roofGeo = track(new THREE.ConeGeometry(0.26, 0.3, 8));
  const roof = new THREE.Mesh(roofGeo, mats.paintedAccent);
  roof.position.set(0, 1.11, 0);
  roof.castShadow = true;
  group.add(roof);

  // Vertical drive spindle / pushrod visible inside gallery
  const spindleGeo = track(new THREE.CylinderGeometry(0.028, 0.028, 0.72, 12));
  const vSpindle = new THREE.Mesh(spindleGeo, mats.steelShaft);
  vSpindle.position.set(0, 0.46, 0);
  group.add(vSpindle);

  // Horizontal windmill rotor shaft & sails
  const rotorPivot = new THREE.Group();
  rotorPivot.position.set(0, 0.88, 0.23);
  group.add(rotorPivot);

  const rotorShaftGeo = track(new THREE.CylinderGeometry(0.032, 0.032, 0.32, 12));
  rotorShaftGeo.rotateX(Math.PI / 2);
  const rotorShaft = new THREE.Mesh(rotorShaftGeo, mats.steelShaft);
  rotorShaft.position.set(0, 0, -0.14);
  rotorPivot.add(rotorShaft);

  const hubGeo = track(new THREE.CylinderGeometry(0.075, 0.075, 0.1, 14));
  hubGeo.rotateX(Math.PI / 2);
  const hub = new THREE.Mesh(hubGeo, mats.metalHardware);
  hub.castShadow = true;
  rotorPivot.add(hub);

  const sparGeo = track(new THREE.BoxGeometry(0.036, 0.62, 0.024));
  const sailGeo = track(new THREE.BoxGeometry(0.14, 0.44, 0.014));
  for (let i = 0; i < 4; i++) {
    const arm = new THREE.Group();
    arm.rotation.z = (i * Math.PI) / 2;

    const spar = new THREE.Mesh(sparGeo, mats.woodDarkTrim);
    spar.position.set(0, 0.31, 0);
    spar.castShadow = true;
    arm.add(spar);

    const sail = new THREE.Mesh(sailGeo, mats.woodSecondary);
    sail.position.set(0.08, 0.38, 0.01);
    sail.rotation.y = 0.22;
    sail.castShadow = true;
    arm.add(sail);

    rotorPivot.add(arm);
  }

  // Exposed internal 90-degree gear pair (rotary mode) OR eccentric crank link (linear mode)
  let topPinionGear: THREE.Group | null = null;
  let rotorCrownGear: THREE.Group | null = null;
  let linearCrankLink: THREE.Mesh | null = null;
  let linearCrankPin: THREE.Mesh | null = null;

  if (driveMode === 'rotary') {
    const wmPitchR = 0.11;
    const wmTeeth = 10;
    const topPinion = buildConicalBevelGear(
      wmPitchR,
      wmPitchR,
      Math.PI / 4,
      wmTeeth,
      0.42,
      0, // tooth 0 at +Z
      mats.metalHardware,
      mats.metalHardware,
      track,
      'topper'
    );
    topPinion.group.position.set(0, 0.88 - wmPitchR, 0);
    group.add(topPinion.group);
    topPinionGear = topPinion.group;

    const rotorGear = buildConicalBevelGear(
      wmPitchR,
      wmPitchR,
      Math.PI / 4,
      wmTeeth,
      0.42,
      Math.PI + Math.PI / wmTeeth, // valley centered at -Z (maps to world -Y)
      mats.woodPrimary,
      mats.metalHardware,
      track,
      'topper'
    );
    // Orient cone axis along -Z toward apex (0, 0.88, 0)
    rotorGear.group.rotation.x = -Math.PI / 2;
    rotorGear.group.position.set(0, 0, -(0.23 - wmPitchR));
    rotorPivot.add(rotorGear.group);
    rotorCrownGear = rotorGear.group;
  } else {
    // Linear reciprocating pushrod drives an eccentric crankpin on the rotor hub
    const crankPinGeo = track(new THREE.CylinderGeometry(0.024, 0.024, 0.08, 10));
    crankPinGeo.rotateX(Math.PI / 2);
    linearCrankPin = new THREE.Mesh(crankPinGeo, mats.metalHardware);
    linearCrankPin.position.set(0.11, 0, -0.22);
    rotorPivot.add(linearCrankPin);

    const linkGeo = track(new THREE.CylinderGeometry(0.018, 0.018, 1.0, 10));
    linearCrankLink = new THREE.Mesh(linkGeo, mats.metalHardware);
    group.add(linearCrankLink);
  }

  return {
    group,
    update: (drive, continuousAngleRad, tipWorldY) => {
      if (driveMode === 'rotary') {
        vSpindle.rotation.y = -continuousAngleRad;
        if (topPinionGear) topPinionGear.rotation.y = -continuousAngleRad;
        // Compatible mesh rotation sign: -Y on rotor moves toward -X matching +Z on topPinion
        rotorPivot.rotation.z = -continuousAngleRad;
        if (rotorCrownGear) rotorCrownGear.rotation.y = 0;
      } else {
        const pinY = computeEffectivePinY(drive, continuousAngleRad, tipWorldY);
        const norm = THREE.MathUtils.clamp((pinY - 0.14) / 0.44, 0, 1);
        const swing = (norm - 0.5) * 1.45;
        rotorPivot.rotation.z = swing;

        const crankR = 0.11;
        const cX = crankR * Math.cos(swing);
        const cY = 0.88 + crankR * Math.sin(swing);
        if (linearCrankLink) {
          const pA = new THREE.Vector3(0, pinY, 0);
          const pB = new THREE.Vector3(cX, cY, 0.01);
          linearCrankLink.scale.set(1, pA.distanceTo(pB), 1);
          alignBarBetweenPoints(linearCrankLink, pA, pB, 'y');
        }
      }
    },
  };
}

/**
 * Builds a modular Cam + Flat-Faced Follower + Pushrod + Topper rig.
 */
export function buildCamModuleRig(
  mod: CamFollowerModule,
  blueprint: AutomataBlueprint,
  mats: AutomataMaterialSet
): ModuleRig {
  const disposables: THREE.BufferGeometry[] = [];
  const track = <T extends THREE.BufferGeometry>(g: T): T => {
    disposables.push(g);
    return g;
  };

  const rootGroup = new THREE.Group();
  rootGroup.position.set(mod.slotX, 0, 0);

  const hlGeo = track(new THREE.TorusGeometry(0.45, 0.025, 10, 28));
  hlGeo.rotateY(Math.PI / 2);
  const highlightRing = new THREE.Mesh(hlGeo, mats.selectionHighlight);
  highlightRing.position.set(0, blueprint.shaftY, 0);
  highlightRing.visible = false;
  rootGroup.add(highlightRing);

  // 1. Cam Rotating Group
  const camPivot = new THREE.Group();
  camPivot.position.set(0, blueprint.shaftY, 0);
  rootGroup.add(camPivot);

  const contour = sampleCamContour(mod.camType, mod.baseRadius, mod.lift, 144);
  const camShape = new THREE.Shape();
  contour.forEach((pt, i) => {
    if (i === 0) camShape.moveTo(pt.x, pt.y);
    else camShape.lineTo(pt.x, pt.y);
  });
  camShape.closePath();

  const hole = new THREE.Path();
  hole.absarc(0, 0, 0.09, 0, Math.PI * 2, true);
  camShape.holes.push(hole);

  const camGeo = track(
    new THREE.ExtrudeGeometry(camShape, {
      depth: 0.24,
      bevelEnabled: true,
      bevelThickness: 0.015,
      bevelSize: CAM_BEVEL_RADIAL,
      bevelSegments: 2,
    })
  );
  camGeo.translate(0, 0, -0.12);
  camGeo.rotateY(-Math.PI / 2);

  const camMesh = new THREE.Mesh(camGeo, mats.woodSecondary);
  camMesh.castShadow = true;
  camMesh.receiveShadow = true;
  camMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  camPivot.add(camMesh);

  const collarGeo = track(new THREE.CylinderGeometry(0.16, 0.16, 0.36, 18));
  collarGeo.rotateZ(Math.PI / 2);
  const collar = new THREE.Mesh(collarGeo, mats.metalHardware);
  collar.castShadow = true;
  collar.userData = { moduleId: mod.id, dragRole: 'module' };
  camPivot.add(collar);

  // 2. Follower & Pushrod Sub-group (bottom of foot rests tangentially on beveled cam)
  const followerGroup = new THREE.Group();
  followerGroup.position.set(0, blueprint.shaftY + mod.baseRadius + CAM_BEVEL_RADIAL, 0);
  rootGroup.add(followerGroup);

  const footGeo = track(new THREE.CylinderGeometry(mod.footRadius, mod.footRadius, 0.08, 24));
  const footMesh = new THREE.Mesh(footGeo, mats.metalHardware);
  footMesh.position.set(0, 0.04, 0);
  footMesh.castShadow = true;
  footMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  followerGroup.add(footMesh);

  // Size rodHeight so at rest (min cam radius), rod top sits at stageDeckY + 0.12
  const restBottomY = blueprint.shaftY + mod.baseRadius + CAM_BEVEL_RADIAL;
  const rodHeight = Math.max(0.7, blueprint.stageDeckY + 0.12 - (restBottomY + 0.08));
  const rodGeo = track(new THREE.CylinderGeometry(0.048, 0.048, rodHeight, 12));
  const rodMesh = new THREE.Mesh(rodGeo, mats.steelShaft);
  rodMesh.position.set(0, 0.08 + rodHeight * 0.5, 0);
  rodMesh.castShadow = true;
  rodMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  followerGroup.add(rodMesh);

  // Brass Guide Collar embedded in Stage Deck (bottom clears deck underside -0.18, top flange +0.06)
  const guideGroup = new THREE.Group();
  guideGroup.position.set(0, blueprint.stageDeckY, 0);
  rootGroup.add(guideGroup);

  const guideGeo = track(new THREE.CylinderGeometry(0.16, 0.18, 0.26, 18));
  const guideMesh = new THREE.Mesh(guideGeo, mats.metalHardware);
  guideMesh.position.set(0, -0.07, 0);
  guideMesh.castShadow = true;
  guideMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  guideGroup.add(guideMesh);

  // 3. Topper Sculpture mounted on Stage Deck
  const topperRig = buildTopperGroup(mod.topper, mod.topperScale, mats, track, 'linear');
  topperRig.group.position.set(0, blueprint.stageDeckY, 0);
  topperRig.group.traverse((obj) => {
    if ((obj as THREE.Mesh).isMesh) {
      obj.userData = { moduleId: mod.id, dragRole: 'module' };
    }
  });
  rootGroup.add(topperRig.group);

  const hitGeo = track(new THREE.BoxGeometry(0.65, blueprint.stageDeckY + 0.9, 1.3));
  const hitMat = new THREE.MeshBasicMaterial({ visible: false });
  const hitMesh = new THREE.Mesh(hitGeo, hitMat);
  hitMesh.position.set(0, (blueprint.stageDeckY + 0.9) * 0.5, 0);
  hitMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  rootGroup.add(hitMesh);

  return {
    id: mod.id,
    type: mod.type,
    rootGroup,
    hitMeshes: [hitMesh, camMesh, footMesh, guideMesh],
    pivotHitMeshes: [],
    highlightRing,
    update: (shaftAngleRad, bp, explodeAmount) => {
      const theta = shaftAngleRad + degToRad(mod.phaseDeg);
      camPivot.rotation.x = -theta;

      const samples = contour.length;
      let maxY = mod.baseRadius * 0.85;
      for (let i = 0; i < samples; i++) {
        const alpha = (i / samples) * Math.PI * 2;
        const r = contour[i].radius + CAM_BEVEL_RADIAL;
        const wa = alpha + theta;
        const pz = r * Math.cos(wa);
        const py = r * Math.sin(wa);
        if (Math.abs(pz) <= mod.footRadius && py > maxY) {
          maxY = py;
        }
      }

      const liftDisp = Math.max(0, maxY - (mod.baseRadius + CAM_BEVEL_RADIAL));
      const normLift = Math.min(1, Math.max(0, liftDisp / Math.max(0.05, mod.lift)));
      followerGroup.position.y = bp.shaftY + maxY + explodeAmount * 0.35;
      guideGroup.position.y = bp.stageDeckY + explodeAmount * 1.15;
      topperRig.group.position.y = bp.stageDeckY + explodeAmount * 1.15;

      const tipAboveDeckWorld = 0.12 + liftDisp;
      topperRig.update(normLift, 0, tipAboveDeckWorld);
    },
    setSelected: (selected) => {
      highlightRing.visible = selected;
    },
    dispose: () => {
      hitMat.dispose();
      for (const g of disposables) g.dispose();
    },
  };
}

/**
 * Builds a modular 4-Bar Crank-Rocker Linkage rig with draggable Fulcrum Pivot on the pegboard
 * and a coaxial guided vertical pushrod driven by a hinged pitman connecting link.
 */
export function buildFourBarModuleRig(
  mod: FourBarLinkageModule,
  blueprint: AutomataBlueprint,
  mats: AutomataMaterialSet,
  solveFn: (shaftAngleRad: number, m: FourBarLinkageModule) => {
    crankPin: { z: number; y: number };
    couplerJoint: { z: number; y: number };
    fulcrum: { z: number; y: number };
    crankAngleRad: number;
    couplerAngleRad: number;
    rockerAngleRad: number;
    pushrodLift: number;
    normalizedSwing: number;
  }
): ModuleRig {
  const disposables: THREE.BufferGeometry[] = [];
  const track = <T extends THREE.BufferGeometry>(g: T): T => {
    disposables.push(g);
    return g;
  };

  const pushCfg = computeFourBarPushrodConfig(mod);

  const rootGroup = new THREE.Group();
  rootGroup.position.set(mod.slotX, 0, 0);

  const hlGeo = track(new THREE.TorusGeometry(0.45, 0.025, 10, 28));
  hlGeo.rotateY(Math.PI / 2);
  const highlightRing = new THREE.Mesh(hlGeo, mats.selectionHighlight);
  highlightRing.position.set(0, blueprint.shaftY, 0);
  highlightRing.visible = false;
  rootGroup.add(highlightRing);

  // 1. Crank Disc + Brass Pin (x ∈ [-0.07, +0.07])
  const crankGroup = new THREE.Group();
  crankGroup.position.set(0, blueprint.shaftY, 0);
  rootGroup.add(crankGroup);

  const crankDiscGeo = track(new THREE.CylinderGeometry(mod.crankRadius + 0.14, mod.crankRadius + 0.14, 0.14, 24));
  crankDiscGeo.rotateZ(Math.PI / 2);
  const crankDisc = new THREE.Mesh(crankDiscGeo, mats.woodSecondary);
  crankDisc.castShadow = true;
  crankDisc.userData = { moduleId: mod.id, dragRole: 'module' };
  crankGroup.add(crankDisc);

  const hubCollarGeo = track(new THREE.CylinderGeometry(0.15, 0.15, 0.24, 16));
  hubCollarGeo.rotateZ(Math.PI / 2);
  const hubCollar = new THREE.Mesh(hubCollarGeo, mats.metalHardware);
  hubCollar.userData = { moduleId: mod.id, dragRole: 'module' };
  crankGroup.add(hubCollar);

  const crankPinGeo = track(new THREE.CylinderGeometry(0.052, 0.052, 0.24, 12));
  crankPinGeo.rotateZ(Math.PI / 2);
  const crankPinMesh = new THREE.Mesh(crankPinGeo, mats.metalHardware);
  crankPinMesh.position.set(0.11, 0, mod.crankRadius);
  crankPinMesh.castShadow = true;
  crankGroup.add(crankPinMesh);

  // 2. Coupler Rod (P1 -> P2) at x = 0.14 (outboard of crankDisc & rocker)
  const couplerGroup = new THREE.Group();
  rootGroup.add(couplerGroup);
  const couplerGeo = track(createLinkBarGeometry(mod.couplerLength, 0.17, 0.07));
  const couplerMesh = new THREE.Mesh(couplerGeo, mats.paintedAccent);
  couplerMesh.position.set(0.14, 0, 0);
  couplerMesh.castShadow = true;
  couplerMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  couplerGroup.add(couplerMesh);

  const jointPinGeo = track(new THREE.CylinderGeometry(0.052, 0.052, 0.24, 12));
  jointPinGeo.rotateZ(Math.PI / 2);
  const jointPin = new THREE.Mesh(jointPinGeo, mats.metalHardware);
  jointPin.position.set(0.11, 0, mod.couplerLength);
  couplerGroup.add(jointPin);

  // 3. Draggable Fulcrum Bracket & Rocker Arm (at P3)
  const fulcrumGroup = new THREE.Group();
  rootGroup.add(fulcrumGroup);

  // Support strut offset to x = -0.15 so it NEVER clips crankDisc ([-0.07, +0.07])
  const strutLen = Math.max(0.2, mod.pivotZ - (-blueprint.boxDepth * 0.5 + 0.2));
  const strutGeo = track(new THREE.BoxGeometry(0.11, 0.14, strutLen));
  const strutMesh = new THREE.Mesh(strutGeo, mats.woodDarkTrim);
  strutMesh.position.set(-0.15, 0, -strutLen * 0.5);
  strutMesh.castShadow = true;
  fulcrumGroup.add(strutMesh);

  const pivotHandleGeo = track(new THREE.CylinderGeometry(0.125, 0.125, 0.34, 16));
  pivotHandleGeo.rotateZ(Math.PI / 2);
  const pivotHandle = new THREE.Mesh(pivotHandleGeo, mats.metalHardware);
  pivotHandle.position.set(0.02, 0, 0);
  pivotHandle.castShadow = true;
  pivotHandle.userData = { moduleId: mod.id, dragRole: 'pivot' };
  fulcrumGroup.add(pivotHandle);

  const pivotRingGeo = track(new THREE.TorusGeometry(0.16, 0.02, 8, 20));
  pivotRingGeo.rotateY(Math.PI / 2);
  const pivotRing = new THREE.Mesh(pivotRingGeo, mats.paintedAccent);
  pivotRing.position.set(0.2, 0, 0);
  pivotRing.userData = { moduleId: mod.id, dragRole: 'pivot' };
  fulcrumGroup.add(pivotRing);

  // Rocker Arm (P3 -> P2) at x = 0.04 with inline brass clevis pin at clevisArmDist
  const rockerGroup = new THREE.Group();
  fulcrumGroup.add(rockerGroup);
  const rockerGeo = track(createLinkBarGeometry(mod.rockerLength, 0.17, 0.075));
  const rockerMesh = new THREE.Mesh(rockerGeo, mats.woodPrimary);
  rockerMesh.position.set(0.04, 0, 0);
  rockerMesh.castShadow = true;
  rockerMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  rockerGroup.add(rockerMesh);

  const clevisPinGeo = track(new THREE.CylinderGeometry(0.048, 0.048, 0.18, 12));
  clevisPinGeo.rotateZ(Math.PI / 2);
  const clevisPin = new THREE.Mesh(clevisPinGeo, mats.metalHardware);
  clevisPin.position.set(-0.02, 0, pushCfg.clevisArmDist);
  rockerGroup.add(clevisPin);

  // Hinged Pitman Link connecting rocker clevis (clevisY, clevisZ) -> guided vertical pushrod bottom (outputStageZ)
  const pitmanGroup = new THREE.Group();
  rootGroup.add(pitmanGroup);
  const pitmanGeo = track(createLinkBarGeometry(pushCfg.linkLength, 0.11, 0.045));
  const pitmanMesh = new THREE.Mesh(pitmanGeo, mats.metalHardware);
  pitmanMesh.position.set(-0.04, 0, 0);
  pitmanMesh.castShadow = true;
  pitmanGroup.add(pitmanMesh);

  const pitmanTopPinGeo = track(new THREE.CylinderGeometry(0.042, 0.042, 0.14, 10));
  pitmanTopPinGeo.rotateZ(Math.PI / 2);
  const pitmanTopPin = new THREE.Mesh(pitmanTopPinGeo, mats.metalHardware);
  pitmanTopPin.position.set(-0.02, 0, pushCfg.linkLength);
  pitmanGroup.add(pitmanTopPin);

  // Coaxial Guided Vertical Pushrod constrained strictly at outputStageZ (ZERO sideways clipping!)
  const pushrodGroup = new THREE.Group();
  rootGroup.add(pushrodGroup);
  const minRodWorldBottom = blueprint.shaftY + pushCfg.minRodBottomY;
  const fourBarRodLen = Math.max(0.45, blueprint.stageDeckY + 0.12 - minRodWorldBottom);
  const pushrodGeo = track(new THREE.CylinderGeometry(0.044, 0.044, fourBarRodLen, 12));
  const pushrodMesh = new THREE.Mesh(pushrodGeo, mats.steelShaft);
  pushrodMesh.position.set(0, fourBarRodLen * 0.5, 0);
  pushrodMesh.castShadow = true;
  pushrodGroup.add(pushrodMesh);

  const deckCollarGroup = new THREE.Group();
  deckCollarGroup.position.set(0, blueprint.stageDeckY, pushCfg.outputStageZ);
  rootGroup.add(deckCollarGroup);
  const collarGeo = track(new THREE.CylinderGeometry(0.16, 0.18, 0.26, 16));
  const collarMesh = new THREE.Mesh(collarGeo, mats.metalHardware);
  collarMesh.position.set(0, -0.07, 0);
  deckCollarGroup.add(collarMesh);

  // 4. Topper Sculpture centered directly over outputStageZ
  const topperRig = buildTopperGroup(mod.topper, mod.topperScale, mats, track, 'linear');
  topperRig.group.position.set(0, blueprint.stageDeckY, pushCfg.outputStageZ);
  topperRig.group.traverse((obj) => {
    if ((obj as THREE.Mesh).isMesh) {
      obj.userData = { moduleId: mod.id, dragRole: 'module' };
    }
  });
  rootGroup.add(topperRig.group);

  const hitGeo = track(new THREE.BoxGeometry(0.65, blueprint.stageDeckY + 0.9, 1.5));
  const hitMat = new THREE.MeshBasicMaterial({ visible: false });
  const hitMesh = new THREE.Mesh(hitGeo, hitMat);
  hitMesh.position.set(0, (blueprint.stageDeckY + 0.9) * 0.5, 0.15);
  hitMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  rootGroup.add(hitMesh);

  return {
    id: mod.id,
    type: mod.type,
    rootGroup,
    hitMeshes: [hitMesh, crankDisc, couplerMesh, rockerMesh],
    pivotHitMeshes: [pivotHandle, pivotRing],
    highlightRing,
    update: (shaftAngleRad, bp, explodeAmount) => {
      const sol = solveFn(shaftAngleRad, mod);
      const pushSol = solveFourBarPushrod(shaftAngleRad, mod, pushCfg, sol as never);

      crankGroup.rotation.x = -sol.crankAngleRad;

      couplerGroup.position.set(0, bp.shaftY + sol.crankPin.y, sol.crankPin.z);
      couplerGroup.rotation.x = -sol.couplerAngleRad;

      fulcrumGroup.position.set(0, bp.shaftY + sol.fulcrum.y, sol.fulcrum.z);
      rockerGroup.rotation.x = -sol.rockerAngleRad;

      pitmanGroup.position.set(0, bp.shaftY + pushSol.clevisY, pushSol.clevisZ);
      pitmanGroup.rotation.x = -pushSol.linkAngleRad;

      pushrodGroup.position.set(0, bp.shaftY + pushSol.pushrodBottomY, pushCfg.outputStageZ);

      deckCollarGroup.position.y = bp.stageDeckY + explodeAmount * 1.15;
      topperRig.group.position.y = bp.stageDeckY + explodeAmount * 1.15;

      const tipAboveDeckWorld =
        bp.shaftY + pushSol.pushrodBottomY + fourBarRodLen - bp.stageDeckY;
      topperRig.update(pushSol.normalizedDrive, 0, tipAboveDeckWorld);
    },
    setSelected: (selected) => {
      highlightRing.visible = selected;
      pivotRing.scale.setScalar(selected ? 1.25 : 1.0);
    },
    dispose: () => {
      hitMat.dispose();
      for (const g of disposables) g.dispose();
    },
  };
}

/**
 * Builds a 90-degree Conical Bevel Gear module driving a vertical spindle & spinning topper
 * with geometrically non-interpenetrating cones and true tooth-to-valley meshing.
 */
export function buildBevelGearModuleRig(
  mod: BevelGearModule,
  blueprint: AutomataBlueprint,
  mats: AutomataMaterialSet
): ModuleRig {
  const disposables: THREE.BufferGeometry[] = [];
  const track = <T extends THREE.BufferGeometry>(g: T): T => {
    disposables.push(g);
    return g;
  };

  const spec = solveBevelGearGeometry(mod.gearRatio, mod.teethCount);

  const rootGroup = new THREE.Group();
  rootGroup.position.set(mod.slotX, 0, 0);

  const hlGeo = track(new THREE.TorusGeometry(0.45, 0.025, 10, 28));
  hlGeo.rotateY(Math.PI / 2);
  const highlightRing = new THREE.Mesh(hlGeo, mats.selectionHighlight);
  highlightRing.position.set(0, blueprint.shaftY, 0);
  highlightRing.visible = false;
  rootGroup.add(highlightRing);

  // Driver conical bevel gear on main shaft (apex at drivenX = driverX + drivenPitchR)
  const gearFaceFrac = 0.4;
  const driverPivot = new THREE.Group();
  driverPivot.position.set(spec.driverX, blueprint.shaftY, 0);
  const driverBuilt = buildConicalBevelGear(
    spec.driverPitchR,
    spec.drivenPitchR,
    spec.coneAngleDriver,
    spec.driverTeeth,
    gearFaceFrac,
    -Math.PI / 2, // tooth 0 aligned at local -X -> world +Y mesh point for any tooth count
    mats.woodPrimary,
    mats.metalHardware,
    track,
    mod.id
  );
  // Cone axis originally +Y; rotate -PI/2 around Z so cone points toward +X
  driverBuilt.group.rotation.z = -Math.PI / 2;
  driverPivot.add(driverBuilt.group);
  rootGroup.add(driverPivot);

  // Driven conical bevel gear on vertical spindle (apex at shaftY = drivenY - driverPitchR)
  const drivenY = blueprint.shaftY + spec.drivenYOffset;
  const drivenGearGroup = new THREE.Group();
  drivenGearGroup.position.set(spec.drivenX, drivenY, 0);

  const drivenBuilt = buildConicalBevelGear(
    spec.drivenPitchR,
    spec.driverPitchR,
    spec.coneAngleDriven,
    spec.drivenTeeth,
    gearFaceFrac,
    -Math.PI / 2 + spec.toothPhaseOffsetRad, // tooth valley centered at world -X mesh point
    mats.metalHardware,
    mats.metalHardware,
    track,
    mod.id
  );
  // Cone axis originally +Y; rotate PI around X so cone points downward (-Y) toward apex
  drivenBuilt.group.rotation.x = Math.PI;
  drivenGearGroup.add(drivenBuilt.group);

  // Vertical spindle rising cleanly into stage deck thrust collar
  const spindleLen = Math.max(0.5, blueprint.stageDeckY - drivenY + 0.08);
  const spindleGeo = track(new THREE.CylinderGeometry(0.055, 0.055, spindleLen, 14));
  const spindleMesh = new THREE.Mesh(spindleGeo, mats.steelShaft);
  spindleMesh.position.set(0, spindleLen * 0.5, 0);
  spindleMesh.castShadow = true;
  drivenGearGroup.add(spindleMesh);
  rootGroup.add(drivenGearGroup);

  // Support bracket from back pegboard wall to vertical spindle + brass radial pillow bearing
  const bracketY = drivenY + 0.26;
  const bracketLen = blueprint.boxDepth * 0.5 - 0.18;
  const bracketGeo = track(new THREE.BoxGeometry(0.15, 0.11, bracketLen));
  const bracketMesh = new THREE.Mesh(bracketGeo, mats.woodDarkTrim);
  bracketMesh.position.set(spec.drivenX, bracketY, -bracketLen * 0.5);
  bracketMesh.castShadow = true;
  rootGroup.add(bracketMesh);

  const pillowBearingGeo = track(new THREE.CylinderGeometry(0.11, 0.11, 0.13, 16));
  const pillowBearing = new THREE.Mesh(pillowBearingGeo, mats.metalHardware);
  pillowBearing.position.set(spec.drivenX, bracketY, 0);
  pillowBearing.castShadow = true;
  rootGroup.add(pillowBearing);

  // Stage Deck thrust collar
  const collarGroup = new THREE.Group();
  collarGroup.position.set(spec.drivenX, blueprint.stageDeckY, 0);
  const collarGeo = track(new THREE.CylinderGeometry(0.17, 0.19, 0.26, 18));
  const collarMesh = new THREE.Mesh(collarGeo, mats.metalHardware);
  collarMesh.position.set(0, -0.07, 0);
  collarGroup.add(collarMesh);
  rootGroup.add(collarGroup);

  // Topper mounted at spindle exit (rotary mode!)
  const topperRig = buildTopperGroup(mod.topper, mod.topperScale, mats, track, 'rotary');
  topperRig.group.position.set(spec.drivenX, blueprint.stageDeckY, 0);
  topperRig.group.traverse((obj) => {
    if ((obj as THREE.Mesh).isMesh) {
      obj.userData = { moduleId: mod.id, dragRole: 'module' };
    }
  });
  rootGroup.add(topperRig.group);

  const hitGeo = track(new THREE.BoxGeometry(0.95, blueprint.stageDeckY + 0.9, 1.4));
  const hitMat = new THREE.MeshBasicMaterial({ visible: false });
  const hitMesh = new THREE.Mesh(hitGeo, hitMat);
  hitMesh.position.set(0, (blueprint.stageDeckY + 0.9) * 0.5, 0);
  hitMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  rootGroup.add(hitMesh);

  return {
    id: mod.id,
    type: mod.type,
    rootGroup,
    hitMeshes: [hitMesh, driverBuilt.hitMesh, drivenBuilt.hitMesh],
    pivotHitMeshes: [],
    highlightRing,
    update: (shaftAngleRad, bp, explodeAmount) => {
      const driverAngle = shaftAngleRad + degToRad(mod.phaseDeg);
      const drivenAngle = driverAngle * spec.effectiveRatio;
      driverPivot.rotation.x = -driverAngle;
      drivenGearGroup.rotation.y = -drivenAngle;

      collarGroup.position.y = bp.stageDeckY + explodeAmount * 1.15;
      topperRig.group.position.y = bp.stageDeckY + explodeAmount * 1.15;
      const drive01 = (Math.sin(drivenAngle) + 1) * 0.5;
      topperRig.update(drive01, drivenAngle, 0);
    },
    setSelected: (selected) => {
      highlightRing.visible = selected;
    },
    dispose: () => {
      hitMat.dispose();
      for (const g of disposables) g.dispose();
    },
  };
}

/**
 * Builds a 90-degree Face Pin-Wheel & Open-Bottom Cantilevered Lantern Pinion module.
 * Replaces conical bevel gears with a Da Vinci / traditional wooden-mill pin-and-stave drive.
 */
export function buildPinLanternModuleRig(
  mod: PinLanternModule,
  blueprint: AutomataBlueprint,
  mats: AutomataMaterialSet
): ModuleRig {
  const disposables: THREE.BufferGeometry[] = [];
  const track = <T extends THREE.BufferGeometry>(g: T): T => {
    disposables.push(g);
    return g;
  };

  const spec = solvePinLanternGeometry(mod.gearRatio, mod.pegCount);

  const rootGroup = new THREE.Group();
  rootGroup.position.set(mod.slotX, 0, 0);

  const driverDiscR = spec.driverPitchR + 1.8 * spec.unitR;
  const hlGeo = track(new THREE.TorusGeometry(driverDiscR + 0.03, 0.025, 10, 32));
  hlGeo.rotateY(Math.PI / 2);
  const highlightRing = new THREE.Mesh(hlGeo, mats.selectionHighlight);
  highlightRing.position.set(spec.driverDiscFaceX - 0.055, blueprint.shaftY, 0);
  highlightRing.visible = false;
  rootGroup.add(highlightRing);

  // 1. Driver Wooden Face Pin-Wheel on Main Horizontal Shaft
  const driverPivot = new THREE.Group();
  driverPivot.position.set(spec.driverDiscFaceX, blueprint.shaftY, 0);
  rootGroup.add(driverPivot);
  const driverDiscGeo = track(
    new THREE.CylinderGeometry(driverDiscR, driverDiscR, 0.11, Math.max(24, spec.driverPegs * 2))
  );
  driverDiscGeo.rotateZ(Math.PI / 2);
  const driverDisc = new THREE.Mesh(driverDiscGeo, mats.woodPrimary);
  driverDisc.position.set(-0.055, 0, 0);
  driverDisc.castShadow = true;
  driverDisc.receiveShadow = true;
  driverDisc.userData = { moduleId: mod.id, dragRole: 'module' };
  driverPivot.add(driverDisc);

  const driverHubR = Math.min(driverDiscR * 0.48, 0.16);
  const driverHubGeo = track(new THREE.CylinderGeometry(driverHubR, driverHubR * 1.08, 0.14, 16));
  driverHubGeo.rotateZ(Math.PI / 2);
  const driverHub = new THREE.Mesh(driverHubGeo, mats.metalHardware);
  driverHub.position.set(-0.11, 0, 0);
  driverHub.castShadow = true;
  driverHub.userData = { moduleId: mod.id, dragRole: 'module' };
  driverPivot.add(driverHub);

  // Protruding tapered brass drive pegs along +X from face of driver disc
  const pegGeo = track(
    new THREE.CylinderGeometry(
      spec.pegRadius * 0.86,
      spec.pegRadius,
      spec.pegLength + 0.015,
      10
    )
  );
  pegGeo.rotateZ(-Math.PI / 2);
  const pegCenterX = (spec.pegLength - 0.015) * 0.5;
  for (let i = 0; i < spec.driverPegs; i++) {
    const a = (i / spec.driverPegs) * Math.PI * 2;
    const peg = new THREE.Mesh(pegGeo, mats.metalHardware);
    peg.position.set(
      pegCenterX,
      spec.driverPitchR * Math.cos(a),
      spec.driverPitchR * Math.sin(a)
    );
    peg.castShadow = true;
    peg.receiveShadow = true;
    peg.userData = { moduleId: mod.id, dragRole: 'module' };
    driverPivot.add(peg);
  }

  // 2. Driven Open-Bottom Cantilevered Lantern Pinion on Vertical Spindle
  const drivenY = blueprint.shaftY + spec.lanternYOffset;
  const drivenLanternGroup = new THREE.Group();
  drivenLanternGroup.position.set(spec.drivenX, drivenY, 0);
  rootGroup.add(drivenLanternGroup);

  const crownR = spec.lanternPitchR + 1.5 * spec.unitR;
  const crownGeo = track(
    new THREE.CylinderGeometry(crownR, crownR, 0.056, Math.max(20, spec.lanternStaves * 2))
  );
  const crownDisc = new THREE.Mesh(crownGeo, mats.metalHardware);
  crownDisc.position.set(0, spec.staveTopY + 0.028, 0);
  crownDisc.castShadow = true;
  crownDisc.receiveShadow = true;
  crownDisc.userData = { moduleId: mod.id, dragRole: 'module' };
  drivenLanternGroup.add(crownDisc);

  const crownBossGeo = track(new THREE.CylinderGeometry(0.1, 0.125, 0.075, 16));
  const crownBoss = new THREE.Mesh(crownBossGeo, mats.woodDarkTrim);
  crownBoss.position.set(0, spec.staveTopY + 0.09, 0);
  crownBoss.castShadow = true;
  crownBoss.userData = { moduleId: mod.id, dragRole: 'module' };
  drivenLanternGroup.add(crownBoss);

  const staveH = spec.staveTopY - spec.staveBottomY;
  const staveMidY = (spec.staveTopY + spec.staveBottomY) * 0.5;
  const staveGeo = track(new THREE.CylinderGeometry(spec.staveRadius, spec.staveRadius, staveH, 10));
  const ferruleGeo = track(
    new THREE.CylinderGeometry(spec.staveRadius * 1.28, spec.staveRadius * 1.28, 0.03, 10)
  );
  for (let j = 0; j < spec.lanternStaves; j++) {
    const b =
      Math.PI + spec.lanternPhaseOffsetRad + (j / spec.lanternStaves) * Math.PI * 2;
    const sx = spec.lanternPitchR * Math.cos(b);
    const sz = -spec.lanternPitchR * Math.sin(b);

    const stave = new THREE.Mesh(staveGeo, mats.steelShaft);
    stave.position.set(sx, staveMidY, sz);
    stave.castShadow = true;
    stave.receiveShadow = true;
    stave.userData = { moduleId: mod.id, dragRole: 'module' };
    drivenLanternGroup.add(stave);

    const ferrule = new THREE.Mesh(ferruleGeo, mats.metalHardware);
    ferrule.position.set(sx, spec.staveTopY - 0.012, sz);
    ferrule.userData = { moduleId: mod.id, dragRole: 'module' };
    drivenLanternGroup.add(ferrule);
  }

  // Vertical steel spindle rising cleanly from crown disc into stage deck thrust collar
  const spindleStartY = spec.staveTopY + 0.05;
  const spindleLen = Math.max(0.5, blueprint.stageDeckY - (drivenY + spindleStartY) + 0.08);
  const spindleGeo = track(new THREE.CylinderGeometry(0.055, 0.055, spindleLen, 14));
  const spindleMesh = new THREE.Mesh(spindleGeo, mats.steelShaft);
  spindleMesh.position.set(0, spindleStartY + spindleLen * 0.5, 0);
  spindleMesh.castShadow = true;
  drivenLanternGroup.add(spindleMesh);

  // Support bracket from back pegboard wall to vertical spindle + brass radial pillow bearing
  const bracketY = drivenY + spec.staveTopY + 0.24;
  const bracketLen = blueprint.boxDepth * 0.5 - 0.18;
  const bracketGeo = track(new THREE.BoxGeometry(0.15, 0.11, bracketLen));
  const bracketMesh = new THREE.Mesh(bracketGeo, mats.woodDarkTrim);
  bracketMesh.position.set(spec.drivenX, bracketY, -bracketLen * 0.5);
  bracketMesh.castShadow = true;
  rootGroup.add(bracketMesh);

  const pillowBearingGeo = track(new THREE.CylinderGeometry(0.11, 0.11, 0.13, 16));
  const pillowBearing = new THREE.Mesh(pillowBearingGeo, mats.metalHardware);
  pillowBearing.position.set(spec.drivenX, bracketY, 0);
  pillowBearing.castShadow = true;
  rootGroup.add(pillowBearing);

  // Stage Deck thrust collar
  const collarGroup = new THREE.Group();
  collarGroup.position.set(spec.drivenX, blueprint.stageDeckY, 0);
  const collarGeo = track(new THREE.CylinderGeometry(0.17, 0.19, 0.26, 18));
  const collarMesh = new THREE.Mesh(collarGeo, mats.metalHardware);
  collarMesh.position.set(0, -0.07, 0);
  collarGroup.add(collarMesh);
  rootGroup.add(collarGroup);

  // Topper mounted at spindle exit (rotary mode!)
  const topperRig = buildTopperGroup(mod.topper, mod.topperScale, mats, track, 'rotary');
  topperRig.group.position.set(spec.drivenX, blueprint.stageDeckY, 0);
  topperRig.group.traverse((obj) => {
    if ((obj as THREE.Mesh).isMesh) {
      obj.userData = { moduleId: mod.id, dragRole: 'module' };
    }
  });
  rootGroup.add(topperRig.group);

  const hitGeo = track(new THREE.BoxGeometry(1.1, blueprint.stageDeckY + 0.9, 1.4));
  const hitMat = new THREE.MeshBasicMaterial({ visible: false });
  const hitMesh = new THREE.Mesh(hitGeo, hitMat);
  hitMesh.position.set(0.05, (blueprint.stageDeckY + 0.9) * 0.5, 0);
  hitMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  rootGroup.add(hitMesh);

  return {
    id: mod.id,
    type: mod.type,
    rootGroup,
    hitMeshes: [hitMesh, driverDisc, crownDisc],
    pivotHitMeshes: [],
    highlightRing,
    update: (shaftAngleRad, bp, explodeAmount) => {
      const driverAngle = shaftAngleRad + degToRad(mod.phaseDeg);
      const drivenAngle = driverAngle * spec.effectiveRatio;
      driverPivot.rotation.x = -driverAngle;
      drivenLanternGroup.rotation.y = -drivenAngle;

      collarGroup.position.y = bp.stageDeckY + explodeAmount * 1.15;
      topperRig.group.position.y = bp.stageDeckY + explodeAmount * 1.15;
      const drive01 = (Math.sin(drivenAngle) + 1) * 0.5;
      topperRig.update(drive01, drivenAngle, 0);
    },
    setSelected: (selected) => {
      highlightRing.visible = selected;
    },
    dispose: () => {
      hitMat.dispose();
      for (const g of disposables) g.dispose();
    },
  };
}

/**
 * Helper to create a tapered ratchet pawl blade geometry in XY, extruded along thickness
 * and rotated so length runs along +Z from pivot (0,0) to engaging chisel tip (pawlLength, 0).
 */
function createRatchetPawlGeometry(
  pawlLength: number,
  thickness: number,
  lockDirection: 1 | -1
): THREE.ExtrudeGeometry {
  const s = new THREE.Shape();
  const rHub = 0.052;
  const tipH = 0.022;
  const hookBias = lockDirection === 1 ? -0.016 : 0.016;
  s.absarc(0, 0, rHub, Math.PI * 0.5, Math.PI * 1.5, false);
  s.lineTo(pawlLength - 0.018, -tipH + hookBias);
  s.lineTo(pawlLength + 0.008, hookBias);
  s.lineTo(pawlLength - 0.018, tipH + hookBias);
  s.lineTo(0, rHub);

  const geo = new THREE.ExtrudeGeometry(s, {
    depth: thickness,
    bevelEnabled: false,
    curveSegments: 14,
  });
  geo.translate(0, 0, -thickness * 0.5);
  geo.rotateY(-Math.PI / 2);
  return geo;
}

/**
 * Builds a modular Ratchet & Pawl one-way stepping/locking mechanism:
 * - Crank disc + Pitman coupler drive a swinging rocker arm around an upper ratchet axle;
 * - A spring-biased driving pawl on the rocker arm advances a sharp sawtooth wheel on the forward stroke;
 * - A stationary holding pawl anchored to the frame locks the wheel against backward slip on return;
 * - A compact 90-degree bevel gear transfers the intermittent stepped motion up to the stage topper.
 */
export function buildRatchetModuleRig(
  mod: RatchetModule,
  blueprint: AutomataBlueprint,
  mats: AutomataMaterialSet
): ModuleRig {
  const disposables: THREE.BufferGeometry[] = [];
  const track = <T extends THREE.BufferGeometry>(g: T): T => {
    disposables.push(g);
    return g;
  };

  const spec = solveRatchetGeometry(mod.teethCount, mod.stepTeeth, mod.lockDirection);
  const axleY = blueprint.shaftY + spec.axleYOffset;

  const rootGroup = new THREE.Group();
  rootGroup.position.set(mod.slotX, 0, 0);

  // Selection highlight ring centered around lower crank disc on main camshaft (leaves upper ratchet teeth unobstructed)
  const hlGeo = track(new THREE.TorusGeometry(0.45, 0.024, 10, 28));
  hlGeo.rotateY(Math.PI / 2);
  const highlightRing = new THREE.Mesh(hlGeo, mats.selectionHighlight);
  highlightRing.position.set(-0.22, blueprint.shaftY, 0);
  highlightRing.visible = false;
  rootGroup.add(highlightRing);

  // 1. Crank Disc + Brass Pin on Main Camshaft (x = -0.22, clears coupler & rocker arm)
  const crankPivot = new THREE.Group();
  crankPivot.position.set(-0.22, blueprint.shaftY, 0);
  rootGroup.add(crankPivot);

  const crankDiscR = Math.max(0.28, spec.crankRadius + 0.09);
  const crankDiscGeo = track(new THREE.CylinderGeometry(crankDiscR, crankDiscR, 0.095, 24));
  crankDiscGeo.rotateZ(Math.PI / 2);
  const crankDisc = new THREE.Mesh(crankDiscGeo, mats.woodSecondary);
  crankDisc.castShadow = true;
  crankDisc.receiveShadow = true;
  crankDisc.userData = { moduleId: mod.id, dragRole: 'module' };
  crankPivot.add(crankDisc);

  const crankHubGeo = track(new THREE.CylinderGeometry(0.11, 0.11, 0.13, 16));
  crankHubGeo.rotateZ(Math.PI / 2);
  const crankHub = new THREE.Mesh(crankHubGeo, mats.metalHardware);
  crankPivot.add(crankHub);

  const crankPinGeo = track(new THREE.CylinderGeometry(0.036, 0.036, 0.13, 14));
  crankPinGeo.rotateZ(Math.PI / 2);
  const crankPinMesh = new THREE.Mesh(crankPinGeo, mats.metalHardware);
  crankPinMesh.position.set(0.055, 0, spec.crankRadius);
  crankPinMesh.castShadow = true;
  crankPivot.add(crankPinMesh);

  // 2. Pitman Coupler Rod (x = -0.14)
  const couplerGroup = new THREE.Group();
  rootGroup.add(couplerGroup);
  const couplerGeo = track(createLinkBarGeometry(spec.couplerLength, 0.095, 0.046));
  const couplerMesh = new THREE.Mesh(couplerGeo, mats.woodPrimary);
  couplerMesh.castShadow = true;
  couplerMesh.receiveShadow = true;
  couplerMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  couplerGroup.add(couplerMesh);

  // 3. Upper Ratchet Axle & Rear-Wall Stationary Stanchion (x = -0.35)
  const backWallInnerZ = -blueprint.boxDepth * 0.5 + 0.24;
  const stanchionX = -0.35;
  const stanchionArmLen = Math.abs(backWallInnerZ) + 0.06;
  const stanchionArmGeo = track(new THREE.BoxGeometry(0.085, 0.14, stanchionArmLen));
  const stanchionArm = new THREE.Mesh(stanchionArmGeo, mats.woodDarkTrim);
  stanchionArm.position.set(stanchionX, axleY, -stanchionArmLen * 0.5 + 0.03);
  stanchionArm.castShadow = true;
  stanchionArm.receiveShadow = true;
  rootGroup.add(stanchionArm);

  const axleBearingGeo = track(new THREE.CylinderGeometry(0.1, 0.1, 0.11, 16));
  axleBearingGeo.rotateZ(Math.PI / 2);
  const axleBearing = new THREE.Mesh(axleBearingGeo, mats.metalHardware);
  axleBearing.position.set(stanchionX, axleY, 0);
  axleBearing.castShadow = true;
  rootGroup.add(axleBearing);

  // Steel Upper Ratchet Axle spanning from stanchion (-0.38) through bevel gear (+0.16)
  const upperAxleLen = 0.55;
  const upperAxleGeo = track(new THREE.CylinderGeometry(0.044, 0.044, upperAxleLen, 14));
  upperAxleGeo.rotateZ(Math.PI / 2);
  const upperAxle = new THREE.Mesh(upperAxleGeo, mats.steelShaft);
  upperAxle.position.set(-0.38 + upperAxleLen * 0.5, axleY, 0);
  upperAxle.castShadow = true;
  rootGroup.add(upperAxle);

  // 4. Swinging Drive Rocker Arm & Driving Pawl Carrier Lug (x = -0.085)
  const swingArmGroup = new THREE.Group();
  swingArmGroup.position.set(-0.085, axleY, 0);
  rootGroup.add(swingArmGroup);

  const armHubGeo = track(new THREE.CylinderGeometry(0.092, 0.092, 0.065, 16));
  armHubGeo.rotateZ(Math.PI / 2);
  const armHub = new THREE.Mesh(armHubGeo, mats.metalHardware);
  swingArmGroup.add(armHub);

  const rockerBarGeo = track(createLinkBarGeometry(spec.armLinkR, 0.105, 0.046));
  const rockerBar = new THREE.Mesh(rockerBarGeo, mats.woodPrimary);
  rockerBar.castShadow = true;
  rockerBar.userData = { moduleId: mod.id, dragRole: 'module' };
  swingArmGroup.add(rockerBar);

  const rockerPinGeo = track(new THREE.CylinderGeometry(0.032, 0.032, 0.12, 14));
  rockerPinGeo.rotateZ(Math.PI / 2);
  const rockerPinMesh = new THREE.Mesh(rockerPinGeo, mats.metalHardware);
  rockerPinMesh.position.set(-0.035, 0, spec.armLinkR);
  swingArmGroup.add(rockerPinMesh);

  // Carrier lug extending out to pawlOrbitR (0.58)
  const localPivZ = mod.lockDirection * spec.pawlOrbitR * Math.cos(spec.drivePivotLocalAngle);
  const localPivY = spec.pawlOrbitR * Math.sin(spec.drivePivotLocalAngle);
  const carrierLugGroup = new THREE.Group();
  carrierLugGroup.rotation.x = -Math.atan2(localPivY, localPivZ);
  const carrierLugGeo = track(createLinkBarGeometry(spec.pawlOrbitR, 0.09, 0.044));
  const carrierLug = new THREE.Mesh(carrierLugGeo, mats.woodSecondary);
  carrierLug.castShadow = true;
  carrierLug.userData = { moduleId: mod.id, dragRole: 'module' };
  carrierLugGroup.add(carrierLug);
  swingArmGroup.add(carrierLugGroup);

  const drivePivotPinGeo = track(new THREE.CylinderGeometry(0.026, 0.026, 0.155, 12));
  drivePivotPinGeo.rotateZ(Math.PI / 2);
  const drivePivotPin = new THREE.Mesh(drivePivotPinGeo, mats.metalHardware);
  drivePivotPin.position.set(0.05, localPivY, localPivZ);
  swingArmGroup.add(drivePivotPin);

  // 5. Sharp Sawtooth Ratchet Wheel at x = 0 (thickness 0.075 -> [-0.0375, +0.0375])
  const wheelPivot = new THREE.Group();
  wheelPivot.position.set(0, axleY, 0);
  rootGroup.add(wheelPivot);

  const contourPts = sampleRatchetContour(
    spec.teethCount,
    mod.lockDirection,
    spec.wheelRootR,
    spec.wheelTipR
  );
  const wheelShape = new THREE.Shape();
  contourPts.forEach((pt, idx) => {
    if (idx === 0) wheelShape.moveTo(pt.x, pt.y);
    else wheelShape.lineTo(pt.x, pt.y);
  });
  wheelShape.closePath();

  const wheelThick = 0.075;
  const wheelGeo = track(
    new THREE.ExtrudeGeometry(wheelShape, {
      depth: wheelThick,
      bevelEnabled: false,
      curveSegments: 1,
    })
  );
  wheelGeo.translate(0, 0, -wheelThick * 0.5);
  wheelGeo.rotateY(-Math.PI / 2);

  const wheelMesh = new THREE.Mesh(wheelGeo, mats.woodDarkTrim);
  wheelMesh.castShadow = true;
  wheelMesh.receiveShadow = true;
  wheelMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  wheelPivot.add(wheelMesh);

  const wheelBossGeo = track(new THREE.CylinderGeometry(0.14, 0.14, wheelThick + 0.02, 20));
  wheelBossGeo.rotateZ(Math.PI / 2);
  const wheelBoss = new THREE.Mesh(wheelBossGeo, mats.metalHardware);
  wheelPivot.add(wheelBoss);

  // High-contrast crimson index stud on wheel faces so step-and-hold motion reads clearly
  const indexStudGeo = track(new THREE.CylinderGeometry(0.038, 0.038, wheelThick + 0.016, 12));
  indexStudGeo.rotateZ(Math.PI / 2);
  const indexStud = new THREE.Mesh(indexStudGeo, mats.paintedAccent);
  indexStud.position.set(0, 0, 0.24);
  wheelPivot.add(indexStud);

  // 6. Driving Pawl (coplanar with ratchet wheel at x = 0)
  const drivePawlGroup = new THREE.Group();
  rootGroup.add(drivePawlGroup);
  const pawlGeo = track(createRatchetPawlGeometry(spec.pawlLength, 0.056, mod.lockDirection));
  const drivePawlMesh = new THREE.Mesh(pawlGeo, mats.metalHardware);
  drivePawlMesh.castShadow = true;
  drivePawlMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  drivePawlGroup.add(drivePawlMesh);

  // 7. Stationary Holding Pawl & Frame Anchor Bracket (at x = +0.076, above wheel & bevel gear)
  const holdPivotCanon = spec.holdPivotCanonAngle;
  const holdPivZ = mod.lockDirection * spec.pawlOrbitR * Math.cos(holdPivotCanon);
  const holdPivY = axleY + spec.pawlOrbitR * Math.sin(holdPivotCanon);

  const holdFrameX = 0.076;
  const overheadY = axleY + 0.54;
  const anchorStartZ = backWallInnerZ - 0.06;
  const anchorEndZ = holdPivZ + 0.045;
  const anchorLen = Math.abs(anchorEndZ - anchorStartZ);
  const anchorArmGeo = track(new THREE.BoxGeometry(0.036, 0.075, anchorLen));
  const anchorArm = new THREE.Mesh(anchorArmGeo, mats.woodDarkTrim);
  anchorArm.position.set(holdFrameX, overheadY, (anchorStartZ + anchorEndZ) * 0.5);
  anchorArm.castShadow = true;
  rootGroup.add(anchorArm);

  const dropH = Math.abs(overheadY - holdPivY) + 0.055;
  const dropLugGeo = track(new THREE.BoxGeometry(0.036, dropH, 0.085));
  const dropLug = new THREE.Mesh(dropLugGeo, mats.woodDarkTrim);
  dropLug.position.set(holdFrameX, (overheadY + holdPivY) * 0.5, holdPivZ);
  dropLug.castShadow = true;
  rootGroup.add(dropLug);

  const holdPivotPinGeo = track(new THREE.CylinderGeometry(0.026, 0.026, 0.14, 12));
  holdPivotPinGeo.rotateZ(Math.PI / 2);
  const holdPivotPin = new THREE.Mesh(holdPivotPinGeo, mats.steelShaft);
  holdPivotPin.position.set(0.032, holdPivY, holdPivZ);
  rootGroup.add(holdPivotPin);

  const holdPawlGroup = new THREE.Group();
  holdPawlGroup.position.set(0, holdPivY, holdPivZ);
  rootGroup.add(holdPawlGroup);
  const holdPawlMesh = new THREE.Mesh(pawlGeo, mats.paintedSecondary);
  holdPawlMesh.castShadow = true;
  holdPawlMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  holdPawlGroup.add(holdPawlMesh);

  // 8. 90-Degree Bevel Transfer from Upper Ratchet Axle to Vertical Stage Spindle
  const bevelFaceFrac = 0.42;
  const driverBevelPivot = new THREE.Group();
  driverBevelPivot.position.set(spec.bevelDriverX, axleY, 0);
  const driverBevelBuilt = buildConicalBevelGear(
    spec.bevelPitchR,
    spec.bevelPitchR,
    Math.PI / 4,
    spec.bevelTeeth,
    bevelFaceFrac,
    -Math.PI / 2,
    mats.woodSecondary,
    mats.metalHardware,
    track,
    mod.id
  );
  driverBevelBuilt.group.rotation.z = -Math.PI / 2;
  driverBevelPivot.add(driverBevelBuilt.group);
  rootGroup.add(driverBevelPivot);

  const drivenY = axleY + spec.bevelDrivenYOffset;
  const drivenSpindleGroup = new THREE.Group();
  drivenSpindleGroup.position.set(spec.bevelDrivenX, drivenY, 0);
  const drivenBevelBuilt = buildConicalBevelGear(
    spec.bevelPitchR,
    spec.bevelPitchR,
    Math.PI / 4,
    spec.bevelTeeth,
    bevelFaceFrac,
    -Math.PI / 2 + Math.PI / spec.bevelTeeth,
    mats.metalHardware,
    mats.metalHardware,
    track,
    mod.id
  );
  drivenBevelBuilt.group.rotation.x = Math.PI;
  drivenSpindleGroup.add(drivenBevelBuilt.group);

  const spindleLen = Math.max(0.5, blueprint.stageDeckY - drivenY + 0.08);
  const spindleGeo = track(new THREE.CylinderGeometry(0.052, 0.052, spindleLen, 14));
  const spindleMesh = new THREE.Mesh(spindleGeo, mats.steelShaft);
  spindleMesh.position.set(0, spindleLen * 0.5, 0);
  spindleMesh.castShadow = true;
  drivenSpindleGroup.add(spindleMesh);
  rootGroup.add(drivenSpindleGroup);

  // Rear-wall pillow bearing bracket for vertical spindle (y = drivenY + 0.26 = 3.11)
  const bracketY = drivenY + 0.26;
  const bracketLen = blueprint.boxDepth * 0.5 - 0.18;
  const bracketGeo = track(new THREE.BoxGeometry(0.14, 0.1, bracketLen));
  const bracketMesh = new THREE.Mesh(bracketGeo, mats.woodDarkTrim);
  bracketMesh.position.set(spec.bevelDrivenX, bracketY, -bracketLen * 0.5);
  bracketMesh.castShadow = true;
  rootGroup.add(bracketMesh);

  const pillowBearingGeo = track(new THREE.CylinderGeometry(0.105, 0.105, 0.12, 16));
  const pillowBearing = new THREE.Mesh(pillowBearingGeo, mats.metalHardware);
  pillowBearing.position.set(spec.bevelDrivenX, bracketY, 0);
  pillowBearing.castShadow = true;
  rootGroup.add(pillowBearing);

  // Stage Deck thrust collar
  const collarGroup = new THREE.Group();
  collarGroup.position.set(spec.bevelDrivenX, blueprint.stageDeckY, 0);
  const collarGeo = track(new THREE.CylinderGeometry(0.17, 0.19, 0.26, 18));
  const collarMesh = new THREE.Mesh(collarGeo, mats.metalHardware);
  collarMesh.position.set(0, -0.07, 0);
  collarGroup.add(collarMesh);
  rootGroup.add(collarGroup);

  // Topper mounted at stepped spindle exit (rotary mode!)
  const topperRig = buildTopperGroup(mod.topper, mod.topperScale, mats, track, 'rotary');
  topperRig.group.position.set(spec.bevelDrivenX, blueprint.stageDeckY, 0);
  topperRig.group.traverse((obj) => {
    if ((obj as THREE.Mesh).isMesh) {
      obj.userData = { moduleId: mod.id, dragRole: 'module' };
    }
  });
  rootGroup.add(topperRig.group);

  const hitGeo = track(new THREE.BoxGeometry(1.05, blueprint.stageDeckY + 0.9, 1.5));
  const hitMat = new THREE.MeshBasicMaterial({ visible: false });
  const hitMesh = new THREE.Mesh(hitGeo, hitMat);
  hitMesh.position.set(0, (blueprint.stageDeckY + 0.9) * 0.5, 0);
  hitMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  rootGroup.add(hitMesh);

  return {
    id: mod.id,
    type: mod.type,
    rootGroup,
    hitMeshes: [hitMesh, crankDisc, wheelMesh, drivePawlMesh, holdPawlMesh],
    pivotHitMeshes: [],
    highlightRing,
    update: (shaftAngleRad, bp, explodeAmount) => {
      const sol = solveRatchet(shaftAngleRad, mod);
      const curAxleY = bp.shaftY + spec.axleYOffset;

      crankPivot.rotation.x = -sol.crankAngleRad;

      couplerGroup.position.set(-0.14, bp.shaftY + sol.crankPin.y, sol.crankPin.z);
      couplerGroup.rotation.x = -sol.couplerAngleRad;

      swingArmGroup.position.set(-0.085, curAxleY, 0);
      swingArmGroup.rotation.x = -sol.armAngleRad;

      wheelPivot.position.set(0, curAxleY, 0);
      wheelPivot.rotation.x = -sol.ratchetWheelAngleRad;

      drivePawlGroup.position.set(0, bp.shaftY + sol.drivePawlPivot.y, sol.drivePawlPivot.z);
      drivePawlGroup.rotation.x = -sol.drivePawlAngleRad;

      holdPawlGroup.position.set(0, bp.shaftY + sol.holdPawlPivot.y, sol.holdPawlPivot.z);
      holdPawlGroup.rotation.x = -sol.holdPawlAngleRad;

      driverBevelPivot.rotation.x = -sol.ratchetWheelAngleRad;
      drivenSpindleGroup.rotation.y = -sol.drivenOutputAngleRad;

      collarGroup.position.y = bp.stageDeckY + explodeAmount * 1.15;
      topperRig.group.position.y = bp.stageDeckY + explodeAmount * 1.15;

      const drive01 = (Math.sin(sol.drivenOutputAngleRad) + 1) * 0.5;
      topperRig.update(drive01, sol.drivenOutputAngleRad, 0);
    },
    setSelected: (selected) => {
      highlightRing.visible = selected;
    },
    dispose: () => {
      hitMat.dispose();
      for (const g of disposables) g.dispose();
    },
  };
}

/**
 * Builds a 90-degree Orthogonal Circular Cam Rotary Friction Drive:
 * - A vertical circular cam disc on the main horizontal shaft presses tangentially onto
 *   the flat underside of a horizontal circular turntable disc on the vertical output spindle;
 * - Pure rolling contact turns the vertical spindle & cross-shaped propeller continuously.
 */
export function buildCircularCamModuleRig(
  mod: CircularCamModule,
  blueprint: AutomataBlueprint,
  mats: AutomataMaterialSet,
  resolveAccumDrivenAngle?: (
    shaftAngleRad: number,
    curMod: CircularCamModule
  ) => number
): ModuleRig {
  const disposables: THREE.BufferGeometry[] = [];
  const track = <T extends THREE.BufferGeometry>(g: T): T => {
    disposables.push(g);
    return g;
  };

  const camShape = mod.camShape ?? 'circle';
  const spec = solveCircularCamGeometry(
    mod.gearRatio,
    mod.driverRadius,
    mod.camOffsetRatio ?? -1,
    camShape
  );

  const rootGroup = new THREE.Group();
  rootGroup.position.set(mod.slotX, 0, 0);

  const isPolygon = spec.camShape !== 'circle';
  const hlGeo = track(
    isPolygon
      ? new THREE.TorusGeometry(spec.drivenDiscRadius + 0.045, 0.022, 10, 32)
      : new THREE.TorusGeometry(spec.driverRadius + 0.055, 0.024, 10, 32)
  );
  if (isPolygon) {
    hlGeo.rotateX(Math.PI / 2);
  } else {
    hlGeo.rotateY(Math.PI / 2);
  }
  const highlightRing = new THREE.Mesh(hlGeo, mats.selectionHighlight);
  highlightRing.position.set(
    isPolygon ? spec.drivenX : spec.driverX,
    isPolygon ? blueprint.shaftY + spec.driverRadius + spec.drivenDiscThickness * 0.5 : blueprint.shaftY,
    0
  );
  highlightRing.visible = false;
  rootGroup.add(highlightRing);

  // 1. Vertical Driver Cam on Horizontal Camshaft (Circle / Triangle / Square; slides along X under tray)
  const driverPivot = new THREE.Group();
  driverPivot.position.set(spec.driverX, blueprint.shaftY, 0);
  rootGroup.add(driverPivot);

  let rimBand: THREE.Mesh;
  let coreDisc: THREE.Mesh;

  if (spec.camShape === 'circle') {
    const rimBandGeo = track(
      new THREE.CylinderGeometry(spec.driverRadius, spec.driverRadius, spec.driverThickness * 0.78, 36)
    );
    rimBandGeo.rotateZ(Math.PI / 2);
    rimBand = new THREE.Mesh(rimBandGeo, mats.woodDarkTrim);
    rimBand.castShadow = true;
    rimBand.receiveShadow = true;
    rimBand.userData = { moduleId: mod.id, dragRole: 'cam-offset' };
    driverPivot.add(rimBand);

    const coreDiscR = Math.max(0.14, spec.driverRadius - 0.032);
    const coreDiscGeo = track(
      new THREE.CylinderGeometry(coreDiscR, coreDiscR, spec.driverThickness, 32)
    );
    coreDiscGeo.rotateZ(Math.PI / 2);
    coreDisc = new THREE.Mesh(coreDiscGeo, mats.woodPrimary);
    coreDisc.castShadow = true;
    coreDisc.receiveShadow = true;
    coreDisc.userData = { moduleId: mod.id, dragRole: 'cam-offset' };
    driverPivot.add(coreDisc);

    // 4 radial brass index pins on vertical cam cheek so circular rotation reads clearly
    const driverPinGeo = track(
      new THREE.CylinderGeometry(0.024, 0.024, spec.driverThickness + 0.018, 12)
    );
    driverPinGeo.rotateZ(Math.PI / 2);
    const pinOrbit = coreDiscR * 0.65;
    for (let i = 0; i < 4; i++) {
      const a = (i * Math.PI) / 2;
      const pin = new THREE.Mesh(driverPinGeo, i === 0 ? mats.paintedAccent : mats.metalHardware);
      pin.position.set(0, pinOrbit * Math.sin(a), pinOrbit * Math.cos(a));
      driverPivot.add(pin);
    }
  } else {
    const sides = spec.camShape === 'triangle' ? 3 : 4;
    const makePolygonPath = (radius: number, reverse = false): THREE.Path => {
      const path = new THREE.Path();
      for (let i = 0; i < sides; i++) {
        const idx = reverse ? (sides - i) % sides : i;
        const alpha = Math.PI * 0.5 - (idx * Math.PI * 2) / sides;
        const sx = -radius * Math.cos(alpha);
        const sy = radius * Math.sin(alpha);
        if (i === 0) path.moveTo(sx, sy);
        else path.lineTo(sx, sy);
      }
      path.closePath();
      return path;
    };

    const rimShape = new THREE.Shape();
    for (let i = 0; i < sides; i++) {
      const alpha = Math.PI * 0.5 - (i * Math.PI * 2) / sides;
      const sx = -spec.driverRadius * Math.cos(alpha);
      const sy = spec.driverRadius * Math.sin(alpha);
      if (i === 0) rimShape.moveTo(sx, sy);
      else rimShape.lineTo(sx, sy);
    }
    rimShape.closePath();
    const rimInnerR = Math.max(0.14, spec.driverRadius - 0.036);
    rimShape.holes.push(makePolygonPath(rimInnerR, true));

    const rimBandGeo = track(
      new THREE.ExtrudeGeometry(rimShape, {
        depth: spec.driverThickness * 0.78,
        bevelEnabled: false,
      })
    );
    rimBandGeo.translate(0, 0, -(spec.driverThickness * 0.78) * 0.5);
    rimBandGeo.rotateY(Math.PI / 2);
    rimBand = new THREE.Mesh(rimBandGeo, mats.woodDarkTrim);
    rimBand.castShadow = true;
    rimBand.receiveShadow = true;
    rimBand.userData = { moduleId: mod.id, dragRole: 'cam-offset' };
    driverPivot.add(rimBand);

    const coreR = Math.max(0.15, spec.driverRadius - 0.032);
    const coreShape = new THREE.Shape();
    for (let i = 0; i < sides; i++) {
      const alpha = Math.PI * 0.5 - (i * Math.PI * 2) / sides;
      const sx = -coreR * Math.cos(alpha);
      const sy = coreR * Math.sin(alpha);
      if (i === 0) coreShape.moveTo(sx, sy);
      else coreShape.lineTo(sx, sy);
    }
    coreShape.closePath();

    const coreDiscGeo = track(
      new THREE.ExtrudeGeometry(coreShape, {
        depth: spec.driverThickness,
        bevelEnabled: false,
      })
    );
    coreDiscGeo.translate(0, 0, -spec.driverThickness * 0.5);
    coreDiscGeo.rotateY(Math.PI / 2);
    coreDisc = new THREE.Mesh(coreDiscGeo, mats.woodPrimary);
    coreDisc.castShadow = true;
    coreDisc.receiveShadow = true;
    coreDisc.userData = { moduleId: mod.id, dragRole: 'cam-offset' };
    driverPivot.add(coreDisc);

    // Vertex index pins pointing toward each polygon corner
    const driverPinGeo = track(
      new THREE.CylinderGeometry(0.022, 0.022, spec.driverThickness + 0.018, 12)
    );
    driverPinGeo.rotateZ(Math.PI / 2);
    const pinOrbit = Math.max(0.135, spec.driverRadius * 0.62);
    for (let i = 0; i < sides; i++) {
      const a = Math.PI * 0.5 - (i * Math.PI * 2) / sides;
      const pin = new THREE.Mesh(driverPinGeo, i === 0 ? mats.paintedAccent : mats.metalHardware);
      pin.position.set(0, pinOrbit * Math.sin(a), pinOrbit * Math.cos(a));
      driverPivot.add(pin);
    }
  }

  const hubR = Math.min(0.108, Math.max(0.088, spec.inradius * 0.68));
  const driverHubGeo = track(new THREE.CylinderGeometry(hubR, hubR, spec.driverThickness + 0.044, 18));
  driverHubGeo.rotateZ(Math.PI / 2);
  const driverHub = new THREE.Mesh(driverHubGeo, mats.metalHardware);
  driverHub.castShadow = true;
  driverHub.userData = { moduleId: mod.id, dragRole: 'cam-offset' };
  driverPivot.add(driverHub);

  // Crimson drag-affordance rings on sliding cam hub (clamped inside polygon inradius so flat faces never clip tray)
  const shiftRingMajorR = Math.min(0.135, Math.max(0.096, spec.inradius * 0.7));
  const shiftRingTubeR = spec.camShape === 'triangle' ? 0.015 : 0.018;
  const shiftRingGeo = track(new THREE.TorusGeometry(shiftRingMajorR, shiftRingTubeR, 8, 22));
  shiftRingGeo.rotateY(Math.PI / 2);
  const shiftRingPos = new THREE.Mesh(shiftRingGeo, mats.paintedAccent);
  shiftRingPos.position.set(spec.driverThickness * 0.5 + 0.024, 0, 0);
  shiftRingPos.userData = { moduleId: mod.id, dragRole: 'cam-offset' };
  driverPivot.add(shiftRingPos);

  const shiftRingNeg = new THREE.Mesh(shiftRingGeo, mats.paintedAccent);
  shiftRingNeg.position.set(-(spec.driverThickness * 0.5 + 0.024), 0, 0);
  shiftRingNeg.userData = { moduleId: mod.id, dragRole: 'cam-offset' };
  driverPivot.add(shiftRingNeg);

  // 2. Horizontal Driven Circular Turntable Disc on Vertical Spindle (centered at drivenX = 0)
  const drivenY = blueprint.shaftY + spec.drivenYOffset;
  const drivenSpindleGroup = new THREE.Group();
  drivenSpindleGroup.position.set(spec.drivenX, drivenY, 0);
  rootGroup.add(drivenSpindleGroup);

  const padH = 0.024;
  const frictionPadGeo = track(
    new THREE.CylinderGeometry(spec.drivenDiscRadius, spec.drivenDiscRadius, padH, 36)
  );
  const frictionPad = new THREE.Mesh(frictionPadGeo, mats.woodDarkTrim);
  frictionPad.position.set(0, padH * 0.5, 0);
  frictionPad.castShadow = true;
  frictionPad.receiveShadow = true;
  frictionPad.userData = { moduleId: mod.id, dragRole: 'module' };
  drivenSpindleGroup.add(frictionPad);

  const upperPlateH = spec.drivenDiscThickness - padH;
  const upperPlateR = Math.max(0.24, spec.drivenDiscRadius - 0.018);
  const upperPlateGeo = track(
    new THREE.CylinderGeometry(upperPlateR, upperPlateR, upperPlateH, 36)
  );
  const upperPlate = new THREE.Mesh(upperPlateGeo, mats.woodPrimary);
  upperPlate.position.set(0, padH + upperPlateH * 0.5, 0);
  upperPlate.castShadow = true;
  upperPlate.receiveShadow = true;
  upperPlate.userData = { moduleId: mod.id, dragRole: 'module' };
  drivenSpindleGroup.add(upperPlate);

  const topBossGeo = track(new THREE.CylinderGeometry(0.115, 0.135, 0.055, 18));
  const topBoss = new THREE.Mesh(topBossGeo, mats.metalHardware);
  topBoss.position.set(0, spec.drivenDiscThickness + 0.026, 0);
  topBoss.castShadow = true;
  drivenSpindleGroup.add(topBoss);

  // 4 radial marker studs on top face of horizontal turntable disc
  const topStudGeo = track(new THREE.CylinderGeometry(0.024, 0.024, 0.02, 12));
  const topStudOrbit = upperPlateR * 0.68;
  for (let i = 0; i < 4; i++) {
    const a = (i * Math.PI) / 2;
    const stud = new THREE.Mesh(topStudGeo, i === 0 ? mats.paintedAccent : mats.metalHardware);
    stud.position.set(
      topStudOrbit * Math.cos(a),
      spec.drivenDiscThickness + 0.008,
      topStudOrbit * Math.sin(a)
    );
    drivenSpindleGroup.add(stud);
  }

  // Vertical steel spindle rising from inside turntable boss into stage deck collar
  // Sized so it remains cleanly engaged inside the brass deck collar even at lowest triangle/square drop
  const spindleStartY = spec.drivenDiscThickness * 0.5;
  const spindleLen = Math.max(0.5, blueprint.stageDeckY + 0.09 - (drivenY + spindleStartY));
  const spindleGeo = track(new THREE.CylinderGeometry(0.052, 0.052, spindleLen, 14));
  const spindleMesh = new THREE.Mesh(spindleGeo, mats.steelShaft);
  spindleMesh.position.set(0, spindleStartY + spindleLen * 0.5, 0);
  spindleMesh.castShadow = true;
  drivenSpindleGroup.add(spindleMesh);

  // 3. Rear-Wall Support Bracket & Brass Radial Pillow Bearing
  const bracketY = drivenY + 0.34;
  const bracketLen = blueprint.boxDepth * 0.5 - 0.18;
  const bracketGeo = track(new THREE.BoxGeometry(0.15, 0.11, bracketLen));
  const bracketMesh = new THREE.Mesh(bracketGeo, mats.woodDarkTrim);
  bracketMesh.position.set(spec.drivenX, bracketY, -bracketLen * 0.5);
  bracketMesh.castShadow = true;
  rootGroup.add(bracketMesh);

  const pillowBearingGeo = track(new THREE.CylinderGeometry(0.11, 0.11, 0.13, 16));
  const pillowBearing = new THREE.Mesh(pillowBearingGeo, mats.metalHardware);
  pillowBearing.position.set(spec.drivenX, bracketY, 0);
  pillowBearing.castShadow = true;
  rootGroup.add(pillowBearing);

  // 4. Stage Deck Thrust Collar
  const collarGroup = new THREE.Group();
  collarGroup.position.set(spec.drivenX, blueprint.stageDeckY, 0);
  const collarGeo = track(new THREE.CylinderGeometry(0.17, 0.19, 0.26, 18));
  const collarMesh = new THREE.Mesh(collarGeo, mats.metalHardware);
  collarMesh.position.set(0, -0.07, 0);
  collarGroup.add(collarMesh);
  rootGroup.add(collarGroup);

  // 5. Topper Sculpture mounted at rotary spindle exit (defaults to Cross-Shaped Propeller)
  const topperRig = buildTopperGroup(mod.topper, mod.topperScale, mats, track, 'rotary');
  topperRig.group.position.set(spec.drivenX, blueprint.stageDeckY, 0);
  topperRig.group.traverse((obj) => {
    if ((obj as THREE.Mesh).isMesh) {
      obj.userData = { moduleId: mod.id, dragRole: 'module' };
    }
  });
  rootGroup.add(topperRig.group);

  const hitGeo = track(new THREE.BoxGeometry(1.0, blueprint.stageDeckY + 0.9, 1.4));
  const hitMat = new THREE.MeshBasicMaterial({ visible: false });
  const hitMesh = new THREE.Mesh(hitGeo, hitMat);
  hitMesh.position.set(0, (blueprint.stageDeckY + 0.9) * 0.5, 0);
  hitMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  rootGroup.add(hitMesh);

  return {
    id: mod.id,
    type: mod.type,
    rootGroup,
    hitMeshes: [hitMesh, frictionPad, upperPlate],
    pivotHitMeshes: [],
    camOffsetHitMeshes: [rimBand, coreDisc, driverHub, shiftRingPos, shiftRingNeg],
    highlightRing,
    update: (shaftAngleRad, bp, explodeAmount) => {
      const sol = solveCircularCam(shaftAngleRad, mod);
      const drivenAngle = resolveAccumDrivenAngle
        ? resolveAccumDrivenAngle(shaftAngleRad, mod)
        : sol.drivenAngleRad;
      driverPivot.rotation.x = -sol.driverAngleRad;
      drivenSpindleGroup.position.y = bp.shaftY + sol.trayBottomY + explodeAmount * 0.28;
      drivenSpindleGroup.rotation.y = -drivenAngle;
      if (isPolygon) {
        highlightRing.position.y =
          drivenSpindleGroup.position.y + spec.drivenDiscThickness * 0.5;
      }

      collarGroup.position.y = bp.stageDeckY + explodeAmount * 1.15;
      topperRig.group.position.y = bp.stageDeckY + explodeAmount * 1.15;

      const drive01 = (Math.sin(drivenAngle) + 1) * 0.5;
      topperRig.update(drive01, drivenAngle, 0, sol.liftDisplacement);
    },
    setSelected: (selected) => {
      highlightRing.visible = selected;
      const maxScale = spec.camShape === 'triangle' ? 1.12 : 1.18;
      const ringScale = selected ? maxScale : 1.0;
      shiftRingPos.scale.setScalar(ringScale);
      shiftRingNeg.scale.setScalar(ringScale);
    },
    dispose: () => {
      hitMat.dispose();
      for (const g of disposables) g.dispose();
    },
  };
}

/**
 * Builds a Twin Eccentric Alternating Cam Drive (一对偏心双凸轮):
 * - Two identical eccentric circular cams keyed 180 degrees apart on the horizontal shaft
 *   sit side-by-side underneath a single shared horizontal turntable tray.
 * - As the camshaft turns, the higher eccentric lobe rises into flush tangential contact with
 *   the tray's underside (while the opposite cam drops clear), alternating between Left Cam
 *   (forward drive + lift) and Right Cam (reverse drive + lift).
 */
export function buildDualEccentricModuleRig(
  mod: DualEccentricModule,
  blueprint: AutomataBlueprint,
  mats: AutomataMaterialSet
): ModuleRig {
  const disposables: THREE.BufferGeometry[] = [];
  const track = <T extends THREE.BufferGeometry>(g: T): T => {
    disposables.push(g);
    return g;
  };

  const spec = solveDualEccentricGeometry(mod.camRadius, mod.eccentricity, mod.gearRatio);

  const rootGroup = new THREE.Group();
  rootGroup.position.set(mod.slotX, 0, 0);

  const hlGeo = track(
    new THREE.TorusGeometry(Math.max(0.16, spec.camRadius - 0.035), 0.022, 10, 32)
  );
  hlGeo.rotateY(Math.PI / 2);
  const highlightRing = new THREE.Mesh(hlGeo, mats.selectionHighlight);
  highlightRing.position.set(0, blueprint.shaftY, 0);
  highlightRing.visible = false;
  rootGroup.add(highlightRing);

  // Central brass spacer sleeve between the Left and Right eccentric cams
  const spacerLen = Math.max(0.18, spec.camHalfSpacing * 2 - spec.camThickness - 0.02);
  const spacerGeo = track(new THREE.CylinderGeometry(0.102, 0.102, spacerLen, 16));
  spacerGeo.rotateZ(Math.PI / 2);
  const spacerMesh = new THREE.Mesh(spacerGeo, mats.metalHardware);
  spacerMesh.position.set(0, blueprint.shaftY, 0);
  spacerMesh.castShadow = true;
  spacerMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  rootGroup.add(spacerMesh);

  // 1. Twin 180-deg Out-of-Phase Eccentric Circular Cams on Main Camshaft
  const rimBandGeo = track(
    new THREE.CylinderGeometry(spec.camRadius, spec.camRadius, spec.camThickness * 0.78, 36)
  );
  rimBandGeo.rotateZ(Math.PI / 2);

  const coreDiscR = Math.max(0.14, spec.camRadius - 0.03);
  const coreDiscGeo = track(
    new THREE.CylinderGeometry(coreDiscR, coreDiscR, spec.camThickness, 32)
  );
  coreDiscGeo.rotateZ(Math.PI / 2);

  const hubGeo = track(new THREE.CylinderGeometry(0.114, 0.114, spec.camThickness + 0.034, 18));
  hubGeo.rotateZ(Math.PI / 2);

  const peakPinGeo = track(new THREE.CylinderGeometry(0.026, 0.026, spec.camThickness + 0.02, 12));
  peakPinGeo.rotateZ(Math.PI / 2);

  const buildEccentricWheel = (xPos: number, lobeAccentMat: THREE.Material) => {
    const pivot = new THREE.Group();
    pivot.position.set(xPos, blueprint.shaftY, 0);
    rootGroup.add(pivot);

    // Shaft-axis brass collar (stays concentric with main shaft)
    const hub = new THREE.Mesh(hubGeo, mats.metalHardware);
    hub.castShadow = true;
    hub.userData = { moduleId: mod.id, dragRole: 'module' };
    pivot.add(hub);

    // Eccentric circular cam body offset by +eccentricity along local +Z
    const rimBand = new THREE.Mesh(rimBandGeo, mats.woodDarkTrim);
    rimBand.position.set(0, 0, spec.eccentricity);
    rimBand.castShadow = true;
    rimBand.receiveShadow = true;
    rimBand.userData = { moduleId: mod.id, dragRole: 'module' };
    pivot.add(rimBand);

    const coreDisc = new THREE.Mesh(coreDiscGeo, mats.woodPrimary);
    coreDisc.position.set(0, 0, spec.eccentricity);
    coreDisc.castShadow = true;
    coreDisc.receiveShadow = true;
    coreDisc.userData = { moduleId: mod.id, dragRole: 'module' };
    pivot.add(coreDisc);

    // High-lobe marker pin on eccentric peak side so alternating 180-deg phase reads immediately
    const peakPin = new THREE.Mesh(peakPinGeo, lobeAccentMat);
    peakPin.position.set(0, 0, spec.eccentricity + coreDiscR * 0.66);
    pivot.add(peakPin);

    return { pivot, rimBand, coreDisc };
  };

  const leftCam = buildEccentricWheel(spec.leftCamX, mats.paintedAccent);
  const rightCam = buildEccentricWheel(spec.rightCamX, mats.paintedSecondary);

  // 2. Shared Horizontal Circular Turntable Tray riding atop the alternating eccentric cams
  const baseDrivenY = blueprint.shaftY + spec.baseDrivenYOffset;
  const drivenSpindleGroup = new THREE.Group();
  drivenSpindleGroup.position.set(spec.drivenX, baseDrivenY, 0);
  rootGroup.add(drivenSpindleGroup);

  const padH = 0.024;
  const frictionPadGeo = track(
    new THREE.CylinderGeometry(spec.drivenDiscRadius, spec.drivenDiscRadius, padH, 36)
  );
  const frictionPad = new THREE.Mesh(frictionPadGeo, mats.woodDarkTrim);
  frictionPad.position.set(0, padH * 0.5, 0);
  frictionPad.castShadow = true;
  frictionPad.receiveShadow = true;
  frictionPad.userData = { moduleId: mod.id, dragRole: 'module' };
  drivenSpindleGroup.add(frictionPad);

  const upperPlateH = spec.drivenDiscThickness - padH;
  const upperPlateR = Math.max(0.24, spec.drivenDiscRadius - 0.018);
  const upperPlateGeo = track(
    new THREE.CylinderGeometry(upperPlateR, upperPlateR, upperPlateH, 36)
  );
  const upperPlate = new THREE.Mesh(upperPlateGeo, mats.woodPrimary);
  upperPlate.position.set(0, padH + upperPlateH * 0.5, 0);
  upperPlate.castShadow = true;
  upperPlate.receiveShadow = true;
  upperPlate.userData = { moduleId: mod.id, dragRole: 'module' };
  drivenSpindleGroup.add(upperPlate);

  const topBossGeo = track(new THREE.CylinderGeometry(0.115, 0.135, 0.055, 18));
  const topBoss = new THREE.Mesh(topBossGeo, mats.metalHardware);
  topBoss.position.set(0, spec.drivenDiscThickness + 0.026, 0);
  topBoss.castShadow = true;
  drivenSpindleGroup.add(topBoss);

  const topStudGeo = track(new THREE.CylinderGeometry(0.024, 0.024, 0.02, 12));
  const topStudOrbit = upperPlateR * 0.68;
  for (let i = 0; i < 4; i++) {
    const a = (i * Math.PI) / 2;
    const stud = new THREE.Mesh(topStudGeo, i === 0 ? mats.paintedAccent : mats.metalHardware);
    stud.position.set(
      topStudOrbit * Math.cos(a),
      spec.drivenDiscThickness + 0.008,
      topStudOrbit * Math.sin(a)
    );
    drivenSpindleGroup.add(stud);
  }

  // Vertical steel quill spindle extending cleanly through rear pillow bearing & deck collar across lift stroke
  const spindleStartY = spec.drivenDiscThickness * 0.5;
  const spindleLen = Math.max(0.55, blueprint.stageDeckY - (baseDrivenY + spindleStartY) + 0.08);
  const spindleGeo = track(new THREE.CylinderGeometry(0.052, 0.052, spindleLen, 14));
  const spindleMesh = new THREE.Mesh(spindleGeo, mats.steelShaft);
  spindleMesh.position.set(0, spindleStartY + spindleLen * 0.5, 0);
  spindleMesh.castShadow = true;
  drivenSpindleGroup.add(spindleMesh);

  // 3. Rear-Wall Support Bracket & Brass Guide Sleeve (clears max eccentric lift safely)
  const bracketY = blueprint.shaftY + spec.maxDrivenYOffset + 0.42;
  const bracketLen = blueprint.boxDepth * 0.5 - 0.18;
  const bracketGeo = track(new THREE.BoxGeometry(0.15, 0.11, bracketLen));
  const bracketMesh = new THREE.Mesh(bracketGeo, mats.woodDarkTrim);
  bracketMesh.position.set(spec.drivenX, bracketY, -bracketLen * 0.5);
  bracketMesh.castShadow = true;
  rootGroup.add(bracketMesh);

  const pillowBearingGeo = track(new THREE.CylinderGeometry(0.11, 0.11, 0.13, 16));
  const pillowBearing = new THREE.Mesh(pillowBearingGeo, mats.metalHardware);
  pillowBearing.position.set(spec.drivenX, bracketY, 0);
  pillowBearing.castShadow = true;
  rootGroup.add(pillowBearing);

  // 4. Stage Deck Thrust Collar
  const collarGroup = new THREE.Group();
  collarGroup.position.set(spec.drivenX, blueprint.stageDeckY, 0);
  const collarGeo = track(new THREE.CylinderGeometry(0.17, 0.19, 0.26, 18));
  const collarMesh = new THREE.Mesh(collarGeo, mats.metalHardware);
  collarMesh.position.set(0, -0.07, 0);
  collarGroup.add(collarMesh);
  rootGroup.add(collarGroup);

  // 5. Topper Sculpture mounted on Stage Deck (rotary alternating drive)
  const topperRig = buildTopperGroup(mod.topper, mod.topperScale, mats, track, 'rotary');
  topperRig.group.position.set(spec.drivenX, blueprint.stageDeckY, 0);
  topperRig.group.traverse((obj) => {
    if ((obj as THREE.Mesh).isMesh) {
      obj.userData = { moduleId: mod.id, dragRole: 'module' };
    }
  });
  rootGroup.add(topperRig.group);

  const hitGeo = track(new THREE.BoxGeometry(1.06, blueprint.stageDeckY + 0.9, 1.4));
  const hitMat = new THREE.MeshBasicMaterial({ visible: false });
  const hitMesh = new THREE.Mesh(hitGeo, hitMat);
  hitMesh.position.set(0, (blueprint.stageDeckY + 0.9) * 0.5, 0);
  hitMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  rootGroup.add(hitMesh);

  return {
    id: mod.id,
    type: mod.type,
    rootGroup,
    hitMeshes: [
      hitMesh,
      spacerMesh,
      leftCam.rimBand,
      leftCam.coreDisc,
      rightCam.rimBand,
      rightCam.coreDisc,
      frictionPad,
      upperPlate,
    ],
    pivotHitMeshes: [],
    highlightRing,
    update: (shaftAngleRad, bp, explodeAmount) => {
      const sol = solveDualEccentric(shaftAngleRad, mod);
      leftCam.pivot.rotation.x = -sol.leftCamAngleRad;
      rightCam.pivot.rotation.x = -sol.rightCamAngleRad;

      drivenSpindleGroup.position.y = bp.shaftY + sol.trayBottomY + explodeAmount * 0.28;
      drivenSpindleGroup.rotation.y = -sol.drivenAngleRad;

      collarGroup.position.y = bp.stageDeckY + explodeAmount * 1.15;
      topperRig.group.position.y = bp.stageDeckY + explodeAmount * 1.15;

      const drive01 = (Math.sin(sol.drivenAngleRad) + 1) * 0.5;
      topperRig.update(drive01, sol.drivenAngleRad, 0);
    },
    setSelected: (selected) => {
      highlightRing.visible = selected;
    },
    dispose: () => {
      hitMat.dispose();
      for (const g of disposables) g.dispose();
    },
  };
}

/**
 * Builds a 90-degree Orthogonal Semicircular Cam Intermittent Rotary Drive (正交半圆凸轮间歇机构):
 * - A vertical half-moon semicircular cam on the horizontal camshaft engages the underside of
 *   the horizontal turntable tray for 180 deg of each revolution (turning the upper arrow),
 *   then rotates its recessed flat side up for the remaining 180 deg so the arrow pauses cleanly.
 */
export function buildSemiCircularCamModuleRig(
  mod: SemiCircularCamModule,
  blueprint: AutomataBlueprint,
  mats: AutomataMaterialSet,
  resolveIntermittentDrivenAngle?: (
    shaftAngleRad: number,
    mod: SemiCircularCamModule
  ) => number
): ModuleRig {
  const disposables: THREE.BufferGeometry[] = [];
  const track = <T extends THREE.BufferGeometry>(g: T): T => {
    disposables.push(g);
    return g;
  };

  const spec = solveSemiCircularCamGeometry(
    mod.gearRatio,
    mod.driverRadius,
    mod.camOffsetRatio ?? -1
  );

  const rootGroup = new THREE.Group();
  rootGroup.position.set(mod.slotX, 0, 0);

  const hlGeo = track(
    new THREE.TorusGeometry(Math.max(0.18, spec.driverRadius - 0.045), 0.022, 10, 32)
  );
  hlGeo.rotateY(Math.PI / 2);
  const highlightRing = new THREE.Mesh(hlGeo, mats.selectionHighlight);
  highlightRing.position.set(spec.driverX, blueprint.shaftY, 0);
  highlightRing.visible = false;
  rootGroup.add(highlightRing);

  // 1. Vertical Semicircular Driver Cam on Horizontal Camshaft (slides independently along X under tray)
  const driverPivot = new THREE.Group();
  driverPivot.position.set(spec.driverX, blueprint.shaftY, 0);
  rootGroup.add(driverPivot);

  // Build wooden half-moon body (core radius r - 0.028 on active 180-deg side + recessed hubRadius on chord side)
  const coreR = Math.max(spec.hubRadius + 0.05, spec.driverRadius - 0.028);
  const coreShape = new THREE.Shape();
  const arcSegs = 32;
  for (let i = 0; i <= arcSegs; i++) {
    const phi = Math.PI * 0.5 - (i / arcSegs) * Math.PI;
    const sx = -coreR * Math.cos(phi);
    const sy = coreR * Math.sin(phi);
    if (i === 0) coreShape.moveTo(sx, sy);
    else coreShape.lineTo(sx, sy);
  }
  for (let i = 0; i <= 16; i++) {
    const psi = -Math.PI * 0.5 + (i / 16) * Math.PI;
    const sx = spec.hubRadius * Math.cos(psi);
    const sy = spec.hubRadius * Math.sin(psi);
    coreShape.lineTo(sx, sy);
  }
  coreShape.closePath();

  const coreGeo = track(
    new THREE.ExtrudeGeometry(coreShape, {
      depth: spec.driverThickness,
      bevelEnabled: false,
      curveSegments: 24,
    })
  );
  coreGeo.translate(0, 0, -spec.driverThickness * 0.5);
  coreGeo.rotateY(Math.PI / 2);
  const coreMesh = new THREE.Mesh(coreGeo, mats.woodPrimary);
  coreMesh.castShadow = true;
  coreMesh.receiveShadow = true;
  coreMesh.userData = { moduleId: mod.id, dragRole: 'cam-offset' };
  driverPivot.add(coreMesh);

  // Curved dark high-friction rim arc spanning active semicircle [−PI/2, +PI/2]
  const rimInnerR = Math.max(spec.hubRadius + 0.04, spec.driverRadius - 0.032);
  const rimShape = new THREE.Shape();
  for (let i = 0; i <= arcSegs; i++) {
    const phi = Math.PI * 0.5 - (i / arcSegs) * Math.PI;
    const sx = -spec.driverRadius * Math.cos(phi);
    const sy = spec.driverRadius * Math.sin(phi);
    if (i === 0) rimShape.moveTo(sx, sy);
    else rimShape.lineTo(sx, sy);
  }
  for (let i = 0; i <= arcSegs; i++) {
    const phi = -Math.PI * 0.5 + (i / arcSegs) * Math.PI;
    const sx = -rimInnerR * Math.cos(phi);
    const sy = rimInnerR * Math.sin(phi);
    rimShape.lineTo(sx, sy);
  }
  rimShape.closePath();

  const rimBandGeo = track(
    new THREE.ExtrudeGeometry(rimShape, {
      depth: spec.driverThickness * 0.8,
      bevelEnabled: false,
      curveSegments: 24,
    })
  );
  rimBandGeo.translate(0, 0, -(spec.driverThickness * 0.8) * 0.5);
  rimBandGeo.rotateY(Math.PI / 2);
  const rimBand = new THREE.Mesh(rimBandGeo, mats.woodDarkTrim);
  rimBand.castShadow = true;
  rimBand.receiveShadow = true;
  rimBand.userData = { moduleId: mod.id, dragRole: 'cam-offset' };
  driverPivot.add(rimBand);

  // Brass shaft collar hub + crimson independent-drag rings
  const driverHubGeo = track(
    new THREE.CylinderGeometry(0.108, 0.108, spec.driverThickness + 0.044, 18)
  );
  driverHubGeo.rotateZ(Math.PI / 2);
  const driverHub = new THREE.Mesh(driverHubGeo, mats.metalHardware);
  driverHub.castShadow = true;
  driverHub.userData = { moduleId: mod.id, dragRole: 'cam-offset' };
  driverPivot.add(driverHub);

  const shiftRingGeo = track(new THREE.TorusGeometry(0.135, 0.018, 8, 22));
  shiftRingGeo.rotateY(Math.PI / 2);
  const shiftRingPos = new THREE.Mesh(shiftRingGeo, mats.paintedAccent);
  shiftRingPos.position.set(spec.driverThickness * 0.5 + 0.024, 0, 0);
  shiftRingPos.userData = { moduleId: mod.id, dragRole: 'cam-offset' };
  driverPivot.add(shiftRingPos);

  const shiftRingNeg = new THREE.Mesh(shiftRingGeo, mats.paintedAccent);
  shiftRingNeg.position.set(-(spec.driverThickness * 0.5 + 0.024), 0, 0);
  shiftRingNeg.userData = { moduleId: mod.id, dragRole: 'cam-offset' };
  driverPivot.add(shiftRingNeg);

  // 3 radial index pins on the semicircular wing (center pin crimson)
  const pinGeo = track(
    new THREE.CylinderGeometry(0.022, 0.022, spec.driverThickness + 0.018, 12)
  );
  pinGeo.rotateZ(Math.PI / 2);
  const pinOrbit = (spec.hubRadius + coreR) * 0.54;
  for (const [idx, phi] of [-Math.PI / 3, 0, Math.PI / 3].entries()) {
    const pin = new THREE.Mesh(pinGeo, idx === 1 ? mats.paintedAccent : mats.metalHardware);
    pin.position.set(0, pinOrbit * Math.sin(phi), pinOrbit * Math.cos(phi));
    driverPivot.add(pin);
  }

  // 2. Horizontal Driven Turntable Disc on Vertical Spindle (centered at drivenX = 0)
  const drivenY = blueprint.shaftY + spec.drivenYOffset;
  const drivenSpindleGroup = new THREE.Group();
  drivenSpindleGroup.position.set(spec.drivenX, drivenY, 0);
  rootGroup.add(drivenSpindleGroup);

  const padH = 0.024;
  const frictionPadGeo = track(
    new THREE.CylinderGeometry(spec.drivenDiscRadius, spec.drivenDiscRadius, padH, 36)
  );
  const frictionPad = new THREE.Mesh(frictionPadGeo, mats.woodDarkTrim);
  frictionPad.position.set(0, padH * 0.5, 0);
  frictionPad.castShadow = true;
  frictionPad.receiveShadow = true;
  frictionPad.userData = { moduleId: mod.id, dragRole: 'module' };
  drivenSpindleGroup.add(frictionPad);

  const upperPlateH = spec.drivenDiscThickness - padH;
  const upperPlateR = Math.max(0.24, spec.drivenDiscRadius - 0.018);
  const upperPlateGeo = track(
    new THREE.CylinderGeometry(upperPlateR, upperPlateR, upperPlateH, 36)
  );
  const upperPlate = new THREE.Mesh(upperPlateGeo, mats.woodPrimary);
  upperPlate.position.set(0, padH + upperPlateH * 0.5, 0);
  upperPlate.castShadow = true;
  upperPlate.receiveShadow = true;
  upperPlate.userData = { moduleId: mod.id, dragRole: 'module' };
  drivenSpindleGroup.add(upperPlate);

  const topBossGeo = track(new THREE.CylinderGeometry(0.115, 0.135, 0.055, 18));
  const topBoss = new THREE.Mesh(topBossGeo, mats.metalHardware);
  topBoss.position.set(0, spec.drivenDiscThickness + 0.026, 0);
  topBoss.castShadow = true;
  drivenSpindleGroup.add(topBoss);

  const topStudGeo = track(new THREE.CylinderGeometry(0.024, 0.024, 0.02, 12));
  const topStudOrbit = upperPlateR * 0.68;
  for (let i = 0; i < 4; i++) {
    const a = (i * Math.PI) / 2;
    const stud = new THREE.Mesh(topStudGeo, i === 0 ? mats.paintedAccent : mats.metalHardware);
    stud.position.set(
      topStudOrbit * Math.cos(a),
      spec.drivenDiscThickness + 0.008,
      topStudOrbit * Math.sin(a)
    );
    drivenSpindleGroup.add(stud);
  }

  const spindleStartY = spec.drivenDiscThickness * 0.5;
  const spindleLen = Math.max(0.5, blueprint.stageDeckY - (drivenY + spindleStartY) + 0.08);
  const spindleGeo = track(new THREE.CylinderGeometry(0.052, 0.052, spindleLen, 14));
  const spindleMesh = new THREE.Mesh(spindleGeo, mats.steelShaft);
  spindleMesh.position.set(0, spindleStartY + spindleLen * 0.5, 0);
  spindleMesh.castShadow = true;
  drivenSpindleGroup.add(spindleMesh);

  // 3. Rear-Wall Support Bracket & Brass Radial Pillow Bearing
  const bracketY = drivenY + 0.34;
  const bracketLen = blueprint.boxDepth * 0.5 - 0.18;
  const bracketGeo = track(new THREE.BoxGeometry(0.15, 0.11, bracketLen));
  const bracketMesh = new THREE.Mesh(bracketGeo, mats.woodDarkTrim);
  bracketMesh.position.set(spec.drivenX, bracketY, -bracketLen * 0.5);
  bracketMesh.castShadow = true;
  rootGroup.add(bracketMesh);

  const pillowBearingGeo = track(new THREE.CylinderGeometry(0.11, 0.11, 0.13, 16));
  const pillowBearing = new THREE.Mesh(pillowBearingGeo, mats.metalHardware);
  pillowBearing.position.set(spec.drivenX, bracketY, 0);
  pillowBearing.castShadow = true;
  rootGroup.add(pillowBearing);

  // 4. Stage Deck Thrust Collar
  const collarGroup = new THREE.Group();
  collarGroup.position.set(spec.drivenX, blueprint.stageDeckY, 0);
  const collarGeo = track(new THREE.CylinderGeometry(0.17, 0.19, 0.26, 18));
  const collarMesh = new THREE.Mesh(collarGeo, mats.metalHardware);
  collarMesh.position.set(0, -0.07, 0);
  collarGroup.add(collarMesh);
  rootGroup.add(collarGroup);

  // 5. Topper Sculpture mounted on Stage Deck (defaults to Directional Arrow)
  const topperRig = buildTopperGroup(mod.topper, mod.topperScale, mats, track, 'rotary');
  topperRig.group.position.set(spec.drivenX, blueprint.stageDeckY, 0);
  topperRig.group.traverse((obj) => {
    if ((obj as THREE.Mesh).isMesh) {
      obj.userData = { moduleId: mod.id, dragRole: 'module' };
    }
  });
  rootGroup.add(topperRig.group);

  const hitGeo = track(new THREE.BoxGeometry(1.0, blueprint.stageDeckY + 0.9, 1.4));
  const hitMat = new THREE.MeshBasicMaterial({ visible: false });
  const hitMesh = new THREE.Mesh(hitGeo, hitMat);
  hitMesh.position.set(0, (blueprint.stageDeckY + 0.9) * 0.5, 0);
  hitMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  rootGroup.add(hitMesh);

  return {
    id: mod.id,
    type: mod.type,
    rootGroup,
    hitMeshes: [hitMesh, frictionPad, upperPlate],
    pivotHitMeshes: [],
    camOffsetHitMeshes: [rimBand, coreMesh, driverHub, shiftRingPos, shiftRingNeg],
    highlightRing,
    update: (shaftAngleRad, bp, explodeAmount) => {
      const sol = solveSemiCircularCam(shaftAngleRad, mod);
      const drivenAngle = resolveIntermittentDrivenAngle
        ? resolveIntermittentDrivenAngle(shaftAngleRad, mod)
        : sol.drivenAngleRad;
      driverPivot.rotation.x = -sol.driverAngleRad;
      drivenSpindleGroup.rotation.y = -drivenAngle;

      collarGroup.position.y = bp.stageDeckY + explodeAmount * 1.15;
      topperRig.group.position.y = bp.stageDeckY + explodeAmount * 1.15;

      const drive01 = (Math.sin(drivenAngle) + 1) * 0.5;
      topperRig.update(drive01, drivenAngle, 0);
    },
    setSelected: (selected) => {
      highlightRing.visible = selected;
      const ringScale = selected ? 1.18 : 1.0;
      shiftRingPos.scale.setScalar(ringScale);
      shiftRingNeg.scale.setScalar(ringScale);
    },
    dispose: () => {
      hitMat.dispose();
      for (const g of disposables) g.dispose();
    },
  };
}
