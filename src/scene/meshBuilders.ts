import * as THREE from 'three';
import {
  BOATMAN_KINEMATICS,
  CAM_BEVEL_RADIAL,
  computeFourBarPushrodConfig,
  degToRad,
  sampleCamContour,
  sampleHockeyCamProfile,
  sampleRatchetContour,
  solveBellCrank,
  solveBevelGearGeometry,
  solveCircularCam,
  solveCircularCamGeometry,
  solveDualEccentric,
  solveDualEccentricGeometry,
  solveFourBarGearSpec,
  solveFourBarPushrod,
  solvePinLanternGeometry,
  solveRatchet,
  solveRatchetGeometry,
  solveScotchYoke,
  solveSemiCircularCam,
  solveSemiCircularCamGeometry,
  solveDigitalNumbers,
  sampleDigitalCamContour,
  evaluateWritingTrajectory,
  sampleWritingCamContours,
  solveWritingCam,
  solveTopperBellowsLinkage,
  solveTopperBoatmanLinkage,
  solveTopperCarpenterLinkage,
  solveTopperClappingLinkage,
  solveTopperDonkeyMill,
  solveTopperDoubleDoorsLinkage,
  solveTopperDuckEggLinkage,
  solveTopperHockeyLinkage,
  solveTopperLoomLinkage,
  solveTopperRattleDrum,
  solveTopperSawyersLinkage,
  solveTopperSeagullLinkage,
  solveTopperWoodpeckerLinkage,
  BELL_CRANK_KINEMATICS,
  BELLOWS_KINEMATICS,
  CARPENTER_KINEMATICS,
  CLAPPING_KINEMATICS,
  DIGITAL_NUMBERS_KINEMATICS,
  DONKEY_MILL_KINEMATICS,
  DOUBLE_DOORS_KINEMATICS,
  DUCK_EGG_KINEMATICS,
  HOCKEY_KINEMATICS,
  LOOM_KINEMATICS,
  RATTLE_DRUM_KINEMATICS,
  SAWYERS_KINEMATICS,
  WRITING_CAM_KINEMATICS,
} from '../engine/kinematics';
import type {
  AutomataBlueprint,
  AutomataModule,
  BellCrankModule,
  BevelGearModule,
  CamFollowerModule,
  CircularCamModule,
  DigitalNumbersModule,
  DualEccentricModule,
  FourBarLinkageModule,
  FourBarSolution,
  PinLanternModule,
  RatchetModule,
  ScotchYokeModule,
  SemiCircularCamModule,
  TopperType,
  WritingCamModule,
} from '../engine/types';
import type { AutomataMaterialSet } from './materials';

export type TopperDriveMode = 'linear' | 'rotary';

export interface TopperRig {
  group: THREE.Group;
  update: (
    drive01: number,
    continuousAngleRad: number,
    pushrodTipAboveDeckWorld: number,
    axialLiftWorld?: number,
    horizontalSlideWorld?: number,
    rotaryDynamics?: { omegaNorm: number; alphaNorm: number }
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
 * Helper to create a monolithic 90-degree L-shaped Bell-Crank elbow lever in XY:
 * pivot corner at (0, 0), input arm along +X (length inLen), output arm along +Y (length outLen).
 */
function createBellCrankLGeometry(
  inLen: number,
  outLen: number,
  width: number,
  thickness: number
): THREE.ExtrudeGeometry {
  const r = width * 0.5;
  const shape = new THREE.Shape();
  shape.absarc(0, 0, r, Math.PI, Math.PI * 1.5, false);
  shape.lineTo(inLen, -r);
  shape.absarc(inLen, 0, r, -Math.PI * 0.5, Math.PI * 0.5, false);
  shape.lineTo(r + 0.02, r);
  shape.quadraticCurveTo(r, r, r, r + 0.02);
  shape.lineTo(r, outLen);
  shape.absarc(0, outLen, r, 0, Math.PI, false);
  shape.lineTo(-r, 0);
  shape.closePath();

  const geo = new THREE.ExtrudeGeometry(shape, {
    depth: thickness,
    bevelEnabled: true,
    bevelThickness: 0.008,
    bevelSize: 0.008,
    bevelSegments: 2,
    curveSegments: 14,
  });
  geo.translate(0, 0, -thickness * 0.5);
  return geo;
}

/**
 * Builds an extruded 2D spur gear rotated onto the YZ plane (tooth 0 along +Z at phaseOffsetRad = 0).
 */
function createSpurGearGeometry(
  pitchRadius: number,
  teethCount: number,
  thickness: number,
  boreRadius: number,
  phaseOffsetRad = 0
): THREE.ExtrudeGeometry {
  const mod = (2 * pitchRadius) / teethCount;
  const outerR = pitchRadius + 0.68 * mod;
  const rootR = Math.max(boreRadius + 0.04, pitchRadius - 0.85 * mod);
  const toothPitch = (Math.PI * 2) / teethCount;

  const shape = new THREE.Shape();
  for (let i = 0; i < teethCount; i++) {
    const c = i * toothPitch + phaseOffsetRad;
    const aRoot0 = c - toothPitch * 0.28;
    const aTip0 = c - toothPitch * 0.14;
    const aTip1 = c + toothPitch * 0.14;
    const aRoot1 = c + toothPitch * 0.28;
    const aNext = (i + 1) * toothPitch + phaseOffsetRad - toothPitch * 0.28;

    const p0x = Math.cos(aRoot0) * rootR;
    const p0y = Math.sin(aRoot0) * rootR;
    if (i === 0) shape.moveTo(p0x, p0y);
    else shape.lineTo(p0x, p0y);

    shape.lineTo(Math.cos(aTip0) * outerR, Math.sin(aTip0) * outerR);
    shape.lineTo(Math.cos(aTip1) * outerR, Math.sin(aTip1) * outerR);
    shape.lineTo(Math.cos(aRoot1) * rootR, Math.sin(aRoot1) * rootR);
    shape.lineTo(Math.cos(aNext) * rootR, Math.sin(aNext) * rootR);
  }
  shape.closePath();

  const hole = new THREE.Path();
  hole.absarc(0, 0, boreRadius, 0, Math.PI * 2, true);
  shape.holes.push(hole);

  const geo = new THREE.ExtrudeGeometry(shape, {
    depth: thickness,
    bevelEnabled: true,
    bevelThickness: 0.008,
    bevelSize: 0.008,
    bevelSegments: 1,
    curveSegments: 12,
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
      update: (drive, _ang, tipWorldY, _lift, horizontalSlideWorld) => {
        if (horizontalSlideWorld !== undefined) {
          const slideLocal = horizontalSlideWorld / Math.max(0.5, scale);
          capGroup.position.set(0, 0.14, slideLocal);
        } else {
          const pinY = tipWorldY > 0.01 ? tipWorldY / scale : 0.14 + drive * 0.45;
          capGroup.position.set(0, pinY, 0);
        }
      },
    };
  }

  if (type === 'sawyers') {
    const K = SAWYERS_KINEMATICS;

    // 1. Deck longitudinal brass shuttle slot & twin timber sawhorses holding a felled log
    const trackGeo = track(new THREE.BoxGeometry(0.11, 0.022, 1.18));
    const trackPlate = new THREE.Mesh(trackGeo, mats.metalHardware);
    trackPlate.position.set(0, 0.014, 0);
    trackPlate.receiveShadow = true;
    group.add(trackPlate);

    const trackSlotGeo = track(new THREE.BoxGeometry(0.036, 0.026, 1.04));
    const trackSlot = new THREE.Mesh(trackSlotGeo, mats.woodDarkTrim);
    trackSlot.position.set(0, 0.016, 0);
    group.add(trackSlot);

    // Timber sawhorses placed cleanly outside X = 0 so shuttle posts slide freely between them
    const legGeo = track(new THREE.CylinderGeometry(0.018, 0.022, 0.31, 8));
    const crossBeamGeo = track(new THREE.BoxGeometry(0.045, 0.038, 0.34));
    for (const sx of [-0.21, 0.21]) {
      const beam = new THREE.Mesh(crossBeamGeo, mats.woodDarkTrim);
      beam.position.set(sx, 0.21, 0);
      beam.castShadow = true;
      group.add(beam);

      for (const sz of [-0.13, 0.13]) {
        const leg = new THREE.Mesh(legGeo, mats.woodDarkTrim);
        leg.position.set(sx * 1.08, 0.12, sz);
        leg.rotation.x = sz > 0 ? -0.32 : 0.32;
        leg.castShadow = true;
        group.add(leg);
      }
    }

    // Two-piece X-aligned timber log with real central kerf cut at x in [-0.02, +0.02]
    const logHalfGeo = track(new THREE.CylinderGeometry(0.128, 0.128, 0.275, 18));
    logHalfGeo.rotateZ(Math.PI / 2);
    const logRingGeo = track(new THREE.CylinderGeometry(0.11, 0.11, 0.28, 16));
    logRingGeo.rotateZ(Math.PI / 2);
    for (const side of [-1, 1] as const) {
      const halfLog = new THREE.Mesh(logHalfGeo, mats.woodDarkTrim);
      halfLog.position.set(side * 0.16, 0.34, 0);
      halfLog.castShadow = true;
      halfLog.receiveShadow = true;
      group.add(halfLog);

      const heartwood = new THREE.Mesh(logRingGeo, mats.woodPrimary);
      heartwood.position.set(side * 0.16, 0.34, 0);
      group.add(heartwood);
    }
    // Lower uncut log core below saw kerf
    const logCoreGeo = track(new THREE.CylinderGeometry(0.068, 0.068, 0.07, 14));
    logCoreGeo.rotateZ(Math.PI / 2);
    const logCore = new THREE.Mesh(logCoreGeo, mats.woodPrimary);
    logCore.position.set(0, 0.295, 0);
    group.add(logCore);

    // 2. Horizontally Translating Crosscut Saw + Under-Log Brass Shuttle Bar
    const sawGroup = new THREE.Group();
    group.add(sawGroup);

    const shuttleBarGeo = track(new THREE.BoxGeometry(0.055, 0.032, K.handleOffset * 2 + 0.08));
    const shuttleBar = new THREE.Mesh(shuttleBarGeo, mats.metalHardware);
    shuttleBar.position.set(0, 0.048, 0);
    shuttleBar.castShadow = true;
    sawGroup.add(shuttleBar);

    const postGeo = track(new THREE.CylinderGeometry(0.017, 0.02, 0.56, 10));
    const handleBarGeo = track(new THREE.CylinderGeometry(0.022, 0.022, 0.16, 12));
    handleBarGeo.rotateZ(Math.PI / 2);
    for (const sgn of [-1, 1] as const) {
      const post = new THREE.Mesh(postGeo, mats.metalHardware);
      post.position.set(0, 0.32, sgn * K.handleOffset);
      post.castShadow = true;
      sawGroup.add(post);

      const handle = new THREE.Mesh(handleBarGeo, mats.woodDarkTrim);
      handle.position.set(0, K.handleY, sgn * K.handleOffset);
      handle.castShadow = true;
      sawGroup.add(handle);
    }

    const bladeGeo = track(new THREE.BoxGeometry(0.014, 0.095, K.handleOffset * 2 - 0.02));
    const blade = new THREE.Mesh(bladeGeo, mats.steelShaft);
    blade.position.set(0, 0.44, 0);
    blade.castShadow = true;
    sawGroup.add(blade);

    const spineGeo = track(new THREE.BoxGeometry(0.022, 0.022, K.handleOffset * 2 + 0.02));
    const spine = new THREE.Mesh(spineGeo, mats.metalHardware);
    spine.position.set(0, 0.495, 0);
    sawGroup.add(spine);

    const toothGeo = track(new THREE.ConeGeometry(0.014, 0.032, 3));
    toothGeo.rotateX(Math.PI);
    for (let i = -9; i <= 9; i++) {
      const tooth = new THREE.Mesh(toothGeo, mats.steelShaft);
      tooth.position.set(0, 0.382, i * 0.048);
      sawGroup.add(tooth);
    }

    // 3. Build Articulated Twin Lumberjacks (Rear at -hipZ facing +Z, Front at +hipZ facing -Z)
    interface SawyerRig {
      torsoPivot: THREE.Group;
      upperArms: [THREE.Mesh, THREE.Mesh];
      forearms: [THREE.Mesh, THREE.Mesh];
      elbows: [THREE.Mesh, THREE.Mesh];
      hands: [THREE.Mesh, THREE.Mesh];
    }

    const buildSawyerFigure = (hipZ: number, facingSign: 1 | -1, shirtMat: THREE.Material): SawyerRig => {
      const plinthGeo = track(new THREE.BoxGeometry(0.24, 0.032, 0.2));
      const plinth = new THREE.Mesh(plinthGeo, mats.woodDarkTrim);
      plinth.position.set(0, 0.016, hipZ);
      plinth.castShadow = true;
      group.add(plinth);

      const legGeo2 = track(new THREE.CylinderGeometry(0.028, 0.032, K.hipY - 0.02, 10));
      const bootGeo = track(new THREE.BoxGeometry(0.06, 0.045, 0.095));
      for (const sx of [-0.056, 0.056]) {
        const leg = new THREE.Mesh(legGeo2, mats.woodSecondary);
        leg.position.set(sx, K.hipY * 0.5 + 0.01, hipZ);
        leg.castShadow = true;
        group.add(leg);

        const boot = new THREE.Mesh(bootGeo, mats.woodDarkTrim);
        boot.position.set(sx, 0.045, hipZ + facingSign * 0.018);
        boot.castShadow = true;
        group.add(boot);
      }

      const hipAxleGeo = track(new THREE.CylinderGeometry(0.024, 0.024, 0.18, 10));
      hipAxleGeo.rotateZ(Math.PI / 2);
      const hipAxle = new THREE.Mesh(hipAxleGeo, mats.metalHardware);
      hipAxle.position.set(0, K.hipY, hipZ);
      group.add(hipAxle);

      const torsoPivot = new THREE.Group();
      torsoPivot.position.set(0, K.hipY, hipZ);
      group.add(torsoPivot);

      const torsoGeo = track(new THREE.BoxGeometry(0.155, K.torsoLen + 0.03, 0.11));
      const torso = new THREE.Mesh(torsoGeo, shirtMat);
      torso.position.set(0, K.torsoLen * 0.5 + 0.01, 0);
      torso.castShadow = true;
      torsoPivot.add(torso);

      const collarGeo = track(new THREE.CylinderGeometry(0.032, 0.036, 0.05, 10));
      const collar = new THREE.Mesh(collarGeo, mats.woodPrimary);
      collar.position.set(0, K.torsoLen + 0.04, 0);
      torsoPivot.add(collar);

      const headGeo = track(new THREE.SphereGeometry(0.068, 14, 12));
      const head = new THREE.Mesh(headGeo, mats.woodPrimary);
      head.position.set(0, K.torsoLen + 0.115, 0);
      head.castShadow = true;
      torsoPivot.add(head);

      const capGeo = track(new THREE.CylinderGeometry(0.066, 0.072, 0.04, 14));
      const cap = new THREE.Mesh(capGeo, mats.woodDarkTrim);
      cap.position.set(0, K.torsoLen + 0.165, 0);
      cap.castShadow = true;
      torsoPivot.add(cap);

      const brimGeo = track(new THREE.BoxGeometry(0.12, 0.015, 0.075));
      const brim = new THREE.Mesh(brimGeo, mats.woodDarkTrim);
      brim.position.set(0, K.torsoLen + 0.148, facingSign * 0.052);
      torsoPivot.add(brim);

      const shoulderAxleGeo = track(new THREE.CylinderGeometry(0.022, 0.022, 0.19, 10));
      shoulderAxleGeo.rotateZ(Math.PI / 2);
      const shoulderAxle = new THREE.Mesh(shoulderAxleGeo, mats.metalHardware);
      shoulderAxle.position.set(0, K.torsoLen, 0);
      torsoPivot.add(shoulderAxle);

      const upperArmGeo = track(new THREE.CylinderGeometry(0.027, 0.024, 1.0, 10));
      const forearmGeo = track(new THREE.CylinderGeometry(0.023, 0.021, 1.0, 10));
      const elbowGeo = track(new THREE.SphereGeometry(0.029, 10, 8));
      const handGeo = track(new THREE.SphereGeometry(0.033, 10, 8));

      const upperArms: [THREE.Mesh, THREE.Mesh] = [
        new THREE.Mesh(upperArmGeo, shirtMat),
        new THREE.Mesh(upperArmGeo, shirtMat),
      ];
      const forearms: [THREE.Mesh, THREE.Mesh] = [
        new THREE.Mesh(forearmGeo, mats.woodPrimary),
        new THREE.Mesh(forearmGeo, mats.woodPrimary),
      ];
      const elbows: [THREE.Mesh, THREE.Mesh] = [
        new THREE.Mesh(elbowGeo, mats.metalHardware),
        new THREE.Mesh(elbowGeo, mats.metalHardware),
      ];
      const hands: [THREE.Mesh, THREE.Mesh] = [
        new THREE.Mesh(handGeo, mats.woodPrimary),
        new THREE.Mesh(handGeo, mats.woodPrimary),
      ];

      for (let i = 0; i < 2; i++) {
        upperArms[i].castShadow = true;
        forearms[i].castShadow = true;
        group.add(upperArms[i], forearms[i], elbows[i], hands[i]);
      }

      return { torsoPivot, upperArms, forearms, elbows, hands };
    };

    const rearSawyer = buildSawyerFigure(-K.hipZ, 1, mats.paintedAccent);
    const frontSawyer = buildSawyerFigure(K.hipZ, -1, mats.paintedSecondary);

    const updateSawyerBranch = (
      rig: SawyerRig,
      branch: ReturnType<typeof solveTopperSawyersLinkage>['rear']
    ) => {
      rig.torsoPivot.rotation.x = branch.torsoLeanRad;
      const sideOffsets = [-0.078, 0.078] as const;
      const gripOffsets = [-0.055, 0.055] as const;
      for (let i = 0; i < 2; i++) {
        const sh = new THREE.Vector3(sideOffsets[i], branch.shoulderY, branch.shoulderZ);
        const el = new THREE.Vector3(sideOffsets[i] * 0.92, branch.elbowY, branch.elbowZ);
        const ha = new THREE.Vector3(gripOffsets[i], branch.handleY, branch.handleZ);

        rig.upperArms[i].scale.set(1, sh.distanceTo(el), 1);
        alignBarBetweenPoints(rig.upperArms[i], sh, el, 'y');

        rig.forearms[i].scale.set(1, el.distanceTo(ha), 1);
        alignBarBetweenPoints(rig.forearms[i], el, ha, 'y');

        rig.elbows[i].position.copy(el);
        rig.hands[i].position.copy(ha);
      }
    };

    return {
      group,
      update: (drive, continuousAngleRad, _tipWorldY, _lift, horizontalSlideWorld) => {
        const rawSlider =
          horizontalSlideWorld !== undefined
            ? horizontalSlideWorld / Math.max(0.5, scale)
            : driveMode === 'rotary'
              ? 0.24 * Math.cos(continuousAngleRad)
              : (drive - 0.5) * 0.5;
        const sol = solveTopperSawyersLinkage(rawSlider);
        sawGroup.position.z = sol.sliderZ;
        updateSawyerBranch(rearSawyer, sol.rear);
        updateSawyerBranch(frontSawyer, sol.front);
      },
    };
  }

  if (type === 'carpenter') {
    const K = CARPENTER_KINEMATICS;

    // 1. Deck shuttle track + slotted carpenter workbench
    const trackGeo = track(new THREE.BoxGeometry(0.11, 0.022, 1.22));
    const trackPlate = new THREE.Mesh(trackGeo, mats.metalHardware);
    trackPlate.position.set(0, 0.014, 0.02);
    trackPlate.receiveShadow = true;
    group.add(trackPlate);

    const trackSlotGeo = track(new THREE.BoxGeometry(0.036, 0.026, 1.08));
    const trackSlot = new THREE.Mesh(trackSlotGeo, mats.woodDarkTrim);
    trackSlot.position.set(0, 0.016, 0.02);
    group.add(trackSlot);

    // Workbench outer legs (x = +-0.155, completely clear of x = 0 drive tang)
    const benchLegGeo = track(new THREE.BoxGeometry(0.044, 0.31, 0.044));
    for (const bx of [-0.155, 0.155]) {
      for (const bz of [-0.32, 0.56]) {
        const leg = new THREE.Mesh(benchLegGeo, mats.woodDarkTrim);
        leg.position.set(bx, 0.165, bz);
        leg.castShadow = true;
        group.add(leg);
      }
    }

    // Slotted twin benchtop rails & timber plank workpiece (open center slot at x in [-0.018, +0.018])
    const benchRailGeo = track(new THREE.BoxGeometry(0.155, 0.045, 0.96));
    const plankHalfGeo = track(new THREE.BoxGeometry(0.095, 0.045, 0.88));
    for (const side of [-1, 1] as const) {
      const benchRail = new THREE.Mesh(benchRailGeo, mats.woodPrimary);
      benchRail.position.set(side * 0.105, 0.33, 0.12);
      benchRail.castShadow = true;
      benchRail.receiveShadow = true;
      group.add(benchRail);

      const plankHalf = new THREE.Mesh(plankHalfGeo, mats.woodSecondary);
      plankHalf.position.set(side * 0.072, 0.375, 0.12);
      plankHalf.castShadow = true;
      group.add(plankHalf);
    }

    // Front brass bench dog stop
    const stopGeo = track(new THREE.BoxGeometry(0.24, 0.055, 0.036));
    const benchStop = new THREE.Mesh(stopGeo, mats.metalHardware);
    benchStop.position.set(0, 0.38, 0.58);
    benchStop.castShadow = true;
    group.add(benchStop);

    // 2. Reciprocating Wooden Hand Plane + Curled Shaving
    const planeGroup = new THREE.Group();
    group.add(planeGroup);

    const shuttleBlockGeo = track(new THREE.BoxGeometry(0.052, 0.032, 0.2));
    const shuttleBlock = new THREE.Mesh(shuttleBlockGeo, mats.metalHardware);
    shuttleBlock.position.set(0, 0.048, 0);
    planeGroup.add(shuttleBlock);

    const driveTangGeo = track(new THREE.BoxGeometry(0.022, 0.37, 0.048));
    const driveTang = new THREE.Mesh(driveTangGeo, mats.metalHardware);
    driveTang.position.set(0, 0.23, 0);
    driveTang.castShadow = true;
    planeGroup.add(driveTang);

    const planeBodyGeo = track(new THREE.BoxGeometry(0.11, 0.068, 0.26));
    const planeBody = new THREE.Mesh(planeBodyGeo, mats.woodDarkTrim);
    planeBody.position.set(0, 0.432, -0.01);
    planeBody.castShadow = true;
    planeGroup.add(planeBody);

    const ironGeo = track(new THREE.BoxGeometry(0.068, 0.11, 0.016));
    const planeIron = new THREE.Mesh(ironGeo, mats.steelShaft);
    planeIron.position.set(0, 0.48, 0.01);
    planeIron.rotation.x = -0.65;
    planeIron.castShadow = true;
    planeGroup.add(planeIron);

    const knobGeo = track(new THREE.SphereGeometry(0.034, 12, 10));
    const frontKnob = new THREE.Mesh(knobGeo, mats.metalHardware);
    frontKnob.position.set(0, 0.495, 0.085);
    planeGroup.add(frontKnob);

    const toteStemGeo = track(new THREE.CylinderGeometry(0.018, 0.022, 0.08, 10));
    const toteStem = new THREE.Mesh(toteStemGeo, mats.woodDarkTrim);
    toteStem.position.set(0, 0.475, K.handleOffsetZ);
    planeGroup.add(toteStem);

    const toteBarGeo = track(new THREE.CylinderGeometry(0.02, 0.02, 0.15, 10));
    toteBarGeo.rotateZ(Math.PI / 2);
    const toteBar = new THREE.Mesh(toteBarGeo, mats.woodDarkTrim);
    toteBar.position.set(0, K.handleY, K.handleOffsetZ);
    planeGroup.add(toteBar);

    const shavingPivot = new THREE.Group();
    shavingPivot.position.set(0, 0.468, 0.015);
    planeGroup.add(shavingPivot);

    const curlArcGeo = track(new THREE.TorusGeometry(0.034, 0.014, 8, 14, Math.PI * 1.35));
    curlArcGeo.rotateY(Math.PI / 2);
    const curlArc = new THREE.Mesh(curlArcGeo, mats.woodPrimary);
    curlArc.position.set(0, 0.035, 0.015);
    curlArc.castShadow = true;
    shavingPivot.add(curlArc);

    // 3. Articulated Woodworker Figure (hipZ = -0.56 facing +Z)
    const plinthGeo = track(new THREE.BoxGeometry(0.24, 0.032, 0.2));
    const plinth = new THREE.Mesh(plinthGeo, mats.woodDarkTrim);
    plinth.position.set(0, 0.016, K.hipZ);
    plinth.castShadow = true;
    group.add(plinth);

    const legGeo2 = track(new THREE.CylinderGeometry(0.03, 0.034, K.hipY - 0.02, 10));
    const bootGeo = track(new THREE.BoxGeometry(0.062, 0.045, 0.095));
    for (const sx of [-0.056, 0.056]) {
      const leg = new THREE.Mesh(legGeo2, mats.woodDarkTrim);
      leg.position.set(sx, K.hipY * 0.5 + 0.01, K.hipZ);
      leg.castShadow = true;
      group.add(leg);

      const boot = new THREE.Mesh(bootGeo, mats.woodDarkTrim);
      boot.position.set(sx, 0.045, K.hipZ + 0.018);
      group.add(boot);
    }

    const torsoPivot = new THREE.Group();
    torsoPivot.position.set(0, K.hipY, K.hipZ);
    group.add(torsoPivot);

    const shirtGeo = track(new THREE.BoxGeometry(0.16, K.torsoLen + 0.03, 0.11));
    const shirt = new THREE.Mesh(shirtGeo, mats.paintedIvory);
    shirt.position.set(0, K.torsoLen * 0.5 + 0.01, 0);
    shirt.castShadow = true;
    torsoPivot.add(shirt);

    const apronGeo = track(new THREE.BoxGeometry(0.145, K.torsoLen * 0.88, 0.025));
    const apron = new THREE.Mesh(apronGeo, mats.paintedSecondary);
    apron.position.set(0, K.torsoLen * 0.44, 0.056);
    torsoPivot.add(apron);

    const headGeo = track(new THREE.SphereGeometry(0.068, 14, 12));
    const head = new THREE.Mesh(headGeo, mats.woodPrimary);
    head.position.set(0, K.torsoLen + 0.115, 0);
    head.castShadow = true;
    torsoPivot.add(head);

    const capGeo = track(new THREE.CylinderGeometry(0.068, 0.074, 0.04, 14));
    const cap = new THREE.Mesh(capGeo, mats.paintedAccent);
    cap.position.set(0, K.torsoLen + 0.165, 0);
    torsoPivot.add(cap);

    const brimGeo = track(new THREE.BoxGeometry(0.12, 0.015, 0.075));
    const brim = new THREE.Mesh(brimGeo, mats.paintedAccent);
    brim.position.set(0, K.torsoLen + 0.148, 0.052);
    torsoPivot.add(brim);

    const upperArmGeo = track(new THREE.CylinderGeometry(0.026, 0.023, 1.0, 10));
    const forearmGeo = track(new THREE.CylinderGeometry(0.022, 0.02, 1.0, 10));
    const elbowGeo = track(new THREE.SphereGeometry(0.028, 10, 8));
    const handGeo = track(new THREE.SphereGeometry(0.032, 10, 8));

    const upperArms: [THREE.Mesh, THREE.Mesh] = [
      new THREE.Mesh(upperArmGeo, mats.paintedIvory),
      new THREE.Mesh(upperArmGeo, mats.paintedIvory),
    ];
    const forearms: [THREE.Mesh, THREE.Mesh] = [
      new THREE.Mesh(forearmGeo, mats.woodPrimary),
      new THREE.Mesh(forearmGeo, mats.woodPrimary),
    ];
    const elbows: [THREE.Mesh, THREE.Mesh] = [
      new THREE.Mesh(elbowGeo, mats.metalHardware),
      new THREE.Mesh(elbowGeo, mats.metalHardware),
    ];
    const hands: [THREE.Mesh, THREE.Mesh] = [
      new THREE.Mesh(handGeo, mats.woodPrimary),
      new THREE.Mesh(handGeo, mats.woodPrimary),
    ];
    for (let i = 0; i < 2; i++) {
      upperArms[i].castShadow = true;
      forearms[i].castShadow = true;
      group.add(upperArms[i], forearms[i], elbows[i], hands[i]);
    }

    return {
      group,
      update: (drive, continuousAngleRad, _tipWorldY, _lift, horizontalSlideWorld) => {
        const rawSlider =
          horizontalSlideWorld !== undefined
            ? horizontalSlideWorld / Math.max(0.5, scale)
            : driveMode === 'rotary'
              ? 0.24 * Math.cos(continuousAngleRad)
              : (drive - 0.5) * 0.5;
        const sol = solveTopperCarpenterLinkage(rawSlider, continuousAngleRad);
        planeGroup.position.z = sol.planeZ;
        shavingPivot.rotation.x = -sol.shavingCurlAngle;
        shavingPivot.position.y = 0.468 + sol.shavingLift;

        torsoPivot.rotation.x = sol.arm.torsoLeanRad;
        const sideOffsets = [-0.08, 0.08] as const;
        const gripOffsets = [-0.052, 0.052] as const;
        for (let i = 0; i < 2; i++) {
          const sh = new THREE.Vector3(sideOffsets[i], sol.arm.shoulderY, sol.arm.shoulderZ);
          const el = new THREE.Vector3(sideOffsets[i] * 0.92, sol.arm.elbowY, sol.arm.elbowZ);
          const ha = new THREE.Vector3(gripOffsets[i], sol.arm.handleY, sol.arm.handleZ);
          upperArms[i].scale.set(1, sh.distanceTo(el), 1);
          alignBarBetweenPoints(upperArms[i], sh, el, 'y');
          forearms[i].scale.set(1, el.distanceTo(ha), 1);
          alignBarBetweenPoints(forearms[i], el, ha, 'y');
          elbows[i].position.copy(el);
          hands[i].position.copy(ha);
        }
      },
    };
  }

  if (type === 'loom') {
    const K = LOOM_KINEMATICS;

    // 1. Deck track & open timber loom frame
    const trackGeo = track(new THREE.BoxGeometry(0.11, 0.022, 1.18));
    const trackPlate = new THREE.Mesh(trackGeo, mats.metalHardware);
    trackPlate.position.set(0, 0.014, 0);
    group.add(trackPlate);

    const postGeo = track(new THREE.BoxGeometry(0.052, 0.78, 0.052));
    for (const px of [-0.23, 0.23]) {
      for (const pz of [-K.beamSpanHalfZ, K.beamSpanHalfZ]) {
        const post = new THREE.Mesh(postGeo, mats.woodDarkTrim);
        post.position.set(px, 0.41, pz);
        post.castShadow = true;
        group.add(post);
      }
    }

    const sideBeamGeo = track(new THREE.BoxGeometry(0.048, 0.052, K.beamSpanHalfZ * 2 + 0.1));
    for (const px of [-0.23, 0.23]) {
      const topSide = new THREE.Mesh(sideBeamGeo, mats.woodDarkTrim);
      topSide.position.set(px, 0.78, 0);
      topSide.castShadow = true;
      group.add(topSide);

      const lowSide = new THREE.Mesh(sideBeamGeo, mats.woodPrimary);
      lowSide.position.set(px, 0.16, 0);
      group.add(lowSide);
    }

    const crossCastleGeo = track(new THREE.BoxGeometry(0.51, 0.048, 0.052));
    for (const pz of [-0.14, 0.14]) {
      const castle = new THREE.Mesh(crossCastleGeo, mats.woodDarkTrim);
      castle.position.set(0, 0.78, pz);
      castle.castShadow = true;
      group.add(castle);
    }

    // Rear warp roller & front cloth beam
    const rollerGeo = track(new THREE.CylinderGeometry(0.048, 0.048, 0.42, 14));
    rollerGeo.rotateZ(Math.PI / 2);
    const warpBeam = new THREE.Mesh(rollerGeo, mats.woodPrimary);
    warpBeam.position.set(0, K.shuttleY, -K.beamSpanHalfZ);
    group.add(warpBeam);

    const clothRollGeo = track(new THREE.CylinderGeometry(0.062, 0.062, 0.36, 16));
    clothRollGeo.rotateZ(Math.PI / 2);
    const clothBeam = new THREE.Mesh(clothRollGeo, mats.paintedAccent);
    clothBeam.position.set(0, K.shuttleY, K.beamSpanHalfZ);
    clothBeam.castShadow = true;
    group.add(clothBeam);

    // 2. Two Open-Bottom Alternating Heddle Frames (x = +-0.18 stiles leave center x=0 completely clear!)
    const buildHeddleFrame = (frameZ: number, barMat: THREE.Material): THREE.Group => {
      const f = new THREE.Group();
      f.position.set(0, K.shuttleY, frameZ);
      const topBarGeo = track(new THREE.BoxGeometry(0.4, 0.028, 0.026));
      const topBar = new THREE.Mesh(topBarGeo, barMat);
      topBar.position.set(0, 0.18, 0);
      f.add(topBar);

      const stileGeo = track(new THREE.BoxGeometry(0.024, 0.24, 0.024));
      for (const sx of [-0.185, 0.185]) {
        const stile = new THREE.Mesh(stileGeo, barMat);
        stile.position.set(sx, 0.07, 0);
        f.add(stile);
      }

      const wireGeo = track(new THREE.CylinderGeometry(0.0045, 0.0045, 0.2, 6));
      for (const wx of [-0.14, -0.095, -0.055, 0.055, 0.095, 0.14]) {
        const wire = new THREE.Mesh(wireGeo, mats.steelShaft);
        wire.position.set(wx, 0.07, 0);
        f.add(wire);
      }
      group.add(f);
      return f;
    };

    const heddleA = buildHeddleFrame(-0.14, mats.woodPrimary);
    const heddleB = buildHeddleFrame(0.14, mats.paintedSecondary);

    // Dynamic warp threads split symmetrically on either side of central shuttle carrier slot
    const warpOffsetsX = [-0.14, -0.095, -0.055, 0.055, 0.095, 0.14] as const;
    const threadGeo = track(new THREE.CylinderGeometry(0.005, 0.005, 1.0, 6));
    const warpSegs: Array<{
      x: number;
      heddle: 'A' | 'B';
      rearSeg: THREE.Mesh;
      midSeg: THREE.Mesh;
      frontSeg: THREE.Mesh;
    }> = [];
    for (let i = 0; i < warpOffsetsX.length; i++) {
      const heddle: 'A' | 'B' = i % 2 === 0 ? 'A' : 'B';
      const mat = heddle === 'A' ? mats.paintedIvory : mats.paintedAccent;
      const rearSeg = new THREE.Mesh(threadGeo, mat);
      const midSeg = new THREE.Mesh(threadGeo, mat);
      const frontSeg = new THREE.Mesh(threadGeo, mat);
      group.add(rearSeg, midSeg, frontSeg);
      warpSegs.push({ x: warpOffsetsX[i], heddle, rearSeg, midSeg, frontSeg });
    }

    // 3. Flying Boat-Shaped Wooden Shuttle driven directly along Z by under-shed brass blade
    const shuttleGroup = new THREE.Group();
    group.add(shuttleGroup);

    const shuttleBarGeo = track(new THREE.BoxGeometry(0.044, 0.028, 0.16));
    const shuttleBar = new THREE.Mesh(shuttleBarGeo, mats.metalHardware);
    shuttleBar.position.set(0, 0.048, 0);
    shuttleGroup.add(shuttleBar);

    const carrierPostGeo = track(new THREE.BoxGeometry(0.018, 0.31, 0.042));
    const carrierPost = new THREE.Mesh(carrierPostGeo, mats.metalHardware);
    carrierPost.position.set(0, 0.21, 0);
    carrierPost.castShadow = true;
    shuttleGroup.add(carrierPost);

    const shuttleBodyGeo = track(new THREE.BoxGeometry(0.072, 0.044, 0.18));
    const shuttleBody = new THREE.Mesh(shuttleBodyGeo, mats.woodDarkTrim);
    shuttleBody.position.set(0, K.shuttleY, 0);
    shuttleBody.castShadow = true;
    shuttleGroup.add(shuttleBody);

    const tipGeo = track(new THREE.ConeGeometry(0.028, 0.065, 10));
    const tipFront = new THREE.Mesh(tipGeo, mats.metalHardware);
    tipFront.rotation.x = Math.PI / 2;
    tipFront.position.set(0, K.shuttleY, 0.122);
    shuttleGroup.add(tipFront);

    const tipRear = new THREE.Mesh(tipGeo, mats.metalHardware);
    tipRear.rotation.x = -Math.PI / 2;
    tipRear.position.set(0, K.shuttleY, -0.122);
    shuttleGroup.add(tipRear);

    const bobbinGeo = track(new THREE.CylinderGeometry(0.016, 0.016, 0.09, 10));
    bobbinGeo.rotateX(Math.PI / 2);
    const bobbin = new THREE.Mesh(bobbinGeo, mats.paintedAccent);
    bobbin.position.set(0, K.shuttleY + 0.015, 0);
    shuttleGroup.add(bobbin);

    // 4. Twin Pivoted Picker Levers at z = +-0.52
    const buildPicker = (pz: number): THREE.Group => {
      const piv = new THREE.Group();
      piv.position.set(0, 0.14, pz);
      const stickGeo = track(new THREE.BoxGeometry(0.036, 0.29, 0.03));
      const stick = new THREE.Mesh(stickGeo, mats.woodSecondary);
      stick.position.set(0, 0.145, 0);
      stick.castShadow = true;
      piv.add(stick);

      const padGeo = track(new THREE.BoxGeometry(0.056, 0.056, 0.034));
      const pad = new THREE.Mesh(padGeo, mats.metalHardware);
      pad.position.set(0, 0.23, 0);
      piv.add(pad);

      group.add(piv);
      return piv;
    };

    const rearPicker = buildPicker(-K.pickerPivotZ);
    const frontPicker = buildPicker(K.pickerPivotZ);

    return {
      group,
      update: (drive, continuousAngleRad, _tipWorldY, _lift, horizontalSlideWorld) => {
        const rawSlider =
          horizontalSlideWorld !== undefined
            ? horizontalSlideWorld / Math.max(0.5, scale)
            : driveMode === 'rotary'
              ? 0.24 * Math.cos(continuousAngleRad)
              : (drive - 0.5) * 0.5;
        const sol = solveTopperLoomLinkage(rawSlider, continuousAngleRad);
        shuttleGroup.position.z = sol.shuttleZ;
        heddleA.position.y = K.shuttleY + sol.heddleAOffsetY;
        heddleB.position.y = K.shuttleY + sol.heddleBOffsetY;
        rearPicker.rotation.x = sol.rearPickerAngleRad;
        frontPicker.rotation.x = sol.frontPickerAngleRad;

        for (const seg of warpSegs) {
          const yA = K.shuttleY + (seg.heddle === 'A' ? sol.heddleAOffsetY : sol.heddleBOffsetY);
          const yB = K.shuttleY + (seg.heddle === 'A' ? sol.heddleAOffsetY : sol.heddleBOffsetY);
          const p0 = new THREE.Vector3(seg.x, K.shuttleY, -K.beamSpanHalfZ);
          const p1 = new THREE.Vector3(seg.x, yA, -0.14);
          const p2 = new THREE.Vector3(seg.x, yB, 0.14);
          const p3 = new THREE.Vector3(seg.x, K.shuttleY, K.beamSpanHalfZ);

          seg.rearSeg.scale.set(1, p0.distanceTo(p1), 1);
          alignBarBetweenPoints(seg.rearSeg, p0, p1, 'y');
          seg.midSeg.scale.set(1, p1.distanceTo(p2), 1);
          alignBarBetweenPoints(seg.midSeg, p1, p2, 'y');
          seg.frontSeg.scale.set(1, p2.distanceTo(p3), 1);
          alignBarBetweenPoints(seg.frontSeg, p2, p3, 'y');
        }
      },
    };
  }

  if (type === 'bellows') {
    const K = BELLOWS_KINEMATICS;

    // 1. Deck shuttle track
    const trackGeo = track(new THREE.BoxGeometry(0.11, 0.022, 1.18));
    const trackPlate = new THREE.Mesh(trackGeo, mats.metalHardware);
    trackPlate.position.set(0, 0.014, 0);
    group.add(trackPlate);

    // 2. Blacksmith Forge Hearth + Glowing Coal Firepot + Mini Anvil (z = +0.54)
    const hearthBaseGeo = track(new THREE.BoxGeometry(0.42, 0.28, 0.42));
    const hearthBase = new THREE.Mesh(hearthBaseGeo, mats.woodDarkTrim);
    hearthBase.position.set(0, 0.15, 0.54);
    hearthBase.castShadow = true;
    hearthBase.receiveShadow = true;
    group.add(hearthBase);

    const hearthRimGeo = track(new THREE.CylinderGeometry(0.16, 0.18, 0.07, 14));
    const hearthRim = new THREE.Mesh(hearthRimGeo, mats.steelShaft);
    hearthRim.position.set(-0.04, 0.325, 0.5);
    hearthRim.castShadow = true;
    group.add(hearthRim);

    // Mini forged steel anvil on forge deck beside firepot
    const anvilBaseGeo = track(new THREE.BoxGeometry(0.08, 0.055, 0.12));
    const anvilBase = new THREE.Mesh(anvilBaseGeo, mats.steelShaft);
    anvilBase.position.set(0.14, 0.32, 0.62);
    group.add(anvilBase);
    const anvilHornGeo = track(new THREE.ConeGeometry(0.032, 0.09, 10));
    anvilHornGeo.rotateX(Math.PI / 2);
    const anvilHorn = new THREE.Mesh(anvilHornGeo, mats.steelShaft);
    anvilHorn.position.set(0.14, 0.355, 0.69);
    group.add(anvilHorn);

    // Glowing coal bed & dynamic flame cluster
    const coalGeo = track(new THREE.DodecahedronGeometry(0.055, 0));
    const coalPositions: Array<[number, number, number]> = [
      [-0.04, 0.355, 0.5],
      [-0.09, 0.345, 0.48],
      [0.01, 0.348, 0.52],
      [-0.05, 0.348, 0.55],
    ];
    for (const [cx, cy, cz] of coalPositions) {
      const coal = new THREE.Mesh(coalGeo, mats.paintedAccent);
      coal.position.set(cx, cy, cz);
      group.add(coal);
    }

    const flameGroup = new THREE.Group();
    flameGroup.position.set(-0.04, 0.37, 0.5);
    group.add(flameGroup);
    const mainFlameGeo = track(new THREE.ConeGeometry(0.068, 0.18, 10));
    const sideFlameGeo = track(new THREE.ConeGeometry(0.042, 0.12, 8));
    const f0 = new THREE.Mesh(mainFlameGeo, mats.paintedAccent);
    f0.position.set(0, 0.09, 0);
    const f1 = new THREE.Mesh(sideFlameGeo, mats.metalHardware);
    f1.position.set(-0.04, 0.06, 0.02);
    const f2 = new THREE.Mesh(sideFlameGeo, mats.metalHardware);
    f2.position.set(0.04, 0.06, -0.02);
    flameGroup.add(f0, f1, f2);

    // Tuyere blast nozzle connecting bellows header (z = +0.14) to firepot (z = +0.34)
    const nozzleGeo = track(new THREE.CylinderGeometry(0.038, 0.052, 0.24, 12));
    nozzleGeo.rotateX(Math.PI / 2);
    const nozzle = new THREE.Mesh(nozzleGeo, mats.metalHardware);
    nozzle.position.set(-0.02, K.bellowsCenterY, 0.24);
    nozzle.castShadow = true;
    group.add(nozzle);

    // 3. Fixed Front Bellows Header & Side Support Stanchions (z = +0.14)
    const postGeo = track(new THREE.BoxGeometry(0.042, 0.34, 0.042));
    for (const sx of [-0.15, 0.15]) {
      const post = new THREE.Mesh(postGeo, mats.woodDarkTrim);
      post.position.set(sx, 0.18, K.frontHeaderZ);
      post.castShadow = true;
      group.add(post);
    }
    const headerGeo = track(new THREE.BoxGeometry(0.32, 0.26, 0.036));
    const frontHeader = new THREE.Mesh(headerGeo, mats.woodDarkTrim);
    frontHeader.position.set(0, K.bellowsCenterY, K.frontHeaderZ);
    frontHeader.castShadow = true;
    group.add(frontHeader);

    // 4. Accordion Pleat Rings & Moving Rear Plunger Board + Horizontal Piston Linkage
    const pleatFrameGeo = track(new THREE.BoxGeometry(0.28, 0.22, 0.032));
    const pleatRibGeo = track(new THREE.BoxGeometry(0.3, 0.24, 0.016));
    const pleatMeshes: THREE.Mesh[] = [];
    for (let i = 0; i < K.pleatCount; i++) {
      const pMesh = new THREE.Mesh(
        i % 2 === 0 ? pleatFrameGeo : pleatRibGeo,
        i % 2 === 0 ? mats.woodSecondary : mats.woodPrimary
      );
      pMesh.position.set(0, K.bellowsCenterY, 0);
      pMesh.castShadow = true;
      group.add(pMesh);
      pleatMeshes.push(pMesh);
    }

    const plungerGroup = new THREE.Group();
    group.add(plungerGroup);

    const rearBoardGeo = track(new THREE.BoxGeometry(0.32, 0.26, 0.04));
    const rearBoard = new THREE.Mesh(rearBoardGeo, mats.woodDarkTrim);
    rearBoard.position.set(0, K.bellowsCenterY, K.rearRestZ);
    rearBoard.castShadow = true;
    plungerGroup.add(rearBoard);

    // Rigid brass tie-arm connecting deck shuttle (0, 0.05, sliderZ) to rear plunger board (0, 0.31, rearRestZ + sliderZ)
    const shuttleShoeGeo = track(new THREE.BoxGeometry(0.055, 0.032, 0.16));
    const shuttleShoe = new THREE.Mesh(shuttleShoeGeo, mats.metalHardware);
    shuttleShoe.position.set(0, 0.048, 0);
    plungerGroup.add(shuttleShoe);

    const tieRodGeo = track(new THREE.BoxGeometry(0.03, 0.03, -K.rearRestZ + 0.06));
    const tieRod = new THREE.Mesh(tieRodGeo, mats.metalHardware);
    tieRod.position.set(0, 0.08, K.rearRestZ * 0.5);
    tieRod.castShadow = true;
    plungerGroup.add(tieRod);

    const clevisGeo = track(new THREE.BoxGeometry(0.038, 0.24, 0.04));
    const clevisPost = new THREE.Mesh(clevisGeo, mats.metalHardware);
    clevisPost.position.set(0, 0.19, K.rearRestZ);
    plungerGroup.add(clevisPost);

    // Rear Blacksmith Rocker Handle pivoted at (-0.82, 0.20)
    const rockerPivot = new THREE.Group();
    rockerPivot.position.set(0, K.rockerPivotY, K.rockerPivotZ);
    group.add(rockerPivot);

    const rockerPlinthGeo = track(new THREE.BoxGeometry(0.14, 0.2, 0.12));
    const rockerPlinth = new THREE.Mesh(rockerPlinthGeo, mats.woodDarkTrim);
    rockerPlinth.position.set(0, 0.1, K.rockerPivotZ);
    group.add(rockerPlinth);

    const rockerArmGeo = track(new THREE.BoxGeometry(0.042, 0.44, 0.045));
    const rockerArm = new THREE.Mesh(rockerArmGeo, mats.woodSecondary);
    rockerArm.position.set(0, 0.21, 0);
    rockerArm.castShadow = true;
    rockerPivot.add(rockerArm);

    const rockerLinkGeo = track(new THREE.CylinderGeometry(0.016, 0.016, 1.0, 8));
    const rockerLink = new THREE.Mesh(rockerLinkGeo, mats.steelShaft);
    group.add(rockerLink);

    return {
      group,
      update: (drive, continuousAngleRad, _tipWorldY, _lift, horizontalSlideWorld) => {
        const rawSlider =
          horizontalSlideWorld !== undefined
            ? horizontalSlideWorld / Math.max(0.5, scale)
            : driveMode === 'rotary'
              ? 0.24 * Math.cos(continuousAngleRad)
              : (drive - 0.5) * 0.5;
        const sol = solveTopperBellowsLinkage(rawSlider);
        plungerGroup.position.z = rawSlider;
        for (let i = 0; i < K.pleatCount; i++) {
          pleatMeshes[i].position.z = sol.pleatZs[i];
        }
        const flameScale = 0.55 + sol.emberGlow01 * 0.95;
        flameGroup.scale.set(0.85 + sol.emberGlow01 * 0.3, flameScale, 0.85 + sol.emberGlow01 * 0.3);

        rockerPivot.rotation.x = sol.rockerAngleRad;
        const rockerPinY = K.rockerPivotY + Math.cos(sol.rockerAngleRad) * 0.14;
        const rockerPinZ = K.rockerPivotZ + Math.sin(sol.rockerAngleRad) * 0.14;
        const pA = new THREE.Vector3(0, rockerPinY, rockerPinZ);
        const pB = new THREE.Vector3(0, K.bellowsCenterY, sol.rearBoardZ - 0.02);
        rockerLink.scale.set(1, pA.distanceTo(pB), 1);
        alignBarBetweenPoints(rockerLink, pA, pB, 'y');
      },
    };
  }

  if (type === 'hockey') {
    const K = HOCKEY_KINEMATICS;

    // 1. Deck shuttle slot & raised slotted ivory hockey rink table
    const trackGeo = track(new THREE.BoxGeometry(0.11, 0.022, 1.2));
    const trackPlate = new THREE.Mesh(trackGeo, mats.metalHardware);
    trackPlate.position.set(0, 0.014, 0);
    group.add(trackPlate);

    const legGeo = track(new THREE.BoxGeometry(0.044, 0.24, 0.044));
    for (const rx of [-0.21, 0.21]) {
      for (const rz of [-0.46, 0.46]) {
        const leg = new THREE.Mesh(legGeo, mats.woodDarkTrim);
        leg.position.set(rx, 0.13, rz);
        leg.castShadow = true;
        group.add(leg);
      }
    }

    // Two ivory ice halves leaving a clean central brass slot at x in [-0.016, +0.016]
    const iceHalfGeo = track(new THREE.BoxGeometry(0.225, 0.032, 1.04));
    const boardSideGeo = track(new THREE.BoxGeometry(0.026, 0.085, 1.06));
    const endWingBoardGeo = track(new THREE.BoxGeometry(0.125, 0.07, 0.026));
    const zoneStripeGeo = track(new THREE.BoxGeometry(0.21, 0.005, 0.022));
    for (const side of [-1, 1] as const) {
      const iceHalf = new THREE.Mesh(iceHalfGeo, mats.paintedIvory);
      iceHalf.position.set(side * 0.13, 0.245, 0);
      iceHalf.castShadow = true;
      iceHalf.receiveShadow = true;
      group.add(iceHalf);

      const sideBoard = new THREE.Mesh(boardSideGeo, mats.woodDarkTrim);
      sideBoard.position.set(side * 0.252, 0.285, 0);
      sideBoard.castShadow = true;
      group.add(sideBoard);

      const centerRed = new THREE.Mesh(zoneStripeGeo, mats.paintedAccent);
      centerRed.position.set(side * 0.13, 0.262, 0);
      group.add(centerRed);

      for (const bz of [-0.2, 0.2]) {
        const blueLine = new THREE.Mesh(zoneStripeGeo, mats.paintedSecondary);
        blueLine.position.set(side * 0.13, 0.262, bz);
        group.add(blueLine);
      }

      for (const ez of [-0.515, 0.515]) {
        const wing = new THREE.Mesh(endWingBoardGeo, mats.woodDarkTrim);
        wing.position.set(side * 0.19, 0.278, ez);
        group.add(wing);
      }
    }

    // Open-top red goal crease posts at z = +-0.515 (stick shafts swing cleanly above open center!)
    const goalPostGeo = track(new THREE.CylinderGeometry(0.013, 0.013, 0.08, 10));
    for (const ez of [-0.515, 0.515]) {
      for (const gx of [-0.125, 0.125]) {
        const post = new THREE.Mesh(goalPostGeo, mats.paintedAccent);
        post.position.set(gx, 0.285, ez);
        group.add(post);
      }
    }

    // 2. Horizontally Reciprocating Crimson Hockey Puck + Twin Shuttle-Mounted Cam Lobes
    const puckGroup = new THREE.Group();
    group.add(puckGroup);

    const shuttleShoeGeo = track(new THREE.BoxGeometry(0.048, 0.028, 0.16));
    const shuttleShoe = new THREE.Mesh(shuttleShoeGeo, mats.metalHardware);
    shuttleShoe.position.set(0, 0.048, 0);
    puckGroup.add(shuttleShoe);

    // Transverse brass crossbar tying central puck tang to left/right cam wedges (clears outer dwell pads)
    const camCrossbarGeo = track(new THREE.BoxGeometry(0.232, 0.022, 0.068));
    const camCrossbar = new THREE.Mesh(camCrossbarGeo, mats.metalHardware);
    camCrossbar.position.set(0, 0.035, 0);
    camCrossbar.castShadow = true;
    puckGroup.add(camCrossbar);

    // Profiled cam wedge matching evaluateHockeyCamLift(u) cleanly on inner cam lanes (|x| = 0.104)
    const camProfilePts = sampleHockeyCamProfile();
    const camShape = new THREE.Shape();
    camProfilePts.forEach((pt, idx) => {
      if (idx === 0) camShape.moveTo(pt.z, pt.y);
      else camShape.lineTo(pt.z, pt.y);
    });
    camShape.closePath();

    const camThickness = 0.02;
    const camGeo = track(
      new THREE.ExtrudeGeometry(camShape, {
        depth: camThickness,
        bevelEnabled: false,
        curveSegments: 12,
      })
    );
    camGeo.translate(0, 0, -camThickness * 0.5);

    const rearCamMesh = new THREE.Mesh(camGeo, mats.woodSecondary);
    rearCamMesh.position.set(-K.sideX + 0.011, 0, 0);
    rearCamMesh.rotation.y = -Math.PI / 2;
    rearCamMesh.castShadow = true;
    puckGroup.add(rearCamMesh);

    const frontCamMesh = new THREE.Mesh(camGeo, mats.woodSecondary);
    frontCamMesh.position.set(K.sideX - 0.011, 0, 0);
    frontCamMesh.rotation.y = Math.PI / 2;
    frontCamMesh.castShadow = true;
    puckGroup.add(frontCamMesh);

    // Stationary outer brass dwell pads on the deck supporting outer roller rim when cam retreats
    const dwellPadGeo = track(new THREE.BoxGeometry(0.016, 0.04, 0.07));
    for (const facingSign of [1, -1] as const) {
      const sideX = -facingSign * K.sideX;
      const dwellPad = new THREE.Mesh(dwellPadGeo, mats.metalHardware);
      dwellPad.position.set(sideX - facingSign * 0.015, 0.043, -facingSign * 0.245);
      dwellPad.castShadow = true;
      group.add(dwellPad);
    }

    const puckStemGeo = track(new THREE.CylinderGeometry(0.014, 0.014, 0.22, 10));
    const puckStem = new THREE.Mesh(puckStemGeo, mats.metalHardware);
    puckStem.position.set(0, 0.16, 0);
    puckGroup.add(puckStem);

    const puckGeo = track(new THREE.CylinderGeometry(K.puckRadius, K.puckRadius, 0.028, 20));
    const puckMesh = new THREE.Mesh(puckGeo, mats.paintedAccent);
    puckMesh.position.set(0, 0.276, 0);
    puckMesh.castShadow = true;
    puckGroup.add(puckMesh);

    const puckDotGeo = track(new THREE.CylinderGeometry(0.014, 0.014, 0.03, 10));
    const puckDot = new THREE.Mesh(puckDotGeo, mats.metalHardware);
    puckDot.position.set(0.024, 0.002, 0);
    puckMesh.add(puckDot);

    // 3. Opposing Articulated Hockey Players, Seesaw Rockers, Return Springs & Drive Rods
    interface HockeyPlayerRig {
      facingSign: 1 | -1;
      inwardArm: THREE.Mesh;
      outerArm: THREE.Mesh;
      rollerWheel: THREE.Mesh;
      springGuide: THREE.Mesh;
      springCoils: THREE.Mesh[];
      torsoRod: THREE.Mesh;
      waistPin: THREE.Mesh;
      waistHorn: THREE.Mesh;
      vertRod: THREE.Mesh;
      hornPin: THREE.Mesh;
      stickHorn: THREE.Mesh;
      wristCollar: THREE.Mesh;
      torsoPivot: THREE.Group;
      upperArms: [THREE.Mesh, THREE.Mesh];
      forearms: [THREE.Mesh, THREE.Mesh];
      elbows: [THREE.Mesh, THREE.Mesh];
      hands: [THREE.Mesh, THREE.Mesh];
      stickShaft: THREE.Mesh;
      stickBlade: THREE.Mesh;
    }

    const buildHockeyPlayer = (facingSign: 1 | -1, jerseyMat: THREE.Material): HockeyPlayerRig => {
      const hipZ = -facingSign * K.hipZ;
      const sideX = -facingSign * K.sideX;
      const pivZ = -facingSign * K.rockerPivotZ;
      const outerX = sideX - facingSign * 0.014;
      const innerX = sideX + facingSign * 0.014;

      const plinthGeo = track(new THREE.BoxGeometry(0.29, 0.032, 0.24));
      const plinth = new THREE.Mesh(plinthGeo, mats.woodDarkTrim);
      plinth.position.set(0, 0.016, hipZ + facingSign * 0.02);
      plinth.castShadow = true;
      group.add(plinth);

      // Cantilevered outer rocker stanchion + horizontal brass pivot axle clearing the cam lane completely
      const stanchionX = -facingSign * 0.172;
      const stanchionGeo = track(new THREE.BoxGeometry(0.032, 0.165, 0.056));
      const stanchion = new THREE.Mesh(stanchionGeo, mats.woodDarkTrim);
      stanchion.position.set(stanchionX, 0.0825, pivZ);
      stanchion.castShadow = true;
      group.add(stanchion);

      const axleSpan = Math.abs(stanchionX - sideX) + 0.028;
      const pivAxleGeo = track(new THREE.CylinderGeometry(0.01, 0.01, axleSpan, 10));
      pivAxleGeo.rotateZ(Math.PI / 2);
      const pivAxle = new THREE.Mesh(pivAxleGeo, mats.metalHardware);
      pivAxle.position.set((stanchionX + sideX) * 0.5, K.rockerPivotY, pivZ);
      group.add(pivAxle);

      // Seesaw rocker arms & wide brass cam roller follower riding cam + dwell pad
      const armBarGeo = track(new THREE.BoxGeometry(0.016, 0.024, 1.0));
      const inwardArm = new THREE.Mesh(armBarGeo, mats.woodSecondary);
      inwardArm.castShadow = true;
      const outerArm = new THREE.Mesh(armBarGeo, mats.woodSecondary);
      outerArm.castShadow = true;
      group.add(inwardArm, outerArm);

      const rollerGeo = track(
        new THREE.CylinderGeometry(K.rollerRadius, K.rollerRadius, 0.048, 16)
      );
      rollerGeo.rotateZ(Math.PI / 2);
      const rollerWheel = new THREE.Mesh(rollerGeo, mats.metalHardware);
      rollerWheel.castShadow = true;
      group.add(rollerWheel);

      // Helical compression return spring under outer rocker arm pressing roller firmly onto cam
      const springSeatGeo = track(new THREE.CylinderGeometry(0.022, 0.026, 0.014, 12));
      const springSeat = new THREE.Mesh(springSeatGeo, mats.metalHardware);
      springSeat.position.set(sideX, 0.036, -facingSign * 0.795);
      group.add(springSeat);

      const springGuideGeo = track(new THREE.CylinderGeometry(0.005, 0.005, 1.0, 8));
      const springGuide = new THREE.Mesh(springGuideGeo, mats.steelShaft);
      group.add(springGuide);

      const springCoilGeo = track(new THREE.TorusGeometry(0.017, 0.0038, 6, 14));
      springCoilGeo.rotateX(Math.PI / 2);
      const springCoils: THREE.Mesh[] = [];
      for (let c = 0; c < 6; c++) {
        const coil = new THREE.Mesh(springCoilGeo, mats.metalHardware);
        group.add(coil);
        springCoils.push(coil);
      }

      // Rigid drive rods, horizontal cross-axles & rear lever horns
      const rodGeo = track(new THREE.CylinderGeometry(0.0085, 0.0085, 1.0, 8));
      const torsoRod = new THREE.Mesh(rodGeo, mats.steelShaft);
      torsoRod.castShadow = true;
      const vertRod = new THREE.Mesh(rodGeo, mats.steelShaft);
      vertRod.castShadow = true;
      group.add(torsoRod, vertRod);

      const waistPinGeo = track(
        new THREE.CylinderGeometry(0.009, 0.009, Math.abs(outerX) + 0.022, 10)
      );
      waistPinGeo.rotateZ(Math.PI / 2);
      const waistPin = new THREE.Mesh(waistPinGeo, mats.metalHardware);
      group.add(waistPin);

      const hornPinGeo = track(
        new THREE.CylinderGeometry(0.009, 0.009, Math.abs(innerX) + 0.022, 10)
      );
      hornPinGeo.rotateZ(Math.PI / 2);
      const hornPin = new THREE.Mesh(hornPinGeo, mats.metalHardware);
      group.add(hornPin);

      const leverBarGeo = track(new THREE.BoxGeometry(0.018, 0.02, 1.0));
      const waistHorn = new THREE.Mesh(leverBarGeo, mats.metalHardware);
      waistHorn.castShadow = true;
      const stickHorn = new THREE.Mesh(leverBarGeo, mats.metalHardware);
      stickHorn.castShadow = true;
      group.add(waistHorn, stickHorn);

      const wristCollarGeo = track(new THREE.CylinderGeometry(0.019, 0.019, 0.088, 12));
      wristCollarGeo.rotateZ(Math.PI / 2);
      const wristCollar = new THREE.Mesh(wristCollarGeo, mats.metalHardware);
      group.add(wristCollar);

      const hipAxleGeo = track(new THREE.CylinderGeometry(0.011, 0.011, 0.19, 10));
      hipAxleGeo.rotateZ(Math.PI / 2);
      const hipAxle = new THREE.Mesh(hipAxleGeo, mats.metalHardware);
      hipAxle.position.set(0, K.hipY, hipZ);
      group.add(hipAxle);

      const legGeo2 = track(new THREE.CylinderGeometry(0.028, 0.032, K.hipY - 0.02, 10));
      const skateGeo = track(new THREE.BoxGeometry(0.056, 0.042, 0.095));
      for (const sx of [-0.056, 0.056]) {
        const leg = new THREE.Mesh(legGeo2, mats.woodDarkTrim);
        leg.position.set(sx, K.hipY * 0.5 + 0.01, hipZ);
        leg.castShadow = true;
        group.add(leg);

        const skate = new THREE.Mesh(skateGeo, mats.steelShaft);
        skate.position.set(sx, 0.042, hipZ + facingSign * 0.016);
        group.add(skate);
      }

      const torsoPivot = new THREE.Group();
      torsoPivot.position.set(0, K.hipY, hipZ);
      group.add(torsoPivot);

      const jerseyGeo = track(new THREE.BoxGeometry(0.16, K.torsoLen + 0.03, 0.11));
      const jersey = new THREE.Mesh(jerseyGeo, jerseyMat);
      jersey.position.set(0, K.torsoLen * 0.5 + 0.01, 0);
      jersey.castShadow = true;
      torsoPivot.add(jersey);

      const stripeGeo = track(new THREE.BoxGeometry(0.164, 0.048, 0.114));
      const stripe = new THREE.Mesh(stripeGeo, mats.paintedIvory);
      stripe.position.set(0, K.torsoLen * 0.35, 0);
      torsoPivot.add(stripe);

      const headGeo = track(new THREE.SphereGeometry(0.066, 14, 12));
      const head = new THREE.Mesh(headGeo, mats.woodPrimary);
      head.position.set(0, K.torsoLen + 0.115, 0);
      head.castShadow = true;
      torsoPivot.add(head);

      const helmetGeo = track(
        new THREE.SphereGeometry(0.072, 14, 10, 0, Math.PI * 2, 0, Math.PI * 0.56)
      );
      const helmet = new THREE.Mesh(helmetGeo, jerseyMat);
      helmet.position.set(0, K.torsoLen + 0.125, 0);
      torsoPivot.add(helmet);

      const upperArmGeo = track(new THREE.CylinderGeometry(0.025, 0.022, 1.0, 10));
      const forearmGeo = track(new THREE.CylinderGeometry(0.021, 0.019, 1.0, 10));
      const elbowGeo = track(new THREE.SphereGeometry(0.026, 10, 8));
      const handGeo = track(new THREE.SphereGeometry(0.03, 10, 8));

      const upperArms: [THREE.Mesh, THREE.Mesh] = [
        new THREE.Mesh(upperArmGeo, jerseyMat),
        new THREE.Mesh(upperArmGeo, jerseyMat),
      ];
      const forearms: [THREE.Mesh, THREE.Mesh] = [
        new THREE.Mesh(forearmGeo, mats.paintedIvory),
        new THREE.Mesh(forearmGeo, mats.paintedIvory),
      ];
      const elbows: [THREE.Mesh, THREE.Mesh] = [
        new THREE.Mesh(elbowGeo, mats.metalHardware),
        new THREE.Mesh(elbowGeo, mats.metalHardware),
      ];
      const hands: [THREE.Mesh, THREE.Mesh] = [
        new THREE.Mesh(handGeo, mats.woodDarkTrim),
        new THREE.Mesh(handGeo, mats.woodDarkTrim),
      ];
      for (let i = 0; i < 2; i++) {
        upperArms[i].castShadow = true;
        forearms[i].castShadow = true;
        group.add(upperArms[i], forearms[i], elbows[i], hands[i]);
      }

      const shaftGeo = track(new THREE.CylinderGeometry(0.014, 0.014, 1.0, 8));
      const stickShaft = new THREE.Mesh(shaftGeo, mats.woodPrimary);
      stickShaft.castShadow = true;
      group.add(stickShaft);

      const bladeGeo = track(new THREE.BoxGeometry(0.11, 0.036, K.bladeHalfThick * 2));
      const stickBlade = new THREE.Mesh(bladeGeo, mats.woodDarkTrim);
      stickBlade.castShadow = true;
      group.add(stickBlade);

      return {
        facingSign,
        inwardArm,
        outerArm,
        rollerWheel,
        springGuide,
        springCoils,
        torsoRod,
        waistPin,
        waistHorn,
        vertRod,
        hornPin,
        stickHorn,
        wristCollar,
        torsoPivot,
        upperArms,
        forearms,
        elbows,
        hands,
        stickShaft,
        stickBlade,
      };
    };

    const rearPlayer = buildHockeyPlayer(1, mats.paintedAccent);
    const frontPlayer = buildHockeyPlayer(-1, mats.paintedSecondary);

    const updateHockeyPlayer = (
      rig: HockeyPlayerRig,
      branch: ReturnType<typeof solveTopperHockeyLinkage>['rear']
    ) => {
      const f = rig.facingSign;
      const sideX = branch.sideX;
      const outerX = sideX - f * 0.014;
      const innerX = sideX + f * 0.014;

      // Rocker seesaw arms & roller follower
      const pPiv = new THREE.Vector3(sideX, branch.rockerPivotY, branch.rockerPivotZ);
      const pRoll = new THREE.Vector3(sideX, branch.rollerY, branch.rollerZ);
      const pOut = new THREE.Vector3(sideX, branch.outerPinY, branch.outerPinZ);
      rig.inwardArm.scale.set(1, 1, pPiv.distanceTo(pRoll));
      alignBarBetweenPoints(rig.inwardArm, pPiv, pRoll, 'z');
      rig.outerArm.scale.set(1, 1, pPiv.distanceTo(pOut));
      alignBarBetweenPoints(rig.outerArm, pPiv, pOut, 'z');
      rig.rollerWheel.position.copy(pRoll);

      // Helical compression spring from plinth seat to outer rocker arm
      const pSpringBase = new THREE.Vector3(sideX, 0.036, -f * 0.795);
      rig.springGuide.scale.set(1, pSpringBase.distanceTo(pOut), 1);
      alignBarBetweenPoints(rig.springGuide, pSpringBase, pOut, 'y');
      for (let c = 0; c < rig.springCoils.length; c++) {
        const t = (c + 1) / (rig.springCoils.length + 1);
        rig.springCoils[c].position.lerpVectors(pSpringBase, pOut, t);
      }

      // Torso drive rod & waist horn
      const pMid = new THREE.Vector3(outerX, branch.midPinY, branch.midPinZ);
      const pWaistSide = new THREE.Vector3(outerX, branch.waistPinY, branch.waistPinZ);
      const pWaistCenter = new THREE.Vector3(0, branch.waistPinY, branch.waistPinZ);
      const pHipCenter = new THREE.Vector3(0, branch.arm.hipY, branch.arm.hipZ);
      rig.torsoRod.scale.set(1, pMid.distanceTo(pWaistSide), 1);
      alignBarBetweenPoints(rig.torsoRod, pMid, pWaistSide, 'y');
      rig.waistPin.position.set(outerX * 0.5, branch.waistPinY, branch.waistPinZ);
      rig.waistHorn.scale.set(1, 1, pHipCenter.distanceTo(pWaistCenter));
      alignBarBetweenPoints(rig.waistHorn, pHipCenter, pWaistCenter, 'z');

      // Vertical stick drive rod & rear stick lever horn
      const pOutInner = new THREE.Vector3(innerX, branch.outerPinY, branch.outerPinZ);
      const pHornSide = new THREE.Vector3(innerX, branch.hornPinY, branch.hornPinZ);
      const pHornCenter = new THREE.Vector3(0, branch.hornPinY, branch.hornPinZ);
      const pGripCenter = new THREE.Vector3(0, branch.gripY, branch.gripZ);
      rig.vertRod.scale.set(1, pOutInner.distanceTo(pHornSide), 1);
      alignBarBetweenPoints(rig.vertRod, pOutInner, pHornSide, 'y');
      rig.hornPin.position.set(innerX * 0.5, branch.hornPinY, branch.hornPinZ);
      rig.stickHorn.scale.set(1, 1, pGripCenter.distanceTo(pHornCenter));
      alignBarBetweenPoints(rig.stickHorn, pGripCenter, pHornCenter, 'z');
      rig.wristCollar.position.copy(pGripCenter);

      // Articulated torso + two-link arms + slanted stick aligned cleanly through wrist collar
      rig.torsoPivot.rotation.x = branch.arm.torsoLeanRad;
      const sideOffsets = [-0.076, 0.076] as const;
      const gripOffsets = [-0.036, 0.036] as const;
      for (let i = 0; i < 2; i++) {
        const sh = new THREE.Vector3(sideOffsets[i], branch.arm.shoulderY, branch.arm.shoulderZ);
        const el = new THREE.Vector3(sideOffsets[i] * 0.9, branch.arm.elbowY, branch.arm.elbowZ);
        const ha = new THREE.Vector3(gripOffsets[i], branch.arm.handleY, branch.arm.handleZ);
        rig.upperArms[i].scale.set(1, sh.distanceTo(el), 1);
        alignBarBetweenPoints(rig.upperArms[i], sh, el, 'y');
        rig.forearms[i].scale.set(1, el.distanceTo(ha), 1);
        alignBarBetweenPoints(rig.forearms[i], el, ha, 'y');
        rig.elbows[i].position.copy(el);
        rig.hands[i].position.copy(ha);
      }

      const bladeEnd = new THREE.Vector3(0, branch.bladeY, branch.bladeZ);
      const stickDir = new THREE.Vector3().subVectors(bladeEnd, pGripCenter).normalize();
      const topButt = pGripCenter.clone().addScaledVector(stickDir, -0.055);
      rig.stickShaft.scale.set(1, topButt.distanceTo(bladeEnd), 1);
      alignBarBetweenPoints(rig.stickShaft, topButt, bladeEnd, 'y');
      rig.stickBlade.position.copy(bladeEnd);
    };

    return {
      group,
      update: (drive, continuousAngleRad, _tipWorldY, _lift, horizontalSlideWorld) => {
        const rawSlider =
          horizontalSlideWorld !== undefined
            ? horizontalSlideWorld / Math.max(0.5, scale)
            : driveMode === 'rotary'
              ? 0.24 * Math.cos(continuousAngleRad)
              : (drive - 0.5) * 0.5;
        const sol = solveTopperHockeyLinkage(rawSlider);
        puckGroup.position.z = sol.puckZ;
        puckMesh.rotation.y = sol.puckSpinRad;
        updateHockeyPlayer(rearPlayer, sol.rear);
        updateHockeyPlayer(frontPlayer, sol.front);
      },
    };
  }

  if (type === 'double-doors') {
    const K = DOUBLE_DOORS_KINEMATICS;
    const B = BELL_CRANK_KINEMATICS;

    // 1. Slotted brass deck track plate along X at z = 0
    const trackGeo = track(new THREE.BoxGeometry(1.12, 0.022, 0.12));
    const trackPlate = new THREE.Mesh(trackGeo, mats.metalHardware);
    trackPlate.position.set(0, 0.014, 0);
    trackPlate.receiveShadow = true;
    group.add(trackPlate);

    // Rear foundation piers (z in [-0.63, -0.38]) supporting the raised chalet threshold above under-door horns (y=0.04)
    const pierGeo = track(new THREE.BoxGeometry(0.068, K.sillTopY - 0.025, 0.25));
    for (const sx of [-0.426, 0.426]) {
      const pier = new THREE.Mesh(pierGeo, mats.woodDarkTrim);
      pier.position.set(sx, (K.sillTopY - 0.025) * 0.5, -0.505);
      pier.castShadow = true;
      group.add(pier);
    }
    const rearBeamGeo = track(new THREE.BoxGeometry(0.84, K.sillTopY - 0.025, 0.04));
    const rearBeam = new THREE.Mesh(rearBeamGeo, mats.woodDarkTrim);
    rearBeam.position.set(0, (K.sillTopY - 0.025) * 0.5, -0.61);
    group.add(rearBeam);

    // Split raised wooden threshold floor (y = 0.095..0.125) with central slot (|x| <= 0.045) for bird perch stem
    const sillSlabGeo = track(new THREE.BoxGeometry(0.415, 0.03, 0.44));
    for (const side of [-1, 1] as const) {
      const slab = new THREE.Mesh(sillSlabGeo, mats.woodDarkTrim);
      slab.position.set(side * (0.045 + 0.415 * 0.5), K.sillTopY - 0.015, -0.41);
      slab.castShadow = true;
      slab.receiveShadow = true;
      group.add(slab);
    }

    // 2. Timber Cuckoo Chalet Frame sitting strictly behind z = K.jambFrontZ (-0.245)
    const chaletDepth = 0.385;
    const chaletCenterZ = K.jambFrontZ - chaletDepth * 0.5;
    const jambGeo = track(new THREE.BoxGeometry(0.068, 0.86, chaletDepth));
    for (const sx of [-0.426, 0.426]) {
      const jamb = new THREE.Mesh(jambGeo, mats.woodDarkTrim);
      jamb.position.set(sx, K.sillTopY + 0.43, chaletCenterZ);
      jamb.castShadow = true;
      group.add(jamb);
    }

    const backWallGeo = track(new THREE.BoxGeometry(0.84, 0.86, 0.036));
    const backWall = new THREE.Mesh(backWallGeo, mats.woodSecondary);
    backWall.position.set(0, K.sillTopY + 0.43, K.jambFrontZ - chaletDepth + 0.018);
    backWall.receiveShadow = true;
    group.add(backWall);

    const innerPanelGeo = track(new THREE.BoxGeometry(0.68, 0.72, 0.02));
    const innerPanel = new THREE.Mesh(innerPanelGeo, mats.paintedIvory);
    innerPanel.position.set(0, K.sillTopY + 0.43, K.jambFrontZ - chaletDepth + 0.042);
    group.add(innerPanel);

    const headerGeo = track(new THREE.BoxGeometry(0.96, 0.075, chaletDepth + 0.05));
    const header = new THREE.Mesh(headerGeo, mats.woodDarkTrim);
    header.position.set(0, K.sillTopY + 0.88, chaletCenterZ + 0.01);
    header.castShadow = true;
    group.add(header);

    // Pitched chalet roof eave cap
    const roofWingGeo = track(new THREE.BoxGeometry(0.56, 0.048, chaletDepth + 0.07));
    for (const side of [-1, 1] as const) {
      const eave = new THREE.Mesh(roofWingGeo, mats.paintedAccent);
      eave.position.set(side * 0.23, K.sillTopY + 1.02, chaletCenterZ + 0.01);
      eave.rotation.z = -side * 0.44;
      eave.castShadow = true;
      group.add(eave);
    }

    // 3. Under-Floor Brass Z-Guide Rail, Cam Ramp & V-Toggle Pop-Out Songbird Carriage (y = 0.072)
    const birdRailGeo = track(new THREE.CylinderGeometry(0.014, 0.014, 0.54, 10));
    birdRailGeo.rotateX(Math.PI / 2);
    const birdRail = new THREE.Mesh(birdRailGeo, mats.steelShaft);
    birdRail.position.set(0, K.birdRodY, -0.32);
    group.add(birdRail);

    const railPostGeo = track(new THREE.CylinderGeometry(0.016, 0.02, 0.06, 10));
    for (const rz of [-0.57, -0.06]) {
      const rp = new THREE.Mesh(railPostGeo, mats.metalHardware);
      rp.position.set(0, 0.046, rz);
      group.add(rp);
    }

    // Stationary sloped brass lift wedge under rear tail that pitches bird forward as carriage emerges
    const camRampGeo = track(new THREE.BoxGeometry(0.036, 0.03, 0.24));
    const camRamp = new THREE.Mesh(camRampGeo, mats.metalHardware);
    camRamp.position.set(0, 0.064, -0.34);
    camRamp.rotation.x = -0.18;
    group.add(camRamp);

    const carriageGroup = new THREE.Group();
    group.add(carriageGroup);

    const sledGeo = track(new THREE.BoxGeometry(0.11, 0.032, 0.11));
    const sledMesh = new THREE.Mesh(sledGeo, mats.metalHardware);
    sledMesh.position.set(0, K.birdRodY, 0);
    sledMesh.castShadow = true;
    carriageGroup.add(sledMesh);

    const perchPivot = new THREE.Group();
    perchPivot.position.set(0, K.birdRodY + 0.028, 0);
    carriageGroup.add(perchPivot);

    const perchStemGeo = track(new THREE.CylinderGeometry(0.013, 0.016, K.birdPerchHeight, 10));
    const perchStem = new THREE.Mesh(perchStemGeo, mats.metalHardware);
    perchStem.position.set(0, K.birdPerchHeight * 0.5, 0);
    perchPivot.add(perchStem);

    // Rear tail lever riding the stationary cam wedge
    const perchTailGeo = track(new THREE.BoxGeometry(0.024, 0.018, 0.11));
    const perchTail = new THREE.Mesh(perchTailGeo, mats.metalHardware);
    perchTail.position.set(0, 0.008, -0.055);
    perchPivot.add(perchTail);

    const birdGroup = new THREE.Group();
    birdGroup.position.set(0, K.birdPerchHeight, K.birdNeckOffsetZ);
    perchPivot.add(birdGroup);

    const birdBodyGeo = track(new THREE.SphereGeometry(0.078, 14, 12));
    birdBodyGeo.scale(0.95, 0.92, 1.2);
    const birdBody = new THREE.Mesh(birdBodyGeo, mats.paintedAccent);
    birdBody.castShadow = true;
    birdGroup.add(birdBody);

    const birdHeadGeo = track(new THREE.SphereGeometry(0.05, 12, 10));
    const birdHead = new THREE.Mesh(birdHeadGeo, mats.paintedAccent);
    birdHead.position.set(0, 0.072, 0.064);
    birdGroup.add(birdHead);

    const birdBeakGeo = track(new THREE.ConeGeometry(0.017, 0.058, 8));
    birdBeakGeo.rotateX(Math.PI * 0.5);
    const birdBeak = new THREE.Mesh(birdBeakGeo, mats.metalHardware);
    birdBeak.position.set(0, 0.068, 0.125);
    birdGroup.add(birdBeak);

    const birdWingGeo = track(new THREE.BoxGeometry(0.018, 0.068, 0.115));
    const birdWingL = new THREE.Mesh(birdWingGeo, mats.paintedSecondary);
    birdWingL.position.set(-0.072, 0.015, -0.01);
    const birdWingR = new THREE.Mesh(birdWingGeo, mats.paintedSecondary);
    birdWingR.position.set(0.072, 0.015, -0.01);
    birdGroup.add(birdWingL, birdWingR);

    // 4. Hinged Wooden Double Doors driven by Outboard Offset-Pivot 4-Bar Coupled Swing Arms (Pa, Pb outboard of base side |x|=0.460)
    const doorHeight = 0.836;
    const doorPivotY = K.sillTopY + 0.003;
    const doorGeo = track(new THREE.BoxGeometry(K.doorWidth, doorHeight, K.doorThick));
    const insetGeo = track(new THREE.BoxGeometry(K.doorWidth - 0.09, doorHeight - 0.22, K.doorThick + 0.006));
    const knobGeo = track(new THREE.SphereGeometry(0.022, 10, 8));

    const outBracketGeo = track(new THREE.BoxGeometry(0.145, 0.024, 0.095));
    const primaryShaftGeo = track(new THREE.CylinderGeometry(0.013, 0.013, K.sillTopY + 0.865, 10));
    const linkPinGeo = track(new THREE.CylinderGeometry(0.011, 0.011, 0.056, 10));
    const hingeLugPinGeo = track(new THREE.CylinderGeometry(0.011, 0.011, 0.044, 10));
    const doorLugBarGeo = track(new THREE.BoxGeometry(0.016, 0.01, 1.0));
    const swingArmGeo = track(new THREE.BoxGeometry(0.018, 0.01, 1.0));
    const coupledArmGeo = track(new THREE.BoxGeometry(0.015, 0.01, 1.0));

    const lowerMainY = 0.136;
    const lowerLugLocalY = 0.02;
    const lowerCoupledY = 0.16;
    const upperMainY = 0.932;
    const upperLugLocalY = doorHeight - 0.02;
    const upperCoupledY = 0.958;

    const lugLocalDx = K.doorLugX - K.doorClosedOuterX;
    const lugLocalDz = K.doorLugZ - K.doorClosedZ;

    const buildDoor = (side: -1 | 1) => {
      // Outboard mounting plates attached to chalet sill/base side (y=0.113) and header corner (y=0.980)
      const lowerBracket = new THREE.Mesh(outBracketGeo, mats.metalHardware);
      lowerBracket.position.set(side * 0.522, K.sillTopY - 0.012, -0.192);
      lowerBracket.castShadow = true;
      group.add(lowerBracket);

      const upperBracket = new THREE.Mesh(outBracketGeo, mats.metalHardware);
      upperBracket.position.set(side * 0.522, K.sillTopY + 0.855, -0.192);
      upperBracket.castShadow = true;
      group.add(upperBracket);

      // Fixed outboard primary rotation shaft Pa = (+-0.510, -0.195) spanning under-sill horn to upper bracket
      const primaryShaft = new THREE.Mesh(primaryShaftGeo, mats.steelShaft);
      primaryShaft.position.set(side * K.pivotX, (K.sillTopY + 0.865) * 0.5 + 0.022, K.pivotZ);
      group.add(primaryShaft);

      // Fixed outboard secondary pivot pins Pb = (+-0.568, -0.164) for coupled swing links (联动摆杆)
      const lowerLinkPin = new THREE.Mesh(linkPinGeo, mats.metalHardware);
      lowerLinkPin.position.set(side * K.linkPivotX, 0.142, K.linkPivotZ);
      group.add(lowerLinkPin);

      const upperLinkPin = new THREE.Mesh(linkPinGeo, mats.metalHardware);
      upperLinkPin.position.set(side * K.linkPivotX, 0.962, K.linkPivotZ);
      group.add(upperLinkPin);

      const doorPivot = new THREE.Group();
      doorPivot.position.set(side * K.doorClosedOuterX, doorPivotY, K.doorClosedZ);
      group.add(doorPivot);

      const leafCenterX = -side * (K.doorWidth * 0.5);
      const doorLeaf = new THREE.Mesh(doorGeo, mats.woodPrimary);
      doorLeaf.position.set(leafCenterX, doorHeight * 0.5, 0);
      doorLeaf.castShadow = true;
      doorLeaf.receiveShadow = true;
      doorPivot.add(doorLeaf);

      const inset = new THREE.Mesh(insetGeo, mats.paintedSecondary);
      inset.position.set(leafCenterX, doorHeight * 0.5, 0);
      doorPivot.add(inset);

      const knob = new THREE.Mesh(knobGeo, mats.metalHardware);
      knob.position.set(-side * (K.doorWidth - 0.055), doorHeight * 0.48, K.doorThick * 0.5 + 0.01);
      doorPivot.add(knob);

      // Rigid door-corner hinge lug brackets connecting H1 (0,0) to outboard lug H2 (+-0.110, -0.015)
      const pH1Local = new THREE.Vector3(0, 0, 0);
      const pH2Local = new THREE.Vector3(side * lugLocalDx, 0, lugLocalDz);
      const lugSpan = pH1Local.distanceTo(pH2Local);
      for (const ly of [lowerLugLocalY, upperLugLocalY]) {
        const lugBar = new THREE.Mesh(doorLugBarGeo, mats.metalHardware);
        lugBar.scale.set(1, 1, lugSpan);
        alignBarBetweenPoints(
          lugBar,
          new THREE.Vector3(0, ly, 0),
          new THREE.Vector3(side * lugLocalDx, ly, lugLocalDz),
          'z'
        );
        lugBar.castShadow = true;
        doorPivot.add(lugBar);

        const pin1 = new THREE.Mesh(hingeLugPinGeo, mats.metalHardware);
        pin1.position.set(0, ly, 0);
        doorPivot.add(pin1);

        const pin2 = new THREE.Mesh(hingeLugPinGeo, mats.metalHardware);
        pin2.position.set(side * lugLocalDx, ly, lugLocalDz);
        doorPivot.add(pin2);
      }

      const mainArmLower = new THREE.Mesh(swingArmGeo, mats.metalHardware);
      const mainArmUpper = new THREE.Mesh(swingArmGeo, mats.metalHardware);
      const coupledArmLower = new THREE.Mesh(coupledArmGeo, mats.paintedAccent);
      const coupledArmUpper = new THREE.Mesh(coupledArmGeo, mats.paintedAccent);
      mainArmLower.castShadow = true;
      mainArmUpper.castShadow = true;
      coupledArmLower.castShadow = true;
      coupledArmUpper.castShadow = true;
      group.add(mainArmLower, mainArmUpper, coupledArmLower, coupledArmUpper);

      return {
        doorPivot,
        mainArmLower,
        mainArmUpper,
        coupledArmLower,
        coupledArmUpper,
      };
    };

    const leftDoor = buildDoor(-1);
    const rightDoor = buildDoor(1);

    const pinSocketGeo = track(new THREE.CylinderGeometry(0.024, 0.026, 0.068, 12));
    const pinSocketL = new THREE.Mesh(pinSocketGeo, mats.metalHardware);
    const pinSocketR = new THREE.Mesh(pinSocketGeo, mats.metalHardware);
    group.add(pinSocketL, pinSocketR);

    const barGeo = track(new THREE.BoxGeometry(0.022, 0.018, 1.0));
    const hornBarL = new THREE.Mesh(barGeo, mats.metalHardware);
    const hornBarR = new THREE.Mesh(barGeo, mats.metalHardware);
    const linkBarL = new THREE.Mesh(barGeo, mats.steelShaft);
    const linkBarR = new THREE.Mesh(barGeo, mats.steelShaft);
    const birdRodGeo = track(new THREE.CylinderGeometry(0.009, 0.009, 1.0, 8));
    birdRodGeo.rotateX(Math.PI / 2);
    const birdRodL = new THREE.Mesh(birdRodGeo, mats.steelShaft);
    const birdRodR = new THREE.Mesh(birdRodGeo, mats.steelShaft);
    group.add(hornBarL, hornBarR, linkBarL, linkBarR, birdRodL, birdRodR);

    return {
      group,
      update: (drive, _continuousAngleRad, _tipWorldY, _lift, horizontalSlideWorld) => {
        const rawSliderX =
          horizontalSlideWorld !== undefined
            ? Math.abs(horizontalSlideWorld) / Math.max(0.5, scale)
            : B.sliderMinBoundX + drive * (B.sliderMaxBoundX - B.sliderMinBoundX);
        const sol = solveTopperDoubleDoorsLinkage(rawSliderX);

        leftDoor.doorPivot.position.set(sol.leftHingeX, doorPivotY, sol.leftHingeZ);
        leftDoor.doorPivot.rotation.y = -sol.doorOpenRad;
        rightDoor.doorPivot.position.set(sol.rightHingeX, doorPivotY, sol.rightHingeZ);
        rightDoor.doorPivot.rotation.y = sol.doorOpenRad;

        const pPaL = new THREE.Vector3(-K.pivotX, lowerMainY, K.pivotZ);
        const pPaR = new THREE.Vector3(K.pivotX, lowerMainY, K.pivotZ);
        const pH1L = new THREE.Vector3(sol.leftHingeX, lowerMainY, sol.leftHingeZ);
        const pH1R = new THREE.Vector3(sol.rightHingeX, lowerMainY, sol.rightHingeZ);
        leftDoor.mainArmLower.scale.set(1, 1, pPaL.distanceTo(pH1L));
        alignBarBetweenPoints(leftDoor.mainArmLower, pPaL, pH1L, 'z');
        rightDoor.mainArmLower.scale.set(1, 1, pPaR.distanceTo(pH1R));
        alignBarBetweenPoints(rightDoor.mainArmLower, pPaR, pH1R, 'z');

        pPaL.y = upperMainY;
        pPaR.y = upperMainY;
        pH1L.y = upperMainY;
        pH1R.y = upperMainY;
        leftDoor.mainArmUpper.scale.set(1, 1, pPaL.distanceTo(pH1L));
        alignBarBetweenPoints(leftDoor.mainArmUpper, pPaL, pH1L, 'z');
        rightDoor.mainArmUpper.scale.set(1, 1, pPaR.distanceTo(pH1R));
        alignBarBetweenPoints(rightDoor.mainArmUpper, pPaR, pH1R, 'z');

        const pPbL = new THREE.Vector3(-K.linkPivotX, lowerCoupledY, K.linkPivotZ);
        const pPbR = new THREE.Vector3(K.linkPivotX, lowerCoupledY, K.linkPivotZ);
        const pH2L = new THREE.Vector3(sol.leftLugX, lowerCoupledY, sol.leftLugZ);
        const pH2R = new THREE.Vector3(sol.rightLugX, lowerCoupledY, sol.rightLugZ);
        leftDoor.coupledArmLower.scale.set(1, 1, pPbL.distanceTo(pH2L));
        alignBarBetweenPoints(leftDoor.coupledArmLower, pPbL, pH2L, 'z');
        rightDoor.coupledArmLower.scale.set(1, 1, pPbR.distanceTo(pH2R));
        alignBarBetweenPoints(rightDoor.coupledArmLower, pPbR, pH2R, 'z');

        pPbL.y = upperCoupledY;
        pPbR.y = upperCoupledY;
        pH2L.y = upperCoupledY;
        pH2R.y = upperCoupledY;
        leftDoor.coupledArmUpper.scale.set(1, 1, pPbL.distanceTo(pH2L));
        alignBarBetweenPoints(leftDoor.coupledArmUpper, pPbL, pH2L, 'z');
        rightDoor.coupledArmUpper.scale.set(1, 1, pPbR.distanceTo(pH2R));
        alignBarBetweenPoints(rightDoor.coupledArmUpper, pPbR, pH2R, 'z');

        carriageGroup.position.set(0, 0, sol.birdCarriageZ);
        perchPivot.rotation.x = sol.birdPitchRad;
        birdWingL.rotation.z = 0.25 + sol.open01 * 0.65;
        birdWingR.rotation.z = -(0.25 + sol.open01 * 0.65);

        const pPinL = new THREE.Vector3(-sol.sliderX, K.pinY, 0);
        const pPinR = new THREE.Vector3(sol.sliderX, K.pinY, 0);
        const pTopPinL = new THREE.Vector3(-sol.sliderX, K.birdRodY, 0);
        const pTopPinR = new THREE.Vector3(sol.sliderX, K.birdRodY, 0);
        const pCarriage = new THREE.Vector3(0, K.birdRodY, sol.birdCarriageZ);
        const pPivotL = new THREE.Vector3(-K.pivotX, K.pinY, K.pivotZ);
        const pPivotR = new THREE.Vector3(K.pivotX, K.pinY, K.pivotZ);
        const pHornL = new THREE.Vector3(sol.leftHornX, K.pinY, sol.leftHornZ);
        const pHornR = new THREE.Vector3(sol.rightHornX, K.pinY, sol.rightHornZ);

        pinSocketL.position.set(-sol.sliderX, K.pinY + 0.014, 0);
        pinSocketR.position.set(sol.sliderX, K.pinY + 0.014, 0);

        hornBarL.scale.set(1, 1, pPivotL.distanceTo(pHornL));
        alignBarBetweenPoints(hornBarL, pPivotL, pHornL, 'z');
        hornBarR.scale.set(1, 1, pPivotR.distanceTo(pHornR));
        alignBarBetweenPoints(hornBarR, pPivotR, pHornR, 'z');

        linkBarL.scale.set(1, 1, pPinL.distanceTo(pHornL));
        alignBarBetweenPoints(linkBarL, pPinL, pHornL, 'z');
        linkBarR.scale.set(1, 1, pPinR.distanceTo(pHornR));
        alignBarBetweenPoints(linkBarR, pPinR, pHornR, 'z');

        birdRodL.scale.set(1, 1, pTopPinL.distanceTo(pCarriage));
        alignBarBetweenPoints(birdRodL, pTopPinL, pCarriage, 'z');
        birdRodR.scale.set(1, 1, pTopPinR.distanceTo(pCarriage));
        alignBarBetweenPoints(birdRodR, pTopPinR, pCarriage, 'z');
      },
    };
  }

  if (type === 'clapping') {
    const K = CLAPPING_KINEMATICS;
    const B = BELL_CRANK_KINEMATICS;

    // 1. Slotted brass deck track plate along X at z = 0
    const trackGeo = track(new THREE.BoxGeometry(1.26, 0.022, 0.11));
    const trackPlate = new THREE.Mesh(trackGeo, mats.metalHardware);
    trackPlate.position.set(0, 0.014, 0);
    trackPlate.receiveShadow = true;
    group.add(trackPlate);

    interface ClappingPuppetRig {
      side: -1 | 1;
      pinSocket: THREE.Mesh;
      outriggerArm: THREE.Mesh;
      rearPinLug: THREE.Mesh;
      armHorn: THREE.Mesh;
      armRod: THREE.Mesh;
      armPivot: THREE.Group;
      headPivot: THREE.Group;
    }

    const buildClappingPuppet = (
      side: -1 | 1,
      vestMat: THREE.Material
    ): ClappingPuppetRig => {
      const hipX = side * K.hipX;
      const facingX = -side;

      // Compact twin-rail plinths at |x| in [0.485, 0.595] strictly outside slider outriggers (|x| <= 0.467)
      const railPlinthGeo = track(new THREE.BoxGeometry(0.11, 0.032, 0.056));
      const legGeo = track(new THREE.CylinderGeometry(0.026, 0.03, K.hipY - 0.01, 10));
      const bootGeo = track(new THREE.BoxGeometry(0.088, 0.044, 0.05));
      for (const sz of [-0.068, 0.068]) {
        const rail = new THREE.Mesh(railPlinthGeo, mats.woodDarkTrim);
        rail.position.set(hipX, 0.016, sz);
        rail.castShadow = true;
        group.add(rail);

        const leg = new THREE.Mesh(legGeo, mats.woodDarkTrim);
        leg.position.set(hipX, K.hipY * 0.5 + 0.012, sz);
        leg.castShadow = true;
        group.add(leg);

        const boot = new THREE.Mesh(bootGeo, mats.woodSecondary);
        boot.position.set(hipX + facingX * 0.014, 0.048, sz);
        group.add(boot);
      }

      // Pelvis bridge & Upright Painted Waistcoat Torso
      const pelvisGeo = track(new THREE.BoxGeometry(0.11, 0.055, 0.18));
      const pelvis = new THREE.Mesh(pelvisGeo, mats.woodDarkTrim);
      pelvis.position.set(hipX, K.hipY, 0);
      group.add(pelvis);

      const torsoGeo = track(new THREE.BoxGeometry(0.125, 0.34, 0.165));
      const torso = new THREE.Mesh(torsoGeo, vestMat);
      torso.position.set(hipX, K.hipY + 0.18, 0);
      torso.castShadow = true;
      group.add(torso);

      const collarGeo = track(new THREE.BoxGeometry(0.132, 0.05, 0.172));
      const collar = new THREE.Mesh(collarGeo, mats.paintedIvory);
      collar.position.set(hipX, K.hipY + 0.32, 0);
      group.add(collar);

      // Pivoting Head + Cap (nods joyfully toward center when clapping!)
      const headPivot = new THREE.Group();
      headPivot.position.set(hipX, K.hipY + 0.36, 0);
      group.add(headPivot);

      const headGeo = track(new THREE.SphereGeometry(0.068, 14, 12));
      const head = new THREE.Mesh(headGeo, mats.woodPrimary);
      head.position.set(0, 0.082, 0);
      head.castShadow = true;
      headPivot.add(head);

      const capGeo = track(
        new THREE.SphereGeometry(0.074, 14, 10, 0, Math.PI * 2, 0, Math.PI * 0.52)
      );
      const cap = new THREE.Mesh(capGeo, vestMat);
      cap.position.set(0, 0.092, 0);
      headPivot.add(cap);

      // Brass Shoulder Cross-Axle extending cleanly to rear crank plane z = K.rodPlaneZ (-0.145)
      const axleMinZ = K.rodPlaneZ - 0.018;
      const axleMaxZ = 0.115;
      const axleLen = axleMaxZ - axleMinZ;
      const shAxleGeo = track(new THREE.CylinderGeometry(0.013, 0.013, axleLen, 12));
      shAxleGeo.rotateX(Math.PI / 2);
      const shAxle = new THREE.Mesh(shAxleGeo, mats.metalHardware);
      shAxle.position.set(hipX, K.shoulderY, (axleMinZ + axleMaxZ) * 0.5);
      group.add(shAxle);

      // Articulated Twin Clapping Arms + Carved Wooden Palms rotating cleanly around Shoulder Axle
      const armPivot = new THREE.Group();
      armPivot.position.set(hipX, K.shoulderY, 0);
      group.add(armPivot);

      const sleeveGeo = track(new THREE.BoxGeometry(0.27, 0.044, 0.042));
      const forearmGeo = track(new THREE.BoxGeometry(0.25, 0.036, 0.036));
      const palmGeo = track(new THREE.BoxGeometry(K.palmHalfThick * 2, 0.105, 0.062));
      const cuffGeo = track(new THREE.CylinderGeometry(0.022, 0.022, 0.04, 10));
      cuffGeo.rotateZ(Math.PI / 2);

      for (const az of [-0.098, 0.098]) {
        const sleeve = new THREE.Mesh(sleeveGeo, vestMat);
        sleeve.position.set(0.13, 0, az);
        sleeve.castShadow = true;
        armPivot.add(sleeve);

        const forearm = new THREE.Mesh(forearmGeo, mats.paintedIvory);
        forearm.position.set(0.38, 0, az);
        forearm.castShadow = true;
        armPivot.add(forearm);

        const cuff = new THREE.Mesh(cuffGeo, mats.metalHardware);
        cuff.position.set(0.26, 0, az);
        armPivot.add(cuff);

        const palm = new THREE.Mesh(palmGeo, mats.woodPrimary);
        palm.position.set(K.armLen + 0.006, 0, az * 0.72);
        palm.castShadow = true;
        armPivot.add(palm);
      }

      // Deck slider socket + rear brass outrigger arm (z = 0 -> rodPlaneZ) + rear crank horn & pushrod
      const pinSocketGeo = track(new THREE.CylinderGeometry(0.023, 0.025, 0.044, 12));
      const pinSocket = new THREE.Mesh(pinSocketGeo, mats.metalHardware);
      group.add(pinSocket);

      const outriggerLen = Math.abs(K.rodPlaneZ) + 0.018;
      const outriggerGeo = track(new THREE.BoxGeometry(0.024, 0.02, outriggerLen));
      const outriggerArm = new THREE.Mesh(outriggerGeo, mats.metalHardware);
      outriggerArm.castShadow = true;
      group.add(outriggerArm);

      const rearLugGeo = track(new THREE.CylinderGeometry(0.015, 0.015, 0.032, 10));
      const rearPinLug = new THREE.Mesh(rearLugGeo, mats.metalHardware);
      group.add(rearPinLug);

      const hornGeo = track(new THREE.BoxGeometry(0.02, 0.022, 1.0));
      const armHorn = new THREE.Mesh(hornGeo, mats.metalHardware);
      armHorn.castShadow = true;
      group.add(armHorn);

      const rodGeo = track(new THREE.CylinderGeometry(0.009, 0.009, 1.0, 8));
      const armRod = new THREE.Mesh(rodGeo, mats.steelShaft);
      armRod.castShadow = true;
      group.add(armRod);

      return { side, pinSocket, outriggerArm, rearPinLug, armHorn, armRod, armPivot, headPivot };
    };

    const leftPuppet = buildClappingPuppet(-1, mats.paintedAccent);
    const rightPuppet = buildClappingPuppet(1, mats.paintedSecondary);

    const updatePuppet = (
      rig: ClappingPuppetRig,
      branch: ReturnType<typeof solveTopperClappingLinkage>['left'],
      sliderX: number
    ) => {
      const px = rig.side * sliderX;
      const pRearPin = new THREE.Vector3(px, K.pinY, K.rodPlaneZ);
      const pShRear = new THREE.Vector3(branch.shoulderX, branch.shoulderY, K.rodPlaneZ);
      const pHornRear = new THREE.Vector3(branch.hornX, branch.hornY, K.rodPlaneZ);

      rig.pinSocket.position.set(px, K.pinY, 0);
      rig.outriggerArm.position.set(px, K.pinY, K.rodPlaneZ * 0.5);
      rig.rearPinLug.position.copy(pRearPin);

      rig.armHorn.scale.set(1, 1, pShRear.distanceTo(pHornRear));
      alignBarBetweenPoints(rig.armHorn, pShRear, pHornRear, 'z');
      rig.armRod.scale.set(1, pRearPin.distanceTo(pHornRear), 1);
      alignBarBetweenPoints(rig.armRod, pRearPin, pHornRear, 'y');

      // Both puppets use direct XY rotation.z (left: armAngle, right: Math.PI - armAngle pointing inward toward x=0)
      rig.armPivot.rotation.y = 0;
      rig.armPivot.rotation.z = branch.armAngleRad;
      rig.headPivot.rotation.z = rig.side === -1 ? -branch.headNodRad : branch.headNodRad;
    };

    return {
      group,
      update: (drive, _continuousAngleRad, _tipWorldY, _lift, horizontalSlideWorld) => {
        const rawSliderX =
          horizontalSlideWorld !== undefined
            ? Math.abs(horizontalSlideWorld) / Math.max(0.5, scale)
            : B.sliderMinBoundX + drive * (B.sliderMaxBoundX - B.sliderMinBoundX);
        const sol = solveTopperClappingLinkage(rawSliderX);
        updatePuppet(leftPuppet, sol.left, sol.sliderX);
        updatePuppet(rightPuppet, sol.right, sol.sliderX);
      },
    };
  }

  if (type === 'duck-egg') {
    ensureRotaryConverter();
    const K = DUCK_EGG_KINEMATICS;

    // 1. Straw-Woven Nest Bowl & Slotted Wooden Egg Chute on -X side (strictly clearing central r=0.16 collar)
    const nestPlinthGeo = track(new THREE.CylinderGeometry(0.165, 0.185, 0.044, 20));
    const nestPlinth = new THREE.Mesh(nestPlinthGeo, mats.woodDarkTrim);
    nestPlinth.position.set(K.nestCenterX, 0.022, 0);
    nestPlinth.castShadow = true;
    nestPlinth.receiveShadow = true;
    group.add(nestPlinth);

    const nestPedestalGeo = track(new THREE.CylinderGeometry(0.11, 0.135, 0.145, 18));
    const nestPedestal = new THREE.Mesh(nestPedestalGeo, mats.woodDarkTrim);
    nestPedestal.position.set(K.nestCenterX, 0.11, 0);
    nestPedestal.castShadow = true;
    group.add(nestPedestal);

    const nestFloorGeo = track(new THREE.CylinderGeometry(0.114, 0.105, 0.03, 18));
    const nestFloor = new THREE.Mesh(nestFloorGeo, mats.woodDarkTrim);
    nestFloor.position.set(K.nestCenterX, 0.19, 0);
    nestFloor.receiveShadow = true;
    group.add(nestFloor);

    const nestRimGeo = track(new THREE.TorusGeometry(0.122, 0.024, 10, 24));
    nestRimGeo.rotateX(Math.PI / 2);
    const nestRim = new THREE.Mesh(nestRimGeo, mats.woodDarkTrim);
    nestRim.position.set(K.nestCenterX, 0.195, 0);
    nestRim.castShadow = true;
    nestRim.receiveShadow = true;
    group.add(nestRim);

    const strawTuftGeo = track(new THREE.BoxGeometry(0.056, 0.014, 0.018));
    for (let i = 0; i < 10; i++) {
      const a = (i / 10) * Math.PI * 2;
      const tuft = new THREE.Mesh(strawTuftGeo, mats.metalHardware);
      tuft.position.set(K.nestCenterX + Math.cos(a) * 0.126, 0.208, Math.sin(a) * 0.126);
      tuft.rotation.y = -a + 0.35;
      tuft.rotation.z = 0.25;
      group.add(tuft);
    }

    // Sculpted twin wooden chute cheek walls (z = +-0.068, clearing eggRadiusZ=0.060) leaving central slot and rolling path visible
    const chuteCheekShape = new THREE.Shape();
    chuteCheekShape.moveTo(-0.44, 0.02);
    chuteCheekShape.lineTo(-0.22, 0.02);
    chuteCheekShape.lineTo(-0.22, 0.36);
    chuteCheekShape.lineTo(-0.27, 0.36);
    chuteCheekShape.lineTo(-0.44, 0.19);
    chuteCheekShape.closePath();
    const chuteCheekGeo = track(
      new THREE.ExtrudeGeometry(chuteCheekShape, {
        depth: 0.016,
        bevelEnabled: true,
        bevelThickness: 0.004,
        bevelSize: 0.004,
        bevelSegments: 1,
      })
    );
    chuteCheekGeo.translate(0, 0, -0.008);
    for (const sz of [-0.068, 0.068]) {
      const cheek = new THREE.Mesh(chuteCheekGeo, mats.woodDarkTrim);
      cheek.position.set(0, 0, sz);
      cheek.castShadow = true;
      cheek.receiveShadow = true;
      group.add(cheek);
    }

    // Fixed brass pivot bracket for eggCarrierArm at (-0.20, 0.20, 0) outside r=0.16 collar
    const eggBaseGeo = track(new THREE.BoxGeometry(0.068, 0.195, 0.072));
    const eggBase = new THREE.Mesh(eggBaseGeo, mats.woodDarkTrim);
    eggBase.position.set(K.eggPivotX, 0.098, 0);
    eggBase.castShadow = true;
    group.add(eggBase);

    const eggAxlePinGeo = track(new THREE.CylinderGeometry(0.015, 0.015, 0.09, 12));
    eggAxlePinGeo.rotateX(Math.PI / 2);
    const eggAxlePin = new THREE.Mesh(eggAxlePinGeo, mats.metalHardware);
    eggAxlePin.position.set(K.eggPivotX, K.eggPivotY, 0);
    group.add(eggAxlePin);

    // 2. Front Pedestal, Webbed Duck Feet, Carved Legs & Under-Belly Rocker Stanchion (+X side, x >= 0.165)
    const frontPlinthGeo = track(new THREE.BoxGeometry(0.18, 0.036, 0.28));
    const frontPlinth = new THREE.Mesh(frontPlinthGeo, mats.woodDarkTrim);
    frontPlinth.position.set(0.255, 0.018, 0);
    frontPlinth.castShadow = true;
    frontPlinth.receiveShadow = true;
    group.add(frontPlinth);

    const webFootGeo = track(new THREE.BoxGeometry(0.125, 0.02, 0.078));
    const toeGeo = track(new THREE.BoxGeometry(0.055, 0.016, 0.02));
    const legShankGeo = track(new THREE.CylinderGeometry(0.02, 0.024, K.hipY - 0.03, 10));
    for (const sz of [-0.105, 0.105]) {
      const foot = new THREE.Mesh(webFootGeo, mats.paintedAccent);
      foot.position.set(0.245, 0.044, sz);
      foot.castShadow = true;
      group.add(foot);

      for (const ta of [-0.28, 0, 0.28]) {
        const toe = new THREE.Mesh(toeGeo, mats.paintedAccent);
        toe.position.set(0.315, 0.042, sz + Math.sin(ta) * 0.035);
        toe.rotation.y = -ta;
        group.add(toe);
      }

      const leg = new THREE.Mesh(legShankGeo, mats.paintedAccent);
      const pFoot = new THREE.Vector3(0.22, 0.048, sz);
      const pHip = new THREE.Vector3(K.hipX, K.hipY, sz);
      leg.scale.set(1, pFoot.distanceTo(pHip) / (K.hipY - 0.03), 1);
      alignBarBetweenPoints(leg, pFoot, pHip, 'y');
      leg.castShadow = true;
      group.add(leg);
    }

    const hipAxleGeo = track(new THREE.CylinderGeometry(0.016, 0.016, 0.25, 12));
    hipAxleGeo.rotateX(Math.PI / 2);
    const hipAxle = new THREE.Mesh(hipAxleGeo, mats.metalHardware);
    hipAxle.position.set(K.hipX, K.hipY, 0);
    group.add(hipAxle);

    // Compact brass stanchion blades (y = 0.02..0.45, z = +-0.032) strictly below belly underside (y=0.47)
    const stanchionGeo = track(new THREE.BoxGeometry(0.024, K.rockerPivotY - 0.02, 0.012));
    for (const sz of [-0.032, 0.032]) {
      const stanchion = new THREE.Mesh(stanchionGeo, mats.metalHardware);
      stanchion.position.set(K.rockerPivotX, (K.rockerPivotY + 0.02) * 0.5, sz);
      stanchion.castShadow = true;
      group.add(stanchion);
    }

    const rockerAxleGeo = track(new THREE.CylinderGeometry(0.016, 0.016, 0.092, 12));
    rockerAxleGeo.rotateX(Math.PI / 2);
    const rockerAxle = new THREE.Mesh(rockerAxleGeo, mats.metalHardware);
    rockerAxle.position.set(K.rockerPivotX, K.rockerPivotY, 0);
    group.add(rockerAxle);

    // 3. Pushrod Clevis, Pitman Rod, Chest Rocker Arm, Egg Carrier Arm & Drive Links
    const clevisCollarGeo = track(new THREE.CylinderGeometry(0.03, 0.032, 0.046, 12));
    const clevisCollar = new THREE.Mesh(clevisCollarGeo, mats.metalHardware);
    clevisCollar.castShadow = true;
    group.add(clevisCollar);

    const rodGeo = track(new THREE.CylinderGeometry(0.011, 0.011, 1.0, 10));
    const linkBarGeo = track(new THREE.BoxGeometry(0.024, 0.02, 1.0));
    const thinWireGeo = track(new THREE.CylinderGeometry(0.007, 0.007, 1.0, 8));

    const pitmanMesh = new THREE.Mesh(rodGeo, mats.steelShaft);
    pitmanMesh.castShadow = true;
    const rockerArmMesh = new THREE.Mesh(linkBarGeo, mats.metalHardware);
    rockerArmMesh.castShadow = true;
    const eggArmMesh = new THREE.Mesh(linkBarGeo, mats.metalHardware);
    eggArmMesh.castShadow = true;
    const eggDragLink = new THREE.Mesh(thinWireGeo, mats.steelShaft);
    const tailGateLink = new THREE.Mesh(thinWireGeo, mats.steelShaft);
    const neckPullWire = new THREE.Mesh(thinWireGeo, mats.steelShaft);
    group.add(pitmanMesh, rockerArmMesh, eggArmMesh, eggDragLink, tailGateLink, neckPullWire);

    const rockerPinMesh = new THREE.Mesh(
      track(new THREE.CylinderGeometry(0.015, 0.015, 0.068, 10).rotateX(Math.PI / 2)),
      mats.metalHardware
    );
    group.add(rockerPinMesh);

    // Egg Cradle Cup + Sculpted Ivory Wooden Egg
    const eggGroup = new THREE.Group();
    group.add(eggGroup);

    const eggGeo = track(new THREE.SphereGeometry(K.eggRadiusZ, 20, 16));
    eggGeo.scale(K.eggRadiusX / K.eggRadiusZ, 0.96, 0.98);
    const eggMesh = new THREE.Mesh(eggGeo, mats.paintedIvory);
    eggMesh.castShadow = true;
    eggMesh.receiveShadow = true;
    eggGroup.add(eggMesh);

    const cradleRingGeo = track(new THREE.TorusGeometry(0.034, 0.008, 8, 16));
    cradleRingGeo.rotateX(Math.PI / 2);
    const cradleRing = new THREE.Mesh(cradleRingGeo, mats.metalHardware);
    cradleRing.position.set(0, -0.048, 0);
    eggGroup.add(cradleRing);

    // 4. Hollowed Carved Wooden Duck Torso, Hinged Rear Belly Trapdoor, Flapping Wings & Quacking Head
    const torsoPivot = new THREE.Group();
    torsoPivot.position.set(K.hipX, K.hipY, 0);
    group.add(torsoPivot);

    // Twin side flank walls (z = +-0.095, inner cavity |z| <= 0.077 clears eggRadiusZ=0.060 and linkage rods)
    const flankShape = new THREE.Shape();
    flankShape.moveTo(-0.47, -0.14);
    flankShape.lineTo(-0.04, -0.15);
    flankShape.quadraticCurveTo(0.14, -0.14, 0.18, 0.02);
    flankShape.quadraticCurveTo(0.19, 0.17, 0.05, 0.22);
    flankShape.quadraticCurveTo(-0.20, 0.23, -0.47, 0.11);
    flankShape.closePath();
    const flankGeo = track(
      new THREE.ExtrudeGeometry(flankShape, {
        depth: 0.024,
        bevelEnabled: true,
        bevelThickness: 0.006,
        bevelSize: 0.006,
        bevelSegments: 2,
        curveSegments: 14,
      })
    );
    flankGeo.translate(0, 0, -0.012);
    for (const sz of [-0.095, 0.095]) {
      const flank = new THREE.Mesh(flankGeo, mats.woodPrimary);
      flank.position.set(0, 0, sz);
      flank.castShadow = true;
      flank.receiveShadow = true;
      torsoPivot.add(flank);
    }

    // Upper arched saddle back (y_local >= +0.140 -> world y >= 0.760, strictly above pushrod & rocker sweep)
    const saddleBackGeo = track(new THREE.BoxGeometry(0.56, 0.09, 0.178));
    const saddleBack = new THREE.Mesh(saddleBackGeo, mats.woodPrimary);
    saddleBack.position.set(-0.165, 0.185, 0);
    saddleBack.castShadow = true;
    torsoPivot.add(saddleBack);

    // Front rounded breast & keel strictly in x_local >= +0.015 (group x >= 0.225), leaving x <= 0.195 unobstructed
    const frontBreastGeo = track(new THREE.SphereGeometry(0.115, 16, 14));
    frontBreastGeo.scale(0.86, 1.12, 0.82);
    const frontBreast = new THREE.Mesh(frontBreastGeo, mats.woodSecondary);
    frontBreast.position.set(0.115, 0.035, 0);
    frontBreast.castShadow = true;
    torsoPivot.add(frontBreast);

    const frontKeelGeo = track(new THREE.BoxGeometry(0.14, 0.044, 0.148));
    const frontKeel = new THREE.Mesh(frontKeelGeo, mats.woodSecondary);
    frontKeel.position.set(0.085, -0.125, 0);
    torsoPivot.add(frontKeel);

    // Front brass throat eyelet mounted cleanly outside frontBreast for external neckPullWire
    const throatEyeletGeo = track(new THREE.CylinderGeometry(0.011, 0.011, 0.022, 10));
    throatEyeletGeo.rotateX(Math.PI / 2);
    const throatEyelet = new THREE.Mesh(throatEyeletGeo, mats.metalHardware);
    throatEyelet.position.set(0.218, 0.06, 0);
    torsoPivot.add(throatEyelet);

    // Perky carved wooden tail feathers above the hinged trapdoor
    const tailPlumeGeo = track(new THREE.BoxGeometry(0.14, 0.052, 0.12));
    const tailPlume = new THREE.Mesh(tailPlumeGeo, mats.woodDarkTrim);
    tailPlume.position.set(-0.46, 0.155, 0);
    tailPlume.rotation.z = -0.45;
    tailPlume.castShadow = true;
    torsoPivot.add(tailPlume);

    // Hinged Rear Belly Trapdoor (tailFlapPivot) with upward brass crank horn inside upper tunnel
    const tailFlapPivot = new THREE.Group();
    tailFlapPivot.position.set(K.flapPivotLocalX, K.flapPivotLocalY, 0);
    torsoPivot.add(tailFlapPivot);

    const flapHingeGeo = track(new THREE.CylinderGeometry(0.015, 0.015, 0.165, 10));
    flapHingeGeo.rotateX(Math.PI / 2);
    const flapHinge = new THREE.Mesh(flapHingeGeo, mats.metalHardware);
    tailFlapPivot.add(flapHinge);

    const flapHornGeo = track(new THREE.BoxGeometry(0.018, 0.06, 0.018));
    const flapHorn = new THREE.Mesh(flapHornGeo, mats.metalHardware);
    flapHorn.position.set(0, 0.028, -0.022);
    tailFlapPivot.add(flapHorn);

    const flapLeafGeo = track(new THREE.BoxGeometry(0.022, K.flapLength, 0.136));
    const flapLeaf = new THREE.Mesh(flapLeafGeo, mats.woodSecondary);
    flapLeaf.position.set(0, -K.flapLength * 0.5, 0);
    flapLeaf.castShadow = true;
    tailFlapPivot.add(flapLeaf);

    const flapBrassStripGeo = track(new THREE.BoxGeometry(0.028, 0.024, 0.142));
    const flapBrassStrip = new THREE.Mesh(flapBrassStripGeo, mats.metalHardware);
    flapBrassStrip.position.set(0, -K.flapLength + 0.022, 0);
    tailFlapPivot.add(flapBrassStrip);

    // Articulated Side Wings mounted outside outer flank walls (z = +-0.125)
    const wingShape = new THREE.Shape();
    wingShape.moveTo(0.08, 0.04);
    wingShape.quadraticCurveTo(0.06, -0.06, -0.10, -0.075);
    wingShape.quadraticCurveTo(-0.34, -0.06, -0.44, 0.01);
    wingShape.quadraticCurveTo(-0.24, 0.085, 0.08, 0.04);
    const wingGeo = track(
      new THREE.ExtrudeGeometry(wingShape, {
        depth: 0.022,
        bevelEnabled: true,
        bevelThickness: 0.006,
        bevelSize: 0.006,
        bevelSegments: 2,
        curveSegments: 12,
      })
    );
    wingGeo.translate(0, 0, -0.011);

    const buildWing = (side: -1 | 1) => {
      const wp = new THREE.Group();
      wp.position.set(0.01, 0.10, side * 0.125);
      const wingMesh = new THREE.Mesh(wingGeo, mats.woodSecondary);
      wingMesh.castShadow = true;
      wp.add(wingMesh);

      const stripeGeo = track(new THREE.BoxGeometry(0.14, 0.036, 0.026));
      const stripe = new THREE.Mesh(stripeGeo, mats.paintedSecondary);
      stripe.position.set(-0.22, -0.015, 0);
      stripe.rotation.z = 0.08;
      wp.add(stripe);

      torsoPivot.add(wp);
      return wp;
    };
    const leftWingPivot = buildWing(-1);
    const rightWingPivot = buildWing(1);

    // Articulated Neck, Duck Head, Button Eyes & Hinged Lower Bill ("Quack!")
    const neckPivot = new THREE.Group();
    neckPivot.position.set(0.125, 0.145, 0);
    torsoPivot.add(neckPivot);

    const neckGeo = track(new THREE.CylinderGeometry(0.05, 0.068, 0.21, 14));
    const neckMesh = new THREE.Mesh(neckGeo, mats.woodPrimary);
    neckMesh.position.set(0.025, 0.095, 0);
    neckMesh.rotation.z = -0.22;
    neckMesh.castShadow = true;
    neckPivot.add(neckMesh);

    const neckCollarGeo = track(new THREE.TorusGeometry(0.06, 0.015, 10, 18));
    neckCollarGeo.rotateX(Math.PI / 2);
    const neckCollar = new THREE.Mesh(neckCollarGeo, mats.paintedIvory);
    neckCollar.position.set(0.035, 0.125, 0);
    neckCollar.rotation.z = -0.18;
    neckPivot.add(neckCollar);

    const headGeo = track(new THREE.SphereGeometry(0.086, 16, 14));
    headGeo.scale(1.16, 0.96, 0.94);
    const headMesh = new THREE.Mesh(headGeo, mats.woodPrimary);
    headMesh.position.set(0.068, 0.225, 0);
    headMesh.castShadow = true;
    neckPivot.add(headMesh);

    const eyeGeo = track(new THREE.SphereGeometry(0.015, 10, 8));
    const eyeSpecGeo = track(new THREE.SphereGeometry(0.006, 8, 6));
    for (const sz of [-0.075, 0.075]) {
      const eye = new THREE.Mesh(eyeGeo, mats.pegHoleDark);
      eye.position.set(0.105, 0.245, sz);
      neckPivot.add(eye);

      const glint = new THREE.Mesh(eyeSpecGeo, mats.paintedIvory);
      glint.position.set(0.114, 0.251, sz + (sz > 0 ? 0.008 : -0.008));
      neckPivot.add(glint);
    }

    // Upper Duckbill (amber-orange carved spatula bill)
    const upperBillGeo = track(new THREE.BoxGeometry(0.135, 0.024, 0.076));
    const upperBill = new THREE.Mesh(upperBillGeo, mats.paintedAccent);
    upperBill.position.set(0.195, 0.216, 0);
    upperBill.rotation.z = -0.06;
    upperBill.castShadow = true;
    neckPivot.add(upperBill);

    const billTipGeo = track(new THREE.CylinderGeometry(0.038, 0.038, 0.024, 12));
    const billTip = new THREE.Mesh(billTipGeo, mats.paintedAccent);
    billTip.position.set(0.258, 0.212, 0);
    neckPivot.add(billTip);

    // Hinged Lower Duckbill driven by visible external brass throat pull-wire
    const lowerBillPivot = new THREE.Group();
    lowerBillPivot.position.set(0.13, 0.198, 0);
    neckPivot.add(lowerBillPivot);

    const lowerBillGeo = track(new THREE.BoxGeometry(0.118, 0.018, 0.064));
    const lowerBill = new THREE.Mesh(lowerBillGeo, mats.paintedAccent);
    lowerBill.position.set(0.058, -0.004, 0);
    lowerBill.castShadow = true;
    lowerBillPivot.add(lowerBill);

    const lowerTipGeo = track(new THREE.CylinderGeometry(0.032, 0.032, 0.018, 12));
    const lowerTip = new THREE.Mesh(lowerTipGeo, mats.paintedAccent);
    lowerTip.position.set(0.114, -0.004, 0);
    lowerBillPivot.add(lowerTip);

    return {
      group,
      update: (drive, continuousAngleRad, tipWorldY, axialLiftWorld = 0) => {
        currentAxialLiftLocal = axialLiftWorld / Math.max(0.5, scale);
        const pinY = computeEffectivePinY(drive, continuousAngleRad, tipWorldY);
        const sol = solveTopperDuckEggLinkage(pinY);

        clevisCollar.position.set(0, sol.pinY, 0);
        rockerPinMesh.position.set(sol.rockerPinX, sol.rockerPinY, 0);

        const pClevis = new THREE.Vector3(0, sol.pinY, 0);
        const pRockerPivot = new THREE.Vector3(K.rockerPivotX, K.rockerPivotY, 0);
        const pRockerPin = new THREE.Vector3(sol.rockerPinX, sol.rockerPinY, 0);

        pitmanMesh.scale.set(1, pClevis.distanceTo(pRockerPin), 1);
        alignBarBetweenPoints(pitmanMesh, pClevis, pRockerPin, 'y');

        rockerArmMesh.scale.set(1, 1, pRockerPivot.distanceTo(pRockerPin));
        alignBarBetweenPoints(rockerArmMesh, pRockerPivot, pRockerPin, 'z');

        const pEggPivot = new THREE.Vector3(K.eggPivotX, K.eggPivotY, 0);
        const pEggCenter = new THREE.Vector3(sol.eggCenterX, sol.eggCenterY, 0);
        const pCradleBottom = new THREE.Vector3(sol.eggCenterX, sol.eggCenterY - 0.048, 0);
        eggArmMesh.scale.set(1, 1, pEggPivot.distanceTo(pCradleBottom));
        alignBarBetweenPoints(eggArmMesh, pEggPivot, pCradleBottom, 'z');

        eggGroup.position.copy(pEggCenter);
        eggGroup.rotation.z = -0.18 + sol.egg01 * 0.45;

        const pDragA = new THREE.Vector3(sol.rockerPinX, sol.rockerPinY, 0.018);
        const pDragB = new THREE.Vector3(sol.eggLugX, sol.eggLugY, 0.018);
        eggDragLink.scale.set(1, pDragA.distanceTo(pDragB), 1);
        alignBarBetweenPoints(eggDragLink, pDragA, pDragB, 'y');

        // Duck posture, rear belly trapdoor, wings & quacking lower bill
        torsoPivot.rotation.z = sol.torsoPitchRad;
        tailFlapPivot.rotation.z = -sol.flapOpenRad;
        leftWingPivot.rotation.x = -sol.wingFlareRad * 0.58;
        leftWingPivot.rotation.z = 0.08 - sol.wingFlareRad * 0.34;
        rightWingPivot.rotation.x = sol.wingFlareRad * 0.58;
        rightWingPivot.rotation.z = 0.08 - sol.wingFlareRad * 0.34;

        neckPivot.rotation.z = sol.neckNodRad;
        lowerBillPivot.rotation.z = -sol.lowerBillOpenRad;

        torsoPivot.updateMatrixWorld(true);
        const flapHornWorld = new THREE.Vector3(0, 0.052, -0.022);
        tailFlapPivot.localToWorld(flapHornWorld);
        group.worldToLocal(flapHornWorld);
        const pRockerBack = new THREE.Vector3(sol.rockerPinX, sol.rockerPinY, -0.022);
        tailGateLink.scale.set(1, pRockerBack.distanceTo(flapHornWorld), 1);
        alignBarBetweenPoints(tailGateLink, pRockerBack, flapHornWorld, 'y');

        const billHornLocal = new THREE.Vector3(0.032, -0.012, 0);
        lowerBillPivot.localToWorld(billHornLocal);
        group.worldToLocal(billHornLocal);
        const pThroatEyelet = new THREE.Vector3(0.218, 0.06, 0);
        torsoPivot.localToWorld(pThroatEyelet);
        group.worldToLocal(pThroatEyelet);
        neckPullWire.scale.set(1, pThroatEyelet.distanceTo(billHornLocal), 1);
        alignBarBetweenPoints(neckPullWire, pThroatEyelet, billHornLocal, 'y');
      },
    };
  }

  if (type === 'rattle-drum') {
    const K = RATTLE_DRUM_KINEMATICS;

    // 1. Turned wooden rosette base & brass thrust collar on stage deck
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

    // 2. Rotating Handle + Crimson Pellet Drum Assembly (rotates around Y & bobs with axialLift)
    const drumPivot = new THREE.Group();
    group.add(drumPivot);

    // Steel quill stem extending below deck bushing so eccentric axial lift never reveals a gap
    const stemGeo = track(new THREE.CylinderGeometry(0.044, 0.044, 0.42, 14));
    const stem = new THREE.Mesh(stemGeo, mats.steelShaft);
    stem.position.set(0, 0.03, 0);
    stem.castShadow = true;
    drumPivot.add(stem);

    // Carved wooden handle & brass ferrules
    const handleLowerGeo = track(new THREE.CylinderGeometry(0.042, 0.052, 0.28, 14));
    const handleLower = new THREE.Mesh(handleLowerGeo, mats.woodPrimary);
    handleLower.position.set(0, 0.36, 0);
    handleLower.castShadow = true;
    drumPivot.add(handleLower);

    const gripBandGeo = track(new THREE.CylinderGeometry(0.054, 0.048, 0.13, 14));
    const gripBand = new THREE.Mesh(gripBandGeo, mats.paintedAccent);
    gripBand.position.set(0, 0.34, 0);
    gripBand.castShadow = true;
    drumPivot.add(gripBand);

    const ferruleGeo = track(new THREE.CylinderGeometry(0.058, 0.058, 0.034, 16));
    for (const fy of [0.225, 0.505]) {
      const ferrule = new THREE.Mesh(ferruleGeo, mats.metalHardware);
      ferrule.position.set(0, fy, 0);
      ferrule.castShadow = true;
      drumPivot.add(ferrule);
    }

    // Crimson-lacquered cylindrical drum shell oriented along Z
    const barrelGeo = track(
      new THREE.CylinderGeometry(K.drumRadius, K.drumRadius, K.drumHalfDepth * 2, 32)
    );
    barrelGeo.rotateX(Math.PI / 2);
    const barrel = new THREE.Mesh(barrelGeo, mats.paintedAccent);
    barrel.position.set(0, K.drumCenterY, 0);
    barrel.castShadow = true;
    barrel.receiveShadow = true;
    drumPivot.add(barrel);

    // Dark walnut waist trim ring around barrel circumference
    const waistBandGeo = track(
      new THREE.CylinderGeometry(K.drumRadius + 0.005, K.drumRadius + 0.005, 0.042, 32)
    );
    waistBandGeo.rotateX(Math.PI / 2);
    const waistBand = new THREE.Mesh(waistBandGeo, mats.woodDarkTrim);
    waistBand.position.set(0, K.drumCenterY, 0);
    waistBand.castShadow = true;
    drumPivot.add(waistBand);

    // Top turned brass finial crest atop the drum rim
    const finialBaseGeo = track(new THREE.CylinderGeometry(0.038, 0.048, 0.032, 14));
    const finialBase = new THREE.Mesh(finialBaseGeo, mats.metalHardware);
    finialBase.position.set(0, K.drumCenterY + K.drumRadius + 0.014, 0);
    drumPivot.add(finialBase);

    const finialKnobGeo = track(new THREE.SphereGeometry(0.032, 14, 10));
    const finialKnob = new THREE.Mesh(finialKnobGeo, mats.metalHardware);
    finialKnob.position.set(0, K.drumCenterY + K.drumRadius + 0.048, 0);
    finialKnob.castShadow = true;
    drumPivot.add(finialKnob);

    // Twin Ivory Parchment Drumheads + Brass Stud Tacks + Impact Pulse Medallions
    const headSkinGeo = track(
      new THREE.CylinderGeometry(K.drumRadius - 0.016, K.drumRadius - 0.016, 0.006, 28)
    );
    headSkinGeo.rotateX(Math.PI / 2);

    const rimRingGeo = track(new THREE.TorusGeometry(K.drumRadius - 0.008, 0.011, 10, 32));
    const tackGeo = track(new THREE.SphereGeometry(0.01, 8, 6));
    const medallionGeo = track(new THREE.CylinderGeometry(0.068, 0.068, 0.004, 24));
    medallionGeo.rotateX(Math.PI / 2);
    const pulseRingGeo = track(new THREE.TorusGeometry(0.115, 0.006, 8, 28));

    const buildDrumHead = (sgnZ: -1 | 1) => {
      const skin = new THREE.Mesh(headSkinGeo, mats.paintedIvory);
      skin.position.set(0, K.drumCenterY, sgnZ * K.drumHalfDepth);
      skin.receiveShadow = true;
      drumPivot.add(skin);

      const rim = new THREE.Mesh(rimRingGeo, mats.woodDarkTrim);
      rim.position.set(0, K.drumCenterY, sgnZ * K.drumHalfDepth);
      drumPivot.add(rim);

      for (let i = 0; i < 16; i++) {
        const a = (i / 16) * Math.PI * 2;
        const tack = new THREE.Mesh(tackGeo, mats.metalHardware);
        tack.position.set(
          Math.cos(a) * (K.drumRadius - 0.01),
          K.drumCenterY + Math.sin(a) * (K.drumRadius - 0.01),
          sgnZ * (K.drumHalfDepth + 0.008)
        );
        drumPivot.add(tack);
      }

      const medallion = new THREE.Mesh(medallionGeo, mats.paintedAccent);
      medallion.position.set(0, K.drumCenterY, sgnZ * (K.drumHalfDepth + 0.004));
      drumPivot.add(medallion);

      const pulseRing = new THREE.Mesh(pulseRingGeo, mats.metalHardware);
      pulseRing.position.set(0, K.drumCenterY, sgnZ * (K.drumHalfDepth + 0.005));
      drumPivot.add(pulseRing);

      return { medallion, pulseRing };
    };
    const frontHead = buildDrumHead(1);
    const backHead = buildDrumHead(-1);

    // 3. Twin Waist Eyelets, Articulated Braided Cords & Pellet Beads
    const eyeletGeo = track(new THREE.TorusGeometry(0.019, 0.0065, 8, 14));
    const cordSegGeo = track(new THREE.CylinderGeometry(0.01, 0.01, 1.0, 8));
    const elbowKnotGeo = track(new THREE.SphereGeometry(0.012, 10, 8));
    const beadGeo = track(new THREE.SphereGeometry(K.beadRadius, 16, 12));
    const beadBeltGeo = track(new THREE.TorusGeometry(K.beadRadius * 0.98, 0.0055, 8, 18));

    const buildCordAssembly = (side: -1 | 1) => {
      const eyelet = new THREE.Mesh(eyeletGeo, mats.metalHardware);
      eyelet.position.set(side * (K.eyeletOffsetX - 0.01), K.drumCenterY, 0);
      eyelet.rotation.y = Math.PI / 2;
      drumPivot.add(eyelet);

      const seg1 = new THREE.Mesh(cordSegGeo, mats.paintedIvory);
      seg1.castShadow = true;
      drumPivot.add(seg1);

      const seg2 = new THREE.Mesh(cordSegGeo, mats.paintedIvory);
      seg2.castShadow = true;
      drumPivot.add(seg2);

      const elbowKnot = new THREE.Mesh(elbowKnotGeo, mats.paintedIvory);
      drumPivot.add(elbowKnot);

      const beadGroup = new THREE.Group();
      const beadMesh = new THREE.Mesh(beadGeo, mats.paintedAccent);
      beadMesh.castShadow = true;
      beadGroup.add(beadMesh);

      const beadBelt = new THREE.Mesh(beadBeltGeo, mats.metalHardware);
      beadBelt.rotation.x = Math.PI / 2;
      beadGroup.add(beadBelt);
      drumPivot.add(beadGroup);

      return { seg1, seg2, elbowKnot, beadGroup };
    };

    const leftCordRig = buildCordAssembly(-1);
    const rightCordRig = buildCordAssembly(1);

    const updateCordRig = (
      rig: ReturnType<typeof buildCordAssembly>,
      cord: ReturnType<typeof solveTopperRattleDrum>['leftCord']
    ) => {
      const p0 = new THREE.Vector3(cord.eyelet.x, cord.eyelet.y, cord.eyelet.z);
      const p1 = new THREE.Vector3(cord.elbow.x, cord.elbow.y, cord.elbow.z);
      const p2 = new THREE.Vector3(cord.bead.x, cord.bead.y, cord.bead.z);

      rig.seg1.scale.set(1, Math.max(1e-3, p0.distanceTo(p1)), 1);
      alignBarBetweenPoints(rig.seg1, p0, p1, 'y');

      rig.seg2.scale.set(1, Math.max(1e-3, p1.distanceTo(p2)), 1);
      alignBarBetweenPoints(rig.seg2, p1, p2, 'y');

      rig.elbowKnot.position.copy(p1);
      rig.beadGroup.position.copy(p2);
    };

    return {
      group,
      update: (drive, continuousAngleRad, tipWorldY, axialLiftWorld = 0, _h, rotaryDynamics) => {
        let drumAngle = continuousAngleRad;
        let omegaNorm = 0;
        let alphaNorm = 0;
        let liftLocal = 0;

        if (driveMode === 'rotary') {
          liftLocal = Math.max(0, axialLiftWorld) / Math.max(0.5, scale);
          if (rotaryDynamics) {
            omegaNorm = rotaryDynamics.omegaNorm;
            alphaNorm = rotaryDynamics.alphaNorm;
          } else {
            omegaNorm = 1.15 * Math.sin(continuousAngleRad);
            alphaNorm = 1.15 * Math.cos(continuousAngleRad);
          }
        } else {
          const pinY = computeEffectivePinY(drive, continuousAngleRad, tipWorldY);
          liftLocal = Math.max(0, pinY - 0.12) * 0.28;
          const thetaEq = drive * Math.PI * 2;
          drumAngle = 1.05 * (1 - Math.cos(thetaEq));
          omegaNorm = 1.15 * Math.sin(thetaEq);
          alphaNorm = 1.15 * Math.cos(thetaEq);
        }

        drumPivot.position.y = liftLocal;
        drumPivot.rotation.y = -drumAngle;

        const sol = solveTopperRattleDrum(drumAngle, omegaNorm, alphaNorm);
        updateCordRig(leftCordRig, sol.leftCord);
        updateCordRig(rightCordRig, sol.rightCord);

        const fPulse = 1 + sol.frontStrikePulse01 * 0.14;
        frontHead.pulseRing.scale.set(fPulse, fPulse, 1);
        frontHead.medallion.scale.set(1 + sol.frontStrikePulse01 * 0.08, 1 + sol.frontStrikePulse01 * 0.08, 1);

        const bPulse = 1 + sol.backStrikePulse01 * 0.14;
        backHead.pulseRing.scale.set(bPulse, bPulse, 1);
        backHead.medallion.scale.set(1 + sol.backStrikePulse01 * 0.08, 1 + sol.backStrikePulse01 * 0.08, 1);
      },
    };
  }

  if (type === 'donkey-mill') {
    const K = DONKEY_MILL_KINEMATICS;

    // 1. Turned wooden rosette base & brass thrust bushing (strictly enclosing central collar)
    const baseGeo = track(new THREE.CylinderGeometry(0.24, 0.28, 0.068, 24));
    const base = new THREE.Mesh(baseGeo, mats.woodDarkTrim);
    base.position.set(0, K.courtyardTopY + 0.034, 0);
    base.castShadow = true;
    base.receiveShadow = true;
    group.add(base);

    const bushingGeo = track(new THREE.CylinderGeometry(0.12, 0.14, 0.09, 16));
    const bushing = new THREE.Mesh(bushingGeo, mats.metalHardware);
    bushing.position.set(0, 0.14, 0);
    bushing.castShadow = true;
    group.add(bushing);

    // 2. Stationary Mill Courtyard Plinth & Inlaid Brass Skating Track Ring
    const stationaryMillGroup = new THREE.Group();
    group.add(stationaryMillGroup);

    // Circular wooden millstone courtyard plinth spanning [0, courtyardTopY] (clears deck slotStrip!)
    const courtyardGeo = track(new THREE.CylinderGeometry(0.84, 0.87, K.courtyardTopY, 44));
    const courtyard = new THREE.Mesh(courtyardGeo, mats.woodPrimary);
    courtyard.position.set(0, K.courtyardTopY * 0.5, 0);
    courtyard.receiveShadow = true;
    courtyard.castShadow = true;
    stationaryMillGroup.add(courtyard);

    // Dark walnut outer border molding around the courtyard plinth
    const courtyardRimGeo = track(new THREE.TorusGeometry(0.84, 0.016, 10, 48));
    courtyardRimGeo.rotateX(Math.PI / 2);
    const courtyardRim = new THREE.Mesh(courtyardRimGeo, mats.woodDarkTrim);
    courtyardRim.position.set(0, K.courtyardTopY * 0.75, 0);
    stationaryMillGroup.add(courtyardRim);

    // Gleaming inlaid Brass Circular Roller-Skating Track Ring at r = orbitRadius (top = K.deckTopY)
    const trackShape = new THREE.Shape();
    trackShape.absarc(0, 0, 0.665, 0, Math.PI * 2, false);
    const trackHole = new THREE.Path();
    trackHole.absarc(0, 0, 0.495, 0, Math.PI * 2, true);
    trackShape.holes.push(trackHole);
    const trackThickness = K.deckTopY - K.courtyardTopY;
    const brassTrackGeo = track(
      new THREE.ExtrudeGeometry(trackShape, {
        depth: trackThickness,
        bevelEnabled: false,
        curveSegments: 48,
      })
    );
    brassTrackGeo.rotateX(-Math.PI / 2);
    const brassTrack = new THREE.Mesh(brassTrackGeo, mats.metalHardware);
    brassTrack.position.set(0, K.courtyardTopY, 0);
    brassTrack.receiveShadow = true;
    stationaryMillGroup.add(brassTrack);

    // Carved stone bedstone base plinth (下磨台底盘, grounded flush on courtyardTopY)
    const basinHeight = 0.15;
    const basinGeo = track(
      new THREE.CylinderGeometry(K.millRadius + 0.035, K.millRadius + 0.055, basinHeight, 28)
    );
    const basin = new THREE.Mesh(basinGeo, mats.woodDarkTrim);
    basin.position.set(0, K.courtyardTopY + basinHeight * 0.5, 0);
    basin.castShadow = true;
    basin.receiveShadow = true;
    stationaryMillGroup.add(basin);

    // Carved flour collection trough around the stone circumference (磨盘接粉环槽)
    const troughGeo = track(new THREE.TorusGeometry(K.millRadius + 0.016, 0.022, 10, 32));
    troughGeo.rotateX(Math.PI / 2);
    const trough = new THREE.Mesh(troughGeo, mats.woodDarkTrim);
    trough.position.set(0, 0.185, 0);
    stationaryMillGroup.add(trough);

    // White flour trace accumulated in the trough
    const flourTraceGeo = track(new THREE.TorusGeometry(K.millRadius + 0.016, 0.014, 8, 32));
    flourTraceGeo.rotateX(Math.PI / 2);
    const flourTrace = new THREE.Mesh(flourTraceGeo, mats.paintedIvory);
    flourTrace.position.set(0, 0.187, 0);
    stationaryMillGroup.add(flourTrace);

    // Stationary lower bedstone (下磨盘 / 磨心)
    const bedstoneGeo = track(new THREE.CylinderGeometry(K.millRadius, K.millRadius, 0.065, 28));
    const bedstone = new THREE.Mesh(bedstoneGeo, mats.paintedIvory);
    bedstone.position.set(0, 0.19, 0);
    bedstone.castShadow = true;
    bedstone.receiveShadow = true;
    stationaryMillGroup.add(bedstone);

    // Radial carved grinding furrows on lower bedstone (磨齿放射纹)
    const furrowGeo = track(new THREE.BoxGeometry(0.012, 0.005, K.millRadius * 0.72));
    for (let f = 0; f < 8; f++) {
      const fa = (f / 8) * Math.PI * 2;
      const furrow = new THREE.Mesh(furrowGeo, mats.woodDarkTrim);
      furrow.position.set(
        Math.cos(fa) * K.millRadius * 0.45,
        0.224,
        Math.sin(fa) * K.millRadius * 0.45
      );
      furrow.rotation.y = -fa;
      stationaryMillGroup.add(furrow);
    }

    // Angled wooden flour spout (出粉溜槽) tucked strictly inside inner track boundary (r < 0.495)
    const spoutAngle = Math.PI * 0.78;
    const spoutR = K.millRadius + 0.028;
    const spoutX = Math.cos(spoutAngle) * spoutR;
    const spoutZ = Math.sin(spoutAngle) * spoutR;
    const spoutGeo = track(new THREE.BoxGeometry(0.056, 0.028, 0.075));
    const spout = new THREE.Mesh(spoutGeo, mats.woodPrimary);
    spout.position.set(spoutX, 0.145, spoutZ);
    spout.rotation.y = -spoutAngle + Math.PI / 2;
    spout.rotation.x = 0.24;
    spout.castShadow = true;
    stationaryMillGroup.add(spout);

    // Turned wooden flour collecting tub (接面木盆, flush on courtyardTopY inside r <= 0.451 < 0.495)
    const tubR = 0.395;
    const tubX = Math.cos(spoutAngle) * tubR;
    const tubZ = Math.sin(spoutAngle) * tubR;
    const spillGeo = track(new THREE.CylinderGeometry(0.018, 0.032, 0.03, 12));
    const spill = new THREE.Mesh(spillGeo, mats.paintedIvory);
    spill.position.set(Math.cos(spoutAngle) * 0.37, K.courtyardTopY + 0.015, Math.sin(spoutAngle) * 0.37);
    stationaryMillGroup.add(spill);

    const tubGeo = track(new THREE.CylinderGeometry(0.056, 0.046, 0.068, 16));
    const tub = new THREE.Mesh(tubGeo, mats.woodPrimary);
    tub.position.set(tubX, K.courtyardTopY + 0.034, tubZ);
    tub.castShadow = true;
    tub.receiveShadow = true;
    stationaryMillGroup.add(tub);

    const tubFlourGeo = track(new THREE.CylinderGeometry(0.051, 0.051, 0.014, 14));
    const tubFlour = new THREE.Mesh(tubFlourGeo, mats.paintedIvory);
    tubFlour.position.set(tubX, K.courtyardTopY + 0.064, tubZ);
    stationaryMillGroup.add(tubFlour);

    // Burlap grain/flour sack (麻布面粉粮袋, flush on courtyardTopY strictly outside r >= 0.68 > 0.665)
    const sackPos = { x: -0.56, z: -0.51 };
    const sackGeo = track(new THREE.CylinderGeometry(0.068, 0.082, 0.17, 14));
    const sack = new THREE.Mesh(sackGeo, mats.paintedIvory);
    sack.position.set(sackPos.x, K.courtyardTopY + 0.085, sackPos.z);
    sack.rotation.z = 0.06;
    sack.castShadow = true;
    sack.receiveShadow = true;
    stationaryMillGroup.add(sack);

    const sackTieGeo = track(new THREE.TorusGeometry(0.062, 0.009, 8, 14));
    sackTieGeo.rotateX(Math.PI / 2);
    const sackTie = new THREE.Mesh(sackTieGeo, mats.metalHardware);
    sackTie.position.set(sackPos.x, K.courtyardTopY + 0.15, sackPos.z);
    stationaryMillGroup.add(sackTie);

    // 3. Rotating Assembly (Rotating with -continuousAngleRad)
    const millRunnerPivot = new THREE.Group();
    group.add(millRunnerPivot);

    // Central spindle steel quill extending down into deck bushing
    const stemGeo = track(new THREE.CylinderGeometry(0.044, 0.044, 0.38, 12));
    const stem = new THREE.Mesh(stemGeo, mats.steelShaft);
    stem.position.set(0, 0.05, 0);
    millRunnerPivot.add(stem);

    // Upper Rotating Runner Stone (上磨盘)
    const runnerGeo = track(new THREE.CylinderGeometry(K.runnerRadius, K.runnerRadius, K.runnerHeight, 28));
    const runner = new THREE.Mesh(runnerGeo, mats.woodDarkTrim);
    runner.position.set(0, 0.22 + K.runnerHeight * 0.5, 0);
    runner.castShadow = true;
    runner.receiveShadow = true;
    millRunnerPivot.add(runner);

    // Runner stone iron reinforcement hoop (铁箍)
    const hoopGeo = track(new THREE.TorusGeometry(K.runnerRadius + 0.004, 0.009, 8, 32));
    hoopGeo.rotateX(Math.PI / 2);
    for (const hy of [0.22 + 0.02, 0.22 + K.runnerHeight - 0.02]) {
      const hoop = new THREE.Mesh(hoopGeo, mats.metalHardware);
      hoop.position.set(0, hy, 0);
      millRunnerPivot.add(hoop);
    }

    // Central feed-eye hole (磨眼)
    const eyeHoleGeo = track(new THREE.CylinderGeometry(0.052, 0.042, 0.04, 16));
    const eyeHole = new THREE.Mesh(eyeHoleGeo, mats.woodDarkTrim);
    eyeHole.position.set(0, 0.22 + K.runnerHeight + 0.01, 0);
    millRunnerPivot.add(eyeHole);

    // Inverted pyramid wooden grain hopper (粮食漏斗 / 倒方斗)
    const hopperGeo = track(new THREE.CylinderGeometry(0.088, 0.048, 0.09, 4));
    hopperGeo.rotateY(Math.PI / 4);
    const hopper = new THREE.Mesh(hopperGeo, mats.woodPrimary);
    hopper.position.set(0, 0.22 + K.runnerHeight + 0.055, 0);
    hopper.castShadow = true;
    millRunnerPivot.add(hopper);

    // Golden corn/wheat kernels filling the hopper (金黄麦粒)
    const grainGeo = track(new THREE.SphereGeometry(0.038, 10, 8));
    const grain = new THREE.Mesh(grainGeo, mats.metalHardware);
    grain.position.set(0, 0.22 + K.runnerHeight + 0.075, 0);
    millRunnerPivot.add(grain);

    // Elevated Overhead Sweeptree / Hitch Boom (高架拉磨横木杠, clearing saddle & torso cleanly!)
    const boomPostGeo = track(new THREE.BoxGeometry(0.048, 0.11, 0.048));
    const boomPost = new THREE.Mesh(boomPostGeo, mats.metalHardware);
    boomPost.position.set(K.runnerRadius * 0.62, 0.39, 0);
    boomPost.castShadow = true;
    millRunnerPivot.add(boomPost);

    const sweeptreeGeo = track(new THREE.BoxGeometry(K.hitchBoomLen - 0.08, 0.036, 0.038));
    const sweeptree = new THREE.Mesh(sweeptreeGeo, mats.woodPrimary);
    sweeptree.position.set(0.11 + (K.hitchBoomLen - 0.08) * 0.5, 0.44, 0);
    sweeptree.castShadow = true;
    sweeptree.receiveShadow = true;
    millRunnerPivot.add(sweeptree);

    // Transverse overhead yoke crossbar over donkey's shoulders (牵引横档)
    const yokeBarGeo = track(new THREE.BoxGeometry(0.21, 0.03, 0.034));
    const yokeBar = new THREE.Mesh(yokeBarGeo, mats.woodDarkTrim);
    yokeBar.position.set(K.orbitRadius, 0.44, 0.01);
    yokeBar.castShadow = true;
    millRunnerPivot.add(yokeBar);

    const hitchRingGeo = track(new THREE.TorusGeometry(0.014, 0.0045, 8, 12));
    for (const rx of [K.orbitRadius - 0.09, K.orbitRadius + 0.09]) {
      const ring = new THREE.Mesh(hitchRingGeo, mats.metalHardware);
      ring.position.set(rx, 0.425, 0.01);
      ring.rotation.y = Math.PI / 2;
      millRunnerPivot.add(ring);

      // Angled brass drop-traces descending cleanly outside shoulders to the collar hames
      const dxTrace = (rx < K.orbitRadius ? 0.014 : -0.014);
      const dyTrace = 0.32 - 0.425;
      const dzTrace = 0.11 - 0.01;
      const traceLen = Math.hypot(dxTrace, dyTrace, dzTrace);
      const dropTraceGeo = track(new THREE.CylinderGeometry(0.0055, 0.0055, traceLen, 8));
      const dropTrace = new THREE.Mesh(dropTraceGeo, mats.metalHardware);
      dropTrace.position.set(rx + dxTrace * 0.5, 0.425 + dyTrace * 0.5, 0.01 + dzTrace * 0.5);
      const dir = new THREE.Vector3(dxTrace, dyTrace, dzTrace).normalize();
      dropTrace.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
      millRunnerPivot.add(dropTrace);
    }

    // 4. The Roller-Skating Wooden Donkey (四蹄穿复古黄铜木滑轮的拉磨小毛驴)
    // Mounted in millRunnerPivot at (K.orbitRadius, 0, 0) facing +Z (tangent to orbit!)
    const donkeyGroup = new THREE.Group();
    donkeyGroup.position.set(K.orbitRadius, 0, 0);
    millRunnerPivot.add(donkeyGroup);

    // Donkey Torso (毛驴身体)
    const torsoPivot = new THREE.Group();
    torsoPivot.position.set(0, K.donkeyHipY, 0);
    donkeyGroup.add(torsoPivot);

    const bodyGeo = track(new THREE.CylinderGeometry(0.086, 0.096, 0.30, 14));
    bodyGeo.rotateX(Math.PI / 2);
    const body = new THREE.Mesh(bodyGeo, mats.woodPrimary);
    body.castShadow = true;
    body.receiveShadow = true;
    torsoPivot.add(body);

    // Rump & belly curvature (圆滚滚的毛驴腹部与后臀)
    const rumpGeo = track(new THREE.SphereGeometry(0.092, 12, 10));
    const rump = new THREE.Mesh(rumpGeo, mats.woodPrimary);
    rump.position.set(0, 0.005, -0.14);
    rump.castShadow = true;
    torsoPivot.add(rump);

    const chestGeo = track(new THREE.SphereGeometry(0.088, 12, 10));
    const chest = new THREE.Mesh(chestGeo, mats.woodPrimary);
    chest.position.set(0, 0.01, 0.13);
    chest.castShadow = true;
    torsoPivot.add(chest);

    // Folk saddle pack blanket (民间红色鞍垫)
    const saddleGeo = track(new THREE.CylinderGeometry(0.096, 0.104, 0.13, 14));
    saddleGeo.rotateX(Math.PI / 2);
    const saddle = new THREE.Mesh(saddleGeo, mats.paintedAccent);
    saddle.position.set(0, 0.008, 0.01);
    saddle.castShadow = true;
    torsoPivot.add(saddle);

    // Harness belly band / girth (肚带)
    const girthGeo = track(new THREE.TorusGeometry(0.098, 0.008, 8, 18));
    girthGeo.rotateX(Math.PI / 2);
    const girth = new THREE.Mesh(girthGeo, mats.metalHardware);
    girth.position.set(0, 0.005, 0.01);
    torsoPivot.add(girth);

    // Shoulder collar / hames (夹板挽具)
    const collarHamesGeo = track(new THREE.TorusGeometry(0.086, 0.014, 8, 16));
    const collarHames = new THREE.Mesh(collarHamesGeo, mats.woodDarkTrim);
    collarHames.position.set(0, 0.02, 0.11);
    collarHames.rotation.x = -0.32;
    torsoPivot.add(collarHames);

    // Neck & Head Pivot (颈部与头部联动)
    const neckPivot = new THREE.Group();
    neckPivot.position.set(0, 0.05, 0.12);
    torsoPivot.add(neckPivot);

    const neckGeo = track(new THREE.CylinderGeometry(0.05, 0.068, 0.16, 12));
    neckGeo.rotateX(-0.55);
    const neck = new THREE.Mesh(neckGeo, mats.woodPrimary);
    neck.position.set(0, 0.06, 0.04);
    neck.castShadow = true;
    neckPivot.add(neck);

    // Stiff dark wooden mane (深色鬃毛脊)
    const maneGeo = track(new THREE.BoxGeometry(0.018, 0.16, 0.05));
    maneGeo.rotateX(-0.55);
    const mane = new THREE.Mesh(maneGeo, mats.woodDarkTrim);
    mane.position.set(0, 0.085, 0.015);
    neckPivot.add(mane);

    // Donkey Head (毛驴头部)
    const headPivot = new THREE.Group();
    headPivot.position.set(0, 0.13, 0.08);
    neckPivot.add(headPivot);

    const headGeo = track(new THREE.CylinderGeometry(0.048, 0.062, 0.13, 12));
    headGeo.rotateX(0.72);
    const head = new THREE.Mesh(headGeo, mats.woodPrimary);
    head.position.set(0, 0, 0.04);
    head.castShadow = true;
    headPivot.add(head);

    // Soft ivory muzzle tip (象牙白口鼻)
    const muzzleGeo = track(new THREE.SphereGeometry(0.046, 12, 10));
    const muzzle = new THREE.Mesh(muzzleGeo, mats.paintedIvory);
    muzzle.position.set(0, -0.045, 0.085);
    headPivot.add(muzzle);

    // The iconic Red Blindfold (传统拉磨红布眼罩)
    const blindfoldGeo = track(new THREE.TorusGeometry(0.056, 0.014, 8, 16));
    blindfoldGeo.rotateX(0.72);
    const blindfold = new THREE.Mesh(blindfoldGeo, mats.paintedAccent);
    blindfold.position.set(0, 0.012, 0.035);
    headPivot.add(blindfold);

    const blindfoldKnotGeo = track(new THREE.SphereGeometry(0.016, 8, 6));
    const blindfoldKnot = new THREE.Mesh(blindfoldKnotGeo, mats.paintedAccent);
    blindfoldKnot.position.set(0, 0.068, 0.018);
    headPivot.add(blindfoldKnot);

    // Twin Articulated Long Ears (招风长耳朵)
    const leftEarPivot = new THREE.Group();
    leftEarPivot.position.set(-0.035, 0.045, -0.01);
    headPivot.add(leftEarPivot);
    const rightEarPivot = new THREE.Group();
    rightEarPivot.position.set(0.035, 0.045, -0.01);
    headPivot.add(rightEarPivot);

    const earGeo = track(new THREE.CylinderGeometry(0.012, 0.024, 0.13, 8));
    earGeo.rotateZ(0.24);
    earGeo.rotateX(-0.35);
    const leftEar = new THREE.Mesh(earGeo, mats.woodDarkTrim);
    leftEar.position.set(-0.015, 0.06, -0.02);
    leftEarPivot.add(leftEar);

    const rightEarGeo = track(new THREE.CylinderGeometry(0.012, 0.024, 0.13, 8));
    rightEarGeo.rotateZ(-0.24);
    rightEarGeo.rotateX(-0.35);
    const rightEar = new THREE.Mesh(rightEarGeo, mats.woodDarkTrim);
    rightEar.position.set(0.015, 0.06, -0.02);
    rightEarPivot.add(rightEar);

    // Tail Assembly (甩动的小毛驴尾巴)
    const tailPivot = new THREE.Group();
    tailPivot.position.set(0, 0.01, -0.15);
    torsoPivot.add(tailPivot);

    const tailRodGeo = track(new THREE.CylinderGeometry(0.008, 0.008, 0.14, 8));
    tailRodGeo.rotateX(0.35);
    const tailRod = new THREE.Mesh(tailRodGeo, mats.woodPrimary);
    tailRod.position.set(0, -0.06, -0.025);
    tailPivot.add(tailRod);

    const tailTuftGeo = track(new THREE.ConeGeometry(0.022, 0.06, 8));
    tailTuftGeo.rotateX(Math.PI - 0.35);
    const tailTuft = new THREE.Mesh(tailTuftGeo, mats.woodDarkTrim);
    tailTuft.position.set(0, -0.13, -0.05);
    tailPivot.add(tailTuft);

    // Four Articulated Skating Legs with Vintage Brass/Wood Roller Skates
    // Parented directly to torsoPivot at hip pins so legs stay 100% attached to the body
    const buildRollerWheel = () => {
      const wheelPivot = new THREE.Group();

      // Warm wooden roller cylinder (木质滚轮本体)
      const treadGeo = track(new THREE.CylinderGeometry(K.wheelRadius, K.wheelRadius, 0.032, 16));
      treadGeo.rotateZ(Math.PI / 2);
      const tread = new THREE.Mesh(treadGeo, mats.woodPrimary);
      tread.castShadow = true;
      wheelPivot.add(tread);

      // Outer dark-wood wheel flanges (深木轮缘)
      const flangeGeo = track(new THREE.CylinderGeometry(K.wheelRadius * 0.94, K.wheelRadius * 0.94, 0.004, 14));
      flangeGeo.rotateZ(Math.PI / 2);
      for (const fx of [-0.017, 0.017]) {
        const flange = new THREE.Mesh(flangeGeo, mats.woodDarkTrim);
        flange.position.set(fx, 0, 0);
        wheelPivot.add(flange);
      }

      // Shiny brass center axle & spoke cross-bar (黄铜轮毂与可见旋转辐条)
      const axleGeo = track(new THREE.CylinderGeometry(0.0055, 0.0055, 0.052, 10));
      axleGeo.rotateZ(Math.PI / 2);
      const axle = new THREE.Mesh(axleGeo, mats.metalHardware);
      wheelPivot.add(axle);

      const spokeGeo = track(new THREE.BoxGeometry(0.044, 0.0045, 0.021));
      const spoke = new THREE.Mesh(spokeGeo, mats.metalHardware);
      wheelPivot.add(spoke);

      return wheelPivot;
    };

    const buildLegRig = (sideX: number, localZ: number) => {
      const hipGroup = new THREE.Group();
      hipGroup.position.set(sideX, -K.hipJointOffsetY, localZ);
      torsoPivot.add(hipGroup);

      // Upper Thigh
      const thighGroup = new THREE.Group();
      hipGroup.add(thighGroup);

      const thighGeo = track(new THREE.CylinderGeometry(0.022, 0.018, K.thighLen, 8));
      const thighMesh = new THREE.Mesh(thighGeo, mats.woodPrimary);
      thighMesh.position.set(0, -K.thighLen * 0.5, 0);
      thighMesh.castShadow = true;
      thighGroup.add(thighMesh);

      const hipStudGeo = track(new THREE.CylinderGeometry(0.015, 0.015, 0.038, 8));
      hipStudGeo.rotateZ(Math.PI / 2);
      const hipStud = new THREE.Mesh(hipStudGeo, mats.metalHardware);
      thighGroup.add(hipStud);

      // Lower Shank
      const shankGroup = new THREE.Group();
      shankGroup.position.set(0, -K.thighLen, 0);
      thighGroup.add(shankGroup);

      const kneeGeo = track(new THREE.SphereGeometry(0.02, 8, 6));
      const kneeMesh = new THREE.Mesh(kneeGeo, mats.woodDarkTrim);
      shankGroup.add(kneeMesh);

      const shankGeo = track(new THREE.CylinderGeometry(0.018, 0.015, K.shankLen, 8));
      const shankMesh = new THREE.Mesh(shankGeo, mats.woodPrimary);
      shankMesh.position.set(0, -K.shankLen * 0.5, 0);
      shankMesh.castShadow = true;
      shankGroup.add(shankMesh);

      // Ankle Pivot -> Hoof + Vintage Brass/Wood Twin-Wheel Roller Skate (每只蹄下一对复古黄铜木滑轮)
      const skateGroup = new THREE.Group();
      skateGroup.position.set(0, -K.shankLen, 0);
      shankGroup.add(skateGroup);

      // Dark wooden hoof seated inside skate stirrup
      const hoofGeo = track(new THREE.CylinderGeometry(0.019, 0.022, 0.022, 10));
      const hoofMesh = new THREE.Mesh(hoofGeo, mats.woodDarkTrim);
      hoofMesh.position.set(0, -0.002, 0);
      hoofMesh.castShadow = true;
      skateGroup.add(hoofMesh);

      // Crimson folk leather strap clasping hoof onto skate plate
      const strapGeo = track(new THREE.BoxGeometry(0.048, 0.011, 0.03));
      const strapMesh = new THREE.Mesh(strapGeo, mats.paintedAccent);
      strapMesh.position.set(0, -0.007, 0);
      skateGroup.add(strapMesh);

      // Brass soleplate chassis
      const plateGeo = track(new THREE.BoxGeometry(0.044, 0.007, 0.072));
      const plateMesh = new THREE.Mesh(plateGeo, mats.metalHardware);
      plateMesh.position.set(0, -0.016, 0);
      plateMesh.castShadow = true;
      skateGroup.add(plateMesh);

      // Front toe-stop & rear heel-guard brass tabs
      const tabGeo = track(new THREE.BoxGeometry(0.040, 0.012, 0.007));
      for (const tz of [-0.033, 0.033]) {
        const tab = new THREE.Mesh(tabGeo, mats.metalHardware);
        tab.position.set(0, -0.011, tz);
        skateGroup.add(tab);
      }

      // Twin brass axle trucks (front & rear)
      const truckGeo = track(new THREE.BoxGeometry(0.046, 0.013, 0.013));
      for (const az of [-K.wheelAxleHalfSpanZ, K.wheelAxleHalfSpanZ]) {
        const truck = new THREE.Mesh(truckGeo, mats.metalHardware);
        truck.position.set(0, -0.02, az);
        skateGroup.add(truck);
      }

      // Front & rear spinning wood-and-brass roller wheels
      const frontWheelPivot = buildRollerWheel();
      frontWheelPivot.position.set(0, -K.wheelCenterOffsetY, K.wheelAxleHalfSpanZ);
      skateGroup.add(frontWheelPivot);

      const rearWheelPivot = buildRollerWheel();
      rearWheelPivot.position.set(0, -K.wheelCenterOffsetY, -K.wheelAxleHalfSpanZ);
      skateGroup.add(rearWheelPivot);

      return { hipGroup, thighGroup, shankGroup, skateGroup, frontWheelPivot, rearWheelPivot };
    };

    const flLeg = buildLegRig(-K.legHalfSpanX, K.shoulderZ);
    const frLeg = buildLegRig(K.legHalfSpanX, K.shoulderZ);
    const rlLeg = buildLegRig(-K.legHalfSpanX, K.hipZ);
    const rrLeg = buildLegRig(K.legHalfSpanX, K.hipZ);

    const updateLegRig = (
      rig: ReturnType<typeof buildLegRig>,
      legSol: ReturnType<typeof solveTopperDonkeyMill>['frontLeftLeg']
    ) => {
      rig.thighGroup.rotation.x = legSol.thighLocalRotX;
      rig.shankGroup.rotation.x = legSol.shankLocalRotX;
      rig.skateGroup.rotation.x = legSol.skateLocalRotX;
      rig.frontWheelPivot.rotation.x = legSol.wheelSpinRad;
      rig.rearWheelPivot.rotation.x = legSol.wheelSpinRad;
    };

    return {
      group,
      update: (drive, continuousAngleRad, tipWorldY, axialLiftWorld = 0) => {
        let millAngle = continuousAngleRad;
        if (driveMode !== 'rotary') {
          const pinY = computeEffectivePinY(drive, continuousAngleRad, tipWorldY);
          millAngle = (pinY - 0.12) * Math.PI * 4;
        }

        const liftLocal = Math.max(0, axialLiftWorld) / Math.max(0.5, scale);
        millRunnerPivot.position.y = liftLocal;
        millRunnerPivot.rotation.y = -millAngle;

        const sol = solveTopperDonkeyMill(millAngle);

        torsoPivot.position.y = K.donkeyHipY + sol.torsoHeaveY;
        torsoPivot.rotation.x = sol.torsoPitchRad;

        neckPivot.rotation.x = sol.headBobRad;
        leftEarPivot.rotation.z = sol.earFlutterRad;
        rightEarPivot.rotation.z = -sol.earFlutterRad;
        tailPivot.rotation.y = sol.tailSwishRad;

        updateLegRig(flLeg, sol.frontLeftLeg);
        updateLegRig(frLeg, sol.frontRightLeg);
        updateLegRig(rlLeg, sol.rearLeftLeg);
        updateLegRig(rrLeg, sol.rearRightLeg);
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
 * Builds a physically realizable 1:1 Spur-Gear Offset 4-Bar Crank-Rocker Linkage rig:
 * - A 16T driver spur gear on the main horizontal camshaft meshes 1:1 with an offset 16T crank gear
 *   supported by an independent bearing subframe, keeping the crank pin and overhead coupler bar
 *   strictly clear of the steel drive shaft across the entire 360-degree revolution.
 * - All parallel planar links occupy disjoint X planes with zero inter-part clipping.
 */
export function buildFourBarModuleRig(
  mod: FourBarLinkageModule,
  blueprint: AutomataBlueprint,
  mats: AutomataMaterialSet,
  solveFn: (shaftAngleRad: number, m: FourBarLinkageModule) => FourBarSolution
): ModuleRig {
  const disposables: THREE.BufferGeometry[] = [];
  const track = <T extends THREE.BufferGeometry>(g: T): T => {
    disposables.push(g);
    return g;
  };

  const gearSpec = solveFourBarGearSpec(mod.pivotZ);
  const pushCfg = computeFourBarPushrodConfig(mod);

  const rootGroup = new THREE.Group();
  rootGroup.position.set(mod.slotX, 0, 0);

  const hlGeo = track(new THREE.TorusGeometry(0.47, 0.025, 10, 28));
  hlGeo.rotateY(Math.PI / 2);
  const highlightRing = new THREE.Mesh(hlGeo, mats.selectionHighlight);
  highlightRing.position.set(0, blueprint.shaftY, 0);
  highlightRing.visible = false;
  rootGroup.add(highlightRing);

  // 1A. Main-Shaft 16T Driver Spur Gear (x ∈ [-0.044, +0.044])
  const driverGearGroup = new THREE.Group();
  driverGearGroup.position.set(0, blueprint.shaftY, 0);
  rootGroup.add(driverGearGroup);

  const driverGearGeo = track(
    createSpurGearGeometry(
      gearSpec.gearPitchR,
      gearSpec.gearTeeth,
      0.072,
      0.088,
      gearSpec.lineAngleRad
    )
  );
  const driverGearMesh = new THREE.Mesh(driverGearGeo, mats.woodPrimary);
  driverGearMesh.castShadow = true;
  driverGearMesh.receiveShadow = true;
  driverGearMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  driverGearGroup.add(driverGearMesh);

  const hubCollarGeo = track(new THREE.CylinderGeometry(0.145, 0.145, 0.14, 16));
  hubCollarGeo.rotateZ(Math.PI / 2);
  const hubCollar = new THREE.Mesh(hubCollarGeo, mats.metalHardware);
  hubCollar.userData = { moduleId: mod.id, dragRole: 'module' };
  driverGearGroup.add(hubCollar);

  // 1B. Stationary Crank Bearing Subframe at x = -0.145 (never touches camshaft)
  const crankWorldY = blueprint.shaftY + gearSpec.crankCenterY;
  const subframeGroup = new THREE.Group();
  rootGroup.add(subframeGroup);

  if (gearSpec.crankCenterZ < 0) {
    // Cantilevered bracket + diagonal brace from rear pegboard wall to crank gear axle
    const backWallZ = -blueprint.boxDepth * 0.5 + 0.21;
    const armLen = Math.max(0.2, gearSpec.crankCenterZ - backWallZ);
    const armGeo = track(new THREE.BoxGeometry(0.1, 0.13, armLen));
    const armMesh = new THREE.Mesh(armGeo, mats.woodDarkTrim);
    armMesh.position.set(-0.145, crankWorldY, backWallZ + armLen * 0.5);
    armMesh.castShadow = true;
    subframeGroup.add(armMesh);

    const braceLen = Math.hypot(armLen, 0.34);
    const braceGeo = track(new THREE.BoxGeometry(0.08, 0.08, braceLen));
    const braceMesh = new THREE.Mesh(braceGeo, mats.woodDarkTrim);
    braceMesh.position.set(-0.145, crankWorldY - 0.17, backWallZ + armLen * 0.5);
    braceMesh.rotation.x = -Math.atan2(0.34, armLen);
    braceMesh.castShadow = true;
    subframeGroup.add(braceMesh);
  } else {
    // Plinth stanchion post on front side when pivot is dragged to rear (-Z)
    const plinthTopY = 0.28;
    const postH = Math.max(0.4, crankWorldY - plinthTopY);
    const postGeo = track(new THREE.BoxGeometry(0.1, postH, 0.14));
    const postMesh = new THREE.Mesh(postGeo, mats.woodDarkTrim);
    postMesh.position.set(-0.145, plinthTopY + postH * 0.5, gearSpec.crankCenterZ);
    postMesh.castShadow = true;
    subframeGroup.add(postMesh);
  }

  const bearingBossGeo = track(new THREE.CylinderGeometry(0.115, 0.115, 0.09, 16));
  bearingBossGeo.rotateZ(Math.PI / 2);
  const bearingBoss = new THREE.Mesh(bearingBossGeo, mats.metalHardware);
  bearingBoss.position.set(-0.095, crankWorldY, gearSpec.crankCenterZ);
  bearingBoss.castShadow = true;
  subframeGroup.add(bearingBoss);

  const crankAxleGeo = track(new THREE.CylinderGeometry(0.058, 0.058, 0.23, 14));
  crankAxleGeo.rotateZ(Math.PI / 2);
  const crankAxle = new THREE.Mesh(crankAxleGeo, mats.steelShaft);
  crankAxle.position.set(-0.06, crankWorldY, gearSpec.crankCenterZ);
  subframeGroup.add(crankAxle);

  // 1C. Offset 16T Driven Crank Spur Gear + Brass Crank Pin (x ∈ [-0.044, +0.044])
  const crankGroup = new THREE.Group();
  crankGroup.position.set(0, crankWorldY, gearSpec.crankCenterZ);
  rootGroup.add(crankGroup);

  const crankGearGeo = track(
    createSpurGearGeometry(
      gearSpec.gearPitchR,
      gearSpec.gearTeeth,
      0.072,
      0.06,
      gearSpec.lineAngleRad + Math.PI + Math.PI / gearSpec.gearTeeth
    )
  );
  const crankGearMesh = new THREE.Mesh(crankGearGeo, mats.woodSecondary);
  crankGearMesh.castShadow = true;
  crankGearMesh.receiveShadow = true;
  crankGearMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  crankGroup.add(crankGearMesh);

  const crankHubGeo = track(new THREE.CylinderGeometry(0.125, 0.125, 0.09, 16));
  crankHubGeo.rotateZ(Math.PI / 2);
  const crankHub = new THREE.Mesh(crankHubGeo, mats.metalHardware);
  crankHub.userData = { moduleId: mod.id, dragRole: 'module' };
  crankGroup.add(crankHub);

  // Slim brass crank web on front face of crank gear (x ∈ [+0.046, +0.060]) supporting crank pin
  const crankWebGeo = track(new THREE.BoxGeometry(0.014, 0.1, mod.crankRadius + 0.04));
  const crankWeb = new THREE.Mesh(crankWebGeo, mats.metalHardware);
  crankWeb.position.set(0.053, 0, mod.crankRadius * 0.5);
  crankWeb.castShadow = true;
  crankGroup.add(crankWeb);

  // Brass crank pin spans from crank gear/web (x=0.025) to coupler outside face (x=+0.242)
  const crankPinGeo = track(new THREE.CylinderGeometry(0.046, 0.046, 0.22, 12));
  crankPinGeo.rotateZ(Math.PI / 2);
  const crankPinMesh = new THREE.Mesh(crankPinGeo, mats.metalHardware);
  crankPinMesh.position.set(0.134, 0, mod.crankRadius);
  crankPinMesh.castShadow = true;
  crankGroup.add(crankPinMesh);

  // 2. Coupler Rod (P1 -> P2) at x = +0.205 (beveled x ∈ [+0.170, +0.240], outboard of rocker)
  const couplerGroup = new THREE.Group();
  rootGroup.add(couplerGroup);
  const couplerGeo = track(createLinkBarGeometry(mod.couplerLength, 0.14, 0.046));
  const couplerMesh = new THREE.Mesh(couplerGeo, mats.paintedAccent);
  couplerMesh.position.set(0.205, 0, 0);
  couplerMesh.castShadow = true;
  couplerMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  couplerGroup.add(couplerMesh);

  const jointPinGeo = track(new THREE.CylinderGeometry(0.046, 0.046, 0.155, 12));
  jointPinGeo.rotateZ(Math.PI / 2);
  const jointPin = new THREE.Mesh(jointPinGeo, mats.metalHardware);
  jointPin.position.set(0.17, 0, mod.couplerLength);
  couplerGroup.add(jointPin);

  // 3. Draggable Fulcrum Bracket & Rocker Arm (at P3)
  const fulcrumGroup = new THREE.Group();
  rootGroup.add(fulcrumGroup);

  // Support strut at x = -0.145 ([-0.195, -0.095]) anchored cleanly at rear pegboard wall
  const strutLen = Math.max(0.2, mod.pivotZ - (-blueprint.boxDepth * 0.5 + 0.21));
  const strutGeo = track(new THREE.BoxGeometry(0.1, 0.13, strutLen));
  const strutMesh = new THREE.Mesh(strutGeo, mats.woodDarkTrim);
  strutMesh.position.set(-0.145, 0, -strutLen * 0.5);
  strutMesh.castShadow = true;
  fulcrumGroup.add(strutMesh);

  const pivotHandleGeo = track(new THREE.CylinderGeometry(0.115, 0.115, 0.42, 16));
  pivotHandleGeo.rotateZ(Math.PI / 2);
  const pivotHandle = new THREE.Mesh(pivotHandleGeo, mats.metalHardware);
  pivotHandle.position.set(0.04, 0, 0);
  pivotHandle.castShadow = true;
  pivotHandle.userData = { moduleId: mod.id, dragRole: 'pivot' };
  fulcrumGroup.add(pivotHandle);

  const pivotRingGeo = track(new THREE.TorusGeometry(0.145, 0.02, 8, 20));
  pivotRingGeo.rotateY(Math.PI / 2);
  const pivotRing = new THREE.Mesh(pivotRingGeo, mats.paintedAccent);
  pivotRing.position.set(0.26, 0, 0);
  pivotRing.userData = { moduleId: mod.id, dragRole: 'pivot' };
  fulcrumGroup.add(pivotRing);

  // Rocker Arm (P3 -> P2) at x = +0.132 (beveled x ∈ [+0.097, +0.167])
  const rockerGroup = new THREE.Group();
  fulcrumGroup.add(rockerGroup);
  const rockerGeo = track(createLinkBarGeometry(mod.rockerLength, 0.14, 0.046));
  const rockerMesh = new THREE.Mesh(rockerGeo, mats.woodPrimary);
  rockerMesh.position.set(0.132, 0, 0);
  rockerMesh.castShadow = true;
  rockerMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  rockerGroup.add(rockerMesh);

  const clevisPinGeo = track(new THREE.CylinderGeometry(0.042, 0.042, 0.135, 12));
  clevisPinGeo.rotateZ(Math.PI / 2);
  const clevisPin = new THREE.Mesh(clevisPinGeo, mats.metalHardware);
  clevisPin.position.set(0.105, 0, pushCfg.clevisArmDist);
  rockerGroup.add(clevisPin);

  // Hinged Pitman Link at x = +0.068 (beveled x ∈ [+0.043, +0.093])
  const pitmanGroup = new THREE.Group();
  rootGroup.add(pitmanGroup);
  const pitmanGeo = track(createLinkBarGeometry(pushCfg.linkLength, 0.095, 0.026));
  const pitmanMesh = new THREE.Mesh(pitmanGeo, mats.metalHardware);
  pitmanMesh.position.set(0.068, 0, 0);
  pitmanMesh.castShadow = true;
  pitmanGroup.add(pitmanMesh);

  const pitmanTopPinGeo = track(new THREE.CylinderGeometry(0.038, 0.038, 0.135, 10));
  pitmanTopPinGeo.rotateZ(Math.PI / 2);
  const pitmanTopPin = new THREE.Mesh(pitmanTopPinGeo, mats.metalHardware);
  pitmanTopPin.position.set(0.028, 0, pushCfg.linkLength);
  pitmanGroup.add(pitmanTopPin);

  // Coaxial Guided Vertical Pushrod at x = 0 (r = 0.036, strictly vertical at outputStageZ)
  const pushrodGroup = new THREE.Group();
  rootGroup.add(pushrodGroup);
  const minRodWorldBottom = blueprint.shaftY + pushCfg.minRodBottomY;
  const fourBarRodLen = Math.max(0.45, blueprint.stageDeckY + 0.12 - minRodWorldBottom);
  const pushrodGeo = track(new THREE.CylinderGeometry(0.036, 0.036, fourBarRodLen, 12));
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

  const hitGeo = track(new THREE.BoxGeometry(0.65, blueprint.stageDeckY + 0.9, 1.9));
  const hitMat = new THREE.MeshBasicMaterial({ visible: false });
  const hitMesh = new THREE.Mesh(hitGeo, hitMat);
  hitMesh.position.set(0, (blueprint.stageDeckY + 0.9) * 0.5, 0.05);
  hitMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  rootGroup.add(hitMesh);

  return {
    id: mod.id,
    type: mod.type,
    rootGroup,
    hitMeshes: [hitMesh, driverGearMesh, crankGearMesh, couplerMesh, rockerMesh],
    pivotHitMeshes: [pivotHandle, pivotRing],
    highlightRing,
    update: (shaftAngleRad, bp, explodeAmount) => {
      const sol = solveFn(shaftAngleRad, mod);
      const pushSol = solveFourBarPushrod(shaftAngleRad, mod, pushCfg, sol);

      driverGearGroup.rotation.x = -sol.driverAngleRad;
      crankGroup.position.set(0, bp.shaftY + sol.crankCenter.y, sol.crankCenter.z);
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

      const axialLiftWorld = sol.trayBottomY - spec.baseDrivenYOffset;
      const drive01 = (Math.sin(sol.drivenAngleRad) + 1) * 0.5;
      topperRig.update(drive01, sol.drivenAngleRad, 0, axialLiftWorld, undefined, {
        omegaNorm: sol.angularVelNorm,
        alphaNorm: sol.angularAccNorm,
      });
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

/**
 * Builds an offset-above-shaft Scotch Yoke pure simple-harmonic horizontal shuttle mechanism.
 * Driver 16T spur gear on the main camshaft drives an overhead crank spur gear directly above it,
 * converting circular crank rotation into pure sinusoidal horizontal push-pull along ±Z
 * on twin steel guide rails without colliding with the main camshaft.
 */
export function buildScotchYokeModuleRig(
  mod: ScotchYokeModule,
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

  const sol0 = solveScotchYoke(0, mod);

  const hlGeo = track(new THREE.TorusGeometry(0.42, 0.024, 10, 28));
  hlGeo.rotateY(Math.PI / 2);
  const highlightRing = new THREE.Mesh(hlGeo, mats.selectionHighlight);
  highlightRing.position.set(0, blueprint.shaftY, 0);
  highlightRing.visible = false;
  rootGroup.add(highlightRing);

  // 1. Main-Shaft 16T Driver Spur Gear (x in [-0.041, +0.041])
  const driverGearGroup = new THREE.Group();
  driverGearGroup.position.set(0, blueprint.shaftY, 0);
  rootGroup.add(driverGearGroup);

  const driverGearGeo = track(
    createSpurGearGeometry(sol0.driverPitchR, sol0.driverTeeth, 0.082, 0.088, 0)
  );
  const driverGearMesh = new THREE.Mesh(driverGearGeo, mats.woodSecondary);
  driverGearMesh.castShadow = true;
  driverGearMesh.receiveShadow = true;
  driverGearMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  driverGearGroup.add(driverGearMesh);

  const driverHubGeo = track(new THREE.CylinderGeometry(0.125, 0.125, 0.13, 18));
  driverHubGeo.rotateZ(Math.PI / 2);
  const driverHub = new THREE.Mesh(driverHubGeo, mats.metalHardware);
  driverHub.castShadow = true;
  driverGearGroup.add(driverHub);

  // 2. Rear Pegboard Crank-Gear Bearing Bracket (strictly behind gears at x = -0.115)
  const crankWorldY = blueprint.shaftY + sol0.crankCenter.y;
  const backInnerZ = -blueprint.boxDepth * 0.5 + 0.16;
  const bracketLen = Math.max(0.5, 0 - backInnerZ + 0.05);
  const bracketGeo = track(new THREE.BoxGeometry(0.095, 0.11, bracketLen));
  const bracketMesh = new THREE.Mesh(bracketGeo, mats.woodDarkTrim);
  bracketMesh.position.set(-0.115, crankWorldY, backInnerZ + bracketLen * 0.5);
  bracketMesh.castShadow = true;
  rootGroup.add(bracketMesh);

  const rearBearingGeo = track(new THREE.CylinderGeometry(0.085, 0.085, 0.11, 16));
  rearBearingGeo.rotateZ(Math.PI / 2);
  const rearBearing = new THREE.Mesh(rearBearingGeo, mats.metalHardware);
  rearBearing.position.set(-0.115, crankWorldY, 0);
  rearBearing.castShadow = true;
  rootGroup.add(rearBearing);

  const crankAxleGeo = track(new THREE.CylinderGeometry(0.044, 0.044, 0.24, 14));
  crankAxleGeo.rotateZ(Math.PI / 2);
  const crankAxle = new THREE.Mesh(crankAxleGeo, mats.steelShaft);
  crankAxle.position.set(-0.055, crankWorldY, 0);
  crankAxle.castShadow = true;
  rootGroup.add(crankAxle);

  // 3. Overhead Driven Crank Spur Gear + Crank Web + Brass Crank Pin
  const crankPivot = new THREE.Group();
  crankPivot.position.set(0, crankWorldY, 0);
  rootGroup.add(crankPivot);

  const crankGearGeo = track(
    createSpurGearGeometry(
      sol0.drivenPitchR,
      sol0.drivenTeeth,
      0.082,
      0.05,
      -Math.PI * 0.5 + Math.PI / sol0.drivenTeeth
    )
  );
  const crankGearMesh = new THREE.Mesh(crankGearGeo, mats.woodPrimary);
  crankGearMesh.castShadow = true;
  crankGearMesh.receiveShadow = true;
  crankGearMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  crankPivot.add(crankGearMesh);

  // Crank face disc / radial crank arm between gear face (+0.042) and yoke slot (+0.082)
  const crankArmGeo = track(new THREE.BoxGeometry(0.026, 0.085, mod.crankRadius + 0.07));
  const crankArm = new THREE.Mesh(crankArmGeo, mats.metalHardware);
  crankArm.position.set(0.058, 0, mod.crankRadius * 0.5);
  crankArm.castShadow = true;
  crankPivot.add(crankArm);

  const crankHubGeo = track(new THREE.CylinderGeometry(0.095, 0.095, 0.13, 16));
  crankHubGeo.rotateZ(Math.PI / 2);
  const crankHub = new THREE.Mesh(crankHubGeo, mats.metalHardware);
  crankHub.position.set(0.015, 0, 0);
  crankPivot.add(crankHub);

  // Brass crank pin protruding cleanly through the sliding bronze die block
  const pinGeo = track(new THREE.CylinderGeometry(0.03, 0.03, 0.13, 14));
  pinGeo.rotateZ(Math.PI / 2);
  const crankPinMesh = new THREE.Mesh(pinGeo, mats.steelShaft);
  crankPinMesh.position.set(0.116, 0, mod.crankRadius);
  crankPinMesh.castShadow = true;
  crankPivot.add(crankPinMesh);

  const pinCapGeo = track(new THREE.CylinderGeometry(0.04, 0.04, 0.02, 14));
  pinCapGeo.rotateZ(Math.PI / 2);
  const pinCap = new THREE.Mesh(pinCapGeo, mats.metalHardware);
  pinCap.position.set(0.168, 0, mod.crankRadius);
  crankPivot.add(pinCap);

  // 4. Sliding Bronze Die Block (rides inside yoke vertical slot at x = +0.118)
  const dieBlockGeo = track(new THREE.BoxGeometry(0.068, 0.092, 0.092));
  const dieBlock = new THREE.Mesh(dieBlockGeo, mats.metalHardware);
  dieBlock.position.set(0.118, blueprint.shaftY + sol0.slotBlockY, sol0.sliderZ);
  dieBlock.castShadow = true;
  dieBlock.userData = { moduleId: mod.id, dragRole: 'module' };
  rootGroup.add(dieBlock);

  // 5. Twin Fixed Horizontal Steel Guide Rails & End Supports (y = shaftY + 1.38)
  const guideRailYWorld = blueprint.shaftY + sol0.guideRailY;
  const frontHangerZ = 0.92;
  const railSpan = frontHangerZ - backInnerZ;
  const railCenterZ = (frontHangerZ + backInnerZ) * 0.5;
  const railGeo = track(new THREE.CylinderGeometry(0.024, 0.024, railSpan, 12));
  railGeo.rotateX(Math.PI / 2);
  for (const rx of [0.062, 0.174]) {
    const rail = new THREE.Mesh(railGeo, mats.steelShaft);
    rail.position.set(rx, guideRailYWorld, railCenterZ);
    rail.castShadow = true;
    rootGroup.add(rail);
  }

  // Rear pegboard guide mount block
  const rearRailMountGeo = track(new THREE.BoxGeometry(0.18, 0.11, 0.14));
  const rearRailMount = new THREE.Mesh(rearRailMountGeo, mats.woodDarkTrim);
  rearRailMount.position.set(0.118, guideRailYWorld, backInnerZ + 0.06);
  rearRailMount.castShadow = true;
  rootGroup.add(rearRailMount);

  // Front vertical deck hanger supporting the front ends of the guide rails
  const deckBottomY = blueprint.stageDeckY - 0.18;
  const hangerH = Math.max(0.24, deckBottomY - guideRailYWorld + 0.05);
  const frontHangerGroup = new THREE.Group();
  const frontHangerGeo = track(new THREE.BoxGeometry(0.18, hangerH, 0.08));
  const frontHanger = new THREE.Mesh(frontHangerGeo, mats.woodDarkTrim);
  frontHanger.position.set(0.118, guideRailYWorld - 0.025 + hangerH * 0.5, frontHangerZ);
  frontHanger.castShadow = true;
  frontHangerGroup.add(frontHanger);

  const railCapGeo = track(new THREE.BoxGeometry(0.19, 0.09, 0.095));
  const railCap = new THREE.Mesh(railCapGeo, mats.metalHardware);
  railCap.position.set(0.118, guideRailYWorld, frontHangerZ);
  frontHangerGroup.add(railCap);
  rootGroup.add(frontHangerGroup);

  // 6. Rigid Vertical-Slot Yoke Carriage Assembly (slides purely along Z)
  const yokeGroup = new THREE.Group();
  yokeGroup.position.set(0, 0, sol0.sliderZ);
  rootGroup.add(yokeGroup);

  const carriageGeo = track(new THREE.BoxGeometry(0.185, 0.105, 0.25));
  const carriageMesh = new THREE.Mesh(carriageGeo, mats.woodPrimary);
  carriageMesh.position.set(0.118, guideRailYWorld, 0);
  carriageMesh.castShadow = true;
  carriageMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  yokeGroup.add(carriageMesh);

  // Two brass linear bushing sleeves around guide rails
  const bushingGeo = track(new THREE.CylinderGeometry(0.036, 0.036, 0.27, 12));
  bushingGeo.rotateX(Math.PI / 2);
  for (const rx of [0.062, 0.174]) {
    const bush = new THREE.Mesh(bushingGeo, mats.metalHardware);
    bush.position.set(rx, guideRailYWorld, 0);
    yokeGroup.add(bush);
  }

  // Open vertical slot frame surrounding die block
  const slotWallH = sol0.slotHalfHeight * 2 + 0.09;
  const slotWallGeo = track(new THREE.BoxGeometry(0.062, slotWallH, 0.044));
  for (const sz of [-0.072, 0.072]) {
    const slotWall = new THREE.Mesh(slotWallGeo, mats.woodSecondary);
    slotWall.position.set(0.118, crankWorldY, sz);
    slotWall.castShadow = true;
    slotWall.userData = { moduleId: mod.id, dragRole: 'module' };
    yokeGroup.add(slotWall);
  }

  // Bottom slot bridge (stays at crankWorldY - slotHalfHeight - 0.04 >= +0.31 above shaft!)
  const slotCapGeo = track(new THREE.BoxGeometry(0.068, 0.055, 0.188));
  const slotBottomCap = new THREE.Mesh(slotCapGeo, mats.metalHardware);
  slotBottomCap.position.set(0.118, crankWorldY - sol0.slotHalfHeight - 0.03, 0);
  slotBottomCap.castShadow = true;
  yokeGroup.add(slotBottomCap);

  const slotTopCap = new THREE.Mesh(slotCapGeo, mats.metalHardware);
  slotTopCap.position.set(0.118, crankWorldY + sol0.slotHalfHeight + 0.03, 0);
  slotTopCap.castShadow = true;
  yokeGroup.add(slotTopCap);

  // Vertical neck linking top of yoke slot up to the horizontal carriage
  const neckBottom = crankWorldY + sol0.slotHalfHeight + 0.05;
  const neckH = Math.max(0.12, guideRailYWorld - 0.04 - neckBottom);
  const neckGeo = track(new THREE.BoxGeometry(0.064, neckH, 0.14));
  const neckMesh = new THREE.Mesh(neckGeo, mats.woodSecondary);
  neckMesh.position.set(0.118, neckBottom + neckH * 0.5, 0);
  neckMesh.castShadow = true;
  yokeGroup.add(neckMesh);

  // Crosshead arm stepping from carriage (x = +0.118) cleanly to stage center (x = 0)
  const crossArmY = guideRailYWorld + 0.08;
  const crossArmGeo = track(new THREE.BoxGeometry(0.16, 0.048, 0.09));
  const crossArm = new THREE.Mesh(crossArmGeo, mats.metalHardware);
  crossArm.position.set(0.058, crossArmY, 0);
  crossArm.castShadow = true;
  yokeGroup.add(crossArm);

  // Vertical brass drive blade rising through the stage deck longitudinal slot at x = 0
  const bladeTopY = blueprint.stageDeckY + 0.055;
  const bladeBottomY = crossArmY;
  const driveBladeH = bladeTopY - bladeBottomY;
  const driveBladeGeo = track(new THREE.BoxGeometry(0.028, driveBladeH, 0.075));
  const driveBlade = new THREE.Mesh(driveBladeGeo, mats.metalHardware);
  driveBlade.position.set(0, bladeBottomY + driveBladeH * 0.5, 0);
  driveBlade.castShadow = true;
  yokeGroup.add(driveBlade);

  // 7. Recessed Brass Stage Deck Guide Track (moves vertically with stage deck on Explode)
  const deckTrackGroup = new THREE.Group();
  deckTrackGroup.position.set(0, blueprint.stageDeckY, 0);
  const deckTrackBezelGeo = track(new THREE.BoxGeometry(0.16, 0.045, 1.04));
  const deckTrackBezel = new THREE.Mesh(deckTrackBezelGeo, mats.metalHardware);
  deckTrackBezel.position.set(0, -0.012, 0);
  deckTrackGroup.add(deckTrackBezel);
  rootGroup.add(deckTrackGroup);

  // 8. Stage Topper (defaults to 'sawyers' Twin Lumberjacks Crosscut Saw)
  const topperRig = buildTopperGroup(mod.topper, mod.topperScale, mats, track, 'linear');
  topperRig.group.position.set(0, blueprint.stageDeckY, 0);
  topperRig.group.traverse((obj) => {
    if ((obj as THREE.Mesh).isMesh) {
      obj.userData = { moduleId: mod.id, dragRole: 'module' };
    }
  });
  rootGroup.add(topperRig.group);

  const hitGeo = track(new THREE.BoxGeometry(0.95, blueprint.stageDeckY + 1.0, 1.8));
  const hitMat = new THREE.MeshBasicMaterial({ visible: false });
  const hitMesh = new THREE.Mesh(hitGeo, hitMat);
  hitMesh.position.set(0, (blueprint.stageDeckY + 1.0) * 0.5, 0);
  hitMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  rootGroup.add(hitMesh);

  return {
    id: mod.id,
    type: mod.type,
    rootGroup,
    hitMeshes: [hitMesh, driverGearMesh, crankGearMesh, carriageMesh],
    pivotHitMeshes: [],
    highlightRing,
    update: (shaftAngleRad, bp, explodeAmount) => {
      const sol = solveScotchYoke(shaftAngleRad, mod);

      driverGearGroup.rotation.x = -sol.driverAngleRad;
      crankPivot.rotation.x = -sol.crankAngleRad;

      dieBlock.position.set(0.118, bp.shaftY + sol.slotBlockY, sol.sliderZ);
      yokeGroup.position.z = sol.sliderZ;

      deckTrackGroup.position.y = bp.stageDeckY + explodeAmount * 1.15;
      topperRig.group.position.y = bp.stageDeckY + explodeAmount * 1.15;

      topperRig.update(sol.normalizedSlide, sol.crankAngleRad, 0.24, 0, sol.sliderZ);
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
 * Builds the Twin 90-degree Bell-Crank / Clamshell Symmetric Opening & Closing Module:
 * Main shaft 16T spur gear -> overhead crank gear -> vertical pitman connecting rod ->
 * twin-rail vertical crosshead -> symmetric coupler links -> Left & Right 90-deg L-shaped
 * bell-cranks -> upper horizontal guide rail with opposing deck drive tangs.
 */
export function buildBellCrankModuleRig(
  mod: BellCrankModule,
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

  const K = BELL_CRANK_KINEMATICS;
  const sol0 = solveBellCrank(0, mod);

  const hlGeo = track(new THREE.TorusGeometry(0.42, 0.024, 10, 28));
  hlGeo.rotateY(Math.PI / 2);
  const highlightRing = new THREE.Mesh(hlGeo, mats.selectionHighlight);
  highlightRing.position.set(0, blueprint.shaftY, 0);
  highlightRing.visible = false;
  rootGroup.add(highlightRing);

  // 1. Main-Shaft 16T Driver Spur Gear
  const driverGearGroup = new THREE.Group();
  driverGearGroup.position.set(0, blueprint.shaftY, 0);
  rootGroup.add(driverGearGroup);

  const driverGearGeo = track(
    createSpurGearGeometry(sol0.driverPitchR, sol0.driverTeeth, 0.072, 0.088, 0)
  );
  const driverGearMesh = new THREE.Mesh(driverGearGeo, mats.woodSecondary);
  driverGearMesh.castShadow = true;
  driverGearMesh.receiveShadow = true;
  driverGearMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  driverGearGroup.add(driverGearMesh);

  const driverHubGeo = track(new THREE.CylinderGeometry(0.115, 0.115, 0.12, 18));
  driverHubGeo.rotateZ(Math.PI / 2);
  const driverHub = new THREE.Mesh(driverHubGeo, mats.metalHardware);
  driverHub.castShadow = true;
  driverGearGroup.add(driverHub);

  // 2. Rear Pegboard Bearing Bracket & Overhead Driven Crank Spur Gear
  const crankWorldY = blueprint.shaftY + sol0.crankCenter.y;
  const backInnerZ = -blueprint.boxDepth * 0.5 + 0.16;
  const bracketLen = Math.max(0.5, 0 - backInnerZ + 0.04);
  const bracketGeo = track(new THREE.BoxGeometry(0.086, 0.1, bracketLen));
  const bracketMesh = new THREE.Mesh(bracketGeo, mats.woodDarkTrim);
  bracketMesh.position.set(-0.095, crankWorldY, backInnerZ + bracketLen * 0.5);
  bracketMesh.castShadow = true;
  rootGroup.add(bracketMesh);

  const rearBearingGeo = track(new THREE.CylinderGeometry(0.078, 0.078, 0.1, 16));
  rearBearingGeo.rotateZ(Math.PI / 2);
  const rearBearing = new THREE.Mesh(rearBearingGeo, mats.metalHardware);
  rearBearing.position.set(-0.095, crankWorldY, 0);
  rootGroup.add(rearBearing);

  const crankAxleGeo = track(new THREE.CylinderGeometry(0.042, 0.042, 0.2, 14));
  crankAxleGeo.rotateZ(Math.PI / 2);
  const crankAxle = new THREE.Mesh(crankAxleGeo, mats.steelShaft);
  crankAxle.position.set(-0.045, crankWorldY, 0);
  rootGroup.add(crankAxle);

  const crankPivot = new THREE.Group();
  crankPivot.position.set(0, crankWorldY, 0);
  rootGroup.add(crankPivot);

  const crankGearGeo = track(
    createSpurGearGeometry(
      sol0.drivenPitchR,
      sol0.drivenTeeth,
      0.072,
      0.044,
      -Math.PI * 0.5 + Math.PI / sol0.drivenTeeth
    )
  );
  const crankGearMesh = new THREE.Mesh(crankGearGeo, mats.woodPrimary);
  crankGearMesh.castShadow = true;
  crankGearMesh.receiveShadow = true;
  crankGearMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  crankPivot.add(crankGearMesh);

  const crankArmGeo = track(new THREE.BoxGeometry(0.024, mod.crankRadius + 0.06, 0.072));
  const crankArm = new THREE.Mesh(crankArmGeo, mats.metalHardware);
  crankArm.position.set(0.052, mod.crankRadius * 0.5, 0);
  crankArm.castShadow = true;
  crankPivot.add(crankArm);

  const crankPinGeo = track(new THREE.CylinderGeometry(0.024, 0.024, 0.095, 14));
  crankPinGeo.rotateZ(Math.PI / 2);
  const crankPinMesh = new THREE.Mesh(crankPinGeo, mats.steelShaft);
  crankPinMesh.position.set(0.088, mod.crankRadius, 0);
  crankPivot.add(crankPinMesh);

  // 3. Pitman Rod & Twin-Rail Guided Vertical Crosshead (z = -0.07 guide frame, clear of gear teeth)
  const pitmanGeo = track(new THREE.CylinderGeometry(0.016, 0.016, 1.0, 10));
  const pitmanMesh = new THREE.Mesh(pitmanGeo, mats.metalHardware);
  pitmanMesh.castShadow = true;
  rootGroup.add(pitmanMesh);

  const guideZ = -0.07;
  const guideBottomY = blueprint.shaftY + 1.04;
  const guideTopY = blueprint.stageDeckY - 0.18;
  const guideSpanY = guideTopY - guideBottomY;
  const vRailGeo = track(new THREE.CylinderGeometry(0.018, 0.018, guideSpanY, 12));
  for (const gx of [-0.19, 0.19]) {
    const vRail = new THREE.Mesh(vRailGeo, mats.steelShaft);
    vRail.position.set(gx, guideBottomY + guideSpanY * 0.5, guideZ);
    vRail.castShadow = true;
    rootGroup.add(vRail);
  }

  // Lower bridge & rear support arm holding the twin vertical guide rails strictly above gear teeth
  const lowerBridgeGeo = track(new THREE.BoxGeometry(0.46, 0.042, 0.075));
  const lowerBridge = new THREE.Mesh(lowerBridgeGeo, mats.woodDarkTrim);
  lowerBridge.position.set(0, guideBottomY, guideZ);
  rootGroup.add(lowerBridge);

  const rearStandoffLen = Math.abs(guideZ - backInnerZ) + 0.04;
  const rearStandoffGeo = track(new THREE.BoxGeometry(0.11, 0.06, rearStandoffLen));
  const rearStandoff = new THREE.Mesh(rearStandoffGeo, mats.woodDarkTrim);
  rearStandoff.position.set(0, guideBottomY, backInnerZ + rearStandoffLen * 0.5);
  rootGroup.add(rearStandoff);

  const crossheadGroup = new THREE.Group();
  rootGroup.add(crossheadGroup);

  const crossBodyGeo = track(new THREE.BoxGeometry(0.44, 0.068, 0.085));
  const crossBody = new THREE.Mesh(crossBodyGeo, mats.woodPrimary);
  crossBody.position.set(0, 0, guideZ);
  crossBody.castShadow = true;
  crossheadGroup.add(crossBody);

  const vBushGeo = track(new THREE.CylinderGeometry(0.03, 0.03, 0.105, 12));
  for (const gx of [-0.19, 0.19]) {
    const vBush = new THREE.Mesh(vBushGeo, mats.metalHardware);
    vBush.position.set(gx, 0, guideZ);
    crossheadGroup.add(vBush);
  }

  // Forward clevis bar carrying crosshead ears into the front bell-crank plane (z = +0.035)
  const bellPlaneZ = 0.035;
  const crossClevisGeo = track(new THREE.BoxGeometry(0.34, 0.048, 0.14));
  const crossClevis = new THREE.Mesh(crossClevisGeo, mats.metalHardware);
  crossClevis.position.set(0, 0, (guideZ + bellPlaneZ) * 0.5);
  crossheadGroup.add(crossClevis);

  const pitmanLugGeo = track(new THREE.BoxGeometry(0.04, 0.055, 0.08));
  const pitmanLug = new THREE.Mesh(pitmanLugGeo, mats.metalHardware);
  pitmanLug.position.set(0.088, -0.02, -0.025);
  crossheadGroup.add(pitmanLug);

  // 4. Twin Rear-Wall Fulcrum Stanchions + 90-degree L-Shaped Bell-Cranks + Symmetric Couplers
  const fulcrumDepth = Math.abs(bellPlaneZ - backInnerZ) + 0.05;
  const fulcrumArmGeo = track(new THREE.BoxGeometry(0.075, 0.085, fulcrumDepth));
  const fulcrumPinGeo = track(new THREE.CylinderGeometry(0.024, 0.024, 0.09, 14));
  fulcrumPinGeo.rotateX(Math.PI / 2);

  for (const fx of [-K.fulcrumX, K.fulcrumX]) {
    const fArm = new THREE.Mesh(fulcrumArmGeo, mats.woodDarkTrim);
    fArm.position.set(fx, blueprint.shaftY + K.fulcrumY, backInnerZ + fulcrumDepth * 0.5 - 0.02);
    fArm.castShadow = true;
    rootGroup.add(fArm);

    const fPin = new THREE.Mesh(fulcrumPinGeo, mats.metalHardware);
    fPin.position.set(fx, blueprint.shaftY + K.fulcrumY, bellPlaneZ);
    rootGroup.add(fPin);
  }

  const bellCrankGeo = track(createBellCrankLGeometry(K.inArmLen, K.outArmLen, 0.068, 0.036));
  const leftBellPivot = new THREE.Group();
  leftBellPivot.position.set(-K.fulcrumX, blueprint.shaftY + K.fulcrumY, bellPlaneZ);
  const leftBellMesh = new THREE.Mesh(bellCrankGeo, mats.woodSecondary);
  leftBellMesh.castShadow = true;
  leftBellMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  leftBellPivot.add(leftBellMesh);
  rootGroup.add(leftBellPivot);

  const rightBellPivot = new THREE.Group();
  rightBellPivot.position.set(K.fulcrumX, blueprint.shaftY + K.fulcrumY, bellPlaneZ);
  const rightBellMesh = new THREE.Mesh(bellCrankGeo, mats.woodSecondary);
  rightBellMesh.rotation.y = Math.PI;
  rightBellMesh.castShadow = true;
  rightBellMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  rightBellPivot.add(rightBellMesh);
  rootGroup.add(rightBellPivot);

  const linkRodGeo = track(new THREE.BoxGeometry(0.036, 1.0, 0.022));
  const leftCoupler = new THREE.Mesh(linkRodGeo, mats.metalHardware);
  const rightCoupler = new THREE.Mesh(linkRodGeo, mats.metalHardware);
  const leftTopLink = new THREE.Mesh(linkRodGeo, mats.metalHardware);
  const rightTopLink = new THREE.Mesh(linkRodGeo, mats.metalHardware);
  leftCoupler.castShadow = true;
  rightCoupler.castShadow = true;
  leftTopLink.castShadow = true;
  rightTopLink.castShadow = true;
  rootGroup.add(leftCoupler, rightCoupler, leftTopLink, rightTopLink);

  // 5. Upper Horizontal Guide Rail & Symmetric Sliding Brass Deck Drive Carriages (z = 0)
  const deckRailWorldY = blueprint.shaftY + K.deckRailY;
  const deckRailSpanX = 1.24;
  const hRailGeo = track(new THREE.CylinderGeometry(0.02, 0.02, deckRailSpanX, 12));
  hRailGeo.rotateZ(Math.PI / 2);
  const hRail = new THREE.Mesh(hRailGeo, mats.steelShaft);
  hRail.position.set(0, deckRailWorldY, 0);
  hRail.castShadow = true;
  rootGroup.add(hRail);

  const deckBottomY = blueprint.stageDeckY - 0.18;
  const hMountH = Math.max(0.18, deckBottomY - deckRailWorldY + 0.04);
  const hMountGeo = track(new THREE.BoxGeometry(0.065, hMountH, 0.09));
  for (const mx of [-0.6, 0.6]) {
    const hMount = new THREE.Mesh(hMountGeo, mats.woodDarkTrim);
    hMount.position.set(mx, deckRailWorldY - 0.02 + hMountH * 0.5, 0);
    rootGroup.add(hMount);
  }

  const buildDeckSlider = () => {
    const g = new THREE.Group();
    const sleeveGeo = track(new THREE.CylinderGeometry(0.032, 0.032, 0.11, 12));
    sleeveGeo.rotateZ(Math.PI / 2);
    const sleeve = new THREE.Mesh(sleeveGeo, mats.metalHardware);
    g.add(sleeve);

    const tangH = blueprint.stageDeckY + 0.055 - deckRailWorldY;
    const tangGeo = track(new THREE.BoxGeometry(0.042, tangH, 0.048));
    const tang = new THREE.Mesh(tangGeo, mats.metalHardware);
    tang.position.set(0, tangH * 0.5, 0);
    tang.castShadow = true;
    g.add(tang);

    const pinLugGeo = track(new THREE.CylinderGeometry(0.014, 0.014, 0.065, 10));
    pinLugGeo.rotateX(Math.PI / 2);
    const pinLug = new THREE.Mesh(pinLugGeo, mats.steelShaft);
    pinLug.position.set(0, 0, bellPlaneZ * 0.5);
    g.add(pinLug);

    rootGroup.add(g);
    return g;
  };
  const leftSliderGroup = buildDeckSlider();
  const rightSliderGroup = buildDeckSlider();

  // 6. Recessed Twin Brass Stage Deck Bezel
  const deckTrackGroup = new THREE.Group();
  deckTrackGroup.position.set(0, blueprint.stageDeckY, 0);
  const deckTrackBezelGeo = track(new THREE.BoxGeometry(1.18, 0.042, 0.14));
  const deckTrackBezel = new THREE.Mesh(deckTrackBezelGeo, mats.metalHardware);
  deckTrackBezel.position.set(0, -0.012, 0);
  deckTrackGroup.add(deckTrackBezel);
  rootGroup.add(deckTrackGroup);

  // 7. Stage Topper (defaults to 'double-doors' Twin Doors Opening)
  const topperRig = buildTopperGroup(mod.topper, mod.topperScale, mats, track, 'linear');
  topperRig.group.position.set(0, blueprint.stageDeckY, 0);
  topperRig.group.traverse((obj) => {
    if ((obj as THREE.Mesh).isMesh) {
      obj.userData = { moduleId: mod.id, dragRole: 'module' };
    }
  });
  rootGroup.add(topperRig.group);

  const hitGeo = track(new THREE.BoxGeometry(1.25, blueprint.stageDeckY + 1.0, 1.3));
  const hitMat = new THREE.MeshBasicMaterial({ visible: false });
  const hitMesh = new THREE.Mesh(hitGeo, hitMat);
  hitMesh.position.set(0, (blueprint.stageDeckY + 1.0) * 0.5, 0);
  hitMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  rootGroup.add(hitMesh);

  return {
    id: mod.id,
    type: mod.type,
    rootGroup,
    hitMeshes: [hitMesh, driverGearMesh, crankGearMesh, leftBellMesh, rightBellMesh],
    pivotHitMeshes: [],
    highlightRing,
    update: (shaftAngleRad, bp, explodeAmount) => {
      const sol = solveBellCrank(shaftAngleRad, mod);

      driverGearGroup.rotation.x = -sol.driverAngleRad;
      crankPivot.rotation.x = -sol.crankAngleRad;

      const pCrank = new THREE.Vector3(0.088, bp.shaftY + sol.crankPin.y, sol.crankPin.z);
      const pCross = new THREE.Vector3(0.088, bp.shaftY + sol.crossheadY, 0);
      pitmanMesh.scale.set(1, pCrank.distanceTo(pCross), 1);
      alignBarBetweenPoints(pitmanMesh, pCrank, pCross, 'y');

      crossheadGroup.position.y = bp.shaftY + sol.crossheadY;

      const alphaL = Math.atan2(
        sol.leftInPin.y - sol.leftFulcrum.y,
        sol.leftInPin.x - sol.leftFulcrum.x
      );
      leftBellPivot.rotation.z = alphaL;
      rightBellPivot.rotation.z = -alphaL;

      const pEarL = new THREE.Vector3(-K.crossheadEarX, bp.shaftY + sol.crossheadY, bellPlaneZ);
      const pEarR = new THREE.Vector3(K.crossheadEarX, bp.shaftY + sol.crossheadY, bellPlaneZ);
      const pInL = new THREE.Vector3(sol.leftInPin.x, bp.shaftY + sol.leftInPin.y, bellPlaneZ);
      const pInR = new THREE.Vector3(sol.rightInPin.x, bp.shaftY + sol.rightInPin.y, bellPlaneZ);
      leftCoupler.scale.set(1, pEarL.distanceTo(pInL), 1);
      alignBarBetweenPoints(leftCoupler, pEarL, pInL, 'y');
      rightCoupler.scale.set(1, pEarR.distanceTo(pInR), 1);
      alignBarBetweenPoints(rightCoupler, pEarR, pInR, 'y');

      const pOutL = new THREE.Vector3(sol.leftOutPin.x, bp.shaftY + sol.leftOutPin.y, bellPlaneZ);
      const pOutR = new THREE.Vector3(sol.rightOutPin.x, bp.shaftY + sol.rightOutPin.y, bellPlaneZ);
      const pSlideL = new THREE.Vector3(-sol.sliderX, bp.shaftY + K.deckRailY, bellPlaneZ);
      const pSlideR = new THREE.Vector3(sol.sliderX, bp.shaftY + K.deckRailY, bellPlaneZ);
      leftTopLink.scale.set(1, pOutL.distanceTo(pSlideL), 1);
      alignBarBetweenPoints(leftTopLink, pOutL, pSlideL, 'y');
      rightTopLink.scale.set(1, pOutR.distanceTo(pSlideR), 1);
      alignBarBetweenPoints(rightTopLink, pOutR, pSlideR, 'y');

      leftSliderGroup.position.set(-sol.sliderX, bp.shaftY + K.deckRailY, 0);
      rightSliderGroup.position.set(sol.sliderX, bp.shaftY + K.deckRailY, 0);

      deckTrackGroup.position.y = bp.stageDeckY + explodeAmount * 1.15;
      topperRig.group.position.y = bp.stageDeckY + explodeAmount * 1.15;

      topperRig.update(sol.open01, sol.crankAngleRad, 0.24, 0, sol.sliderX);
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
 * Creates a classic beveled hexagonal 7-segment display bar (horizontal or vertical).
 */
function createHexSegmentGeometry(
  length: number,
  width: number,
  depth: number,
  horizontal: boolean
): THREE.ExtrudeGeometry {
  const hl = length * 0.5;
  const hw = width * 0.5;
  const tip = Math.min(hw, hl * 0.28);
  const shape = new THREE.Shape();
  if (horizontal) {
    shape.moveTo(-hl, 0);
    shape.lineTo(-hl + tip, hw);
    shape.lineTo(hl - tip, hw);
    shape.lineTo(hl, 0);
    shape.lineTo(hl - tip, -hw);
    shape.lineTo(-hl + tip, -hw);
  } else {
    shape.moveTo(0, -hl);
    shape.lineTo(hw, -hl + tip);
    shape.lineTo(hw, hl - tip);
    shape.lineTo(0, hl);
    shape.lineTo(-hw, hl - tip);
    shape.lineTo(-hw, -hl + tip);
  }
  shape.closePath();

  const geo = new THREE.ExtrudeGeometry(shape, {
    depth,
    bevelEnabled: true,
    bevelThickness: 0.004,
    bevelSize: 0.004,
    bevelSegments: 1,
  });
  geo.translate(0, 0, -depth * 0.5);
  return geo;
}

/**
 * Builds the 7-Cam Digital Numbers Display (`Digital Numbers by yh246801`):
 * - 7 programmed coaxial brass cams on the main camshaft inside the showcase box
 * - 7 guided vertical steel pushrods rising through aligned brass deck bushings
 * - Twin brushed-steel side cheek plates supporting an upper horizontal pivot bar
 * - 7 collar crank horns & rigid diagonal coupler links driving 7 bent steel actuator wires
 *   that flip beveled emerald-green 7-segment bars cleanly between upright ON (0°) and -90° back OFF.
 */
export function buildDigitalNumbersModuleRig(
  mod: DigitalNumbersModule,
  blueprint: AutomataBlueprint,
  mats: AutomataMaterialSet
): ModuleRig {
  const disposables: THREE.BufferGeometry[] = [];
  const track = <T extends THREE.BufferGeometry>(g: T): T => {
    disposables.push(g);
    return g;
  };

  const K = DIGITAL_NUMBERS_KINEMATICS;
  const clampedSlotX = Math.max(-1.85, Math.min(1.85, mod.slotX));

  const rootGroup = new THREE.Group();
  rootGroup.position.set(clampedSlotX, 0, 0);

  const hlGeo = track(new THREE.TorusGeometry(0.5, 0.024, 10, 32));
  hlGeo.rotateY(Math.PI / 2);
  const highlightRing = new THREE.Mesh(hlGeo, mats.selectionHighlight);
  highlightRing.position.set(-K.plateHalfSpanX - 0.08, blueprint.shaftY, 0);
  highlightRing.visible = false;
  rootGroup.add(highlightRing);

  // Precompute all 7 cam contours once for mesh generation and fast runtime contact solving
  const camContours = Array.from({ length: K.slotCount }, (_, i) =>
    sampleDigitalCamContour(i, mod.dwellRatio, mod.countDirection, 240)
  );

  // 1. 7-Cam Coaxial Bank on Main Shaft (inside showcase box)
  const camBankPivot = new THREE.Group();
  camBankPivot.position.set(0, blueprint.shaftY, 0);
  rootGroup.add(camBankPivot);

  const camCollarGeo = track(new THREE.CylinderGeometry(0.12, 0.12, 0.095, 18));
  camCollarGeo.rotateZ(Math.PI / 2);
  const hitMeshes: THREE.Mesh[] = [];

  for (let i = 0; i < K.slotCount; i++) {
    const camX = (i - 3) * K.camSpacingX;
    const contour = camContours[i];
    const camShape = new THREE.Shape();
    contour.forEach((pt, idx) => {
      if (idx === 0) camShape.moveTo(pt.x, pt.y);
      else camShape.lineTo(pt.x, pt.y);
    });
    camShape.closePath();

    const bore = new THREE.Path();
    bore.absarc(0, 0, 0.09, 0, Math.PI * 2, true);
    camShape.holes.push(bore);

    const camGeo = track(
      new THREE.ExtrudeGeometry(camShape, {
        depth: K.camThickness,
        bevelEnabled: true,
        bevelThickness: 0.008,
        bevelSize: K.camBevelRadial,
        bevelSegments: 2,
      })
    );
    camGeo.translate(0, 0, -K.camThickness * 0.5);
    camGeo.rotateY(-Math.PI / 2);

    const camMesh = new THREE.Mesh(camGeo, mats.metalHardware);
    camMesh.position.set(camX, 0, 0);
    camMesh.castShadow = true;
    camMesh.receiveShadow = true;
    camMesh.userData = { moduleId: mod.id, dragRole: 'module' };
    camBankPivot.add(camMesh);
    hitMeshes.push(camMesh);

    const collar = new THREE.Mesh(camCollarGeo, mats.woodDarkTrim);
    collar.position.set(camX, 0, 0);
    collar.castShadow = true;
    camBankPivot.add(collar);
  }

  // Internal guide rail inside box keeping the 7 vertical steel pushrods aligned vertically
  const guideRailY = blueprint.shaftY + 0.9;
  const backInnerZ = -blueprint.boxDepth * 0.5 + 0.16;
  const guideRailGeo = track(new THREE.BoxGeometry(K.plateHalfSpanX * 2 - 0.02, 0.05, 0.08));
  const guideRail = new THREE.Mesh(guideRailGeo, mats.woodDarkTrim);
  guideRail.position.set(0, guideRailY, 0);
  guideRail.castShadow = true;
  rootGroup.add(guideRail);

  const rearBracketLen = Math.max(0.4, 0 - backInnerZ + 0.03);
  const rearBracketGeo = track(new THREE.BoxGeometry(0.072, 0.072, rearBracketLen));
  for (const bx of [-K.plateHalfSpanX + 0.08, K.plateHalfSpanX - 0.08]) {
    const bracket = new THREE.Mesh(rearBracketGeo, mats.woodDarkTrim);
    bracket.position.set(bx, guideRailY, backInnerZ + rearBracketLen * 0.5);
    bracket.castShadow = true;
    rootGroup.add(bracket);
  }

  const innerSleeveGeo = track(new THREE.CylinderGeometry(0.024, 0.024, 0.078, 12));
  for (let i = 0; i < K.slotCount; i++) {
    const sleeve = new THREE.Mesh(innerSleeveGeo, mats.metalHardware);
    sleeve.position.set((i - 3) * K.camSpacingX, guideRailY, 0);
    rootGroup.add(sleeve);
  }

  // 2. 7 Guided Vertical Cam-Follower Pushrods
  const restCamTopY = blueprint.shaftY + K.camBaseRadius + K.camBevelRadial;
  const restClevisY = blueprint.stageDeckY + K.rodMinDeckY;
  const shoeH = 0.04;
  const rodLen = Math.max(0.8, restClevisY - (restCamTopY + shoeH));

  const shoeGeo = track(new THREE.BoxGeometry(0.05, shoeH, K.followerFootRadius * 2));
  const pushrodGeo = track(new THREE.CylinderGeometry(0.014, 0.014, rodLen, 10));
  const rodClevisPinGeo = track(new THREE.CylinderGeometry(0.014, 0.014, 0.048, 10));
  rodClevisPinGeo.rotateZ(Math.PI / 2);

  const followerGroups: THREE.Group[] = [];
  for (let i = 0; i < K.slotCount; i++) {
    const camX = (i - 3) * K.camSpacingX;
    const fg = new THREE.Group();
    fg.position.set(camX, restCamTopY, 0);

    const shoe = new THREE.Mesh(shoeGeo, mats.metalHardware);
    shoe.position.set(0, shoeH * 0.5, 0);
    shoe.castShadow = true;
    fg.add(shoe);

    const rod = new THREE.Mesh(pushrodGeo, mats.steelShaft);
    rod.position.set(0, shoeH + rodLen * 0.5, 0);
    rod.castShadow = true;
    fg.add(rod);

    const clevisPin = new THREE.Mesh(rodClevisPinGeo, mats.metalHardware);
    clevisPin.position.set(0.024, shoeH + rodLen, 0);
    fg.add(clevisPin);

    rootGroup.add(fg);
    followerGroups.push(fg);
  }

  // 3. Stage Assembly (Deck Escutcheon, Twin Steel Cheek Plates, Upper Pivot Bar, Rockers & 7-Segment Display)
  const stageGroup = new THREE.Group();
  stageGroup.position.set(0, blueprint.stageDeckY, 0);
  rootGroup.add(stageGroup);

  const escutcheonGeo = track(new THREE.BoxGeometry(K.plateHalfSpanX * 2 + 0.06, 0.036, 0.18));
  const escutcheon = new THREE.Mesh(escutcheonGeo, mats.metalHardware);
  escutcheon.position.set(0, -0.008, 0);
  escutcheon.castShadow = true;
  escutcheon.userData = { moduleId: mod.id, dragRole: 'module' };
  stageGroup.add(escutcheon);

  const deckBushGeo = track(new THREE.CylinderGeometry(0.032, 0.038, 0.12, 14));
  for (let i = 0; i < K.slotCount; i++) {
    const bush = new THREE.Mesh(deckBushGeo, mats.metalHardware);
    bush.position.set((i - 3) * K.camSpacingX, -0.032, 0);
    bush.castShadow = true;
    stageGroup.add(bush);
  }

  const displayScaleGroup = new THREE.Group();
  displayScaleGroup.visible = mod.topper !== 'none';
  stageGroup.add(displayScaleGroup);

  // Twin Brushed-Steel Cheek Plates & Upper Horizontal Pivot Bar (faithful to pin_poster.jpg)
  const plateGeo = track(new THREE.BoxGeometry(0.038, 0.44, 0.48));
  const footBracketGeo = track(new THREE.BoxGeometry(0.085, 0.032, 0.52));
  const slotBoltGeo = track(new THREE.CylinderGeometry(0.019, 0.019, 0.048, 10));
  slotBoltGeo.rotateZ(Math.PI / 2);

  for (const side of [-1, 1] as const) {
    const px = side * K.plateHalfSpanX;
    const plate = new THREE.Mesh(plateGeo, mats.steelShaft);
    plate.position.set(px, 0.22, 0.08);
    plate.castShadow = true;
    plate.receiveShadow = true;
    plate.userData = { moduleId: mod.id, dragRole: 'module' };
    displayScaleGroup.add(plate);
    hitMeshes.push(plate);

    const foot = new THREE.Mesh(footBracketGeo, mats.woodDarkTrim);
    foot.position.set(px, 0.016, 0.08);
    foot.castShadow = true;
    displayScaleGroup.add(foot);

    for (const [bz, by] of [
      [-0.1, 0.09],
      [-0.1, 0.35],
      [0.2, 0.09],
    ] as const) {
      const bolt = new THREE.Mesh(slotBoltGeo, mats.metalHardware);
      bolt.position.set(px, by, bz);
      displayScaleGroup.add(bolt);
    }
  }

  const pivotBarGeo = track(new THREE.CylinderGeometry(0.022, 0.022, K.plateHalfSpanX * 2 + 0.05, 16));
  pivotBarGeo.rotateZ(Math.PI / 2);
  const pivotBar = new THREE.Mesh(pivotBarGeo, mats.steelShaft);
  pivotBar.position.set(0, K.pivotBarY, K.pivotBarZ);
  pivotBar.castShadow = true;
  displayScaleGroup.add(pivotBar);

  // Precalculate exact ON crank pin angle so the lower crank horn sits rigidly on the collar
  const solOnCal = solveDigitalNumbers(0, { phaseDeg: 0, dwellRatio: 0.55, countDirection: 1 }, camContours);
  const pinOnCal = solOnCal.segments[0].crankPin;
  const hornLocalVec = new THREE.Vector3(
    0,
    pinOnCal.y - K.pivotBarY,
    pinOnCal.z - K.pivotBarZ
  );

  // Wire builder helper: turns local [x, y, z] polyline into connected steel wire + rounded elbows
  const wireRadius = 0.0105;
  const wireSphereGeo = track(new THREE.SphereGeometry(wireRadius * 1.15, 8, 8));
  const addBentWire = (parent: THREE.Group, pts: Array<[number, number, number]>) => {
    for (let k = 0; k < pts.length - 1; k++) {
      const pA = new THREE.Vector3(...pts[k]);
      const pB = new THREE.Vector3(...pts[k + 1]);
      const segLen = pA.distanceTo(pB);
      if (segLen < 1e-4) continue;
      const segGeo = track(new THREE.CylinderGeometry(wireRadius, wireRadius, segLen, 8));
      const segMesh = new THREE.Mesh(segGeo, mats.steelShaft);
      alignBarBetweenPoints(segMesh, pA, pB, 'y');
      segMesh.castShadow = true;
      parent.add(segMesh);
      if (k > 0) {
        const elbow = new THREE.Mesh(wireSphereGeo, mats.steelShaft);
        elbow.position.copy(pA);
        parent.add(elbow);
      }
    }
  };

  // Shared geometries for collars, horns, couplers, and bold beveled 7-segment hexagonal bars
  const pivotCollarGeo = track(new THREE.CylinderGeometry(0.034, 0.034, 0.058, 14));
  pivotCollarGeo.rotateZ(Math.PI / 2);
  const hornBarGeo = track(new THREE.BoxGeometry(0.016, 0.02, K.crankLen));
  const pinDotGeo = track(new THREE.CylinderGeometry(0.013, 0.013, 0.036, 10));
  pinDotGeo.rotateZ(Math.PI / 2);
  const couplerGeo = track(new THREE.CylinderGeometry(0.012, 0.012, 1.0, 10));

  const horizSegFaceGeo = track(createHexSegmentGeometry(0.28, 0.068, 0.026, true));
  const horizSegBackGeo = track(createHexSegmentGeometry(0.294, 0.082, 0.018, true));
  const vertSegFaceGeo = track(createHexSegmentGeometry(0.28, 0.068, 0.026, false));
  const vertSegBackGeo = track(createHexSegmentGeometry(0.294, 0.082, 0.018, false));

  // Local wire paths in collarPivot[i] space (origin = (camX[i], pivotBarY, pivotBarZ); digit center X = -camX[i])
  const wireSpecs: Array<{
    pts: Array<[number, number, number]>;
    segLocalPos: [number, number, number];
    horizontal: boolean;
  }> = [
    // Slot 0 (camX = -0.57) -> a (top horizontal at Y = 0.80)
    {
      pts: [
        [0, 0, 0],
        [0, 0.8, 0],
        [0.435, 0.8, 0],
      ],
      segLocalPos: [0.57, 0.8, 0],
      horizontal: true,
    },
    // Slot 1 (camX = -0.38) -> f (upper-left vertical at X = -0.172, Y = 0.645)
    {
      pts: [
        [0, 0, 0],
        [0, 0.645, 0],
        [0.176, 0.645, 0],
      ],
      segLocalPos: [0.208, 0.645, 0],
      horizontal: false,
    },
    // Slot 2 (camX = -0.19) -> e (lower-left vertical at X = -0.172, Y = 0.335)
    {
      pts: [
        [0, 0, 0],
        [0, 0.09, 0],
        [-0.075, 0.09, 0],
        [-0.075, 0.335, 0],
        [-0.012, 0.335, 0],
      ],
      segLocalPos: [0.018, 0.335, 0],
      horizontal: false,
    },
    // Slot 3 (camX = 0.00) -> d (bottom horizontal at Y = 0.18)
    {
      pts: [
        [0, 0, 0],
        [0, 0.15, 0],
      ],
      segLocalPos: [0, 0.18, 0],
      horizontal: true,
    },
    // Slot 4 (camX = +0.19) -> c (lower-right vertical at X = +0.172, Y = 0.335)
    {
      pts: [
        [0, 0, 0],
        [0, 0.09, 0],
        [0.075, 0.09, 0],
        [0.075, 0.335, 0],
        [0.012, 0.335, 0],
      ],
      segLocalPos: [-0.018, 0.335, 0],
      horizontal: false,
    },
    // Slot 5 (camX = +0.38) -> g (middle horizontal at Y = 0.49, stepped -0.02 in Z behind c/b)
    {
      pts: [
        [0, 0, 0],
        [0, 0.09, -0.02],
        [0, 0.49, -0.02],
        [-0.245, 0.49, -0.02],
        [-0.245, 0.49, 0],
      ],
      segLocalPos: [-0.38, 0.49, 0],
      horizontal: true,
    },
    // Slot 6 (camX = +0.57) -> b (upper-right vertical at X = +0.172, Y = 0.645)
    {
      pts: [
        [0, 0, 0],
        [0, 0.645, 0],
        [-0.366, 0.645, 0],
      ],
      segLocalPos: [-0.398, 0.645, 0],
      horizontal: false,
    },
  ];

  const collarPivots: THREE.Group[] = [];
  const couplerMeshes: THREE.Mesh[] = [];

  for (let i = 0; i < K.slotCount; i++) {
    const camX = (i - 3) * K.camSpacingX;
    const collarPivot = new THREE.Group();
    collarPivot.position.set(camX, K.pivotBarY, K.pivotBarZ);

    const collarSleeve = new THREE.Mesh(pivotCollarGeo, mats.metalHardware);
    collarPivot.add(collarSleeve);

    // Rigid Lower Crank Horn on collar
    const hornMesh = new THREE.Mesh(hornBarGeo, mats.metalHardware);
    hornMesh.scale.set(1, 1, 1);
    const hornStart = new THREE.Vector3(0.024, 0, 0);
    const hornEnd = new THREE.Vector3(0.024, hornLocalVec.y, hornLocalVec.z);
    alignBarBetweenPoints(hornMesh, hornStart, hornEnd, 'z');
    hornMesh.castShadow = true;
    collarPivot.add(hornMesh);

    const hornPin = new THREE.Mesh(pinDotGeo, mats.steelShaft);
    hornPin.position.copy(hornEnd);
    collarPivot.add(hornPin);

    // Bent Steel Actuator Wire & Green Beveled 7-Segment Bar
    const spec = wireSpecs[i];
    addBentWire(collarPivot, spec.pts);

    const segBack = new THREE.Mesh(
      spec.horizontal ? horizSegBackGeo : vertSegBackGeo,
      mats.digitalEdgeDark
    );
    segBack.position.set(spec.segLocalPos[0], spec.segLocalPos[1], spec.segLocalPos[2] - 0.002);
    segBack.castShadow = true;
    segBack.userData = { moduleId: mod.id, dragRole: 'module' };
    collarPivot.add(segBack);

    const segFace = new THREE.Mesh(
      spec.horizontal ? horizSegFaceGeo : vertSegFaceGeo,
      mats.digitalGreen
    );
    segFace.position.set(spec.segLocalPos[0], spec.segLocalPos[1], spec.segLocalPos[2] + 0.002);
    segFace.castShadow = true;
    segFace.userData = { moduleId: mod.id, dragRole: 'module' };
    collarPivot.add(segFace);
    hitMeshes.push(segFace);

    displayScaleGroup.add(collarPivot);
    collarPivots.push(collarPivot);

    const coupler = new THREE.Mesh(couplerGeo, mats.steelShaft);
    coupler.castShadow = true;
    displayScaleGroup.add(coupler);
    couplerMeshes.push(coupler);
  }

  const hitGeo = track(new THREE.BoxGeometry(1.62, blueprint.stageDeckY + 1.35, 1.25));
  const hitMat = new THREE.MeshBasicMaterial({ visible: false });
  const hitMesh = new THREE.Mesh(hitGeo, hitMat);
  hitMesh.position.set(0, (blueprint.stageDeckY + 1.35) * 0.5, 0);
  hitMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  rootGroup.add(hitMesh);
  hitMeshes.unshift(hitMesh);

  return {
    id: mod.id,
    type: mod.type,
    rootGroup,
    hitMeshes,
    pivotHitMeshes: [],
    highlightRing,
    update: (shaftAngleRad, bp, explodeAmount) => {
      const sol = solveDigitalNumbers(shaftAngleRad, mod, camContours);

      camBankPivot.rotation.x = -sol.driverAngleRad;
      stageGroup.position.y = bp.stageDeckY + explodeAmount * 1.15;

      for (let i = 0; i < K.slotCount; i++) {
        const seg = sol.segments[i];
        followerGroups[i].position.y = bp.shaftY + seg.camTopY;

        collarPivots[i].rotation.x = seg.armTiltRad;

        const pRod = new THREE.Vector3(seg.camX + 0.024, seg.rodTopDeckY, 0);
        const pPin = new THREE.Vector3(
          seg.camX + 0.024,
          seg.crankPin.y,
          seg.crankPin.z
        );
        couplerMeshes[i].scale.set(1, pRod.distanceTo(pPin), 1);
        alignBarBetweenPoints(couplerMeshes[i], pRod, pPin, 'y');
      }
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
 * Builds the 3-Cam Programmable Writing Automaton ("Cams as a code -> Drawing as execution"):
 * - 3 coaxial sculpted brass roller-follower cams inside the showcase box (X, Y, Pen-Lift Z)
 * - Guided vertical steel pushrods with miniature rolling brass follower wheels
 * - Open-frame expressive automaton torso & head overlooking an inclined walnut drafting easel
 * - Rigid planar bell-cranks, coaxial shoulder horns, 4-bar parallelogram arm & wrist pen sleeve
 * - Pre-allocated instanced ink trail rendering crisp live "1 2" handwriting on the cream sheet.
 */
export function buildWritingCamModuleRig(
  mod: WritingCamModule,
  blueprint: AutomataBlueprint,
  mats: AutomataMaterialSet
): ModuleRig {
  const disposables: THREE.BufferGeometry[] = [];
  const track = <T extends THREE.BufferGeometry>(g: T): T => {
    disposables.push(g);
    return g;
  };

  const K = WRITING_CAM_KINEMATICS;
  const clampedSlotX = Math.max(-1.85, Math.min(1.85, mod.slotX));

  const rootGroup = new THREE.Group();
  rootGroup.position.set(clampedSlotX, 0, 0);

  const hlGeo = track(new THREE.TorusGeometry(0.5, 0.024, 10, 32));
  hlGeo.rotateY(Math.PI / 2);
  const highlightRing = new THREE.Mesh(hlGeo, mats.selectionHighlight);
  highlightRing.position.set(-0.46, blueprint.shaftY, 0);
  highlightRing.visible = false;
  rootGroup.add(highlightRing);

  const camContours = sampleWritingCamContours(mod.digitPattern, mod.strokeScale);
  const camXs = [-K.camSpacingX, 0, K.camSpacingX];

  // 1. 3-Cam Coaxial Bank on Main Shaft (inside showcase box)
  const camBankPivot = new THREE.Group();
  camBankPivot.position.set(0, blueprint.shaftY, 0);
  rootGroup.add(camBankPivot);

  const camCollarGeo = track(new THREE.CylinderGeometry(0.125, 0.125, 0.105, 18));
  camCollarGeo.rotateZ(Math.PI / 2);
  const hitMeshes: THREE.Mesh[] = [];

  for (let i = 0; i < K.camCount; i++) {
    const camX = camXs[i];
    const contour = camContours[i];
    const camShape = new THREE.Shape();
    contour.forEach((pt, idx) => {
      if (idx === 0) camShape.moveTo(pt.x, pt.y);
      else camShape.lineTo(pt.x, pt.y);
    });
    camShape.closePath();

    const bore = new THREE.Path();
    bore.absarc(0, 0, 0.09, 0, Math.PI * 2, true);
    camShape.holes.push(bore);

    const camGeo = track(
      new THREE.ExtrudeGeometry(camShape, {
        depth: K.camThickness,
        bevelEnabled: true,
        bevelThickness: 0.005,
        bevelSize: K.camBevelRadial,
        bevelSegments: 2,
      })
    );
    camGeo.translate(0, 0, -K.camThickness * 0.5);
    camGeo.rotateY(-Math.PI / 2);

    const camMesh = new THREE.Mesh(camGeo, mats.metalHardware);
    camMesh.position.set(camX, 0, 0);
    camMesh.castShadow = true;
    camMesh.receiveShadow = true;
    camMesh.userData = { moduleId: mod.id, dragRole: 'module' };
    camBankPivot.add(camMesh);
    hitMeshes.push(camMesh);

    const collar = new THREE.Mesh(camCollarGeo, mats.woodDarkTrim);
    collar.position.set(camX, 0, 0);
    collar.castShadow = true;
    camBankPivot.add(collar);
  }

  // Internal wooden guide rail & brass sleeves inside showcase box
  const guideRailY = blueprint.shaftY + 0.9;
  const backInnerZ = -blueprint.boxDepth * 0.5 + 0.16;
  const guideRailGeo = track(new THREE.BoxGeometry(0.76, 0.055, 0.09));
  const guideRail = new THREE.Mesh(guideRailGeo, mats.woodDarkTrim);
  guideRail.position.set(0, guideRailY, 0);
  guideRail.castShadow = true;
  rootGroup.add(guideRail);

  const rearBracketLen = Math.max(0.4, 0 - backInnerZ + 0.03);
  const rearBracketGeo = track(new THREE.BoxGeometry(0.072, 0.072, rearBracketLen));
  for (const bx of [-0.32, 0.32]) {
    const bracket = new THREE.Mesh(rearBracketGeo, mats.woodDarkTrim);
    bracket.position.set(bx, guideRailY, backInnerZ + rearBracketLen * 0.5);
    bracket.castShadow = true;
    rootGroup.add(bracket);
  }

  const innerSleeveGeo = track(new THREE.CylinderGeometry(0.025, 0.025, 0.082, 12));
  for (const cx of camXs) {
    const sleeve = new THREE.Mesh(innerSleeveGeo, mats.metalHardware);
    sleeve.position.set(cx, guideRailY, 0);
    rootGroup.add(sleeve);
  }

  // 2. 3 Guided Vertical Pushrods with Roller Follower Wheels
  const restRollerCenterY = blueprint.shaftY + K.camBaseRadius + K.rollerRadius;
  const restClevisY = blueprint.stageDeckY + K.rodMinDeckY;
  const rodLen = Math.max(0.8, restClevisY - restRollerCenterY);

  const rollerWheelGeo = track(new THREE.CylinderGeometry(K.rollerRadius, K.rollerRadius, 0.038, 16));
  rollerWheelGeo.rotateZ(Math.PI / 2);
  const forkCheekGeo = track(new THREE.BoxGeometry(0.01, 0.044, 0.034));
  const pushrodGeo = track(new THREE.CylinderGeometry(0.015, 0.015, rodLen - 0.03, 10));
  const clevisPinGeo = track(new THREE.CylinderGeometry(0.014, 0.014, 0.05, 10));
  clevisPinGeo.rotateZ(Math.PI / 2);

  const followerGroups: THREE.Group[] = [];
  const rollerMeshes: THREE.Mesh[] = [];
  for (let i = 0; i < K.camCount; i++) {
    const fg = new THREE.Group();
    fg.position.set(camXs[i], restRollerCenterY, 0);

    const roller = new THREE.Mesh(rollerWheelGeo, mats.steelShaft);
    roller.castShadow = true;
    fg.add(roller);
    rollerMeshes.push(roller);

    for (const fx of [-0.024, 0.024]) {
      const cheek = new THREE.Mesh(forkCheekGeo, mats.metalHardware);
      cheek.position.set(fx, 0.018, 0);
      fg.add(cheek);
    }

    const rod = new THREE.Mesh(pushrodGeo, mats.steelShaft);
    rod.position.set(0, 0.03 + (rodLen - 0.03) * 0.5, 0);
    rod.castShadow = true;
    fg.add(rod);

    const clevisPin = new THREE.Mesh(clevisPinGeo, mats.metalHardware);
    clevisPin.position.set(0, rodLen, 0);
    fg.add(clevisPin);

    rootGroup.add(fg);
    followerGroups.push(fg);
  }

  // 3. Stage Deck Assembly (Escutcheon, Open-Frame Torso & Head, 3 Bell-Cranks, 2-Link Writing Arm & Easel)
  const stageGroup = new THREE.Group();
  stageGroup.position.set(0, blueprint.stageDeckY, 0);
  rootGroup.add(stageGroup);

  const escutcheonGeo = track(new THREE.BoxGeometry(0.84, 0.036, 0.24));
  const escutcheon = new THREE.Mesh(escutcheonGeo, mats.metalHardware);
  escutcheon.position.set(0, -0.008, 0);
  escutcheon.castShadow = true;
  escutcheon.userData = { moduleId: mod.id, dragRole: 'module' };
  stageGroup.add(escutcheon);
  hitMeshes.push(escutcheon);

  const deckBushGeo = track(new THREE.CylinderGeometry(0.032, 0.038, 0.12, 14));
  for (const cx of camXs) {
    const bush = new THREE.Mesh(deckBushGeo, mats.metalHardware);
    bush.position.set(cx, -0.032, 0);
    bush.castShadow = true;
    stageGroup.add(bush);
  }

  const automatonScaleGroup = new THREE.Group();
  automatonScaleGroup.visible = mod.topper !== 'none';
  stageGroup.add(automatonScaleGroup);

  // 3a. Open-Front Automaton Torso Frame & Expressive Head (matching PDA-0 hardware silhouette)
  const pillarGeo = track(new THREE.BoxGeometry(0.056, 0.78, 0.09));
  for (const px of [-0.35, 0.35]) {
    const pillar = new THREE.Mesh(pillarGeo, mats.woodSecondary);
    pillar.position.set(px, 0.39, -0.11);
    pillar.castShadow = true;
    pillar.receiveShadow = true;
    pillar.userData = { moduleId: mod.id, dragRole: 'module' };
    automatonScaleGroup.add(pillar);
    hitMeshes.push(pillar);
  }

  const topBeamGeo = track(new THREE.BoxGeometry(0.76, 0.058, 0.11));
  const topBeam = new THREE.Mesh(topBeamGeo, mats.woodPrimary);
  topBeam.position.set(0, 0.78, -0.11);
  topBeam.castShadow = true;
  automatonScaleGroup.add(topBeam);

  const rearSpineGeo = track(new THREE.BoxGeometry(0.74, 0.064, 0.04));
  const rearSpine = new THREE.Mesh(rearSpineGeo, mats.woodDarkTrim);
  rearSpine.position.set(0, 0.44, -0.145);
  rearSpine.castShadow = true;
  automatonScaleGroup.add(rearSpine);

  const liftSpineGeo = track(new THREE.BoxGeometry(0.28, 0.05, 0.04));
  const liftSpine = new THREE.Mesh(liftSpineGeo, mats.woodDarkTrim);
  liftSpine.position.set(-0.22, 0.6, -0.145);
  automatonScaleGroup.add(liftSpine);

  // Expressive Box Head with 2 Dark Oval Eyes gazing down toward the writing sheet
  const neckGeo = track(new THREE.CylinderGeometry(0.055, 0.065, 0.11, 14));
  const neck = new THREE.Mesh(neckGeo, mats.metalHardware);
  neck.position.set(0, 0.84, -0.11);
  automatonScaleGroup.add(neck);

  const headPivot = new THREE.Group();
  headPivot.position.set(0, 0.9, -0.11);
  headPivot.rotation.set(0.14, -0.12, 0);
  automatonScaleGroup.add(headPivot);

  const headBoxGeo = track(new THREE.BoxGeometry(0.56, 0.28, 0.22));
  const headBox = new THREE.Mesh(headBoxGeo, mats.paintedIvory);
  headBox.position.set(0, 0.14, 0);
  headBox.castShadow = true;
  headBox.receiveShadow = true;
  headBox.userData = { moduleId: mod.id, dragRole: 'module' };
  headPivot.add(headBox);
  hitMeshes.push(headBox);

  const headCapGeo = track(new THREE.BoxGeometry(0.58, 0.026, 0.24));
  const headCap = new THREE.Mesh(headCapGeo, mats.woodDarkTrim);
  headCap.position.set(0, 0.285, 0);
  headPivot.add(headCap);

  const eyeGeo = track(new THREE.CylinderGeometry(0.032, 0.032, 0.018, 16));
  eyeGeo.rotateX(Math.PI / 2);
  eyeGeo.scale(0.85, 1.45, 1);
  for (const ex of [-0.12, 0.12]) {
    const eye = new THREE.Mesh(eyeGeo, mats.pegHoleDark);
    eye.position.set(ex, 0.15, 0.112);
    headPivot.add(eye);
  }

  // Brass Axle Pins anchoring Bell-Cranks back to the Rear Spine
  const makeRearPin = (x: number, y: number, zFront: number) => {
    const zBack = -0.145;
    const len = zFront - zBack + 0.03;
    const pinGeo = track(new THREE.CylinderGeometry(0.016, 0.016, len, 10));
    pinGeo.rotateX(Math.PI / 2);
    const pin = new THREE.Mesh(pinGeo, mats.metalHardware);
    pin.position.set(x, y, zBack + len * 0.5 - 0.01);
    automatonScaleGroup.add(pin);
  };
  makeRearPin(K.bell0.pivot.x, K.bell0.pivot.y, K.bell0.pivot.z);
  makeRearPin(K.bell1.pivot.x, K.bell1.pivot.y, K.bell1.pivot.z);
  makeRearPin(K.bell2.pivot.x, K.bell2.pivot.y, K.bell2.pivot.z);
  makeRearPin(K.bell2.hornCenter.x, K.bell2.hornCenter.y, K.bell2.pivot.z);

  // 3b. Bell-Crank Linkages (Steel Bars, Couplers, Tie-Rods)
  const linkUnitGeo = track(new THREE.CylinderGeometry(0.011, 0.011, 1.0, 10));
  const hubGeo = track(new THREE.CylinderGeometry(0.026, 0.026, 0.032, 12));
  hubGeo.rotateX(Math.PI / 2);
  const jointPinGeo = track(new THREE.SphereGeometry(0.016, 10, 10));

  const createDynamicBar = (mat: THREE.Material) => {
    const m = new THREE.Mesh(linkUnitGeo, mat);
    m.castShadow = true;
    automatonScaleGroup.add(m);
    return m;
  };
  const createJointDot = (mat: THREE.Material) => {
    const m = new THREE.Mesh(jointPinGeo, mat);
    automatonScaleGroup.add(m);
    return m;
  };

  for (const pt of [
    K.bell0.pivot,
    K.bell1.pivot,
    K.bell2.pivot,
    { ...K.bell2.hornCenter, z: K.bell2.pivot.z },
  ]) {
    const hub = new THREE.Mesh(hubGeo, mats.metalHardware);
    hub.position.set(pt.x, pt.y, pt.z);
    automatonScaleGroup.add(hub);
  }

  // Bell-Crank 0 (X) + Bell-Crank 1 (Y) + Bell-Crank 2 (Lift)
  const bell0Coupler = createDynamicBar(mats.steelShaft);
  const bell0InArm = createDynamicBar(mats.metalHardware);
  const bell0OutArm = createDynamicBar(mats.metalHardware);
  const bell0TieRod = createDynamicBar(mats.steelShaft);
  const bell0HornArm = createDynamicBar(mats.metalHardware);
  const bell0InDot = createJointDot(mats.metalHardware);
  const bell0OutDot = createJointDot(mats.metalHardware);
  const bell0HornDot = createJointDot(mats.metalHardware);

  const bell1Coupler = createDynamicBar(mats.steelShaft);
  const bell1InArm = createDynamicBar(mats.metalHardware);
  const bell1OutArm = createDynamicBar(mats.metalHardware);
  const bell1TieRod = createDynamicBar(mats.steelShaft);
  const bell1HornArm = createDynamicBar(mats.metalHardware);
  const bell1InDot = createJointDot(mats.metalHardware);
  const bell1OutDot = createJointDot(mats.metalHardware);
  const bell1HornDot = createJointDot(mats.metalHardware);

  const bell2Coupler = createDynamicBar(mats.steelShaft);
  const bell2InArm = createDynamicBar(mats.metalHardware);
  const bell2OutArm = createDynamicBar(mats.metalHardware);
  const bell2TieRod = createDynamicBar(mats.steelShaft);
  const bell2HornArm = createDynamicBar(mats.metalHardware);
  const bell2LiftFork = createDynamicBar(mats.metalHardware);
  const bell2InDot = createJointDot(mats.metalHardware);
  const bell2OutDot = createJointDot(mats.metalHardware);
  const bell2HornDot = createJointDot(mats.metalHardware);

  // 3c. Inclined Shoulder Turret Column, Parallel 2-Link Writing Arm & Sliding Pen Sleeve
  const solZero = solveWritingCam(0, mod, camContours);
  const ct = Math.cos(K.paperTiltRad);
  const st = Math.sin(K.paperTiltRad);
  const normalVec = new THREE.Vector3(0, ct, st);
  const shoulderVec = new THREE.Vector3(
    solZero.shoulderPos.x,
    solZero.shoulderPos.y,
    solZero.shoulderPos.z
  );

  // Sturdy bracket from left pillar to shoulder column base
  const shoulderBracket = createDynamicBar(mats.woodDarkTrim);
  const pPillarAnchor = new THREE.Vector3(-0.35, 0.56, -0.11);
  const pColBase = shoulderVec.clone().addScaledVector(normalVec, -0.14);
  shoulderBracket.scale.set(1.6, pPillarAnchor.distanceTo(pColBase), 1.6);
  alignBarBetweenPoints(shoulderBracket, pPillarAnchor, pColBase, 'y');

  const shoulderColLen = 0.28;
  const shoulderColGeo = track(new THREE.CylinderGeometry(0.028, 0.028, shoulderColLen, 16));
  const shoulderCol = new THREE.Mesh(shoulderColGeo, mats.steelShaft);
  const pColTop = shoulderVec.clone().addScaledVector(normalVec, 0.14);
  alignBarBetweenPoints(shoulderCol, pColBase, pColTop, 'y');
  shoulderCol.castShadow = true;
  automatonScaleGroup.add(shoulderCol);

  // Sliding Brass Lift Collar on Shoulder Column + Parallel Lift Bow Wire
  const liftCollarGeo = track(new THREE.CylinderGeometry(0.042, 0.042, 0.036, 16));
  const liftCollar = new THREE.Mesh(liftCollarGeo, mats.metalHardware);
  liftCollar.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), normalVec);
  automatonScaleGroup.add(liftCollar);

  const armPlaneGroup = new THREE.Group();
  armPlaneGroup.position.copy(shoulderVec);
  armPlaneGroup.rotation.x = K.paperTiltRad;
  automatonScaleGroup.add(armPlaneGroup);

  const createFlatArmGeo = (length: number, width: number, thickness: number) => {
    const r = width * 0.5;
    const shape = new THREE.Shape();
    shape.absarc(0, 0, r, Math.PI * 0.5, Math.PI * 1.5, false);
    shape.lineTo(length, -r);
    shape.absarc(length, 0, r, -Math.PI * 0.5, Math.PI * 0.5, false);
    shape.lineTo(0, r);
    const geo = new THREE.ExtrudeGeometry(shape, {
      depth: thickness,
      bevelEnabled: true,
      bevelThickness: 0.008,
      bevelSize: 0.008,
      bevelSegments: 2,
      curveSegments: 16,
    });
    geo.translate(0, 0, -thickness * 0.5);
    geo.rotateX(Math.PI / 2); // now length along +X, width along Z, thickness along Y (normal +N)
    return track(geo);
  };

  const upperArmPivot = new THREE.Group();
  const upperArmMesh = new THREE.Mesh(
    createFlatArmGeo(K.upperArmLen, 0.072, 0.03),
    mats.woodSecondary
  );
  upperArmMesh.castShadow = true;
  upperArmMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  upperArmPivot.add(upperArmMesh);
  armPlaneGroup.add(upperArmPivot);
  hitMeshes.push(upperArmMesh);

  const forearmPivot = new THREE.Group();
  const forearmMesh = new THREE.Mesh(
    createFlatArmGeo(K.forearmLen, 0.058, 0.026),
    mats.woodSecondary
  );
  forearmMesh.position.set(0, 0.018, 0);
  forearmMesh.castShadow = true;
  forearmMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  forearmPivot.add(forearmMesh);
  armPlaneGroup.add(forearmPivot);
  hitMeshes.push(forearmMesh);

  // Parallel Forearm Control Drum & Parallelogram Link
  const drumLinkShoulder = createDynamicBar(mats.metalHardware);
  const parallelForearmRod = createDynamicBar(mats.steelShaft);
  const drumLinkElbow = createDynamicBar(mats.metalHardware);

  const jointCapGeo = track(new THREE.CylinderGeometry(0.032, 0.032, 0.054, 14));
  const elbowCap = new THREE.Mesh(jointCapGeo, mats.metalHardware);
  elbowCap.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), normalVec);
  automatonScaleGroup.add(elbowCap);

  const wristBushingGeo = track(new THREE.CylinderGeometry(0.03, 0.03, 0.12, 16));
  const wristBushing = new THREE.Mesh(wristBushingGeo, mats.metalHardware);
  wristBushing.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), normalVec);
  wristBushing.castShadow = true;
  automatonScaleGroup.add(wristBushing);

  // Tapered Wooden Stylus / Pen sliding axially inside wrist bushing + overhead lift wire
  const penGroup = new THREE.Group();
  penGroup.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), normalVec);
  automatonScaleGroup.add(penGroup);

  const penNibGeo = track(new THREE.ConeGeometry(0.017, 0.055, 14));
  penNibGeo.rotateX(Math.PI);
  const penNib = new THREE.Mesh(penNibGeo, mats.pegHoleDark);
  penNib.position.set(0, 0.0275, 0);
  penNib.castShadow = true;
  penGroup.add(penNib);

  const penFerruleGeo = track(new THREE.CylinderGeometry(0.022, 0.018, 0.035, 14));
  const penFerrule = new THREE.Mesh(penFerruleGeo, mats.metalHardware);
  penFerrule.position.set(0, 0.072, 0);
  penGroup.add(penFerrule);

  const penShaftGeo = track(new THREE.CylinderGeometry(0.022, 0.022, K.penLength - 0.09, 14));
  const penShaft = new THREE.Mesh(penShaftGeo, mats.paintedAccent);
  penShaft.position.set(0, 0.09 + (K.penLength - 0.09) * 0.5, 0);
  penShaft.castShadow = true;
  penGroup.add(penShaft);

  const penCapGeo = track(new THREE.SphereGeometry(0.024, 12, 10));
  const penCap = new THREE.Mesh(penCapGeo, mats.metalHardware);
  penCap.position.set(0, K.penLength, 0);
  penGroup.add(penCap);

  const liftBowWire1 = createDynamicBar(mats.metalHardware);
  const liftBowWire2 = createDynamicBar(mats.metalHardware);

  // 3d. Inclined Walnut Drafting Easel, Cream Paper Sheet & Live Instanced Ink Trail
  const deskGroup = new THREE.Group();
  deskGroup.position.set(K.paperCenter.x, K.paperCenter.y, K.paperCenter.z);
  deskGroup.rotation.x = K.paperTiltRad;
  automatonScaleGroup.add(deskGroup);

  // Walnut Drafting Board + Wedge Pedestal resting flush on stageDeckY
  const boardW = K.paperWidth + 0.1;
  const boardH = K.paperHeight + 0.08;
  const boardGeo = track(new THREE.BoxGeometry(boardW, 0.036, boardH));
  const boardMesh = new THREE.Mesh(boardGeo, mats.woodDarkTrim);
  boardMesh.position.set(0, -0.022, 0);
  boardMesh.castShadow = true;
  boardMesh.receiveShadow = true;
  boardMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  deskGroup.add(boardMesh);
  hitMeshes.push(boardMesh);

  // Pedestal legs supporting angled board flush onto stageDeckY (world y = 0)
  const easelStandShape = new THREE.Shape();
  const zBack = K.paperCenter.z - (boardH * 0.45) * ct;
  const yBack = K.paperCenter.y + (boardH * 0.45) * st - 0.035;
  const zFront = K.paperCenter.z + (boardH * 0.42) * ct;
  const yFront = Math.max(0.015, K.paperCenter.y - (boardH * 0.42) * st - 0.035);
  easelStandShape.moveTo(zBack, 0);
  easelStandShape.lineTo(zFront, 0);
  easelStandShape.lineTo(zFront, yFront);
  easelStandShape.lineTo(zBack, yBack);
  easelStandShape.closePath();
  const easelStandGeo = track(
    new THREE.ExtrudeGeometry(easelStandShape, { depth: 0.048, bevelEnabled: false })
  );
  easelStandGeo.rotateY(-Math.PI / 2);
  for (const sx of [-0.28, 0.28]) {
    const stand = new THREE.Mesh(easelStandGeo, mats.woodDarkTrim);
    stand.position.set(K.paperCenter.x + sx + 0.024, 0, 0);
    stand.castShadow = true;
    stand.receiveShadow = true;
    automatonScaleGroup.add(stand);
  }

  // Ivory Paper Sheet + Two Miniature Brass Corner Clips
  const paperGeo = track(new THREE.BoxGeometry(K.paperWidth, 0.008, K.paperHeight));
  const paperMesh = new THREE.Mesh(paperGeo, mats.paintedIvory);
  paperMesh.position.set(0, -0.004, 0);
  paperMesh.receiveShadow = true;
  deskGroup.add(paperMesh);

  const clipGeo = track(new THREE.BoxGeometry(0.055, 0.014, 0.028));
  for (const cx of [-K.paperWidth * 0.38, K.paperWidth * 0.38]) {
    const clip = new THREE.Mesh(clipGeo, mats.metalHardware);
    clip.position.set(cx, 0.004, -K.paperHeight * 0.46);
    deskGroup.add(clip);
  }

  // Precompute oriented ink stroke segments along pen-down path for zero-allocation live drawing!
  const inkSampleCount = 320;
  const inkSegments: Array<{ t: number; x: number; z: number; len: number; angle: number }> = [];
  let prevPenDownPt: { u: number; v: number } | null = null;
  for (let i = 0; i < inkSampleCount; i++) {
    const t = i / inkSampleCount;
    const pt = evaluateWritingTrajectory(t, mod.digitPattern, mod.strokeScale);
    if (pt.penLift < 0.14) {
      const pu = (pt.u - 0.5) * K.paperDrawW;
      const pv = (pt.v - 0.5) * K.paperDrawH;
      if (prevPenDownPt) {
        const dx = pu - prevPenDownPt.u;
        const dz = -(pv - prevPenDownPt.v);
        const segLen = Math.hypot(dx, dz);
        if (segLen > 1e-4 && segLen < 0.08) {
          inkSegments.push({
            t,
            x: 0.5 * (pu + prevPenDownPt.u),
            z: 0.5 * (-pv - prevPenDownPt.v),
            len: segLen + 0.014,
            angle: Math.atan2(dx, dz),
          });
        }
      }
      prevPenDownPt = { u: pu, v: pv };
    } else {
      prevPenDownPt = null;
    }
  }

  const inkSegGeo = track(new THREE.BoxGeometry(0.017, 0.003, 1.0));
  const maxInkCount = Math.max(1, inkSegments.length);
  const inkInstanced = new THREE.InstancedMesh(inkSegGeo, mats.writingInk, maxInkCount);
  inkInstanced.instanceMatrix.setUsage(THREE.StaticDrawUsage);
  const dummy = new THREE.Object3D();
  for (let i = 0; i < inkSegments.length; i++) {
    const seg = inkSegments[i];
    dummy.position.set(seg.x, 0.0022, seg.z);
    dummy.rotation.set(0, seg.angle, 0);
    dummy.scale.set(1, 1, seg.len);
    dummy.updateMatrix();
    inkInstanced.setMatrixAt(i, dummy.matrix);
  }
  inkInstanced.instanceMatrix.needsUpdate = true;
  inkInstanced.count = 0;
  deskGroup.add(inkInstanced);

  const hitGeo = track(new THREE.BoxGeometry(1.5, blueprint.stageDeckY + 1.4, 1.8));
  const hitMat = new THREE.MeshBasicMaterial({ visible: false });
  const hitMesh = new THREE.Mesh(hitGeo, hitMat);
  hitMesh.position.set(-0.2, (blueprint.stageDeckY + 1.4) * 0.5, 0.2);
  hitMesh.userData = { moduleId: mod.id, dragRole: 'module' };
  rootGroup.add(hitMesh);
  hitMeshes.unshift(hitMesh);

  // Scratch vectors to avoid per-frame allocations
  const vA = new THREE.Vector3();
  const vB = new THREE.Vector3();
  const vC = new THREE.Vector3();
  const vD = new THREE.Vector3();

  const placeBar = (
    bar: THREE.Mesh,
    a: { x: number; y: number; z: number },
    b: { x: number; y: number; z: number },
    radiusScale = 1
  ) => {
    vA.set(a.x, a.y, a.z);
    vB.set(b.x, b.y, b.z);
    const len = Math.max(1e-4, vA.distanceTo(vB));
    bar.scale.set(radiusScale, len, radiusScale);
    alignBarBetweenPoints(bar, vA, vB, 'y');
  };

  return {
    id: mod.id,
    type: mod.type,
    rootGroup,
    hitMeshes,
    pivotHitMeshes: [],
    highlightRing,
    update: (shaftAngleRad, bp, explodeAmount) => {
      const sol = solveWritingCam(shaftAngleRad, mod, camContours);

      camBankPivot.rotation.x = -sol.driverAngleRad;
      stageGroup.position.y = bp.stageDeckY + explodeAmount * 1.15;

      for (let i = 0; i < K.camCount; i++) {
        followerGroups[i].position.y = bp.shaftY + sol.camRollerCenterY[i];
        rollerMeshes[i].rotation.x = sol.driverAngleRad * 4;
      }

      // Bell-Crank 0 (X)
      placeBar(bell0Coupler, sol.bellCrankX.rodClevis, sol.bellCrankX.inPin);
      placeBar(bell0InArm, sol.bellCrankX.pivot, sol.bellCrankX.inPin, 1.15);
      placeBar(bell0OutArm, sol.bellCrankX.pivot, sol.bellCrankX.outPin, 1.15);
      placeBar(bell0TieRod, sol.bellCrankX.outPin, sol.bellCrankX.hornPin);
      placeBar(bell0HornArm, sol.bellCrankX.hornCenter, sol.bellCrankX.hornPin, 1.2);
      bell0InDot.position.set(sol.bellCrankX.inPin.x, sol.bellCrankX.inPin.y, sol.bellCrankX.inPin.z);
      bell0OutDot.position.set(sol.bellCrankX.outPin.x, sol.bellCrankX.outPin.y, sol.bellCrankX.outPin.z);
      bell0HornDot.position.set(sol.bellCrankX.hornPin.x, sol.bellCrankX.hornPin.y, sol.bellCrankX.hornPin.z);

      // Bell-Crank 1 (Y)
      placeBar(bell1Coupler, sol.bellCrankY.rodClevis, sol.bellCrankY.inPin);
      placeBar(bell1InArm, sol.bellCrankY.pivot, sol.bellCrankY.inPin, 1.15);
      placeBar(bell1OutArm, sol.bellCrankY.pivot, sol.bellCrankY.outPin, 1.15);
      placeBar(bell1TieRod, sol.bellCrankY.outPin, sol.bellCrankY.hornPin);
      placeBar(bell1HornArm, sol.bellCrankY.hornCenter, sol.bellCrankY.hornPin, 1.2);
      bell1InDot.position.set(sol.bellCrankY.inPin.x, sol.bellCrankY.inPin.y, sol.bellCrankY.inPin.z);
      bell1OutDot.position.set(sol.bellCrankY.outPin.x, sol.bellCrankY.outPin.y, sol.bellCrankY.outPin.z);
      bell1HornDot.position.set(sol.bellCrankY.hornPin.x, sol.bellCrankY.hornPin.y, sol.bellCrankY.hornPin.z);

      // Bell-Crank 2 (Pen Lift)
      placeBar(bell2Coupler, sol.bellCrankLift.rodClevis, sol.bellCrankLift.inPin);
      placeBar(bell2InArm, sol.bellCrankLift.pivot, sol.bellCrankLift.inPin, 1.15);
      placeBar(bell2OutArm, sol.bellCrankLift.pivot, sol.bellCrankLift.outPin, 1.15);
      placeBar(bell2TieRod, sol.bellCrankLift.outPin, sol.bellCrankLift.hornPin);
      placeBar(bell2HornArm, sol.bellCrankLift.hornCenter, sol.bellCrankLift.hornPin, 1.2);
      bell2InDot.position.set(sol.bellCrankLift.inPin.x, sol.bellCrankLift.inPin.y, sol.bellCrankLift.inPin.z);
      bell2OutDot.position.set(sol.bellCrankLift.outPin.x, sol.bellCrankLift.outPin.y, sol.bellCrankLift.outPin.z);
      bell2HornDot.position.set(sol.bellCrankLift.hornPin.x, sol.bellCrankLift.hornPin.y, sol.bellCrankLift.hornPin.z);

      // Shoulder Lift Collar + Fork + Overhead Lift Wire to Pen Top
      vC.copy(shoulderVec).addScaledVector(normalVec, 0.045 + sol.liftCollarOffset);
      liftCollar.position.copy(vC);
      placeBar(bell2LiftFork, sol.bellCrankLift.hornPin, { x: vC.x, y: vC.y, z: vC.z });

      // 2-Link Upper Arm + Forearm + Parallelogram Link in tilted UV plane
      upperArmPivot.rotation.y = sol.shoulderAngleRad;
      forearmPivot.position.set(
        K.upperArmLen * Math.cos(sol.shoulderAngleRad),
        0,
        -K.upperArmLen * Math.sin(sol.shoulderAngleRad)
      );
      forearmPivot.rotation.y = sol.forearmAngleRad;

      vB.set(sol.elbowPos.x, sol.elbowPos.y, sol.elbowPos.z);
      vD.set(sol.wristPos.x, sol.wristPos.y, sol.wristPos.z);
      elbowCap.position.copy(vB);
      wristBushing.position.copy(vD);

      placeBar(drumLinkShoulder, sol.shoulderPos, sol.shoulderDrumPin, 0.85);
      placeBar(parallelForearmRod, sol.shoulderDrumPin, sol.elbowHornPin, 0.8);
      placeBar(drumLinkElbow, sol.elbowPos, sol.elbowHornPin, 0.85);

      // Sliding Pen & Overhead Lift Bow Wire
      penGroup.position.set(sol.penTipPos.x, sol.penTipPos.y, sol.penTipPos.z);
      const vElbowLift = vB.clone().addScaledVector(normalVec, 0.08 + sol.liftCollarOffset);
      placeBar(liftBowWire1, { x: vC.x, y: vC.y, z: vC.z }, { x: vElbowLift.x, y: vElbowLift.y, z: vElbowLift.z }, 0.65);
      placeBar(liftBowWire2, { x: vElbowLift.x, y: vElbowLift.y, z: vElbowLift.z }, sol.penTopPos, 0.65);

      // Update live ink count on the paper pad up to current cycle progress
      const prog = sol.cycleProgress01;
      let activeCount = 0;
      for (let i = 0; i < inkSegments.length; i++) {
        if (inkSegments[i].t <= prog) activeCount = i + 1;
        else break;
      }
      inkInstanced.count = activeCount;
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


