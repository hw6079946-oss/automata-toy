import type {
  AutomataModule,
  BevelGearModule,
  BevelGearSolution,
  CamFollowerModule,
  CamFollowerSolution,
  CamProfileType,
  CircularCamModule,
  CircularCamShape,
  CircularCamSolution,
  DualEccentricModule,
  DualEccentricSolution,
  FourBarLinkageModule,
  FourBarSolution,
  PinLanternModule,
  PinLanternSolution,
  RatchetModule,
  RatchetSolution,
  SemiCircularCamModule,
  SemiCircularCamSolution,
} from './types';

export const SHAFT_MIN_X = -2.55;
export const SHAFT_MAX_X = 2.55;
export const MIN_MODULE_SPACING = 1.18;
export const CAM_BEVEL_RADIAL = 0.012;

export function degToRad(deg: number): number {
  return (deg * Math.PI) / 180;
}

export function normalizeAngleRad(rad: number): number {
  const twoPi = Math.PI * 2;
  return ((rad % twoPi) + twoPi) % twoPi;
}

/**
 * Evaluates the polar radius R(alpha) of a classic wooden automaton cam
 * in its local unrotated frame where alpha = 0 points along +Z and alpha = PI/2 points along +Y.
 */
export function evaluateCamRadius(
  camType: CamProfileType,
  alpha: number,
  baseRadius: number,
  lift: number
): number {
  const a = normalizeAngleRad(alpha);

  switch (camType) {
    case 'eccentric': {
      const e = lift * 0.5;
      const r = baseRadius + e;
      const cosA = Math.cos(a);
      const disc = Math.max(0.0001, r * r - e * e * cosA * cosA);
      return e * Math.sin(a) + Math.sqrt(disc);
    }
    case 'pear': {
      const lobe = Math.sin(a) > 0 ? Math.pow(Math.sin(a), 1.65) : 0;
      return baseRadius + lift * lobe;
    }
    case 'snail': {
      const t = a / (Math.PI * 2);
      if (t <= 0.94) {
        return baseRadius + lift * (t / 0.94);
      }
      const dropT = (t - 0.94) / 0.06;
      return baseRadius + lift * (1 - dropT);
    }
    case 'heart': {
      const lobe = 0.5 * (1 - Math.cos(2 * a));
      return baseRadius + lift * lobe;
    }
  }
}

/**
 * Generates 2D contour points (z, y) for building ExtrudeGeometry or SVG diagrams.
 */
export function sampleCamContour(
  camType: CamProfileType,
  baseRadius: number,
  lift: number,
  segments = 144
): Array<{ x: number; y: number; radius: number }> {
  const pts: Array<{ x: number; y: number; radius: number }> = [];
  for (let i = 0; i < segments; i++) {
    const alpha = (i / segments) * Math.PI * 2;
    const radius = evaluateCamRadius(camType, alpha, baseRadius, lift);
    pts.push({
      x: Math.cos(alpha) * radius,
      y: Math.sin(alpha) * radius,
      radius,
    });
  }
  return pts;
}

/**
 * Solves exact flat-faced follower height riding tangentially on a rotated, beveled cam contour.
 * Accounts for CAM_BEVEL_RADIAL so the follower foot never penetrates the physical 3D ExtrudeGeometry.
 */
export function solveCamFollower(
  camType: CamProfileType,
  shaftAngleRad: number,
  phaseDeg: number,
  baseRadius: number,
  lift: number,
  footRadius = 0.34
): CamFollowerSolution {
  const theta = shaftAngleRad + degToRad(phaseDeg);
  const samples = 144;
  let maxY = baseRadius * 0.85;
  let contactZ = 0;

  for (let i = 0; i < samples; i++) {
    const alpha = (i / samples) * Math.PI * 2;
    const r = evaluateCamRadius(camType, alpha, baseRadius, lift) + CAM_BEVEL_RADIAL;
    const worldAlpha = alpha + theta;
    const pz = r * Math.cos(worldAlpha);
    const py = r * Math.sin(worldAlpha);

    if (Math.abs(pz) <= footRadius && py > maxY) {
      maxY = py;
      contactZ = pz;
    }
  }

  const minExpected = baseRadius + CAM_BEVEL_RADIAL;
  const maxExpected = baseRadius + lift + CAM_BEVEL_RADIAL;
  const span = Math.max(0.05, maxExpected - minExpected);
  const liftDisplacement = Math.max(0, maxY - minExpected);
  const normalizedLift = Math.min(1, Math.max(0, liftDisplacement / span));

  return {
    followerBottomY: maxY,
    liftDisplacement,
    normalizedLift,
    contactOffsetZ: contactZ,
  };
}

/**
 * Solves 4-bar crank-rocker linkage in the local YZ plane (X is shaft axis).
 * Shaft center is (0, 0). Fixed rocker fulcrum is (pivotZ, pivotY).
 * Uses circle-circle intersection between crank pin (P1, couplerLength) and fulcrum (P3, rockerLength).
 */
export function solveFourBarLinkage(
  shaftAngleRad: number,
  module: Pick<
    FourBarLinkageModule,
    'phaseDeg' | 'crankRadius' | 'couplerLength' | 'rockerLength' | 'pivotZ' | 'pivotY' | 'outputArmRatio'
  >
): FourBarSolution {
  const crankAngleRad = shaftAngleRad + degToRad(module.phaseDeg);
  const p1z = module.crankRadius * Math.cos(crankAngleRad);
  const p1y = module.crankRadius * Math.sin(crankAngleRad);

  const p3z = module.pivotZ;
  const p3y = module.pivotY;

  const dz = p3z - p1z;
  const dy = p3y - p1y;
  const dRaw = Math.hypot(dz, dy);
  const d = Math.max(0.001, dRaw);

  const r1 = module.couplerLength;
  const r2 = module.rockerLength;

  const minReach = Math.abs(r1 - r2) + 0.02;
  const maxReach = r1 + r2 - 0.02;
  const isOverStretched = dRaw > maxReach || dRaw < minReach;

  const clampedD = Math.min(maxReach, Math.max(minReach, d));

  const a = (r1 * r1 - r2 * r2 + clampedD * clampedD) / (2 * clampedD);
  const hSq = Math.max(0, r1 * r1 - a * a);
  const h = Math.sqrt(hSq);

  const uz = dz / d;
  const uy = dy / d;

  const mz = p1z + a * uz;
  const my = p1y + a * uy;

  const branchSign = module.pivotZ >= 0 ? 1 : -1;
  const p2z = mz - branchSign * h * uy;
  const p2y = my + branchSign * h * uz;

  const couplerAngleRad = Math.atan2(p2y - p1y, p2z - p1z);
  const rockerAngleRad = Math.atan2(p2y - p3y, p2z - p3z);

  const refAngle = module.pivotZ >= 0 ? Math.PI : 0;
  let deltaRocker = rockerAngleRad - refAngle;
  while (deltaRocker > Math.PI) deltaRocker -= Math.PI * 2;
  while (deltaRocker < -Math.PI) deltaRocker += Math.PI * 2;

  const maxSwingAngle = Math.asin(Math.min(0.95, module.crankRadius / Math.max(0.3, module.rockerLength)));
  const normalizedSwing = Math.max(-1, Math.min(1, deltaRocker / Math.max(0.15, maxSwingAngle)));

  const outputArmLen = module.rockerLength * (module.outputArmRatio || 0.75);
  const pushrodLift = Math.sin(rockerAngleRad) * outputArmLen * 0.65;

  return {
    crankPin: { z: p1z, y: p1y },
    couplerJoint: { z: p2z, y: p2y },
    fulcrum: { z: p3z, y: p3y },
    crankAngleRad,
    couplerAngleRad,
    rockerAngleRad,
    pushrodLift,
    normalizedSwing,
    isOverStretched,
  };
}

export interface FourBarPushrodConfig {
  clevisArmDist: number;
  outputStageZ: number;
  linkLength: number;
  minRodBottomY: number;
  maxRodBottomY: number;
}

export interface FourBarPushrodSolution {
  clevisZ: number;
  clevisY: number;
  pushrodBottomY: number;
  linkAngleRad: number;
  normalizedDrive: number;
  liftAboveMin: number;
}

/**
 * Computes the fixed stage-collar Z coordinate (midpoint of clevis horizontal travel)
 * and rigid connecting-link length so the guided vertical pushrod stays coaxial inside
 * the stage deck collar without ever clipping sideways through the wood deck.
 */
export function computeFourBarPushrodConfig(
  module: Pick<
    FourBarLinkageModule,
    'phaseDeg' | 'crankRadius' | 'couplerLength' | 'rockerLength' | 'pivotZ' | 'pivotY' | 'outputArmRatio'
  >
): FourBarPushrodConfig {
  const clevisArmDist = Math.min(0.54, module.rockerLength * 0.52);
  let minZ = Infinity;
  let maxZ = -Infinity;
  const samples = 36;
  const clevisPoints: Array<{ z: number; y: number }> = [];

  for (let i = 0; i < samples; i++) {
    const theta = (i / samples) * Math.PI * 2;
    const sol = solveFourBarLinkage(theta, module);
    const cz = sol.fulcrum.z + clevisArmDist * Math.cos(sol.rockerAngleRad);
    const cy = sol.fulcrum.y + clevisArmDist * Math.sin(sol.rockerAngleRad);
    clevisPoints.push({ z: cz, y: cy });
    if (cz < minZ) minZ = cz;
    if (cz > maxZ) maxZ = cz;
  }

  const outputStageZ = Math.max(-1.05, Math.min(1.05, (minZ + maxZ) * 0.5));
  const maxDz = clevisPoints.reduce((acc, p) => Math.max(acc, Math.abs(outputStageZ - p.z)), 0);
  const linkLength = Math.max(0.42, maxDz + 0.24);

  let minRodBottomY = Infinity;
  let maxRodBottomY = -Infinity;
  for (const p of clevisPoints) {
    const dz = outputStageZ - p.z;
    const dy = Math.sqrt(Math.max(0.01, linkLength * linkLength - dz * dz));
    const rby = p.y + dy;
    if (rby < minRodBottomY) minRodBottomY = rby;
    if (rby > maxRodBottomY) maxRodBottomY = rby;
  }

  return {
    clevisArmDist,
    outputStageZ,
    linkLength,
    minRodBottomY,
    maxRodBottomY,
  };
}

/**
 * Solves the exact position of the guided vertical pushrod and pitman connecting link
 * connecting the arcing rocker clevis to the vertical stage collar at outputStageZ.
 */
export function solveFourBarPushrod(
  shaftAngleRad: number,
  module: Pick<
    FourBarLinkageModule,
    'phaseDeg' | 'crankRadius' | 'couplerLength' | 'rockerLength' | 'pivotZ' | 'pivotY' | 'outputArmRatio'
  >,
  cfg: FourBarPushrodConfig,
  sol?: FourBarSolution
): FourBarPushrodSolution {
  const s = sol ?? solveFourBarLinkage(shaftAngleRad, module);
  const clevisZ = s.fulcrum.z + cfg.clevisArmDist * Math.cos(s.rockerAngleRad);
  const clevisY = s.fulcrum.y + cfg.clevisArmDist * Math.sin(s.rockerAngleRad);
  const dz = cfg.outputStageZ - clevisZ;
  const dy = Math.sqrt(Math.max(0.01, cfg.linkLength * cfg.linkLength - dz * dz));
  const pushrodBottomY = clevisY + dy;
  const linkAngleRad = Math.atan2(dy, dz);
  const span = Math.max(0.05, cfg.maxRodBottomY - cfg.minRodBottomY);
  const liftAboveMin = Math.max(0, pushrodBottomY - cfg.minRodBottomY);
  const normalizedDrive = Math.min(1, Math.max(0, liftAboveMin / span));

  return {
    clevisZ,
    clevisY,
    pushrodBottomY,
    linkAngleRad,
    normalizedDrive,
    liftAboveMin,
  };
}

export interface BevelGearGeometrySpec {
  driverPitchR: number;
  drivenPitchR: number;
  driverTeeth: number;
  drivenTeeth: number;
  effectiveRatio: number;
  moduleSize: number;
  drivenX: number;
  driverX: number;
  drivenYOffset: number;
  coneAngleDriver: number;
  coneAngleDriven: number;
  toothPhaseOffsetRad: number;
}

/**
 * Solves non-interpenetrating 90-degree conical bevel gear pitch cones whose axes intersect
 * at (drivenX, shaftY, 0), guaranteeing identical tooth circular pitch and tangential meshing.
 */
export function solveBevelGearGeometry(
  gearRatio: number,
  teethCount: number
): BevelGearGeometrySpec {
  const driverTeeth = Math.max(10, Math.round(teethCount));
  const drivenTeeth = Math.max(8, Math.round(driverTeeth / Math.max(0.45, gearRatio)));
  const effectiveRatio = driverTeeth / drivenTeeth;

  // Shared pitch radius per tooth so both gears have identical tooth spacing & depth
  const unitR = Math.min(
    0.58 / driverTeeth,
    0.52 / drivenTeeth,
    0.86 / (driverTeeth + drivenTeeth)
  );
  const driverPitchR = driverTeeth * unitR;
  const drivenPitchR = drivenTeeth * unitR;
  const moduleSize = 2 * unitR;

  const drivenX = 0.22;
  // Pitch cone apex lies at (drivenX, shaftY, 0)
  const driverX = drivenX - drivenPitchR;
  const drivenYOffset = driverPitchR;

  const coneAngleDriver = Math.atan2(driverPitchR, drivenPitchR);
  const coneAngleDriven = Math.atan2(drivenPitchR, driverPitchR);
  const toothPhaseOffsetRad = Math.PI / drivenTeeth;

  return {
    driverPitchR,
    drivenPitchR,
    driverTeeth,
    drivenTeeth,
    effectiveRatio,
    moduleSize,
    drivenX,
    driverX,
    drivenYOffset,
    coneAngleDriver,
    coneAngleDriven,
    toothPhaseOffsetRad,
  };
}

export function solveBevelGear(
  shaftAngleRad: number,
  module: Pick<BevelGearModule, 'phaseDeg' | 'gearRatio' | 'teethCount'>
): BevelGearSolution {
  const spec = solveBevelGearGeometry(module.gearRatio, module.teethCount);
  const driverAngleRad = shaftAngleRad + degToRad(module.phaseDeg);
  const drivenAngleRad = driverAngleRad * spec.effectiveRatio;
  return {
    driverAngleRad,
    drivenAngleRad,
  };
}

export interface PinLanternGeometrySpec {
  driverPitchR: number;
  lanternPitchR: number;
  driverPegs: number;
  lanternStaves: number;
  effectiveRatio: number;
  unitR: number;
  pegRadius: number;
  staveRadius: number;
  pegLength: number;
  drivenX: number;
  driverDiscFaceX: number;
  pegTipX: number;
  lanternYOffset: number;
  staveTopY: number;
  staveBottomY: number;
  lanternPhaseOffsetRad: number;
}

/**
 * Solves a 90-degree Face Pin-Wheel & Open-Bottom Cantilevered Lantern Pinion.
 * An open-bottom cantilevered top-crown lantern pinion avoids bottom-flange interference
 * across every speed ratio (0.5x..2.5x) while maintaining equal circular pitch at the mesh point.
 */
export function solvePinLanternGeometry(
  gearRatio: number,
  pegCount: number
): PinLanternGeometrySpec {
  const driverPegs = Math.max(8, Math.round(pegCount));
  const lanternStaves = Math.max(6, Math.round(driverPegs / Math.max(0.45, gearRatio)));
  const effectiveRatio = driverPegs / lanternStaves;

  const unitR = Math.min(
    0.52 / driverPegs,
    0.44 / lanternStaves,
    0.78 / (driverPegs + lanternStaves)
  );
  const driverPitchR = driverPegs * unitR;
  const lanternPitchR = lanternStaves * unitR;

  const pegRadius = 0.55 * unitR;
  const staveRadius = 0.55 * unitR;
  const pegLength = 3.8 * unitR;

  const drivenX = 0.24;
  const driverDiscFaceX = drivenX - lanternPitchR - 2.3 * unitR;
  const pegTipX = driverDiscFaceX + pegLength;
  const lanternYOffset = driverPitchR;
  const staveTopY = 1.8 * unitR;
  const staveBottomY = -1.6 * unitR;
  const lanternPhaseOffsetRad = Math.PI / lanternStaves;

  return {
    driverPitchR,
    lanternPitchR,
    driverPegs,
    lanternStaves,
    effectiveRatio,
    unitR,
    pegRadius,
    staveRadius,
    pegLength,
    drivenX,
    driverDiscFaceX,
    pegTipX,
    lanternYOffset,
    staveTopY,
    staveBottomY,
    lanternPhaseOffsetRad,
  };
}

export function solvePinLantern(
  shaftAngleRad: number,
  module: Pick<PinLanternModule, 'phaseDeg' | 'gearRatio' | 'pegCount'>
): PinLanternSolution {
  const spec = solvePinLanternGeometry(module.gearRatio, module.pegCount);
  const driverAngleRad = shaftAngleRad + degToRad(module.phaseDeg);
  const drivenAngleRad = driverAngleRad * spec.effectiveRatio;
  return {
    driverAngleRad,
    drivenAngleRad,
  };
}

export const CIRCULAR_CAM_MAX_OFFSET = 0.38;

export interface CircularCamGeometrySpec {
  camShape: CircularCamShape;
  driverRadius: number;
  inradius: number;
  contactRadius: number;
  drivenDiscRadius: number;
  camOffsetRatio: number;
  maxOffset: number;
  effectiveRatio: number;
  driverX: number;
  drivenX: number;
  drivenYOffset: number;
  driverThickness: number;
  drivenDiscThickness: number;
}

export function clampCircularCamOffset(rawOffset: number, snapThreshold = 0): number {
  const clamped = Math.max(-1, Math.min(1, Number.isFinite(rawOffset) ? rawOffset : -1));
  if (snapThreshold > 0) {
    for (const target of [-1, 0, 1]) {
      if (Math.abs(clamped - target) <= snapThreshold) {
        return target;
      }
    }
  }
  return Math.round(clamped * 100) / 100;
}

/**
 * Solves a 90-degree Orthogonal Cam Rotary Friction Drive (Circle / Equilateral Triangle / Square)
 * with adjustable wheel X offset under the horizontal turntable tray:
 * - Circle: constant tray height R, smooth continuous rolling contact.
 * - Triangle (3-sided) / Square (4-sided): tray rides flush atop the rotating polygon at
 *   trayBottomY = max_k(R * sin(alpha_k + theta)), bobbing between inradius R*cos(PI/N) and R
 *   while modulating rotary speed synchronously with instantaneous contact radius.
 */
export function solveCircularCamGeometry(
  gearRatio: number,
  driverRadius: number,
  camOffsetRatio = -1,
  camShape: CircularCamShape = 'circle'
): CircularCamGeometrySpec {
  const clampedRatio = Math.max(0.5, Math.min(3.0, gearRatio));
  const effectiveDriverR = Math.max(0.28, Math.min(0.48, driverRadius));
  const clampedOffset = Math.max(-1, Math.min(1, Number.isFinite(camOffsetRatio) ? camOffsetRatio : -1));
  const shape: CircularCamShape =
    camShape === 'triangle' || camShape === 'square' ? camShape : 'circle';
  const sides = shape === 'triangle' ? 3 : shape === 'square' ? 4 : 0;
  const inradius =
    sides > 0 ? effectiveDriverR * Math.cos(Math.PI / sides) : effectiveDriverR;

  const drivenDiscRadius = 0.48;
  const maxOffset = CIRCULAR_CAM_MAX_OFFSET;
  const drivenX = 0;
  const driverX = drivenX + clampedOffset * maxOffset;
  const contactRadius = Math.abs(driverX - drivenX);
  const effectiveRatio = Math.abs(clampedOffset) < 1e-4 ? 0 : -clampedOffset * clampedRatio;
  const drivenYOffset = effectiveDriverR;

  return {
    camShape: shape,
    driverRadius: effectiveDriverR,
    inradius,
    contactRadius,
    drivenDiscRadius,
    camOffsetRatio: clampedOffset,
    maxOffset,
    effectiveRatio,
    driverX,
    drivenX,
    drivenYOffset,
    driverThickness: 0.086,
    drivenDiscThickness: 0.072,
  };
}

export function solveCircularCam(
  shaftAngleRad: number,
  module: Pick<CircularCamModule, 'phaseDeg' | 'gearRatio' | 'driverRadius'> & {
    camOffsetRatio?: number;
    camShape?: CircularCamShape;
  }
): CircularCamSolution {
  const spec = solveCircularCamGeometry(
    module.gearRatio,
    module.driverRadius,
    module.camOffsetRatio ?? -1,
    module.camShape ?? 'circle'
  );
  const driverAngleRad = shaftAngleRad + degToRad(module.phaseDeg);

  if (spec.camShape === 'circle') {
    return {
      driverAngleRad,
      drivenAngleRad: driverAngleRad * spec.effectiveRatio,
      trayBottomY: spec.driverRadius,
      liftDisplacement: 0,
    };
  }

  const sides = spec.camShape === 'triangle' ? 3 : 4;
  const halfSector = Math.PI / sides;
  const sector = 2 * halfSector;
  const sectorIdx = Math.floor(driverAngleRad / sector);
  const s = driverAngleRad - sectorIdx * sector;

  const trayBottomY =
    s <= halfSector
      ? spec.driverRadius * Math.cos(s)
      : spec.driverRadius * Math.cos(sector - s);
  const liftDisplacement = Math.max(0, trayBottomY - spec.inradius);

  const sectorIntegral = 2 * Math.sin(halfSector);
  const inSectorIntegral =
    s <= halfSector
      ? Math.sin(s)
      : sectorIntegral - Math.sin(sector - s);
  const phi = sectorIdx * sectorIntegral + inSectorIntegral;

  return {
    driverAngleRad,
    drivenAngleRad: phi * spec.effectiveRatio,
    trayBottomY,
    liftDisplacement,
  };
}

export interface SemiCircularCamGeometrySpec {
  driverRadius: number;
  hubRadius: number;
  contactRadius: number;
  drivenDiscRadius: number;
  camOffsetRatio: number;
  maxOffset: number;
  effectiveRatio: number;
  driverX: number;
  drivenX: number;
  drivenYOffset: number;
  driverThickness: number;
  drivenDiscThickness: number;
  stepPerRevRad: number;
}

/**
 * Solves a 90-degree Orthogonal Semicircular Cam Intermittent Rotary Drive:
 * - Vertical half-moon cam (180-deg active circular arc of radius r + recessed shaft hub hubRadius)
 *   sits on the horizontal camshaft beneath the flat horizontal turntable tray at drivenYOffset = r.
 * - During the active half revolution (phase in [0, PI]), the circular rim contacts the tray
 *   tangentially (gap === 0) and turns the vertical spindle & arrow topper continuously.
 * - During the relief half revolution (phase in (PI, 2*PI)), the cutaway face clears the tray
 *   (gap > 0) so the turntable & arrow pause cleanly (dwell) with zero penetration.
 */
export function solveSemiCircularCamGeometry(
  gearRatio: number,
  driverRadius: number,
  camOffsetRatio = -1
): SemiCircularCamGeometrySpec {
  const clampedRatio = Math.max(0.5, Math.min(2.5, gearRatio));
  const effectiveDriverR = Math.max(0.28, Math.min(0.46, driverRadius));
  const clampedOffset = Math.max(-1, Math.min(1, Number.isFinite(camOffsetRatio) ? camOffsetRatio : -1));
  const hubRadius = 0.145;
  const drivenDiscRadius = 0.48;
  const maxOffset = CIRCULAR_CAM_MAX_OFFSET;
  const drivenX = 0;
  const driverX = drivenX + clampedOffset * maxOffset;
  const contactRadius = Math.abs(driverX - drivenX);
  const effectiveRatio = Math.abs(clampedOffset) < 1e-4 ? 0 : -clampedOffset * clampedRatio;

  return {
    driverRadius: effectiveDriverR,
    hubRadius,
    contactRadius,
    drivenDiscRadius,
    camOffsetRatio: clampedOffset,
    maxOffset,
    effectiveRatio,
    driverX,
    drivenX,
    drivenYOffset: effectiveDriverR,
    driverThickness: 0.088,
    drivenDiscThickness: 0.072,
    stepPerRevRad: Math.PI * effectiveRatio,
  };
}

export function solveSemiCircularCam(
  shaftAngleRad: number,
  module: Pick<SemiCircularCamModule, 'phaseDeg' | 'gearRatio' | 'driverRadius'> & {
    camOffsetRatio?: number;
  }
): SemiCircularCamSolution {
  const spec = solveSemiCircularCamGeometry(
    module.gearRatio,
    module.driverRadius,
    module.camOffsetRatio ?? -1
  );
  const theta = shaftAngleRad + degToRad(module.phaseDeg);
  const rev = Math.floor(theta / (Math.PI * 2));
  const phase = normalizeAngleRad(theta);
  const isEngaged = phase <= Math.PI;
  const activeAdvance = isEngaged ? phase : Math.PI;
  const drivenAngleRad = (rev * Math.PI + activeAdvance) * spec.effectiveRatio;

  const topY = isEngaged
    ? spec.driverRadius
    : Math.max(spec.hubRadius, spec.driverRadius * Math.abs(Math.cos(phase)));
  const gap = Math.max(0, spec.driverRadius - topY);

  return {
    driverAngleRad: theta,
    drivenAngleRad,
    isEngaged,
    gap,
  };
}

export interface DualEccentricGeometrySpec {
  camRadius: number;
  eccentricity: number;
  gearRatio: number;
  camHalfSpacing: number;
  leftCamX: number;
  rightCamX: number;
  camThickness: number;
  drivenX: number;
  drivenDiscRadius: number;
  drivenDiscThickness: number;
  baseDrivenYOffset: number;
  maxDrivenYOffset: number;
  swingAmplitudeRad: number;
}

/**
 * Solves a Twin Eccentric Alternating Cam Drive under a single horizontal turntable tray:
 * - Left eccentric cam (x = -camHalfSpacing) and Right eccentric cam (x = +camHalfSpacing)
 *   are keyed 180 degrees out of phase on the horizontal camshaft.
 * - The shared horizontal tray rests tangentially on the higher cam at trayBottomY = max(leftTop, rightTop),
 *   so Left Cam lifts & drives forward during sin(theta) > 0 while Right Cam drops clear,
 *   then Right Cam lifts & drives in reverse during sin(theta) < 0 while Left Cam drops clear.
 */
export function solveDualEccentricGeometry(
  camRadius: number,
  eccentricity: number,
  gearRatio: number
): DualEccentricGeometrySpec {
  const r = Math.max(0.28, Math.min(0.4, camRadius));
  const e = Math.max(0.04, Math.min(0.12, eccentricity));
  const clampedRatio = Math.max(0.5, Math.min(2.5, gearRatio));
  const camHalfSpacing = 0.25;
  const camThickness = 0.078;
  const drivenDiscRadius = 0.48;
  const drivenDiscThickness = 0.072;
  const swingAmplitudeRad = clampedRatio * Math.PI * 0.9;

  return {
    camRadius: r,
    eccentricity: e,
    gearRatio: clampedRatio,
    camHalfSpacing,
    leftCamX: -camHalfSpacing,
    rightCamX: camHalfSpacing,
    camThickness,
    drivenX: 0,
    drivenDiscRadius,
    drivenDiscThickness,
    baseDrivenYOffset: r,
    maxDrivenYOffset: r + e,
    swingAmplitudeRad,
  };
}

export function solveDualEccentric(
  shaftAngleRad: number,
  module: Pick<DualEccentricModule, 'phaseDeg' | 'camRadius' | 'eccentricity' | 'gearRatio'>
): DualEccentricSolution {
  const spec = solveDualEccentricGeometry(
    module.camRadius,
    module.eccentricity,
    module.gearRatio
  );
  const theta = shaftAngleRad + degToRad(module.phaseDeg);
  const leftCamAngleRad = theta;
  const rightCamAngleRad = theta + Math.PI;

  const sinT = Math.sin(theta);
  const leftCamTopY = spec.camRadius + spec.eccentricity * sinT;
  const rightCamTopY = spec.camRadius - spec.eccentricity * sinT;
  const trayBottomY = Math.max(leftCamTopY, rightCamTopY);
  const leftGap = Math.max(0, trayBottomY - leftCamTopY);
  const rightGap = Math.max(0, trayBottomY - rightCamTopY);

  // Smooth C-infinity alternating integral: d(driven)/d(theta) = K * sin(theta)
  // Left Cam active (sin > 0) -> forward swing; Right Cam active (sin < 0) -> reverse swing
  const K = spec.swingAmplitudeRad * 0.5;
  const drivenAngleRad = K * (1 - Math.cos(theta));
  const activeCam: DualEccentricSolution['activeCam'] =
    sinT > 0.04 ? 'left' : sinT < -0.04 ? 'right' : 'handoff';

  return {
    leftCamAngleRad,
    rightCamAngleRad,
    leftCamTopY,
    rightCamTopY,
    trayBottomY,
    leftGap,
    rightGap,
    drivenAngleRad,
    activeCam,
  };
}

export interface RatchetGeometrySpec {
  teethCount: number;
  effectiveStepTeeth: number;
  lockDirection: 1 | -1;
  axleYOffset: number;
  armLinkR: number;
  crankRadius: number;
  couplerLength: number;
  wheelRootR: number;
  wheelTipR: number;
  pawlOrbitR: number;
  pawlLength: number;
  toothPitchRad: number;
  stepAngleRad: number;
  slackAngleRad: number;
  halfSwingRad: number;
  thetaAtMin: number;
  thetaAtMax: number;
  phaseSpanToMax: number;
  phaseSpanToMin: number;
  drivePivotLocalAngle: number;
  holdPivotCanonAngle: number;
  bevelPitchR: number;
  bevelTeeth: number;
  bevelDriverX: number;
  bevelDrivenX: number;
  bevelDrivenYOffset: number;
}

const _ratchetSpecCache = new Map<string, RatchetGeometrySpec>();

export function evaluateRatchetWheelRadius(
  alpha: number,
  teethCount: number,
  lockDirection: 1 | -1 = 1
): number {
  const rRoot = 0.34;
  const rTip = 0.44;
  const pitch = (Math.PI * 2) / Math.max(8, Math.round(teethCount));
  const tRaw = normalizeAngleRad(alpha) / pitch;
  const tFrac = tRaw - Math.floor(tRaw);
  const t = lockDirection === 1 ? tFrac : (1 - tFrac) % 1;
  if (t < 0.06) {
    return rRoot + (rTip - rRoot) * (t / 0.06);
  }
  if (t < 0.9) {
    return rTip - (rTip - rRoot) * ((t - 0.06) / 0.84);
  }
  return rRoot;
}

/**
 * Generates exact sharp sawtooth vertices in local XY (x = cos*r, y = sin*r) for ExtrudeGeometry.
 */
export function sampleRatchetContour(
  teethCount: number,
  lockDirection: 1 | -1 = 1,
  rRoot = 0.34,
  rTip = 0.44
): Array<{ x: number; y: number }> {
  const N = Math.max(8, Math.round(teethCount));
  const P = (Math.PI * 2) / N;
  const pts: Array<{ x: number; y: number }> = [];
  for (let k = 0; k < N; k++) {
    const base = k * P;
    const offsets = [
      { a: 0, r: rRoot },
      { a: 0.055 * P, r: rTip },
      { a: 0.89 * P, r: rRoot },
      { a: 0.96 * P, r: rRoot },
    ];
    if (lockDirection === 1) {
      for (const o of offsets) {
        const ang = base + o.a;
        pts.push({ x: Math.cos(ang) * o.r, y: Math.sin(ang) * o.r });
      }
    } else {
      for (let idx = offsets.length - 1; idx >= 0; idx--) {
        const o = offsets[idx];
        const ang = base + (P - o.a);
        pts.push({ x: Math.cos(ang) * o.r, y: Math.sin(ang) * o.r });
      }
    }
  }
  return pts;
}

function solveRatchetCrankRocker(
  theta: number,
  crankRadius: number,
  couplerLength: number,
  armLinkR: number,
  axleYOffset: number
): {
  p1z: number;
  p1y: number;
  p2z: number;
  p2y: number;
  armAngle: number;
  couplerAngle: number;
} {
  const p1z = crankRadius * Math.cos(theta);
  const p1y = crankRadius * Math.sin(theta);
  const dz = 0 - p1z;
  const dy = axleYOffset - p1y;
  const d = Math.max(0.001, Math.hypot(dz, dy));
  const minR = Math.abs(couplerLength - armLinkR) + 0.005;
  const maxR = couplerLength + armLinkR - 0.005;
  const clampedD = Math.min(maxR, Math.max(minR, d));
  const a = (couplerLength * couplerLength - armLinkR * armLinkR + clampedD * clampedD) / (2 * clampedD);
  const h = Math.sqrt(Math.max(0, couplerLength * couplerLength - a * a));
  const uz = dz / d;
  const uy = dy / d;
  // +Z rocker branch around axle (0, axleYOffset)
  const p2z = p1z + a * uz + h * uy;
  const p2y = p1y + a * uy - h * uz;
  const armAngle = Math.atan2(p2y - axleYOffset, p2z);
  const couplerAngle = Math.atan2(p2y - p1y, p2z - p1z);
  return { p1z, p1y, p2z, p2y, armAngle, couplerAngle };
}

function solvePawlTipContact(
  pivotAngle: number,
  canonWheelAngle: number,
  teethCount: number,
  pawlOrbitR: number,
  pawlLength: number
): { tipAngleRad: number; pawlAngleRad: number } {
  const tipOff = 0.016;
  let rSurf = 0.34 + tipOff;
  let delta = 0.57;
  for (let it = 0; it < 5; it++) {
    const cosD = Math.max(
      -1,
      Math.min(1, (pawlOrbitR * pawlOrbitR + rSurf * rSurf - pawlLength * pawlLength) / (2 * pawlOrbitR * rSurf))
    );
    delta = Math.acos(cosD);
    rSurf = evaluateRatchetWheelRadius(pivotAngle + delta - canonWheelAngle, teethCount, 1) + tipOff;
  }
  const cosD = Math.max(
    -1,
    Math.min(1, (pawlOrbitR * pawlOrbitR + rSurf * rSurf - pawlLength * pawlLength) / (2 * pawlOrbitR * rSurf))
  );
  delta = Math.acos(cosD);
  const ac = pivotAngle + delta;
  const pz = pawlOrbitR * Math.cos(pivotAngle);
  const py = pawlOrbitR * Math.sin(pivotAngle);
  const cz = rSurf * Math.cos(ac);
  const cy = rSurf * Math.sin(ac);
  return {
    tipAngleRad: ac,
    pawlAngleRad: Math.atan2(cy - py, cz - pz),
  };
}

export function solveRatchetGeometry(
  teethCount: number,
  stepTeeth: number,
  lockDirection: 1 | -1 = 1
): RatchetGeometrySpec {
  const N = Math.max(8, Math.round(teethCount));
  const maxStep = Math.max(1, Math.min(3, Math.floor(N * 0.28)));
  const effectiveStepTeeth = Math.max(1, Math.min(maxStep, Math.round(stepTeeth)));
  const key = `${N}:${effectiveStepTeeth}:${lockDirection}`;
  const cached = _ratchetSpecCache.get(key);
  if (cached) return cached;

  const axleYOffset = 0.98;
  const armLinkR = 0.44;
  const wheelRootR = 0.34;
  const wheelTipR = 0.44;
  const pawlOrbitR = 0.58;
  const pawlLength = 0.34;

  const toothPitchRad = (Math.PI * 2) / N;
  const stepAngleRad = effectiveStepTeeth * toothPitchRad;
  const slackAngleRad = 0.16 * toothPitchRad;
  const halfSwingRad = 0.5 * (stepAngleRad + slackAngleRad);

  const dMin = Math.sqrt(
    axleYOffset * axleYOffset + armLinkR * armLinkR - 2 * axleYOffset * armLinkR * Math.sin(halfSwingRad)
  );
  const dMax = Math.sqrt(
    axleYOffset * axleYOffset + armLinkR * armLinkR + 2 * axleYOffset * armLinkR * Math.sin(halfSwingRad)
  );
  const couplerLength = 0.5 * (dMax + dMin);
  const crankRadius = 0.5 * (dMax - dMin);

  // Locate exact dead centers (minimum & maximum armAngle) over [0, 2*PI)
  let minA = Infinity;
  let maxA = -Infinity;
  let thetaAtMin = 0;
  let thetaAtMax = 0;
  const coarse = 360;
  for (let i = 0; i < coarse; i++) {
    const t = (i / coarse) * Math.PI * 2;
    const sol = solveRatchetCrankRocker(t, crankRadius, couplerLength, armLinkR, axleYOffset);
    if (sol.armAngle < minA) {
      minA = sol.armAngle;
      thetaAtMin = t;
    }
    if (sol.armAngle > maxA) {
      maxA = sol.armAngle;
      thetaAtMax = t;
    }
  }
  const refine = (centerT: number, findMin: boolean): number => {
    let bestT = centerT;
    let bestV = findMin ? Infinity : -Infinity;
    const step = (Math.PI * 2) / coarse;
    for (let k = -20; k <= 20; k++) {
      const t = centerT + (k / 20) * step;
      const sol = solveRatchetCrankRocker(t, crankRadius, couplerLength, armLinkR, axleYOffset);
      if (findMin ? sol.armAngle < bestV : sol.armAngle > bestV) {
        bestV = sol.armAngle;
        bestT = t;
      }
    }
    return normalizeAngleRad(bestT);
  };
  thetaAtMin = refine(thetaAtMin, true);
  thetaAtMax = refine(thetaAtMax, false);

  const phaseSpanToMax = normalizeAngleRad(thetaAtMax - thetaAtMin);
  const phaseSpanToMin = normalizeAngleRad(thetaAtMin - thetaAtMax);

  const baseDriveContact = Math.round((Math.PI * 0.5) / toothPitchRad) * toothPitchRad - 0.04;
  const drivePivotLocalAngle = baseDriveContact - 0.57 - (-halfSwingRad + slackAngleRad);
  const holdPivotCanonAngle = Math.PI - 0.04 - 0.57;

  const spec: RatchetGeometrySpec = {
    teethCount: N,
    effectiveStepTeeth,
    lockDirection,
    axleYOffset,
    armLinkR,
    crankRadius,
    couplerLength,
    wheelRootR,
    wheelTipR,
    pawlOrbitR,
    pawlLength,
    toothPitchRad,
    stepAngleRad,
    slackAngleRad,
    halfSwingRad,
    thetaAtMin,
    thetaAtMax,
    phaseSpanToMax,
    phaseSpanToMin,
    drivePivotLocalAngle,
    holdPivotCanonAngle,
    bevelPitchR: 0.22,
    bevelTeeth: 12,
    bevelDriverX: 0.14,
    bevelDrivenX: 0.36,
    bevelDrivenYOffset: 0.22,
  };
  _ratchetSpecCache.set(key, spec);
  return spec;
}

/**
 * Solves Ratchet & Pawl one-way stepping & locking kinematics.
 * Accepts unwrapped continuous shaftAngleRad so stepped wheel progress accumulates seamlessly across revolutions.
 */
export function solveRatchet(
  shaftAngleRad: number,
  module: Pick<RatchetModule, 'phaseDeg' | 'teethCount' | 'stepTeeth' | 'lockDirection'>
): RatchetSolution {
  const spec = solveRatchetGeometry(module.teethCount, module.stepTeeth, module.lockDirection);
  const theta = shaftAngleRad + degToRad(module.phaseDeg);
  const cr = solveRatchetCrankRocker(
    theta,
    spec.crankRadius,
    spec.couplerLength,
    spec.armLinkR,
    spec.axleYOffset
  );

  let canonWheelAngle: number;
  let isLocked: boolean;
  let advanceInRev: number;

  if (spec.lockDirection === 1) {
    const u = theta - spec.thetaAtMin;
    const rev = Math.floor(u / (Math.PI * 2));
    const phase = normalizeAngleRad(u);
    const inForward = phase <= spec.phaseSpanToMax;
    if (inForward) {
      advanceInRev = Math.max(
        0,
        Math.min(spec.stepAngleRad, cr.armAngle - (-spec.halfSwingRad + spec.slackAngleRad))
      );
      isLocked = advanceInRev <= 1e-5;
    } else {
      advanceInRev = spec.stepAngleRad;
      isLocked = true;
    }
    canonWheelAngle = rev * spec.stepAngleRad + advanceInRev;
  } else {
    const u = theta - spec.thetaAtMax;
    const rev = Math.floor(u / (Math.PI * 2));
    const phase = normalizeAngleRad(u);
    const inForward = phase <= spec.phaseSpanToMin;
    const flippedArm = -cr.armAngle;
    if (inForward) {
      advanceInRev = Math.max(
        0,
        Math.min(spec.stepAngleRad, flippedArm - (-spec.halfSwingRad + spec.slackAngleRad))
      );
      isLocked = advanceInRev <= 1e-5;
    } else {
      advanceInRev = spec.stepAngleRad;
      isLocked = true;
    }
    canonWheelAngle = rev * spec.stepAngleRad + advanceInRev;
  }

  const ratchetWheelAngleRad = spec.lockDirection * canonWheelAngle;
  const canonArmForPawl = spec.lockDirection === 1 ? cr.armAngle : -cr.armAngle;
  const canonDrivePivAngle = spec.drivePivotLocalAngle + canonArmForPawl;
  const driveSolved = solvePawlTipContact(
    canonDrivePivAngle,
    canonWheelAngle,
    spec.teethCount,
    spec.pawlOrbitR,
    spec.pawlLength
  );
  const holdSolved = solvePawlTipContact(
    spec.holdPivotCanonAngle,
    canonWheelAngle,
    spec.teethCount,
    spec.pawlOrbitR,
    spec.pawlLength
  );

  const drivePivZCanon = spec.pawlOrbitR * Math.cos(canonDrivePivAngle);
  const drivePivY = spec.axleYOffset + spec.pawlOrbitR * Math.sin(canonDrivePivAngle);
  const holdPivZCanon = spec.pawlOrbitR * Math.cos(spec.holdPivotCanonAngle);
  const holdPivY = spec.axleYOffset + spec.pawlOrbitR * Math.sin(spec.holdPivotCanonAngle);

  const drivePawlPivot = {
    z: spec.lockDirection * drivePivZCanon,
    y: drivePivY,
  };
  const holdPawlPivot = {
    z: spec.lockDirection * holdPivZCanon,
    y: holdPivY,
  };
  const drivePawlAngleRad =
    spec.lockDirection === 1 ? driveSolved.pawlAngleRad : Math.PI - driveSolved.pawlAngleRad;
  const holdPawlAngleRad =
    spec.lockDirection === 1 ? holdSolved.pawlAngleRad : Math.PI - holdSolved.pawlAngleRad;

  return {
    crankPin: { z: cr.p1z, y: cr.p1y },
    rockerPin: { z: cr.p2z, y: cr.p2y },
    crankAngleRad: theta,
    couplerAngleRad: cr.couplerAngle,
    armAngleRad: cr.armAngle,
    ratchetWheelAngleRad,
    drivenOutputAngleRad: ratchetWheelAngleRad,
    drivePawlPivot,
    drivePawlAngleRad,
    holdPawlPivot,
    holdPawlAngleRad,
    isLocked,
  };
}

/**
 * Solves symmetric wing-horn joint in XY for the Flapping Seagull pushrod-yoke linkage.
 * Wing hinge is at (wingPivotX, wingPivotY); crosshead pin is at (crossheadX, crossheadY).
 */
export function solveTopperSeagullLinkage(
  crossheadY: number,
  wingPivotX = 0.14,
  wingPivotY = 0.96,
  crossheadX = 0.18,
  hornRadius = 0.3,
  linkLen = 0.6
): { hornX: number; hornY: number; wingAngleRad: number } {
  const dx = crossheadX - wingPivotX;
  const dy = crossheadY - wingPivotY;
  const dRaw = Math.hypot(dx, dy);
  const minR = Math.abs(hornRadius - linkLen) + 0.02;
  const maxR = hornRadius + linkLen - 0.02;
  const d = Math.min(maxR, Math.max(minR, Math.max(0.001, dRaw)));

  const a = (hornRadius * hornRadius - linkLen * linkLen + d * d) / (2 * d);
  const h = Math.sqrt(Math.max(0, hornRadius * hornRadius - a * a));
  const ux = dx / Math.max(0.001, dRaw);
  const uy = dy / Math.max(0.001, dRaw);

  // Pick the outward (+X) elbow branch for the right wing
  const hx = wingPivotX + a * ux - h * uy;
  const hy = wingPivotY + a * uy + h * ux;
  const wingAngleRad = Math.atan2(hy - wingPivotY, hx - wingPivotX);
  return { hornX: hx, hornY: hy, wingAngleRad };
}

/**
 * Solves 2D YZ triangle closure for the Pecking Woodpecker belly-lever pushrod linkage.
 * Fulcrum is at (fulcrumZ, fulcrumY); pushrod pin is at (0, pinY).
 */
export function solveTopperWoodpeckerLinkage(
  pinY: number,
  fulcrumZ = -0.22,
  fulcrumY = 0.72,
  leverLen = 0.27,
  linkLen = 0.39
): { leverPinZ: number; leverPinY: number; birdPitchRad: number } {
  const dz = 0 - fulcrumZ;
  const dy = pinY - fulcrumY;
  const dRaw = Math.hypot(dz, dy);
  const minR = Math.abs(leverLen - linkLen) + 0.02;
  const maxR = leverLen + linkLen - 0.02;
  const d = Math.min(maxR, Math.max(minR, Math.max(0.001, dRaw)));

  const a = (leverLen * leverLen - linkLen * linkLen + d * d) / (2 * d);
  const h = Math.sqrt(Math.max(0, leverLen * leverLen - a * a));
  const uz = dz / Math.max(0.001, dRaw);
  const uy = dy / Math.max(0.001, dRaw);

  // Lower-forward lever branch (towards +Z and below fulcrum)
  const lz = fulcrumZ + a * uz + h * uy;
  const ly = fulcrumY + a * uy - h * uz;
  const leverAngle = Math.atan2(ly - fulcrumY, lz - fulcrumZ);
  // Neutral lever points downward-forward at -0.62 rad; pitching nose-down increases rotation.x
  const birdPitchRad = -(leverAngle - (-0.62));
  return { leverPinZ: lz, leverPinY: ly, birdPitchRad };
}

export interface TopperBoatmanSolution {
  boatPitchRad: number;
  keelPinZ: number;
  keelPinY: number;
  sailorSeatZ: number;
  sailorSeatY: number;
  sailorPitchRad: number;
  dorsalHornZ: number;
  dorsalHornY: number;
  shoulderLocalZ: number;
  shoulderLocalY: number;
  handLocalZ: number;
  handLocalY: number;
}

export const BOATMAN_KINEMATICS = {
  boatFulcrumZ: -0.34,
  boatFulcrumY: 0.8,
  boatLeverLen: 0.7,
  boatLinkLen: 0.4,
  neutralLeverAngle: -0.22,
  seatLocalZ: 0.22,
  seatLocalY: 0.14,
  anchorZ: -0.18,
  anchorY: 0.5,
  sailorHornLen: 0.12,
  sailorLinkLen: 0.44,
  neutralHornAngle: 3.24,
  torsoHeight: 0.23,
  oarlockLocalZ: 0.5,
  oarlockLocalY: 0.24,
  armLenYZ: 0.25,
  inboardOarLenYZ: 0.22,
} as const;

/**
 * Solves the 3-stage rigid-body rowing linkage for the Boatman / Buoy topper:
 * 1) Pushrod crosshead (0, pinY) drives the boat keel crank arm around boatFulcrum;
 * 2) Fixed rear stanchion anchor (anchorZ, anchorY) pulls/pushes the sailor's dorsal horn;
 * 3) Sailor shoulder and boat oarlock close the arm + inboard-oar triangle so hands never detach.
 */
export function solveTopperBoatmanLinkage(pinY: number): TopperBoatmanSolution {
  const K = BOATMAN_KINEMATICS;

  // Stage 1: Pushrod (0, pinY) -> Keel crank pin around boatFulcrum
  const dz1 = 0 - K.boatFulcrumZ;
  const dy1 = pinY - K.boatFulcrumY;
  const d1Raw = Math.max(0.001, Math.hypot(dz1, dy1));
  const minR1 = Math.abs(K.boatLeverLen - K.boatLinkLen) + 0.02;
  const maxR1 = K.boatLeverLen + K.boatLinkLen - 0.02;
  const d1 = Math.min(maxR1, Math.max(minR1, d1Raw));
  const u1z = dz1 / d1Raw;
  const u1y = dy1 / d1Raw;
  const a1 = (K.boatLeverLen * K.boatLeverLen - K.boatLinkLen * K.boatLinkLen + d1 * d1) / (2 * d1);
  const h1 = Math.sqrt(Math.max(0, K.boatLeverLen * K.boatLeverLen - a1 * a1));
  // Lower-forward keel pin branch (+Z, below fulcrum)
  const keelPinZ = K.boatFulcrumZ + a1 * u1z - h1 * u1y;
  const keelPinY = K.boatFulcrumY + a1 * u1y + h1 * u1z;
  const leverAngle = Math.atan2(keelPinY - K.boatFulcrumY, keelPinZ - K.boatFulcrumZ);
  const boatPitchRad = -(leverAngle - K.neutralLeverAngle);

  // Sailor seat transform under boatPivot.rotation.x = boatPitchRad
  const cosB = Math.cos(boatPitchRad);
  const sinB = Math.sin(boatPitchRad);
  const sailorSeatY = K.boatFulcrumY + K.seatLocalY * cosB - K.seatLocalZ * sinB;
  const sailorSeatZ = K.boatFulcrumZ + K.seatLocalY * sinB + K.seatLocalZ * cosB;

  // Stage 2: Fixed anchor -> Sailor dorsal crank horn around sailorSeat
  const dz2 = K.anchorZ - sailorSeatZ;
  const dy2 = K.anchorY - sailorSeatY;
  const d2Raw = Math.max(0.001, Math.hypot(dz2, dy2));
  const minR2 = Math.abs(K.sailorHornLen - K.sailorLinkLen) + 0.02;
  const maxR2 = K.sailorHornLen + K.sailorLinkLen - 0.02;
  const d2 = Math.min(maxR2, Math.max(minR2, d2Raw));
  const u2z = dz2 / d2Raw;
  const u2y = dy2 / d2Raw;
  const a2 = (K.sailorHornLen * K.sailorHornLen - K.sailorLinkLen * K.sailorLinkLen + d2 * d2) / (2 * d2);
  const h2 = Math.sqrt(Math.max(0, K.sailorHornLen * K.sailorHornLen - a2 * a2));
  // Aft (-Z) dorsal horn branch
  const dorsalHornZ = sailorSeatZ + a2 * u2z + h2 * u2y;
  const dorsalHornY = sailorSeatY + a2 * u2y - h2 * u2z;
  let dorsalAngle = Math.atan2(dorsalHornY - sailorSeatY, dorsalHornZ - sailorSeatZ);
  if (dorsalAngle < 0) dorsalAngle += Math.PI * 2;
  const worldTorsoTilt = -(dorsalAngle - K.neutralHornAngle);
  const sailorPitchRad = worldTorsoTilt - boatPitchRad;

  // Stage 3: Shoulder -> Arm & Inboard Oar -> Oarlock in boatPivot local YZ plane
  const shoulderLocalZ = K.seatLocalZ + K.torsoHeight * Math.sin(sailorPitchRad);
  const shoulderLocalY = K.seatLocalY + K.torsoHeight * Math.cos(sailorPitchRad);
  const dz3 = K.oarlockLocalZ - shoulderLocalZ;
  const dy3 = K.oarlockLocalY - shoulderLocalY;
  const d3Raw = Math.max(0.001, Math.hypot(dz3, dy3));
  const minR3 = Math.abs(K.armLenYZ - K.inboardOarLenYZ) + 0.02;
  const maxR3 = K.armLenYZ + K.inboardOarLenYZ - 0.02;
  const d3 = Math.min(maxR3, Math.max(minR3, d3Raw));
  const u3z = dz3 / d3Raw;
  const u3y = dy3 / d3Raw;
  const a3 = (K.armLenYZ * K.armLenYZ - K.inboardOarLenYZ * K.inboardOarLenYZ + d3 * d3) / (2 * d3);
  const h3 = Math.sqrt(Math.max(0, K.armLenYZ * K.armLenYZ - a3 * a3));
  // Lower elbow/grip branch (u3z > 0 -> -h3 * u3z drops hand below shoulder-oarlock chord)
  const handLocalZ = shoulderLocalZ + a3 * u3z + h3 * u3y;
  const handLocalY = shoulderLocalY + a3 * u3y - h3 * u3z;

  return {
    boatPitchRad,
    keelPinZ,
    keelPinY,
    sailorSeatZ,
    sailorSeatY,
    sailorPitchRad,
    dorsalHornZ,
    dorsalHornY,
    shoulderLocalZ,
    shoulderLocalY,
    handLocalZ,
    handLocalY,
  };
}

/**
 * Clamps a dragged module's X position along the camshaft, avoiding overlap with sibling modules.
 */
export function clampModuleSlotX(
  targetX: number,
  movingModuleId: string | null,
  existingModules: AutomataModule[]
): number {
  let clamped = Math.max(SHAFT_MIN_X, Math.min(SHAFT_MAX_X, targetX));
  const others = existingModules
    .filter((m) => m.id !== movingModuleId)
    .map((m) => m.slotX)
    .sort((a, b) => a - b);

  for (let pass = 0; pass < 4; pass++) {
    for (const ox of others) {
      const dist = clamped - ox;
      if (Math.abs(dist) < MIN_MODULE_SPACING) {
        const pushDir = dist >= 0 ? 1 : -1;
        clamped = ox + pushDir * MIN_MODULE_SPACING;
        clamped = Math.max(SHAFT_MIN_X, Math.min(SHAFT_MAX_X, clamped));
      }
    }
  }
  return Math.round(clamped * 100) / 100;
}

/**
 * Finds an open X slot on the camshaft when adding a new modular block.
 */
export function findNearestFreeSlotX(
  preferredX: number,
  existingModules: AutomataModule[]
): number {
  const candidates = [0, -1.25, 1.25, -2.25, 2.25, -0.65, 0.65, -1.8, 1.8];
  const sorted = [preferredX, ...candidates].map((x) =>
    Math.max(SHAFT_MIN_X, Math.min(SHAFT_MAX_X, x))
  );

  for (const cand of sorted) {
    const clear = existingModules.every(
      (m) => Math.abs(m.slotX - cand) >= MIN_MODULE_SPACING
    );
    if (clear) {
      return Math.round(cand * 100) / 100;
    }
  }
  return clampModuleSlotX(preferredX, null, existingModules);
}

/**
 * Clamps dragged 4-bar fulcrum coordinates within the physical wooden box pegboard envelope.
 */
export function clampFourBarPivot(pivotZ: number, pivotY: number): { pivotZ: number; pivotY: number } {
  const clampedZ = Math.max(-1.25, Math.min(1.25, pivotZ));
  const safeZ = Math.abs(clampedZ) < 0.55 ? (clampedZ >= 0 ? 0.55 : -0.55) : clampedZ;
  const clampedY = Math.max(0.55, Math.min(1.35, pivotY));
  return {
    pivotZ: Math.round(safeZ * 100) / 100,
    pivotY: Math.round(clampedY * 100) / 100,
  };
}

/**
 * Generates a full 360-degree displacement curve (0..359 deg) for live cam/linkage inspection diagrams.
 */
export function sampleCycleCurve(module: AutomataModule, steps = 72): number[] {
  const values: number[] = [];
  const fourBarCfg =
    module.type === 'four-bar-linkage' ? computeFourBarPushrodConfig(module) : null;
  for (let i = 0; i < steps; i++) {
    const theta = (i / steps) * Math.PI * 2;
    if (module.type === 'cam-follower') {
      const sol = solveCamFollower(
        module.camType,
        theta,
        module.phaseDeg,
        module.baseRadius,
        module.lift,
        module.footRadius
      );
      values.push(sol.normalizedLift);
    } else if (module.type === 'four-bar-linkage' && fourBarCfg) {
      const pushSol = solveFourBarPushrod(theta, module, fourBarCfg);
      values.push(pushSol.normalizedDrive);
    } else if (module.type === 'bevel-gear') {
      const sol = solveBevelGear(theta, module);
      values.push((((sol.drivenAngleRad % (Math.PI * 2)) + Math.PI * 2) % (Math.PI * 2)) / (Math.PI * 2));
    } else if (module.type === 'pin-lantern') {
      const sol = solvePinLantern(theta, module);
      values.push((((sol.drivenAngleRad % (Math.PI * 2)) + Math.PI * 2) % (Math.PI * 2)) / (Math.PI * 2));
    } else if (module.type === 'circular-cam') {
      const spec = solveCircularCamGeometry(
        module.gearRatio,
        module.driverRadius,
        module.camOffsetRatio,
        module.camShape ?? 'circle'
      );
      if (Math.abs(spec.effectiveRatio) < 1e-5) {
        values.push(0);
      } else {
        const sol = solveCircularCam(theta, module);
        if (spec.camShape === 'circle') {
          values.push(
            (((sol.drivenAngleRad % (Math.PI * 2)) + Math.PI * 2) % (Math.PI * 2)) / (Math.PI * 2)
          );
        } else {
          const span = Math.max(0.01, spec.driverRadius - spec.inradius);
          values.push(Math.min(1, Math.max(0, sol.liftDisplacement / span)));
        }
      }
    } else if (module.type === 'dual-eccentric') {
      const thetaPhase = theta + degToRad(module.phaseDeg);
      values.push(0.5 * (1 - Math.cos(thetaPhase)));
    } else if (module.type === 'semi-circular-cam') {
      const spec = solveSemiCircularCamGeometry(
        module.gearRatio,
        module.driverRadius,
        module.camOffsetRatio
      );
      if (Math.abs(spec.effectiveRatio) < 1e-5) {
        values.push(0);
      } else {
        const thetaPhase = normalizeAngleRad(theta + degToRad(module.phaseDeg));
        values.push(thetaPhase <= Math.PI ? thetaPhase / Math.PI : 1);
      }
    } else if (module.type === 'ratchet') {
      const spec = solveRatchetGeometry(module.teethCount, module.stepTeeth, module.lockDirection);
      const baseTheta = spec.lockDirection === 1 ? spec.thetaAtMin : spec.thetaAtMax;
      const startVal = solveRatchet(baseTheta - degToRad(module.phaseDeg), module).ratchetWheelAngleRad;
      const curVal = solveRatchet(baseTheta + theta - degToRad(module.phaseDeg), module).ratchetWheelAngleRad;
      const stepProg = Math.abs(curVal - startVal) / Math.max(0.01, spec.stepAngleRad);
      values.push(Math.min(1, Math.max(0, stepProg)));
    }
  }
  return values;
}
