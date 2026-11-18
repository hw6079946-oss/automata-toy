import type {
  AutomataModule,
  BellCrankModule,
  BellCrankSolution,
  BevelGearModule,
  BevelGearSolution,
  CamFollowerModule,
  CamFollowerSolution,
  CamProfileType,
  CircularCamModule,
  CircularCamShape,
  CircularCamSolution,
  DigitalNumbersModule,
  DigitalNumbersSegmentSolution,
  DigitalNumbersSolution,
  DigitalSegmentId,
  DualEccentricModule,
  DualEccentricSolution,
  FourBarLinkageModule,
  FourBarSolution,
  PinLanternModule,
  PinLanternSolution,
  RatchetModule,
  RatchetSolution,
  ScotchYokeModule,
  ScotchYokeSolution,
  SemiCircularCamModule,
  SemiCircularCamSolution,
  WritingCamBellCrankSolution,
  WritingCamModule,
  WritingCamSolution,
  WritingDigitPattern,
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

export const FOUR_BAR_GEAR_TEETH = 16;
export const FOUR_BAR_CRANK_OFFSET_Z = 0.8;
export const FOUR_BAR_CRANK_OFFSET_Y = 0.26;

export function solveFourBarGearSpec(pivotZ: number): {
  crankCenterZ: number;
  crankCenterY: number;
  centerDist: number;
  gearPitchR: number;
  gearTeeth: number;
  lineAngleRad: number;
} {
  const s = pivotZ >= 0 ? 1 : -1;
  const crankCenterZ = -s * FOUR_BAR_CRANK_OFFSET_Z;
  const crankCenterY = FOUR_BAR_CRANK_OFFSET_Y;
  const centerDist = Math.hypot(crankCenterZ, crankCenterY);
  const gearPitchR = centerDist * 0.5;
  return {
    crankCenterZ,
    crankCenterY,
    centerDist,
    gearPitchR,
    gearTeeth: FOUR_BAR_GEAR_TEETH,
    lineAngleRad: Math.atan2(crankCenterY, crankCenterZ),
  };
}

/**
 * Solves a physically realizable 1:1 Spur-Gear Offset 4-Bar Crank-Rocker Linkage in local YZ:
 * - Main horizontal shaft (0, 0) carries a 16T driver spur gear.
 * - Offset 16T crank spur gear is mounted at (-sign(pivotZ)*0.80, +0.26) on an independent bearing
 *   subframe and counter-rotates 1:1, placing the crank pin + overhead coupler strictly clear of
 *   the continuous steel camshaft across all 360 degrees.
 */
export function solveFourBarLinkage(
  shaftAngleRad: number,
  module: Pick<
    FourBarLinkageModule,
    'phaseDeg' | 'crankRadius' | 'couplerLength' | 'rockerLength' | 'pivotZ' | 'pivotY' | 'outputArmRatio'
  >
): FourBarSolution {
  const gearSpec = solveFourBarGearSpec(module.pivotZ);
  const driverAngleRad = shaftAngleRad + degToRad(module.phaseDeg);
  const crankAngleRad = -driverAngleRad;

  const p1z = gearSpec.crankCenterZ + module.crankRadius * Math.cos(crankAngleRad);
  const p1y = gearSpec.crankCenterY + module.crankRadius * Math.sin(crankAngleRad);

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

  // Overhead branch keeps rocker-coupler joint P2 above the P1-P3 chord, arching coupler clear of shaft
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
    crankCenter: { z: gearSpec.crankCenterZ, y: gearSpec.crankCenterY },
    crankPin: { z: p1z, y: p1y },
    couplerJoint: { z: p2z, y: p2y },
    fulcrum: { z: p3z, y: p3y },
    driverAngleRad,
    crankAngleRad,
    couplerAngleRad,
    rockerAngleRad,
    gearPitchR: gearSpec.gearPitchR,
    gearTeeth: gearSpec.gearTeeth,
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
  const clevisArmDist = Math.min(0.46, module.rockerLength * 0.46);
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
  const linkLength = Math.max(0.3, maxDz + 0.16);

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
  const angularVelNorm = K * Math.sin(theta);
  const angularAccNorm = K * Math.cos(theta);
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
    angularVelNorm,
    angularAccNorm,
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

export const SCOTCH_YOKE_CENTER_DIST = 0.76;
export const SCOTCH_YOKE_GUIDE_RAIL_Y = 1.38;

/**
 * Computes exact meshing spur-gear pitch radii and slot geometry for the offset-above-shaft Scotch Yoke.
 * Placing the crank gear vertically above the main camshaft guarantees zero shaft collision across all strokes.
 */
export function solveScotchYokeGeometry(
  crankRadius: number,
  gearRatio: number
): {
  crankCenter: { z: number; y: number };
  crankRadius: number;
  gearRatio: number;
  driverTeeth: number;
  drivenTeeth: number;
  driverPitchR: number;
  drivenPitchR: number;
  slotHalfHeight: number;
  guideRailY: number;
} {
  const safeR = Math.max(0.18, Math.min(0.36, crankRadius));
  const safeRatio = Math.max(0.75, Math.min(1.5, gearRatio));
  const driverTeeth = 16;
  const drivenTeeth = Math.max(10, Math.round(driverTeeth / safeRatio));
  const unitR = SCOTCH_YOKE_CENTER_DIST / (driverTeeth + drivenTeeth);
  const driverPitchR = driverTeeth * unitR;
  const drivenPitchR = drivenTeeth * unitR;
  const slotHalfHeight = safeR + 0.055;

  return {
    crankCenter: { z: 0, y: SCOTCH_YOKE_CENTER_DIST },
    crankRadius: safeR,
    gearRatio: driverTeeth / drivenTeeth,
    driverTeeth,
    drivenTeeth,
    driverPitchR,
    drivenPitchR,
    slotHalfHeight,
    guideRailY: SCOTCH_YOKE_GUIDE_RAIL_Y,
  };
}

/**
 * Solves pure simple harmonic horizontal reciprocation (SHM) of the Scotch Yoke mechanism:
 * sliderZ = crankRadius * cos(crankAngleRad), slotBlockY = crankCenter.y + crankRadius * sin(crankAngleRad).
 */
export function solveScotchYoke(
  shaftAngleRad: number,
  module: Pick<ScotchYokeModule, 'phaseDeg' | 'crankRadius' | 'gearRatio'>
): ScotchYokeSolution {
  const geo = solveScotchYokeGeometry(module.crankRadius, module.gearRatio);
  const driverAngleRad = shaftAngleRad + degToRad(module.phaseDeg);
  // External meshing spur gears counter-rotate; phase offset aligns crank at +Z (maximum forward stroke) when theta = 0
  const crankAngleRad = -driverAngleRad * geo.gearRatio;
  const pinZ = geo.crankRadius * Math.cos(crankAngleRad);
  const pinY = geo.crankCenter.y + geo.crankRadius * Math.sin(crankAngleRad);

  return {
    driverAngleRad,
    crankAngleRad,
    crankCenter: geo.crankCenter,
    crankPin: { z: pinZ, y: pinY },
    sliderZ: pinZ,
    slotBlockY: pinY,
    normalizedSlide: 0.5 * (1 + Math.cos(crankAngleRad)),
    driverTeeth: geo.driverTeeth,
    drivenTeeth: geo.drivenTeeth,
    driverPitchR: geo.driverPitchR,
    drivenPitchR: geo.drivenPitchR,
    slotHalfHeight: geo.slotHalfHeight,
    guideRailY: geo.guideRailY,
  };
}

export const SAWYERS_KINEMATICS = {
  hipZ: 0.85,
  hipY: 0.42,
  torsoLen: 0.3,
  upperArmLen: 0.35,
  forearmLen: 0.35,
  handleOffset: 0.52,
  handleY: 0.56,
  torsoLeanGain: 0.9,
} as const;

export interface SawyerArmBranch {
  hipZ: number;
  hipY: number;
  torsoLeanRad: number;
  shoulderZ: number;
  shoulderY: number;
  elbowZ: number;
  elbowY: number;
  handleZ: number;
  handleY: number;
}

export interface TopperSawyersSolution {
  sliderZ: number;
  rear: SawyerArmBranch;
  front: SawyerArmBranch;
}

function solveSingleSawyerBranch(
  hipZ: number,
  torsoLeanRad: number,
  handleZ: number
): SawyerArmBranch {
  const K = SAWYERS_KINEMATICS;
  const shoulderZ = hipZ + Math.sin(torsoLeanRad) * K.torsoLen;
  const shoulderY = K.hipY + Math.cos(torsoLeanRad) * K.torsoLen;

  const dz = handleZ - shoulderZ;
  const dy = K.handleY - shoulderY;
  const dRaw = Math.max(0.001, Math.hypot(dz, dy));
  const minR = Math.abs(K.upperArmLen - K.forearmLen) + 0.02;
  const maxR = K.upperArmLen + K.forearmLen - 0.02;
  const d = Math.min(maxR, Math.max(minR, dRaw));
  const uz = dz / dRaw;
  const uy = dy / dRaw;

  const a = (K.upperArmLen * K.upperArmLen - K.forearmLen * K.forearmLen + d * d) / (2 * d);
  const h = Math.sqrt(Math.max(0, K.upperArmLen * K.upperArmLen - a * a));

  // Natural downward-bent elbow branch:
  // If facing +Z (uz > 0), (+h*uy, -h*uz) drops Y downward;
  // If facing -Z (uz < 0), (-h*uy, +h*uz) drops Y downward.
  const downSign = uz >= 0 ? 1 : -1;
  const elbowZ = shoulderZ + a * uz + downSign * h * uy;
  const elbowY = shoulderY + a * uy - downSign * h * uz;

  return {
    hipZ,
    hipY: K.hipY,
    torsoLeanRad,
    shoulderZ,
    shoulderY,
    elbowZ,
    elbowY,
    handleZ,
    handleY: K.handleY,
  };
}

/**
 * Solves exact 2-link articulated arm & torso lean kinematics for two opposing lumberjacks
 * driving a shared crosscut timber saw driven by pure horizontal reciprocation sliderZ.
 */
export function solveTopperSawyersLinkage(sliderZ: number): TopperSawyersSolution {
  const K = SAWYERS_KINEMATICS;
  const clampedZ = Math.max(-0.38, Math.min(0.38, sliderZ));
  const torsoLeanRad = clampedZ * K.torsoLeanGain;

  return {
    sliderZ: clampedZ,
    rear: solveSingleSawyerBranch(-K.hipZ, torsoLeanRad, clampedZ - K.handleOffset),
    front: solveSingleSawyerBranch(K.hipZ, torsoLeanRad, clampedZ + K.handleOffset),
  };
}

export const CARPENTER_KINEMATICS = {
  hipZ: -0.56,
  hipY: 0.42,
  torsoLen: 0.3,
  upperArmLen: 0.4,
  forearmLen: 0.4,
  handleOffsetZ: -0.12,
  handleY: 0.5,
  torsoLeanGain: 0.95,
} as const;

export interface TopperCarpenterSolution {
  planeZ: number;
  shavingCurlAngle: number;
  shavingLift: number;
  arm: SawyerArmBranch;
}

/**
 * Solves rigid-body 2-link arm IK and torso lean for the Carpenter Planing Workbench topper.
 * Hands stay locked onto the hand plane tote as sliderZ sweeps purely horizontally.
 */
export function solveTopperCarpenterLinkage(
  sliderZ: number,
  crankAngleRad = 0
): TopperCarpenterSolution {
  const K = CARPENTER_KINEMATICS;
  const clampedZ = Math.max(-0.38, Math.min(0.38, sliderZ));
  const lean = clampedZ * K.torsoLeanGain;
  const shoulderZ = K.hipZ + Math.sin(lean) * K.torsoLen;
  const shoulderY = K.hipY + Math.cos(lean) * K.torsoLen;
  const handleZ = clampedZ + K.handleOffsetZ;

  const dz = handleZ - shoulderZ;
  const dy = K.handleY - shoulderY;
  const dRaw = Math.max(0.001, Math.hypot(dz, dy));
  const minR = Math.abs(K.upperArmLen - K.forearmLen) + 0.02;
  const maxR = K.upperArmLen + K.forearmLen - 0.02;
  const d = Math.min(maxR, Math.max(minR, dRaw));
  const uz = dz / dRaw;
  const uy = dy / dRaw;

  const a = (K.upperArmLen * K.upperArmLen - K.forearmLen * K.forearmLen + d * d) / (2 * d);
  const h = Math.sqrt(Math.max(0, K.upperArmLen * K.upperArmLen - a * a));
  const elbowZ = shoulderZ + a * uz + h * uy;
  const elbowY = shoulderY + a * uy - h * uz;

  // Curl wood shaving upward during forward cutting stroke (+Z direction)
  const forwardStroke01 = Math.max(0, (clampedZ + 0.36) / 0.72);
  const cutActive = Math.sin(crankAngleRad) <= 0 ? 1 : 0.45;
  const shavingCurlAngle = 0.25 + forwardStroke01 * cutActive * 0.85;
  const shavingLift = forwardStroke01 * 0.045;

  return {
    planeZ: clampedZ,
    shavingCurlAngle,
    shavingLift,
    arm: {
      hipZ: K.hipZ,
      hipY: K.hipY,
      torsoLeanRad: lean,
      shoulderZ,
      shoulderY,
      elbowZ,
      elbowY,
      handleZ,
      handleY: K.handleY,
    },
  };
}

export const LOOM_KINEMATICS = {
  shuttleY: 0.37,
  beamSpanHalfZ: 0.46,
  pickerPivotZ: 0.52,
  pickerKickStart: 0.16,
  heddleMaxOffset: 0.055,
} as const;

export interface TopperLoomSolution {
  shuttleZ: number;
  heddleAOffsetY: number;
  heddleBOffsetY: number;
  rearPickerAngleRad: number;
  frontPickerAngleRad: number;
}

/**
 * Solves flying-shuttle horizontal position, alternating heddle-frame shed opening,
 * and end-stroke picker arm rebound angles for the Weaving Loom topper.
 */
export function solveTopperLoomLinkage(
  sliderZ: number,
  crankAngleRad = 0
): TopperLoomSolution {
  const K = LOOM_KINEMATICS;
  const clampedZ = Math.max(-0.38, Math.min(0.38, sliderZ));
  const shedWave = Math.sin(crankAngleRad);
  const heddleAOffsetY = shedWave * K.heddleMaxOffset;
  const heddleBOffsetY = -heddleAOffsetY;

  const rearKick = Math.max(0, -clampedZ - K.pickerKickStart) / 0.22;
  const frontKick = Math.max(0, clampedZ - K.pickerKickStart) / 0.22;

  return {
    shuttleZ: clampedZ,
    heddleAOffsetY,
    heddleBOffsetY,
    rearPickerAngleRad: -rearKick * 0.48,
    frontPickerAngleRad: frontKick * 0.48,
  };
}

export const BELLOWS_KINEMATICS = {
  frontHeaderZ: 0.14,
  rearRestZ: -0.36,
  bellowsCenterY: 0.31,
  pleatCount: 5,
  rockerPivotZ: -0.82,
  rockerPivotY: 0.2,
} as const;

export interface TopperBellowsSolution {
  rearBoardZ: number;
  gap: number;
  pleatZs: number[];
  emberGlow01: number;
  rockerAngleRad: number;
}

/**
 * Solves accordion bellows compression spacing, glowing charcoal forge intensity,
 * and rear rocker-arm pitch for the Forge Bellows horizontal SHM topper.
 */
export function solveTopperBellowsLinkage(sliderZ: number): TopperBellowsSolution {
  const K = BELLOWS_KINEMATICS;
  const clampedZ = Math.max(-0.38, Math.min(0.38, sliderZ));
  const rearBoardZ = K.rearRestZ + clampedZ;
  const gap = Math.max(0.08, K.frontHeaderZ - rearBoardZ);
  const pleatZs: number[] = [];
  for (let i = 1; i <= K.pleatCount; i++) {
    pleatZs.push(rearBoardZ + (i / (K.pleatCount + 1)) * gap);
  }
  const compression01 = Math.max(0, Math.min(1, (clampedZ + 0.36) / 0.72));
  const rockerAngleRad = Math.atan2(
    rearBoardZ - K.rockerPivotZ,
    K.bellowsCenterY - K.rockerPivotY
  ) * 0.42;

  return {
    rearBoardZ,
    gap,
    pleatZs,
    emberGlow01: compression01,
    rockerAngleRad,
  };
}

export const HOCKEY_KINEMATICS = {
  sideX: 0.115,
  rockerPivotZ: 0.56,
  rockerPivotY: 0.145,
  rollerArmLen: 0.32,
  midArmLen: 0.11,
  outerArmLen: 0.24,
  outerOffsetAngle: Math.PI - 0.25,
  rollerRadius: 0.022,
  camBaseY: 0.063,
  camMaxLift: 0.094,
  hipZ: 0.82,
  hipY: 0.44,
  torsoLen: 0.3,
  waistHornR: 0.11,
  waistRestLean: 0.1,
  upperArmLen: 0.24,
  forearmLen: 0.24,
  gripLocalFwd: 0.195,
  gripLocalDown: -0.155,
  stickHornR: 0.1,
  hornRestAngle: 2.95,
  bladeLen: 0.305,
  bladeRestAngle: -1.16,
  puckRadius: 0.056,
  bladeHalfThick: 0.008,
  iceTopY: 0.261,
} as const;

/**
 * Evaluates the contoured cam lobe mounted on each side of the hockey puck shuttle.
 * u = local Z offset from shuttle center toward the player's side;
 * returns vertical roller lift above camBaseY (0 .. camMaxLift).
 */
export function evaluateHockeyCamLift(u: number): number {
  if (u <= -0.2 || u >= 0.18) return 0;
  if (u <= 0.04) {
    const t = (u + 0.2) / 0.24;
    return HOCKEY_KINEMATICS.camMaxLift * 0.5 * (1 - Math.cos(Math.PI * t));
  }
  if (u <= 0.14) {
    const s = (u - 0.04) / 0.1;
    return HOCKEY_KINEMATICS.camMaxLift - 0.06 * 0.5 * (1 - Math.cos(Math.PI * s));
  }
  const e = (u - 0.14) / 0.04;
  return (HOCKEY_KINEMATICS.camMaxLift - 0.06) * 0.5 * (1 + Math.cos(Math.PI * e));
}

/**
 * Samples the 2D side silhouette (z, y) of the hockey shuttle cam bar for ExtrudeGeometry,
 * keeping top contact heights identical to evaluateHockeyCamLift(z).
 */
export function sampleHockeyCamProfile(): Array<{ z: number; y: number }> {
  const pts: Array<{ z: number; y: number }> = [];
  const baseBottomY = 0.024;
  const steps = 40;
  const zMin = -0.21;
  const zMax = 0.19;
  pts.push({ z: zMin, y: baseBottomY });
  for (let i = 0; i <= steps; i++) {
    const z = zMin + (i / steps) * (zMax - zMin);
    pts.push({ z, y: HOCKEY_KINEMATICS.camBaseY + evaluateHockeyCamLift(z) });
  }
  pts.push({ z: zMax, y: baseBottomY });
  return pts;
}

function intersectCirclePairZY(
  c1z: number,
  c1y: number,
  r1: number,
  c2z: number,
  c2y: number,
  r2: number,
  pickBranch: 1 | -1
): { z: number; y: number } {
  const dz = c2z - c1z;
  const dy = c2y - c1y;
  const dRaw = Math.max(1e-5, Math.hypot(dz, dy));
  const clamped = Math.min(r1 + r2 - 1e-5, Math.max(Math.abs(r1 - r2) + 1e-5, dRaw));
  const a = (r1 * r1 - r2 * r2 + clamped * clamped) / (2 * clamped);
  const h = Math.sqrt(Math.max(0, r1 * r1 - a * a));
  const uz = dz / dRaw;
  const uy = dy / dRaw;
  return {
    z: c1z + a * uz + pickBranch * h * uy,
    y: c1y + a * uy - pickBranch * h * uz,
  };
}

const HOCKEY_DERIVED_LENGTHS = (() => {
  const K = HOCKEY_KINEMATICS;
  const pivZ = -K.rockerPivotZ;
  const pivY = K.rockerPivotY;
  const rollerRestY = K.camBaseY + K.rollerRadius;
  const a0 = Math.asin((rollerRestY - pivY) / K.rollerArmLen);
  const midZ0 = pivZ + K.midArmLen * Math.cos(a0 + K.outerOffsetAngle);
  const midY0 = pivY + K.midArmLen * Math.sin(a0 + K.outerOffsetAngle);
  const outZ0 = pivZ + K.outerArmLen * Math.cos(a0 + K.outerOffsetAngle);
  const outY0 = pivY + K.outerArmLen * Math.sin(a0 + K.outerOffsetAngle);

  const hipZ = -K.hipZ;
  const hipY = K.hipY;
  const lean0 = K.waistRestLean;
  const waistZ0 = hipZ - K.waistHornR * Math.cos(lean0);
  const waistY0 = hipY - K.waistHornR * Math.sin(lean0);
  const torsoRodLen = Math.hypot(waistZ0 - midZ0, waistY0 - midY0);

  const shZ0 = hipZ + K.torsoLen * Math.sin(lean0);
  const shY0 = hipY + K.torsoLen * Math.cos(lean0);
  const gZ0 = shZ0 + K.gripLocalFwd * Math.cos(lean0) + K.gripLocalDown * Math.sin(lean0);
  const gY0 = shY0 - K.gripLocalFwd * Math.sin(lean0) + K.gripLocalDown * Math.cos(lean0);
  const hornZ0 = gZ0 + K.stickHornR * Math.cos(K.hornRestAngle);
  const hornY0 = gY0 + K.stickHornR * Math.sin(K.hornRestAngle);
  const vertRodLen = Math.hypot(hornZ0 - outZ0, hornY0 - outY0);

  return {
    torsoRodLen,
    vertRodLen,
    bladeRelAngle: K.bladeRestAngle - K.hornRestAngle,
  };
})();

export interface HockeyPlayerBranch {
  sideX: number;
  rockerPivotZ: number;
  rockerPivotY: number;
  rockerAngleRad: number;
  rollerZ: number;
  rollerY: number;
  camLift: number;
  onDwellPad: boolean;
  midPinZ: number;
  midPinY: number;
  outerPinZ: number;
  outerPinY: number;
  waistPinZ: number;
  waistPinY: number;
  hornPinZ: number;
  hornPinY: number;
  torsoRodLen: number;
  vertRodLen: number;
  arm: SawyerArmBranch;
  gripZ: number;
  gripY: number;
  bladeZ: number;
  bladeY: number;
  strike01: number;
}

export interface TopperHockeySolution {
  puckZ: number;
  puckSpinRad: number;
  rear: HockeyPlayerBranch;
  front: HockeyPlayerBranch;
}

function solveSingleHockeyPlayer(
  facingSign: 1 | -1,
  puckZ: number
): HockeyPlayerBranch {
  const K = HOCKEY_KINEMATICS;
  const L = HOCKEY_DERIVED_LENGTHS;
  const sideX = -facingSign * K.sideX;

  // Solve in canonical rear-facing space (player at z = -K.hipZ, facing +Z), then mirror by facingSign
  const canonicalPuckZ = facingSign * puckZ;
  const pivZ = -K.rockerPivotZ;
  const pivY = K.rockerPivotY;
  const rollerRestY = K.camBaseY + K.rollerRadius;

  // Fixed-point tangent contact between roller arc and sliding cam lobe
  let rollerZ = pivZ + K.rollerArmLen;
  let camLift = 0;
  for (let it = 0; it < 4; it++) {
    camLift = evaluateHockeyCamLift(rollerZ - canonicalPuckZ);
    const dy = rollerRestY + camLift - pivY;
    rollerZ = pivZ + Math.sqrt(Math.max(1e-4, K.rollerArmLen * K.rollerArmLen - dy * dy));
  }
  const rollerY = rollerRestY + camLift;
  const alpha = Math.atan2(rollerY - pivY, rollerZ - pivZ);

  const midPinZ = pivZ + K.midArmLen * Math.cos(alpha + K.outerOffsetAngle);
  const midPinY = pivY + K.midArmLen * Math.sin(alpha + K.outerOffsetAngle);
  const outerPinZ = pivZ + K.outerArmLen * Math.cos(alpha + K.outerOffsetAngle);
  const outerPinY = pivY + K.outerArmLen * Math.sin(alpha + K.outerOffsetAngle);

  // 1. Torso waist horn driven rigidly by torsoRodLen from rocker midPin
  const hipZ = -K.hipZ;
  const hipY = K.hipY;
  const waist = intersectCirclePairZY(hipZ, hipY, K.waistHornR, midPinZ, midPinY, L.torsoRodLen, 1);
  const lean = Math.atan2(hipY - waist.y, hipZ - waist.z);

  const shoulderZ = hipZ + K.torsoLen * Math.sin(lean);
  const shoulderY = hipY + K.torsoLen * Math.cos(lean);
  const gripZ = shoulderZ + K.gripLocalFwd * Math.cos(lean) + K.gripLocalDown * Math.sin(lean);
  const gripY = shoulderY - K.gripLocalFwd * Math.sin(lean) + K.gripLocalDown * Math.cos(lean);

  // 2. Two-link shoulder-elbow-grip arm IK
  const dz = gripZ - shoulderZ;
  const dy = gripY - shoulderY;
  const dRaw = Math.max(0.001, Math.hypot(dz, dy));
  const minR = Math.abs(K.upperArmLen - K.forearmLen) + 0.02;
  const maxR = K.upperArmLen + K.forearmLen - 0.02;
  const d = Math.min(maxR, Math.max(minR, dRaw));
  const uz = dz / dRaw;
  const uy = dy / dRaw;
  const a = (K.upperArmLen * K.upperArmLen - K.forearmLen * K.forearmLen + d * d) / (2 * d);
  const h = Math.sqrt(Math.max(0, K.upperArmLen * K.upperArmLen - a * a));
  const elbowZ = shoulderZ + a * uz + h * uy;
  const elbowY = shoulderY + a * uy - h * uz;

  // 3. Stick rear lever horn driven rigidly by vertRodLen from rocker outerPin
  const horn = intersectCirclePairZY(gripZ, gripY, K.stickHornR, outerPinZ, outerPinY, L.vertRodLen, 1);
  const hornAngle = Math.atan2(horn.y - gripY, horn.z - gripZ);
  const bladeAngle = hornAngle + L.bladeRelAngle;
  const bladeZ = gripZ + K.bladeLen * Math.cos(bladeAngle);
  const bladeY = gripY + K.bladeLen * Math.sin(bladeAngle);

  // Map canonical coordinates back to world (+facingSign = +1 rear, -1 front)
  const mapZ = (z: number) => facingSign * z;

  return {
    sideX,
    rockerPivotZ: mapZ(pivZ),
    rockerPivotY: pivY,
    rockerAngleRad: facingSign * alpha,
    rollerZ: mapZ(rollerZ),
    rollerY,
    camLift,
    onDwellPad: camLift <= 1e-4,
    midPinZ: mapZ(midPinZ),
    midPinY,
    outerPinZ: mapZ(outerPinZ),
    outerPinY,
    waistPinZ: mapZ(waist.z),
    waistPinY: waist.y,
    hornPinZ: mapZ(horn.z),
    hornPinY: horn.y,
    torsoRodLen: L.torsoRodLen,
    vertRodLen: L.vertRodLen,
    arm: {
      hipZ: mapZ(hipZ),
      hipY,
      torsoLeanRad: facingSign * lean,
      shoulderZ: mapZ(shoulderZ),
      shoulderY,
      elbowZ: mapZ(elbowZ),
      elbowY,
      handleZ: mapZ(gripZ),
      handleY: gripY,
    },
    gripZ: mapZ(gripZ),
    gripY,
    bladeZ: mapZ(bladeZ),
    bladeY,
    strike01: camLift / K.camMaxLift,
  };
}

/**
 * Solves opposing tabletop ice-hockey player strikes via physical shuttle-mounted cam lobes,
 * under-ice roller seesaw rockers, and rigid vertical rods driving each player's torso waist
 * horn and hockey stick rear lever horn with zero puck clipping.
 */
export function solveTopperHockeyLinkage(sliderZ: number): TopperHockeySolution {
  const clampedZ = Math.max(-0.38, Math.min(0.38, sliderZ));
  return {
    puckZ: clampedZ,
    puckSpinRad: clampedZ * 6.5,
    rear: solveSingleHockeyPlayer(1, clampedZ),
    front: solveSingleHockeyPlayer(-1, clampedZ),
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
  const clampedZ = Math.max(-1.2, Math.min(1.2, pivotZ));
  const safeZ = Math.abs(clampedZ) < 0.55 ? (clampedZ >= 0 ? 0.55 : -0.55) : clampedZ;
  const clampedY = Math.max(0.65, Math.min(1.3, pivotY));
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
    } else if (module.type === 'scotch-yoke') {
      const sol = solveScotchYoke(theta, module);
      values.push(sol.normalizedSlide);
    } else if (module.type === 'bell-crank') {
      const sol = solveBellCrank(theta, module);
      values.push(sol.open01);
    } else if (module.type === 'digital-numbers') {
      const sol = solveDigitalNumbers(theta, module);
      const activeSum = sol.segments.reduce((acc, seg) => acc + seg.on01, 0);
      values.push(activeSum / sol.segments.length);
    } else if (module.type === 'writing-cam') {
      const sol = solveWritingCam(theta, module);
      values.push(0.55 * sol.paperU + 0.45 * sol.paperV);
    }
  }
  return values;
}

export const BELL_CRANK_KINEMATICS = {
  centerDist: 0.62,
  pitmanLen: 0.64,
  fulcrumX: 0.46,
  fulcrumY: 1.62,
  crossheadEarX: 0.14,
  inArmLen: 0.3,
  couplerLen: 0.39,
  outArmLen: 0.3,
  deckRailY: 2.0,
  topLinkLen: 0.24,
  minCrankRadius: 0.1,
  maxCrankRadius: 0.16,
  sliderMinBoundX: 0.11,
  sliderMaxBoundX: 0.455,
} as const;

function intersectCirclePair2D(
  c1x: number,
  c1y: number,
  r1: number,
  c2x: number,
  c2y: number,
  r2: number,
  pickBranch: 1 | -1
): { x: number; y: number } {
  const dx = c2x - c1x;
  const dy = c2y - c1y;
  const dRaw = Math.max(1e-5, Math.hypot(dx, dy));
  const clamped = Math.min(r1 + r2 - 1e-5, Math.max(Math.abs(r1 - r2) + 1e-5, dRaw));
  const a = (r1 * r1 - r2 * r2 + clamped * clamped) / (2 * clamped);
  const h = Math.sqrt(Math.max(0, r1 * r1 - a * a));
  const ux = dx / dRaw;
  const uy = dy / dRaw;
  return {
    x: c1x + a * ux + pickBranch * h * uy,
    y: c1y + a * uy - pickBranch * h * ux,
  };
}

export function solveBellCrankGeometry(
  crankRadius: number,
  gearRatio: number
): {
  crankCenter: { z: number; y: number };
  crankRadius: number;
  gearRatio: number;
  driverTeeth: number;
  drivenTeeth: number;
  driverPitchR: number;
  drivenPitchR: number;
  minSliderX: number;
  maxSliderX: number;
} {
  const K = BELL_CRANK_KINEMATICS;
  const safeR = Math.max(K.minCrankRadius, Math.min(K.maxCrankRadius, crankRadius));
  const safeRatio = Math.max(0.75, Math.min(1.5, gearRatio));
  const driverTeeth = 16;
  const drivenTeeth = Math.max(10, Math.round(driverTeeth / safeRatio));
  const unitR = K.centerDist / (driverTeeth + drivenTeeth);
  const driverPitchR = driverTeeth * unitR;
  const drivenPitchR = drivenTeeth * unitR;

  const evalSliderAtCrossheadY = (cy: number): number => {
    const leftInPin = intersectCirclePair2D(
      -K.fulcrumX,
      K.fulcrumY,
      K.inArmLen,
      -K.crossheadEarX,
      cy,
      K.couplerLen,
      -1
    );
    const alpha = Math.atan2(leftInPin.y - K.fulcrumY, leftInPin.x - -K.fulcrumX);
    const outAngle = alpha + Math.PI * 0.5;
    const leftOutPinX = -K.fulcrumX + K.outArmLen * Math.cos(outAngle);
    const leftOutPinY = K.fulcrumY + K.outArmLen * Math.sin(outAngle);
    const dy = K.deckRailY - leftOutPinY;
    const dx = Math.sqrt(Math.max(1e-5, K.topLinkLen * K.topLinkLen - dy * dy));
    return Math.abs(leftOutPinX + dx);
  };

  const minCrossheadY = K.centerDist - safeR + K.pitmanLen;
  const maxCrossheadY = K.centerDist + safeR + K.pitmanLen;
  const minSliderX = evalSliderAtCrossheadY(minCrossheadY);
  const maxSliderX = evalSliderAtCrossheadY(maxCrossheadY);

  return {
    crankCenter: { z: 0, y: K.centerDist },
    crankRadius: safeR,
    gearRatio: driverTeeth / drivenTeeth,
    driverTeeth,
    drivenTeeth,
    driverPitchR,
    drivenPitchR,
    minSliderX,
    maxSliderX,
  };
}

/**
 * Solves the Twin 90-degree Bell-Crank / Clamshell Symmetric Opening & Closing Mechanism:
 * Main shaft gear -> overhead offset crank gear -> pitman rod -> guided vertical crosshead ->
 * symmetric coupler links -> Left & Right 90-degree L-shaped bell-cranks -> opposing horizontal deck sliders.
 */
export function solveBellCrank(
  shaftAngleRad: number,
  module: Pick<BellCrankModule, 'phaseDeg' | 'crankRadius' | 'gearRatio'>
): BellCrankSolution {
  const K = BELL_CRANK_KINEMATICS;
  const geo = solveBellCrankGeometry(module.crankRadius, module.gearRatio);
  const driverAngleRad = shaftAngleRad + degToRad(module.phaseDeg);
  const crankAngleRad = -driverAngleRad * geo.gearRatio;

  const pinZ = geo.crankRadius * Math.sin(crankAngleRad);
  const pinY = geo.crankCenter.y + geo.crankRadius * Math.cos(crankAngleRad);
  const crossheadY = pinY + Math.sqrt(Math.max(1e-5, K.pitmanLen * K.pitmanLen - pinZ * pinZ));

  const leftFulcrum = { x: -K.fulcrumX, y: K.fulcrumY };
  const rightFulcrum = { x: K.fulcrumX, y: K.fulcrumY };

  const leftInPin = intersectCirclePair2D(
    leftFulcrum.x,
    leftFulcrum.y,
    K.inArmLen,
    -K.crossheadEarX,
    crossheadY,
    K.couplerLen,
    -1
  );
  const rightInPin = { x: -leftInPin.x, y: leftInPin.y };

  const alphaL = Math.atan2(leftInPin.y - leftFulcrum.y, leftInPin.x - leftFulcrum.x);
  const outAngleL = alphaL + Math.PI * 0.5;
  const leftOutPin = {
    x: leftFulcrum.x + K.outArmLen * Math.cos(outAngleL),
    y: leftFulcrum.y + K.outArmLen * Math.sin(outAngleL),
  };
  const rightOutPin = { x: -leftOutPin.x, y: leftOutPin.y };

  const dy = K.deckRailY - leftOutPin.y;
  const dx = Math.sqrt(Math.max(1e-5, K.topLinkLen * K.topLinkLen - dy * dy));
  const sliderX = Math.abs(leftOutPin.x + dx);

  const span = Math.max(1e-4, geo.maxSliderX - geo.minSliderX);
  const open01 = Math.max(0, Math.min(1, (sliderX - geo.minSliderX) / span));

  return {
    driverAngleRad,
    crankAngleRad,
    crankCenter: geo.crankCenter,
    crankPin: { z: pinZ, y: pinY },
    crossheadY,
    leftFulcrum,
    rightFulcrum,
    leftInPin,
    rightInPin,
    leftOutPin,
    rightOutPin,
    sliderX,
    open01,
    driverTeeth: geo.driverTeeth,
    drivenTeeth: geo.drivenTeeth,
    driverPitchR: geo.driverPitchR,
    drivenPitchR: geo.drivenPitchR,
  };
}

export const DOUBLE_DOORS_KINEMATICS = {
  pivotX: 0.51,
  pivotZ: -0.195,
  linkPivotX: 0.568,
  linkPivotZ: -0.164,
  doorClosedOuterX: 0.386,
  doorClosedZ: -0.226,
  doorLugX: 0.496,
  doorLugZ: -0.241,
  jambFrontZ: -0.245,
  baseOuterX: 0.46,
  doorWidth: 0.384,
  doorThick: 0.028,
  hornR: 0.24,
  linkLen: 0.28,
  pinY: 0.04,
  sillTopY: 0.125,
  birdToggleLen: 0.465,
  birdRodY: 0.072,
  birdPerchHeight: 0.24,
  birdNeckOffsetZ: 0.045,
  birdBeakTipOffset: 0.12,
} as const;

const DOUBLE_DOORS_CAL = (() => {
  const K = DOUBLE_DOORS_KINEMATICS;
  const minS = BELL_CRANK_KINEMATICS.sliderMinBoundX;
  const horn0 = intersectCirclePair2D(-K.pivotX, K.pivotZ, K.hornR, -minS, 0, K.linkLen, -1);
  const dx1 = K.doorClosedOuterX - K.pivotX;
  const dz1 = K.doorClosedZ - K.pivotZ;
  const dx2 = K.doorLugX - K.doorClosedOuterX;
  const dz2 = K.doorLugZ - K.doorClosedZ;
  const lugDist = Math.hypot(dx2, dz2);
  const coupledArmLen = Math.hypot(K.doorLugX - K.linkPivotX, K.doorLugZ - K.linkPivotZ);
  const candPos = intersectCirclePair2D(
    K.doorClosedOuterX,
    K.doorClosedZ,
    lugDist,
    K.linkPivotX,
    K.linkPivotZ,
    coupledArmLen,
    1
  );
  const errPos = Math.hypot(candPos.x - K.doorLugX, candPos.y - K.doorLugZ);
  return {
    hornAngleAtClosed: Math.atan2(horn0.y - K.pivotZ, horn0.x - -K.pivotX),
    dx1,
    dz1,
    armLen: Math.hypot(dx1, dz1),
    lugDist,
    coupledArmLen,
    lugBranchSign: (errPos < 1e-4 ? 1 : -1) as 1 | -1,
    baseDoorDirAngle: Math.atan2(dz2, dx2),
  };
})();

export interface TopperDoubleDoorsSolution {
  sliderX: number;
  armSwingRad: number;
  doorOpenRad: number;
  open01: number;
  leftHornX: number;
  leftHornZ: number;
  rightHornX: number;
  rightHornZ: number;
  leftHingeX: number;
  leftHingeZ: number;
  rightHingeX: number;
  rightHingeZ: number;
  leftLugX: number;
  leftLugZ: number;
  rightLugX: number;
  rightLugZ: number;
  birdCarriageZ: number;
  birdPitchRad: number;
  birdHeadZ: number;
  birdBeakZ: number;
}

/**
 * Solves symmetric XY/XZ closed-chain offset-pivot 4-bar door linkage + central V-toggle cuckoo-bird
 * pop-out carriage driven by Twin Bell-Crank deck sliders (+-sliderX, z=0).
 * Both primary rotation pivots (+-0.510, -0.195) and coupled swing-link pivots (+-0.568, -0.164) are offset
 * strictly outboard of the door edges (+-0.386) and chalet base sides (+-0.460) so the main swing arm
 * and coupled swing link swing each door outward and forward (0..93 deg) with zero side/jamb collision.
 */
export function solveTopperDoubleDoorsLinkage(sliderXInput: number): TopperDoubleDoorsSolution {
  const K = DOUBLE_DOORS_KINEMATICS;
  const B = BELL_CRANK_KINEMATICS;
  const sliderX = Math.max(B.sliderMinBoundX, Math.min(B.sliderMaxBoundX, sliderXInput));
  const hornL = intersectCirclePair2D(-K.pivotX, K.pivotZ, K.hornR, -sliderX, 0, K.linkLen, -1);
  const hornAngle = Math.atan2(hornL.y - K.pivotZ, hornL.x - -K.pivotX);
  let armSwingRad = hornAngle - DOUBLE_DOORS_CAL.hornAngleAtClosed;
  if (armSwingRad < -Math.PI) armSwingRad += Math.PI * 2;
  if (armSwingRad > Math.PI) armSwingRad -= Math.PI * 2;
  armSwingRad = Math.max(0, armSwingRad);

  const ca = Math.cos(armSwingRad);
  const sa = Math.sin(armSwingRad);
  const rightHingeX = K.pivotX + DOUBLE_DOORS_CAL.dx1 * ca + DOUBLE_DOORS_CAL.dz1 * sa;
  const rightHingeZ = K.pivotZ - DOUBLE_DOORS_CAL.dx1 * sa + DOUBLE_DOORS_CAL.dz1 * ca;

  const lugR = intersectCirclePair2D(
    rightHingeX,
    rightHingeZ,
    DOUBLE_DOORS_CAL.lugDist,
    K.linkPivotX,
    K.linkPivotZ,
    DOUBLE_DOORS_CAL.coupledArmLen,
    DOUBLE_DOORS_CAL.lugBranchSign
  );
  const curDoorDirAngle = Math.atan2(lugR.y - rightHingeZ, lugR.x - rightHingeX);
  let doorDelta = -(curDoorDirAngle - DOUBLE_DOORS_CAL.baseDoorDirAngle);
  if (doorDelta < -Math.PI) doorDelta += Math.PI * 2;
  if (doorDelta > Math.PI) doorDelta -= Math.PI * 2;
  const doorOpenRad = Math.max(0, doorDelta);

  const open01 = Math.max(
    0,
    Math.min(1, (sliderX - B.sliderMinBoundX) / (B.sliderMaxBoundX - B.sliderMinBoundX))
  );

  // Rigid V-toggle linkage: sliderX^2 + birdCarriageZ^2 = birdToggleLen^2
  const birdCarriageZ = -Math.sqrt(Math.max(1e-4, K.birdToggleLen * K.birdToggleLen - sliderX * sliderX));
  const birdPitchRad = open01 * 0.48;
  const birdCenterZ =
    birdCarriageZ + K.birdNeckOffsetZ + K.birdPerchHeight * Math.sin(birdPitchRad);
  const birdHeadZ = birdCenterZ + 0.064 * Math.cos(birdPitchRad);
  const birdBeakZ = birdCenterZ + K.birdBeakTipOffset * Math.cos(birdPitchRad);

  return {
    sliderX,
    armSwingRad,
    doorOpenRad,
    open01,
    leftHornX: hornL.x,
    leftHornZ: hornL.y,
    rightHornX: -hornL.x,
    rightHornZ: hornL.y,
    leftHingeX: -rightHingeX,
    leftHingeZ: rightHingeZ,
    rightHingeX,
    rightHingeZ,
    leftLugX: -lugR.x,
    leftLugZ: lugR.y,
    rightLugX: lugR.x,
    rightLugZ: lugR.y,
    birdCarriageZ,
    birdPitchRad,
    birdHeadZ,
    birdBeakZ,
  };
}

export const CLAPPING_KINEMATICS = {
  hipX: 0.54,
  hipY: 0.36,
  shoulderY: 0.68,
  armHornR: 0.16,
  armRodLen: 0.65,
  pinY: 0.05,
  armLen: 0.515,
  palmHalfThick: 0.014,
  clapRestAngle: -0.06,
  rodPlaneZ: -0.145,
} as const;

const CLAPPING_CAL = (() => {
  const K = CLAPPING_KINEMATICS;
  const minS = BELL_CRANK_KINEMATICS.sliderMinBoundX;
  const horn0 = intersectCirclePair2D(
    -K.hipX,
    K.shoulderY,
    K.armHornR,
    -minS,
    K.pinY,
    K.armRodLen,
    1
  );
  return {
    hornAngleAtClap: Math.atan2(horn0.y - K.shoulderY, horn0.x - -K.hipX),
  };
})();

export interface ClappingPuppetBranch {
  shoulderX: number;
  shoulderY: number;
  hornX: number;
  hornY: number;
  wristX: number;
  wristY: number;
  armAngleRad: number;
  headNodRad: number;
}

export interface TopperClappingSolution {
  sliderX: number;
  clap01: number;
  left: ClappingPuppetBranch;
  right: ClappingPuppetBranch;
}

/**
 * Solves symmetric twin clapping wooden puppets driven directly by Twin Bell-Crank deck sliders (+-sliderX, pinY).
 * Pushrods and crank horns run cleanly in rear plane z = rodPlaneZ (-0.145), strictly behind legs/torso/arms.
 * Moving sliders inward swings both pairs of arms upward to clap palms face-to-face at x=0 with zero overlap.
 */
export function solveTopperClappingLinkage(sliderXInput: number): TopperClappingSolution {
  const K = CLAPPING_KINEMATICS;
  const B = BELL_CRANK_KINEMATICS;
  const sliderX = Math.max(B.sliderMinBoundX, Math.min(B.sliderMaxBoundX, sliderXInput));

  const hornL = intersectCirclePair2D(
    -K.hipX,
    K.shoulderY,
    K.armHornR,
    -sliderX,
    K.pinY,
    K.armRodLen,
    1
  );
  const hornAngle = Math.atan2(hornL.y - K.shoulderY, hornL.x - -K.hipX);
  const armAngle = hornAngle - CLAPPING_CAL.hornAngleAtClap + K.clapRestAngle;

  const wristXL = -K.hipX + K.armLen * Math.cos(armAngle);
  const wristYL = K.shoulderY + K.armLen * Math.sin(armAngle);

  const clap01 = Math.max(
    0,
    Math.min(1, 1 - (sliderX - B.sliderMinBoundX) / (B.sliderMaxBoundX - B.sliderMinBoundX))
  );
  const headNod = clap01 * 0.16;

  return {
    sliderX,
    clap01,
    left: {
      shoulderX: -K.hipX,
      shoulderY: K.shoulderY,
      hornX: hornL.x,
      hornY: hornL.y,
      wristX: wristXL,
      wristY: wristYL,
      armAngleRad: armAngle,
      headNodRad: headNod,
    },
    right: {
      shoulderX: K.hipX,
      shoulderY: K.shoulderY,
      hornX: -hornL.x,
      hornY: hornL.y,
      wristX: -wristXL,
      wristY: wristYL,
      armAngleRad: Math.PI - armAngle,
      headNodRad: headNod,
    },
  };
}

export const DUCK_EGG_KINEMATICS = {
  hipX: 0.21,
  hipY: 0.64,
  rockerPivotX: 0.19,
  rockerPivotY: 0.48,
  rockerArmLen: 0.24,
  pitmanLen: 0.2,
  eggPivotX: -0.2,
  eggPivotY: 0.2,
  eggArmLen: 0.3,
  eggLugRadius: 0.14,
  eggStartRad: (82 * Math.PI) / 180,
  eggEndRad: (168 * Math.PI) / 180,
  eggRadiusX: 0.078,
  eggRadiusZ: 0.06,
  flapPivotLocalX: -0.47,
  flapPivotLocalY: 0.04,
  flapLength: 0.2,
  flapOpenMaxRad: (78 * Math.PI) / 180,
  wingMaxRad: 0.68,
  lowerBillMaxRad: 0.44,
  nestCenterX: -0.493,
  nestFloorY: 0.125,
  minPinY: 0.12,
  travelFullPinY: 0.56,
  maxPinY: 0.72,
} as const;

function smoothstep01(edge0: number, edge1: number, x: number): number {
  const t = Math.max(0, Math.min(1, (x - edge0) / Math.max(1e-6, edge1 - edge0)));
  return t * t * (3 - 2 * t);
}

const DUCK_EGG_CAL = (() => {
  const K = DUCK_EGG_KINEMATICS;
  const evalRockerAngle = (pinY: number): number => {
    const p = intersectCirclePair2D(
      K.rockerPivotX,
      K.rockerPivotY,
      K.rockerArmLen,
      0,
      pinY,
      K.pitmanLen,
      1
    );
    return Math.atan2(p.y - K.rockerPivotY, -(p.x - K.rockerPivotX));
  };
  const angAtMin = evalRockerAngle(K.minPinY);
  const angAtFull = evalRockerAngle(K.travelFullPinY);
  return {
    angAtMin,
    angSpan: Math.max(1e-4, angAtFull - angAtMin),
  };
})();

export interface TopperDuckEggSolution {
  pinY: number;
  rockerPinX: number;
  rockerPinY: number;
  rockerAngleRad: number;
  travel: number;
  flap01: number;
  egg01: number;
  quack01: number;
  flapOpenRad: number;
  torsoPitchRad: number;
  neckNodRad: number;
  lowerBillOpenRad: number;
  wingFlareRad: number;
  eggAngleRad: number;
  eggCenterX: number;
  eggCenterY: number;
  eggLugX: number;
  eggLugY: number;
}

/**
 * Solves Kazuaki Harada's "Egg-Laying Duck" (Decoy) closed-chain pushrod-rocker topper:
 * 1) Guided vertical pushrod clevis (0, pinY) drives the overhead chest rocker lever
 *    around (rockerPivotX, rockerPivotY) with exact rigid pitman & rocker lengths;
 * 2) Rising rocker travel sequentially swings the rear belly trapdoor open (flap01)
 *    strictly before releasing the wooden egg (egg01) along the slotted chute into the nest;
 * 3) Once the egg lands in the nest, the duck puffs its chest, flares both wings, and opens
 *    its hinged lower bill in a triumphant quack (quack01) until the snail drop cam snaps shut.
 */
export function solveTopperDuckEggLinkage(pinYInput: number): TopperDuckEggSolution {
  const K = DUCK_EGG_KINEMATICS;
  const pinY = Math.max(K.minPinY, Math.min(K.maxPinY, pinYInput));

  const p = intersectCirclePair2D(
    K.rockerPivotX,
    K.rockerPivotY,
    K.rockerArmLen,
    0,
    pinY,
    K.pitmanLen,
    1
  );
  const rockerAngleRad = Math.atan2(p.y - K.rockerPivotY, -(p.x - K.rockerPivotX));
  const travel = Math.max(0, Math.min(1.28, (rockerAngleRad - DUCK_EGG_CAL.angAtMin) / DUCK_EGG_CAL.angSpan));

  const flap01 = smoothstep01(0.05, 0.42, travel);
  const egg01 = smoothstep01(0.38, 0.86, travel);
  const quack01 = smoothstep01(0.72, 1.02, travel);

  const flapOpenRad = flap01 * K.flapOpenMaxRad;
  // Strain posture lifts rear tail slightly during laying, then puffs chest up proudly during quack
  const strainWave = Math.sin(Math.min(1, travel / 0.82) * Math.PI);
  const torsoPitchRad = -0.11 * strainWave + 0.13 * quack01;
  const neckNodRad = 0.08 * strainWave - 0.16 * quack01;
  const lowerBillOpenRad = quack01 * K.lowerBillMaxRad;
  const wingFlareRad = 0.02 + 0.36 * strainWave + 0.62 * quack01;

  const eggAngleRad = K.eggStartRad + egg01 * (K.eggEndRad - K.eggStartRad);
  const eggCenterX = K.eggPivotX + K.eggArmLen * Math.cos(eggAngleRad);
  const eggCenterY = K.eggPivotY + K.eggArmLen * Math.sin(eggAngleRad);
  const lugAngle = eggAngleRad - 0.55;
  const eggLugX = K.eggPivotX + K.eggLugRadius * Math.cos(lugAngle);
  const eggLugY = K.eggPivotY + K.eggLugRadius * Math.sin(lugAngle);

  return {
    pinY,
    rockerPinX: p.x,
    rockerPinY: p.y,
    rockerAngleRad,
    travel,
    flap01,
    egg01,
    quack01,
    flapOpenRad,
    torsoPitchRad,
    neckNodRad,
    lowerBillOpenRad,
    wingFlareRad,
    eggAngleRad,
    eggCenterX,
    eggCenterY,
    eggLugX,
    eggLugY,
  };
}

export const RATTLE_DRUM_KINEMATICS = {
  drumCenterY: 0.78,
  drumRadius: 0.24,
  drumHalfDepth: 0.068,
  eyeletOffsetX: 0.256,
  cordLength: 0.245,
  beadRadius: 0.046,
} as const;

export interface RattleDrumCordSolution {
  eyelet: { x: number; y: number; z: number };
  elbow: { x: number; y: number; z: number };
  bead: { x: number; y: number; z: number };
  totalCordLength: number;
  strike01: number;
  strikeSide: 1 | -1;
}

export interface TopperRattleDrumSolution {
  drumAngleRad: number;
  swingNorm: number;
  centrifugal01: number;
  leftCord: RattleDrumCordSolution;
  rightCord: RattleDrumCordSolution;
  frontStrikePulse01: number;
  backStrikePulse01: number;
}

/**
 * Solves a traditional pellet drum (拨浪鼓) mounted on a reciprocating rotary spindle:
 * - Twin cords attach to +/-X brass waist eyelets outside the crimson drum barrel.
 * - Angular acceleration (alphaNorm) and rotational speed (omegaNorm) lag & whip each cord
 *   symmetrically across opposite Z faces so Left & Right beads strike opposite parchment
 *   drumheads on each half-stroke while wrapping cleanly around the barrel rim with zero clipping.
 */
export function solveTopperRattleDrum(
  drumAngleRad: number,
  omegaNorm: number,
  alphaNorm: number
): TopperRattleDrumSolution {
  const K = RATTLE_DRUM_KINEMATICS;
  // Phase-lagged torsional response: inertia lags opposite to acceleration and trails spin
  const rawSwing = -0.95 * alphaNorm + 0.55 * omegaNorm;
  const swingNorm = Math.max(-1, Math.min(1, rawSwing / 0.82));
  const centrifugal01 = Math.max(0, Math.min(1, Math.abs(omegaNorm) / 0.85));

  const solveSideCord = (side: -1 | 1): RattleDrumCordSolution => {
    // Opposite sides (+X vs -X) whip into opposite Z faces under the same Y-axis drum rotation
    const u = side * swingNorm;
    const absU = Math.min(1, Math.abs(u));
    const sgnZ: 1 | -1 = u >= 0 ? 1 : -1;
    const wrap01 = smoothstep01(0.24, 0.84, absU);
    const flare01 = Math.max(centrifugal01 * 0.96, Math.min(1, absU * 1.35));

    const dx = side * (0.24 + 0.78 * flare01);
    const dy = -(0.96 - 0.86 * flare01);
    const dz = u * 1.45;
    const dl = Math.max(1e-5, Math.hypot(dx, dy, dz));
    const ux = dx / dl;
    const uy = dy / dl;
    const uz = dz / dl;

    const p0x = side * K.eyeletOffsetX;
    const p0y = K.drumCenterY;
    const p0z = 0;

    const p1FlareX = p0x + ux * K.cordLength * 0.42;
    const p1FlareY = p0y + uy * K.cordLength * 0.42;
    const p1FlareZ = p0z + uz * K.cordLength * 0.42;

    const p2FlareX = p0x + ux * K.cordLength;
    const p2FlareY = p0y + uy * K.cordLength;
    const p2FlareZ = p0z + uz * K.cordLength;

    const pCornerX = side * 0.274;
    const pCornerY = K.drumCenterY - 0.005;
    const pCornerZ = sgnZ * 0.096;

    const pStrikeX = side * 0.128;
    const pStrikeY = K.drumCenterY - 0.01;
    const pStrikeZ = sgnZ * (K.drumHalfDepth + K.beadRadius + 0.002);

    const p1x = p1FlareX + (pCornerX - p1FlareX) * wrap01;
    const p1y = p1FlareY + (pCornerY - p1FlareY) * wrap01;
    const p1z = p1FlareZ + (pCornerZ - p1FlareZ) * wrap01;

    const goalX = p2FlareX + (pStrikeX - p2FlareX) * wrap01;
    const goalY = p2FlareY + (pStrikeY - p2FlareY) * wrap01;
    const goalZ = p2FlareZ + (pStrikeZ - p2FlareZ) * wrap01;

    const seg1 = Math.hypot(p1x - p0x, p1y - p0y, p1z - p0z);
    const rem = Math.max(0.02, K.cordLength - seg1);
    const gx = goalX - p1x;
    const gy = goalY - p1y;
    const gz = goalZ - p1z;
    const gl = Math.max(1e-5, Math.hypot(gx, gy, gz));

    let p2x = p1x + (gx / gl) * rem;
    let p2y = p1y + (gy / gl) * rem;
    let p2z = p1z + (gz / gl) * rem;

    // Guarantee bead sphere clears drum cylinder & parchment face tangentially
    const rFace = Math.hypot(p2x, p2y - K.drumCenterY);
    const minZ = K.drumHalfDepth + K.beadRadius;
    if (rFace < K.drumRadius + K.beadRadius - 1e-3 && Math.abs(p2z) < minZ) {
      p2z = sgnZ * minZ;
      const dzRem = p2z - p1z;
      const xyRem = Math.sqrt(Math.max(1e-6, rem * rem - dzRem * dzRem));
      const dxy = Math.max(1e-5, Math.hypot(p2x - p1x, p2y - p1y));
      p2x = p1x + ((p2x - p1x) / dxy) * xyRem;
      p2y = p1y + ((p2y - p1y) / dxy) * xyRem;
    }

    const seg2 = Math.hypot(p2x - p1x, p2y - p1y, p2z - p1z);
    const strike01 = smoothstep01(0.84, 0.98, absU);

    return {
      eyelet: { x: p0x, y: p0y, z: p0z },
      elbow: { x: p1x, y: p1y, z: p1z },
      bead: { x: p2x, y: p2y, z: p2z },
      totalCordLength: seg1 + seg2,
      strike01,
      strikeSide: sgnZ,
    };
  };

  const leftCord = solveSideCord(-1);
  const rightCord = solveSideCord(1);
  const frontStrikePulse01 = Math.max(
    leftCord.strikeSide === 1 ? leftCord.strike01 : 0,
    rightCord.strikeSide === 1 ? rightCord.strike01 : 0
  );
  const backStrikePulse01 = Math.max(
    leftCord.strikeSide === -1 ? leftCord.strike01 : 0,
    rightCord.strikeSide === -1 ? rightCord.strike01 : 0
  );

  return {
    drumAngleRad,
    swingNorm,
    centrifugal01,
    leftCord,
    rightCord,
    frontStrikePulse01,
    backStrikePulse01,
  };
}

export const DONKEY_MILL_KINEMATICS = {
  millRadius: 0.28,
  millHeight: 0.16,
  runnerRadius: 0.21,
  runnerHeight: 0.12,
  orbitRadius: 0.58,
  hitchBoomLen: 0.58,
  donkeyHipY: 0.30,
  shoulderZ: 0.125,
  hipZ: -0.125,
  legHalfSpanX: 0.056,
  hipJointOffsetY: 0.02,
  thighLen: 0.125,
  shankLen: 0.125,
  skateDrop: 0.036,
  wheelRadius: 0.0145,
  wheelAxleHalfSpanZ: 0.022,
  wheelCenterOffsetY: 0.0215,
  courtyardTopY: 0.032,
  deckTopY: 0.036,
  gaitStepsPerRev: 4,
  strideHalfZ: 0.068,
  liftHeight: 0.024,
  glideDuty: 0.6,
} as const;

export interface DonkeyLegSolution {
  thighAngleRad: number;
  kneeAngleRad: number;
  thighLocalRotX: number;
  shankLocalRotX: number;
  skateLocalRotX: number;
  skatePitchRad: number;
  wheelSpinRad: number;
  isGliding: boolean;
  hip: { x: number; y: number; z: number };
  knee: { x: number; y: number; z: number };
  hoof: { x: number; y: number; z: number };
  ankle: { x: number; y: number; z: number };
  groundClearance: number;
}

export interface TopperDonkeyMillSolution {
  millAngleRad: number;
  gaitPhaseRad: number;
  donkeyOrbitAngleRad: number;
  donkeyPos: { x: number; y: number; z: number };
  donkeyFacingAngleRad: number;
  frontLeftLeg: DonkeyLegSolution;
  frontRightLeg: DonkeyLegSolution;
  rearLeftLeg: DonkeyLegSolution;
  rearRightLeg: DonkeyLegSolution;
  headBobRad: number;
  earFlutterRad: number;
  tailSwishRad: number;
  torsoHeaveY: number;
  torsoPitchRad: number;
  hitchTipWorld: { x: number; y: number; z: number };
}

/**
 * Solves a traditional folk Donkey Grinding Mill (毛驴拉磨) rotary topper with
 * vintage brass-and-wood roller skates on all four hooves:
 * - A stationary courtyard plinth & lower bedstone rest on the deck with an inlaid
 *   circular brass skating track at orbitRadius = 0.58 (deckTopY = 0.036);
 * - The vertical spindle turns the upper runner stone and overhead hitch boom;
 * - A blindfolded wooden donkey wears twin-wheel brass/wood roller skates on each hoof
 *   and glides smoothly in an alternating diagonal skating stride:
 *   each skate rolls flush along the brass track during its glide stroke (groundClearance == 0)
 *   and lifts cleanly on its recovery swing while all 8 roller wheels spin forward smoothly.
 */
export function solveTopperDonkeyMill(continuousAngleRad: number): TopperDonkeyMillSolution {
  const K = DONKEY_MILL_KINEMATICS;
  const millAngleRad = continuousAngleRad;
  const gaitPhaseRad = continuousAngleRad * K.gaitStepsPerRev;

  const torsoHeaveY = 0.004 * Math.cos(2 * gaitPhaseRad);
  const torsoPitchRad = 0.018 * Math.sin(gaitPhaseRad);
  const headBobRad = 0.065 * Math.sin(gaitPhaseRad);
  const earFlutterRad = 0.11 * Math.cos(gaitPhaseRad);
  const tailSwishRad = 0.16 * Math.sin(gaitPhaseRad);

  const ca = Math.cos(torsoPitchRad);
  const sa = Math.sin(torsoPitchRad);

  const solveLeg = (
    baseX: number,
    baseZ: number,
    legPhase: number,
    kneeDirSign: 1 | -1
  ): DonkeyLegSolution => {
    const wrapped = (((legPhase / (Math.PI * 2)) % 1) + 1) % 1;
    const isGliding = wrapped < K.glideDuty;

    let zOffset: number;
    let lift: number;
    let skatePitchRad: number;

    if (isGliding) {
      const s = wrapped / K.glideDuty;
      zOffset = K.strideHalfZ * Math.cos(Math.PI * s);
      lift = 0;
      skatePitchRad = 0;
    } else {
      const s = (wrapped - K.glideDuty) / (1 - K.glideDuty);
      zOffset = -K.strideHalfZ * Math.cos(Math.PI * s);
      const sinLift = Math.sin(Math.PI * s);
      lift = K.liftHeight * sinLift * sinLift;
      skatePitchRad = 0.16 * sinLift * sinLift;
    }

    // Exact hip pin position in donkeyGroup space from torsoPivot transform
    const localHipY = -K.hipJointOffsetY;
    const hip = {
      x: baseX,
      y: K.donkeyHipY + torsoHeaveY + (localHipY * ca - baseZ * sa),
      z: localHipY * sa + baseZ * ca,
    };

    const ankle = {
      x: baseX,
      y: K.deckTopY + K.skateDrop + lift,
      z: baseZ + zOffset,
    };

    const dy = ankle.y - hip.y;
    const dz = ankle.z - hip.z;
    const dRaw = Math.max(1e-5, Math.hypot(dy, dz));
    const minD = Math.abs(K.thighLen - K.shankLen) + 0.01;
    const maxD = K.thighLen + K.shankLen - 0.006;
    const d = Math.min(maxD, Math.max(minD, dRaw));
    const uy = dy / dRaw;
    const uz = dz / dRaw;

    const a = (K.thighLen * K.thighLen - K.shankLen * K.shankLen + d * d) / (2 * d);
    const h = Math.sqrt(Math.max(0, K.thighLen * K.thighLen - a * a));

    const knee = {
      x: hip.x,
      y: hip.y + a * uy + kneeDirSign * h * uz,
      z: hip.z + a * uz - kneeDirSign * h * uy,
    };

    const thighWorldAngle = Math.atan2(knee.z - hip.z, -(knee.y - hip.y));
    const shankWorldAngle = Math.atan2(ankle.z - knee.z, -(ankle.y - knee.y));

    const thighLocalRotX = -thighWorldAngle - torsoPitchRad;
    const shankLocalRotX = thighWorldAngle - shankWorldAngle;
    const skateLocalRotX = shankWorldAngle - skatePitchRad;

    const cp = Math.cos(skatePitchRad);
    const sp = Math.sin(skatePitchRad);
    const frontWheelBottomY =
      ankle.y - K.wheelCenterOffsetY * cp + K.wheelAxleHalfSpanZ * sp - K.wheelRadius;
    const rearWheelBottomY =
      ankle.y - K.wheelCenterOffsetY * cp - K.wheelAxleHalfSpanZ * sp - K.wheelRadius;
    const lowestWheelY = Math.min(frontWheelBottomY, rearWheelBottomY);

    const wheelSpinRad = (K.orbitRadius * millAngleRad + zOffset) / K.wheelRadius;

    return {
      thighAngleRad: thighWorldAngle,
      kneeAngleRad: thighWorldAngle - shankWorldAngle,
      thighLocalRotX,
      shankLocalRotX,
      skateLocalRotX,
      skatePitchRad,
      wheelSpinRad,
      isGliding,
      hip,
      knee,
      hoof: {
        x: ankle.x,
        y: lowestWheelY,
        z: ankle.z,
      },
      ankle,
      groundClearance: lowestWheelY - K.deckTopY,
    };
  };

  // Diagonal skating cadence: FL+RR glide while FR+RL swing forward, offset by PI
  const frontLeftLeg = solveLeg(-K.legHalfSpanX, K.shoulderZ, gaitPhaseRad, 1);
  const rearRightLeg = solveLeg(K.legHalfSpanX, K.hipZ, gaitPhaseRad, -1);
  const frontRightLeg = solveLeg(K.legHalfSpanX, K.shoulderZ, gaitPhaseRad + Math.PI, 1);
  const rearLeftLeg = solveLeg(-K.legHalfSpanX, K.hipZ, gaitPhaseRad + Math.PI, -1);

  const orbitAngle = -millAngleRad;
  const donkeyPos = {
    x: K.orbitRadius * Math.cos(orbitAngle),
    y: 0,
    z: K.orbitRadius * Math.sin(orbitAngle),
  };
  const donkeyFacingAngleRad = orbitAngle + Math.PI / 2;

  const hitchTipWorld = {
    x: K.hitchBoomLen * Math.cos(orbitAngle),
    y: 0.44,
    z: K.hitchBoomLen * Math.sin(orbitAngle),
  };

  return {
    millAngleRad,
    gaitPhaseRad,
    donkeyOrbitAngleRad: orbitAngle,
    donkeyPos,
    donkeyFacingAngleRad,
    frontLeftLeg,
    frontRightLeg,
    rearLeftLeg,
    rearRightLeg,
    headBobRad,
    earFlutterRad,
    tailSwishRad,
    torsoHeaveY,
    torsoPitchRad,
    hitchTipWorld,
  };
}

export const DIGITAL_NUMBERS_KINEMATICS = {
  slotCount: 7,
  segmentOrder: ['a', 'f', 'e', 'd', 'c', 'g', 'b'] as readonly DigitalSegmentId[],
  camSpacingX: 0.19,
  camThickness: 0.06,
  camBaseRadius: 0.42,
  camLift: 0.15,
  camBevelRadial: 0.008,
  followerFootRadius: 0.032,
  rodMinDeckY: 0.04,
  rodMaxDeckY: 0.19,
  pivotBarY: 0.35,
  pivotBarZ: 0.2,
  crankLen: 0.069,
  couplerLen: 0.308,
  plateHalfSpanX: 0.74,
  minDwellRatio: 0.4,
  maxDwellRatio: 0.7,
} as const;

/**
 * Standard 7-segment display bitmasks for digits 0..9 in slot order:
 * [0: a (top), 1: f (upper-left), 2: e (lower-left), 3: d (bottom), 4: c (lower-right), 5: g (middle), 6: b (upper-right)]
 */
export const DIGITAL_SEGMENT_PATTERNS: readonly (readonly [
  number,
  number,
  number,
  number,
  number,
  number,
  number,
])[] = [
  [1, 1, 1, 1, 1, 0, 1], // 0
  [0, 0, 0, 0, 1, 0, 1], // 1
  [1, 0, 1, 1, 0, 1, 1], // 2
  [1, 0, 0, 1, 1, 1, 1], // 3
  [0, 1, 0, 0, 1, 1, 1], // 4
  [1, 1, 0, 1, 1, 1, 0], // 5
  [1, 1, 1, 1, 1, 1, 0], // 6
  [1, 0, 0, 0, 1, 0, 1], // 7
  [1, 1, 1, 1, 1, 1, 1], // 8
  [1, 1, 0, 1, 1, 1, 1], // 9
];

const DIGITAL_CRANK_CAL = (() => {
  const K = DIGITAL_NUMBERS_KINEMATICS;
  const solveCollarPin = (rodDeckY: number): { z: number; y: number; phi: number } => {
    const p = intersectCirclePair2D(
      K.pivotBarZ,
      K.pivotBarY,
      K.crankLen,
      0,
      rodDeckY,
      K.couplerLen,
      -1
    );
    return { z: p.x, y: p.y, phi: Math.atan2(p.y - K.pivotBarY, p.x - K.pivotBarZ) };
  };
  const onState = solveCollarPin(K.rodMinDeckY);
  const offState = solveCollarPin(K.rodMaxDeckY);
  return {
    phiOn: onState.phi,
    phiOff: offState.phi,
    phiSpan: offState.phi - onState.phi,
  };
})();

/**
 * Evaluates the polar radius R(alpha) of cam slot `segIndex` (0..6) at local contour angle `alpha`,
 * where alpha = +PI/2 points straight up (+Y) at shaftAngleRad = 0.
 * Matching yh246801's lower-horn linkage: low valley keeps the pushrod down so the crank horn
 * pulls the segment upright (ON, 0 deg); high cam lobe lifts the pushrod to swing the lower
 * crank horn forward +90 deg, folding the segment arm -90 deg backward (OFF).
 */
export function evaluateDigitalNumbersCamRadius(
  segIndex: number,
  alpha: number,
  dwellRatio = 0.55,
  countDirection: 1 | -1 = 1
): number {
  const K = DIGITAL_NUMBERS_KINEMATICS;
  const safeDwell = Math.max(K.minDwellRatio, Math.min(K.maxDwellRatio, dwellRatio));
  const twoPi = Math.PI * 2;
  const sectorRad = twoPi / 10;
  const halfDwell = 0.5 * safeDwell * sectorRad;
  const transWidth = (1 - safeDwell) * sectorRad;

  // Map cam contour angle so shaftAngleRad = k * 36 deg brings sector k to +PI/2 (12 o'clock)
  const thetaAtCenter = normalizeAngleRad(Math.PI * 0.5 - alpha);
  const shifted = normalizeAngleRad(thetaAtCenter + halfDwell);
  const sectorIdx = Math.floor(shifted / sectorRad) % 10;
  const inSector = shifted - sectorIdx * sectorRad;

  const digitAt = (idx: number): number =>
    countDirection === 1 ? ((idx % 10) + 10) % 10 : ((10 - (idx % 10)) % 10 + 10) % 10;

  const vCurr = 1 - (DIGITAL_SEGMENT_PATTERNS[digitAt(sectorIdx)][segIndex] ?? 0);
  let activeVal = vCurr;

  const dwellSpan = safeDwell * sectorRad;
  if (inSector > dwellSpan) {
    const nextIdx = (sectorIdx + 1) % 10;
    const vNext = 1 - (DIGITAL_SEGMENT_PATTERNS[digitAt(nextIdx)][segIndex] ?? 0);
    const u = Math.min(1, Math.max(0, (inSector - dwellSpan) / Math.max(1e-5, transWidth)));
    const blend = 0.5 - 0.5 * Math.cos(Math.PI * u);
    activeVal = vCurr + (vNext - vCurr) * blend;
  }

  return K.camBaseRadius + K.camLift * activeVal;
}

export function sampleDigitalCamContour(
  segIndex: number,
  dwellRatio = 0.55,
  countDirection: 1 | -1 = 1,
  samples = 240
): Array<{ x: number; y: number; radius: number }> {
  const pts: Array<{ x: number; y: number; radius: number }> = [];
  for (let i = 0; i < samples; i++) {
    const alpha = (i / samples) * Math.PI * 2;
    const radius = evaluateDigitalNumbersCamRadius(segIndex, alpha, dwellRatio, countDirection);
    pts.push({
      x: Math.cos(alpha) * radius,
      y: Math.sin(alpha) * radius,
      radius,
    });
  }
  return pts;
}

/**
 * Solves all 7 coaxial cams, guided vertical pushrods, upper pivot-bar collar bell-cranks,
 * and rigid arm tilt angles (0 rad = upright ON facing +Z, -PI/2 rad = folded back horizontally OFF)
 * for the 7-Cam Digital Numbers Display (`Digital Numbers by yh246801`).
 */
export function solveDigitalNumbers(
  shaftAngleRad: number,
  module: Pick<DigitalNumbersModule, 'phaseDeg' | 'dwellRatio' | 'countDirection'>,
  precomputedContours?: Array<Array<{ x: number; y: number; radius: number }>>
): DigitalNumbersSolution {
  const K = DIGITAL_NUMBERS_KINEMATICS;
  const driverAngleRad = shaftAngleRad + degToRad(module.phaseDeg);
  const safeDwell = Math.max(K.minDwellRatio, Math.min(K.maxDwellRatio, module.dwellRatio));
  const dir: 1 | -1 = module.countDirection === -1 ? -1 : 1;

  const twoPi = Math.PI * 2;
  const sectorRad = twoPi / 10;
  const normTheta = normalizeAngleRad(driverAngleRad);
  const nearestSector = Math.round(normTheta / sectorRad) % 10;
  let deltaFromCenter = normTheta - nearestSector * sectorRad;
  if (deltaFromCenter > Math.PI) deltaFromCenter -= twoPi;
  if (deltaFromCenter < -Math.PI) deltaFromCenter += twoPi;
  const activeDigit =
    dir === 1 ? nearestSector : ((10 - nearestSector) % 10 + 10) % 10;
  const isTransitioning = Math.abs(deltaFromCenter) > 0.5 * safeDwell * sectorRad;

  const minCamTopY = K.camBaseRadius + K.camBevelRadial;
  const maxCamTopY = K.camBaseRadius + K.camLift + K.camBevelRadial;
  const segments: DigitalNumbersSegmentSolution[] = [];

  for (let i = 0; i < K.slotCount; i++) {
    const contour =
      precomputedContours?.[i] ??
      sampleDigitalCamContour(i, safeDwell, dir, 240);
    const n = contour.length;
    let camTopY = K.camBaseRadius * 0.85;
    for (let s = 0; s < n; s++) {
      const alpha = (s / n) * twoPi;
      const r = contour[s].radius + K.camBevelRadial;
      const wa = alpha + driverAngleRad;
      const pz = r * Math.cos(wa);
      const py = r * Math.sin(wa);
      if (Math.abs(pz) <= K.followerFootRadius && py > camTopY) {
        camTopY = py;
      }
    }

    const rawLift = Math.max(0, camTopY - minCamTopY);
    const off01 = Math.min(1, Math.max(0, rawLift / Math.max(1e-4, maxCamTopY - minCamTopY)));
    const on01 = 1 - off01;
    const rodTopDeckY = K.rodMinDeckY + rawLift;

    const pin = intersectCirclePair2D(
      K.pivotBarZ,
      K.pivotBarY,
      K.crankLen,
      0,
      rodTopDeckY,
      K.couplerLen,
      -1
    );
    const phi = Math.atan2(pin.y - K.pivotBarY, pin.x - K.pivotBarZ);
    const armTiltRad =
      -Math.PI * 0.5 * Math.min(1, Math.max(0, (phi - DIGITAL_CRANK_CAL.phiOn) / DIGITAL_CRANK_CAL.phiSpan));

    segments.push({
      slotIndex: i,
      segmentId: K.segmentOrder[i],
      camX: (i - 3) * K.camSpacingX,
      camTopY,
      rodTopDeckY,
      crankPin: { z: pin.x, y: pin.y },
      armTiltRad,
      on01,
    });
  }

  return {
    driverAngleRad,
    activeDigit,
    isTransitioning,
    segments,
  };
}

export const WRITING_CAM_KINEMATICS = {
  camCount: 3,
  camSpacingX: 0.24,
  camThickness: 0.072,
  camBaseRadius: 0.41,
  camBevelRadial: 0.005,
  rollerRadius: 0.015,
  rodMinDeckY: 0.14,
  xyStrokeSpan: 0.22,
  liftStrokeSpan: 0.15,
  contourSamples: 360,
  paperTiltRad: 0.32,
  paperCenter: { x: -0.42, y: 0.14, z: 0.58 },
  paperWidth: 0.78,
  paperHeight: 0.56,
  paperDrawW: 0.52,
  paperDrawH: 0.34,
  shoulderLocal: { u: 0.12, v: 0.62, n: 0.24 },
  upperArmLen: 0.54,
  forearmLen: 0.52,
  penLength: 0.32,
  penMaxLift: 0.072,
  penMaxPitchRad: 0.24,
  // Closed-chain Bell-Crank 0 (Cam 0 at x=-0.24 -> Shoulder Yaw Horn 1)
  bell0: {
    rodX: -0.24,
    pivot: { x: -0.02, y: 0.44, z: -0.02 },
    inLen: 0.22,
    coupLen: 0.19,
    outLen: 0.26,
    nOffset: -0.04,
    hornR: 0.24,
    tieLen: 0.36,
    branchSign: -1 as const,
    hornBaseOffset: (97.4 * Math.PI) / 180 - (-152.3 * Math.PI) / 180,
  },
  // Closed-chain Bell-Crank 1 (Cam 1 at x=0.00 -> Coaxial Forearm Drum Horn 2)
  bell1: {
    rodX: 0.0,
    pivot: { x: 0.18, y: 0.44, z: 0.02 },
    inLen: 0.18,
    coupLen: 0.19,
    outLen: 0.28,
    nOffset: -0.1,
    hornR: 0.24,
    tieLen: 0.56,
    branchSign: -1 as const,
    hornBaseOffset: (91.7 * Math.PI) / 180 - (-43.8 * Math.PI) / 180,
  },
  // Closed-chain Bell-Crank 2 (Cam 2 at x=+0.24 -> Shoulder Lift Collar Rocker)
  bell2: {
    rodX: 0.24,
    pivot: { x: 0.08, y: 0.44, z: 0.06 },
    inLen: 0.16,
    coupLen: 0.19,
    outLen: 0.16,
    hornCenter: { x: -0.2, y: 0.6, z: 0.06 },
    hornR: 0.12,
    tieLen: 0.31,
  },
} as const;

const WRITING_FRAME = (() => {
  const K = WRITING_CAM_KINEMATICS;
  const ct = Math.cos(K.paperTiltRad);
  const st = Math.sin(K.paperTiltRad);
  const U = { x: 1, y: 0, z: 0 };
  const V = { x: 0, y: st, z: -ct };
  const N = { x: 0, y: ct, z: st };
  const shoulderPos = {
    x: K.paperCenter.x + K.shoulderLocal.u * U.x + K.shoulderLocal.v * V.x + K.shoulderLocal.n * N.x,
    y: K.paperCenter.y + K.shoulderLocal.u * U.y + K.shoulderLocal.v * V.y + K.shoulderLocal.n * N.y,
    z: K.paperCenter.z + K.shoulderLocal.u * U.z + K.shoulderLocal.v * V.z + K.shoulderLocal.n * N.z,
  };
  return { U, V, N, shoulderPos };
})();

function easeCosine01(t: number): number {
  const c = Math.max(0, Math.min(1, t));
  return 0.5 - 0.5 * Math.cos(Math.PI * c);
}

/**
 * Evaluates normalized cyclic pen trajectory (u, v in [0, 1], penLift in [0, 1])
 * over phase t01 in [0, 1) for Programmable Drawing Automaton modes '12', '1', or '2'.
 */
export function evaluateWritingTrajectory(
  t01: number,
  digitPattern: WritingDigitPattern = '12',
  strokeScale = 1.0
): {
  u: number;
  v: number;
  penLift: number;
  strokeState: 'writing-1' | 'pen-lift-jump' | 'writing-2' | 'pen-lift-return';
} {
  const t = ((t01 % 1) + 1) % 1;
  const safeScale = Math.max(0.75, Math.min(1.15, strokeScale));
  let u = 0.5;
  let v = 0.5;
  let penLift = 0;
  let strokeState: 'writing-1' | 'pen-lift-jump' | 'writing-2' | 'pen-lift-return' = 'writing-1';

  if (digitPattern === '12') {
    if (t < 0.05) {
      u = 0.15;
      v = 0.72;
      penLift = easeCosine01(1 - t / 0.05);
      strokeState = penLift > 0.25 ? 'pen-lift-return' : 'writing-1';
    } else if (t < 0.13) {
      const s = easeCosine01((t - 0.05) / 0.08);
      u = 0.15 + 0.11 * s;
      v = 0.72 + 0.16 * s;
      penLift = 0;
      strokeState = 'writing-1';
    } else if (t < 0.28) {
      const s = easeCosine01((t - 0.13) / 0.15);
      u = 0.26;
      v = 0.88 - 0.74 * s;
      penLift = 0;
      strokeState = 'writing-1';
    } else if (t < 0.44) {
      const raw = (t - 0.28) / 0.16;
      const s = easeCosine01(Math.max(0, Math.min(1, (raw - 0.15) / 0.7)));
      u = 0.26 + 0.26 * s;
      v = 0.14 + 0.58 * s;
      penLift = raw < 0.5 ? easeCosine01(raw / 0.4) : easeCosine01((1 - raw) / 0.4);
      strokeState = 'pen-lift-jump';
    } else if (t < 0.6) {
      const s = easeCosine01((t - 0.44) / 0.16);
      const ang = Math.PI + s * (-Math.PI * 1.12);
      u = 0.68 + 0.16 * Math.cos(ang);
      v = 0.72 + 0.16 * Math.sin(ang);
      penLift = 0;
      strokeState = 'writing-2';
    } else if (t < 0.72) {
      const angEnd = Math.PI - Math.PI * 1.12;
      const uStart = 0.68 + 0.16 * Math.cos(angEnd);
      const vStart = 0.72 + 0.16 * Math.sin(angEnd);
      const s = easeCosine01((t - 0.6) / 0.12);
      u = uStart + (0.51 - uStart) * s;
      v = vStart + (0.14 - vStart) * s;
      penLift = 0;
      strokeState = 'writing-2';
    } else if (t < 0.82) {
      const s = easeCosine01((t - 0.72) / 0.1);
      u = 0.51 + 0.35 * s;
      v = 0.14;
      penLift = 0;
      strokeState = 'writing-2';
    } else if (t < 0.88) {
      u = 0.86;
      v = 0.14;
      penLift = easeCosine01((t - 0.82) / 0.06);
      strokeState = 'pen-lift-return';
    } else if (t < 0.97) {
      const s = easeCosine01((t - 0.88) / 0.09);
      u = 0.86 + (0.15 - 0.86) * s;
      v = 0.14 + (0.72 - 0.14) * s;
      penLift = 1;
      strokeState = 'pen-lift-return';
    } else {
      u = 0.15;
      v = 0.72;
      penLift = 1;
      strokeState = 'pen-lift-return';
    }
  } else if (digitPattern === '1') {
    if (t < 0.06) {
      u = 0.38;
      v = 0.7;
      penLift = easeCosine01(1 - t / 0.06);
      strokeState = penLift > 0.25 ? 'pen-lift-return' : 'writing-1';
    } else if (t < 0.24) {
      const s = easeCosine01((t - 0.06) / 0.18);
      u = 0.38 + 0.14 * s;
      v = 0.7 + 0.18 * s;
      penLift = 0;
      strokeState = 'writing-1';
    } else if (t < 0.62) {
      const s = easeCosine01((t - 0.24) / 0.38);
      u = 0.52;
      v = 0.88 - 0.74 * s;
      penLift = 0;
      strokeState = 'writing-1';
    } else if (t < 0.72) {
      u = 0.52;
      v = 0.14;
      penLift = easeCosine01((t - 0.62) / 0.1);
      strokeState = 'pen-lift-return';
    } else if (t < 0.95) {
      const s = easeCosine01((t - 0.72) / 0.23);
      u = 0.52 + (0.38 - 0.52) * s;
      v = 0.14 + (0.7 - 0.14) * s;
      penLift = 1;
      strokeState = 'pen-lift-return';
    } else {
      u = 0.38;
      v = 0.7;
      penLift = 1;
      strokeState = 'pen-lift-return';
    }
  } else {
    // digitPattern === '2'
    if (t < 0.06) {
      u = 0.3;
      v = 0.72;
      penLift = easeCosine01(1 - t / 0.06);
      strokeState = penLift > 0.25 ? 'pen-lift-return' : 'writing-2';
    } else if (t < 0.32) {
      const s = easeCosine01((t - 0.06) / 0.26);
      const ang = Math.PI + s * (-Math.PI * 1.12);
      u = 0.5 + 0.2 * Math.cos(ang);
      v = 0.7 + 0.18 * Math.sin(ang);
      penLift = 0;
      strokeState = 'writing-2';
    } else if (t < 0.52) {
      const angEnd = Math.PI - Math.PI * 1.12;
      const uStart = 0.5 + 0.2 * Math.cos(angEnd);
      const vStart = 0.7 + 0.18 * Math.sin(angEnd);
      const s = easeCosine01((t - 0.32) / 0.2);
      u = uStart + (0.28 - uStart) * s;
      v = vStart + (0.14 - vStart) * s;
      penLift = 0;
      strokeState = 'writing-2';
    } else if (t < 0.7) {
      const s = easeCosine01((t - 0.52) / 0.18);
      u = 0.28 + 0.44 * s;
      v = 0.14;
      penLift = 0;
      strokeState = 'writing-2';
    } else if (t < 0.78) {
      u = 0.72;
      v = 0.14;
      penLift = easeCosine01((t - 0.7) / 0.08);
      strokeState = 'pen-lift-return';
    } else if (t < 0.96) {
      const s = easeCosine01((t - 0.78) / 0.18);
      u = 0.72 + (0.3 - 0.72) * s;
      v = 0.14 + (0.72 - 0.14) * s;
      penLift = 1;
      strokeState = 'pen-lift-return';
    } else {
      u = 0.3;
      v = 0.72;
      penLift = 1;
      strokeState = 'pen-lift-return';
    }
  }

  return {
    u: 0.5 + (u - 0.5) * safeScale,
    v: 0.5 + (v - 0.5) * safeScale,
    penLift,
    strokeState,
  };
}

function solveWritingArmIK(u: number, v: number): { theta1: number; phi2: number } {
  const K = WRITING_CAM_KINEMATICS;
  const pu = (u - 0.5) * K.paperDrawW;
  const pv = (v - 0.5) * K.paperDrawH;
  const du = pu - K.shoulderLocal.u;
  const dv = pv - K.shoulderLocal.v;
  const d2 = du * du + dv * dv;
  const d = Math.max(1e-5, Math.sqrt(d2));
  const ang = Math.atan2(dv, du);
  const cosAlpha = Math.max(
    -1,
    Math.min(1, (d2 + K.upperArmLen * K.upperArmLen - K.forearmLen * K.forearmLen) / (2 * K.upperArmLen * d))
  );
  const alpha = Math.acos(cosAlpha);
  const theta1 = ang - alpha;
  const phi2 = Math.atan2(dv - K.upperArmLen * Math.sin(theta1), du - K.upperArmLen * Math.cos(theta1));
  return { theta1, phi2 };
}

function solveWritingXYBellCrank(
  rodY: number,
  cfg: typeof WRITING_CAM_KINEMATICS.bell0 | typeof WRITING_CAM_KINEMATICS.bell1
): WritingCamBellCrankSolution & { jointAngle: number } {
  const { U, V, N, shoulderPos } = WRITING_FRAME;
  const p1 = intersectCirclePair2D(cfg.pivot.x, cfg.pivot.y, cfg.inLen, cfg.rodX, rodY, cfg.coupLen, 1);
  const p2 = intersectCirclePair2D(cfg.pivot.x, cfg.pivot.y, cfg.inLen, cfg.rodX, rodY, cfg.coupLen, -1);
  const inPin2D = p1.x < p2.x ? p1 : p2;
  const angIn = Math.atan2(inPin2D.y - cfg.pivot.y, inPin2D.x - cfg.pivot.x);
  const angOut = angIn - Math.PI * 0.5;
  const outPin = {
    x: cfg.pivot.x + cfg.outLen * Math.cos(angOut),
    y: cfg.pivot.y + cfg.outLen * Math.sin(angOut),
    z: cfg.pivot.z,
  };

  const hornCenter = {
    x: shoulderPos.x + cfg.nOffset * N.x,
    y: shoulderPos.y + cfg.nOffset * N.y,
    z: shoulderPos.z + cfg.nOffset * N.z,
  };
  const dx = outPin.x - hornCenter.x;
  const dy = outPin.y - hornCenter.y;
  const dz = outPin.z - hornCenter.z;
  const qu = dx * U.x + dy * U.y + dz * U.z;
  const qv = dx * V.x + dy * V.y + dz * V.z;
  const qn = dx * N.x + dy * N.y + dz * N.z;
  const r2d = Math.sqrt(Math.max(1e-6, cfg.tieLen * cfg.tieLen - qn * qn));
  const g = intersectCirclePair2D(0, 0, cfg.hornR, qu, qv, r2d, cfg.branchSign);
  const gamma = Math.atan2(g.y, g.x);
  const hornPin = {
    x: hornCenter.x + g.x * U.x + g.y * V.x,
    y: hornCenter.y + g.x * U.y + g.y * V.y,
    z: hornCenter.z + g.x * U.z + g.y * V.z,
  };

  return {
    pivot: { ...cfg.pivot },
    rodClevis: { x: cfg.rodX, y: rodY, z: 0 },
    inPin: { x: inPin2D.x, y: inPin2D.y, z: cfg.pivot.z },
    outPin,
    hornPin,
    hornCenter,
    jointAngle: gamma - cfg.hornBaseOffset,
  };
}

function invertWritingXYBellCrank(
  targetJointAngle: number,
  cfg: typeof WRITING_CAM_KINEMATICS.bell0 | typeof WRITING_CAM_KINEMATICS.bell1
): number {
  const K = WRITING_CAM_KINEMATICS;
  let lo: number = K.rodMinDeckY;
  let hi: number = K.rodMinDeckY + K.xyStrokeSpan;
  const fLo = solveWritingXYBellCrank(lo, cfg).jointAngle;
  const fHi = solveWritingXYBellCrank(hi, cfg).jointAngle;
  const target = Math.max(Math.min(fLo, fHi), Math.min(Math.max(fLo, fHi), targetJointAngle));
  for (let it = 0; it < 28; it++) {
    const mid = 0.5 * (lo + hi);
    const val = solveWritingXYBellCrank(mid, cfg).jointAngle;
    if ((fHi < fLo && val > target) || (fHi > fLo && val < target)) lo = mid;
    else hi = mid;
  }
  return 0.5 * (lo + hi);
}

function solveWritingLiftBellCrank(rodY: number): WritingCamBellCrankSolution {
  const K = WRITING_CAM_KINEMATICS;
  const cfg = K.bell2;
  const p1 = intersectCirclePair2D(cfg.pivot.x, cfg.pivot.y, cfg.inLen, cfg.rodX, rodY, cfg.coupLen, 1);
  const p2 = intersectCirclePair2D(cfg.pivot.x, cfg.pivot.y, cfg.inLen, cfg.rodX, rodY, cfg.coupLen, -1);
  const inPin2D = p1.x > p2.x ? p1 : p2;
  const angIn = Math.atan2(inPin2D.y - cfg.pivot.y, inPin2D.x - cfg.pivot.x);
  const angOut = angIn + Math.PI * 0.5;
  const outPin = {
    x: cfg.pivot.x + cfg.outLen * Math.cos(angOut),
    y: cfg.pivot.y + cfg.outLen * Math.sin(angOut),
    z: cfg.pivot.z,
  };
  const h = intersectCirclePair2D(
    cfg.hornCenter.x,
    cfg.hornCenter.y,
    cfg.hornR,
    outPin.x,
    outPin.y,
    cfg.tieLen,
    -1
  );
  return {
    pivot: { ...cfg.pivot },
    rodClevis: { x: cfg.rodX, y: rodY, z: 0 },
    inPin: { x: inPin2D.x, y: inPin2D.y, z: cfg.pivot.z },
    outPin,
    hornPin: { x: h.x, y: h.y, z: cfg.pivot.z },
    hornCenter: { ...cfg.hornCenter },
  };
}

/**
 * Generates the 3 coaxial roller-envelope cam contours ([camX, camY, camLift])
 * encoding the trajectory of `digitPattern` ('12', '1', or '2') at `strokeScale`.
 */
export function sampleWritingCamContours(
  digitPattern: WritingDigitPattern = '12',
  strokeScale = 1.0,
  samples = WRITING_CAM_KINEMATICS.contourSamples
): [
  Array<{ x: number; y: number; radius: number }>,
  Array<{ x: number; y: number; radius: number }>,
  Array<{ x: number; y: number; radius: number }>,
] {
  const K = WRITING_CAM_KINEMATICS;
  const pitchOffset = K.camBaseRadius + K.rollerRadius;
  const envOffset = K.rollerRadius + K.camBevelRadial;
  const rp0: number[] = [];
  const rp1: number[] = [];
  const rp2: number[] = [];

  for (let i = 0; i < samples; i++) {
    const t = i / samples;
    const traj = evaluateWritingTrajectory(t, digitPattern, strokeScale);
    const { theta1, phi2 } = solveWritingArmIK(traj.u, traj.v);
    const ry0 = invertWritingXYBellCrank(theta1, K.bell0);
    const ry1 = invertWritingXYBellCrank(phi2, K.bell1);
    rp0.push(pitchOffset + (ry0 - K.rodMinDeckY));
    rp1.push(pitchOffset + (ry1 - K.rodMinDeckY));
    rp2.push(pitchOffset + K.liftStrokeSpan * traj.penLift);
  }

  const toEnvelope = (rp: number[]) => {
    const pts: Array<{ x: number; y: number; radius: number }> = [];
    for (let i = 0; i < samples; i++) {
      const iPrev = (i - 1 + samples) % samples;
      const iNext = (i + 1) % samples;
      const aPrev = Math.PI * 0.5 - (iPrev / samples) * Math.PI * 2;
      const aNext = Math.PI * 0.5 - (iNext / samples) * Math.PI * 2;
      const aCurr = Math.PI * 0.5 - (i / samples) * Math.PI * 2;
      const tx = rp[iNext] * Math.cos(aNext) - rp[iPrev] * Math.cos(aPrev);
      const ty = rp[iNext] * Math.sin(aNext) - rp[iPrev] * Math.sin(aPrev);
      const tl = Math.max(1e-8, Math.hypot(tx, ty));
      const nx = -ty / tl;
      const ny = tx / tl;
      const px = rp[i] * Math.cos(aCurr) - envOffset * nx;
      const py = rp[i] * Math.sin(aCurr) - envOffset * ny;
      pts.push({ x: px, y: py, radius: Math.hypot(px, py) });
    }
    return pts;
  };

  return [toEnvelope(rp0), toEnvelope(rp1), toEnvelope(rp2)];
}

function evaluateWritingRollerCenterY(
  contour: Array<{ x: number; y: number; radius: number }>,
  driverAngleRad: number
): number {
  const K = WRITING_CAM_KINEMATICS;
  const rEff = K.rollerRadius + K.camBevelRadial;
  const ct = Math.cos(driverAngleRad);
  const st = Math.sin(driverAngleRad);
  let bestY: number = K.camBaseRadius;
  const n = contour.length;
  for (let s = 0; s < n; s++) {
    const pt = contour[s];
    const pz = pt.x * ct - pt.y * st;
    const py = pt.x * st + pt.y * ct;
    if (Math.abs(pz) <= rEff) {
      const cy = py + Math.sqrt(Math.max(0, rEff * rEff - pz * pz));
      if (cy > bestY) bestY = cy;
    }
  }
  return bestY;
}

/**
 * Forward rigid-body solver for the 3-Cam Programmable Writing Automaton:
 * 1) Evaluates roller follower center height on each of the 3 extruded coaxial cams;
 * 2) Solves all 3 vertical pushrods, 3 planar bell-cranks, 3D shoulder horns, and lift collar;
 * 3) Computes exact 2-link arm + parallel forearm rod + wrist pen-lift coordinates over the inclined desk.
 */
export function solveWritingCam(
  shaftAngleRad: number,
  module: Pick<WritingCamModule, 'phaseDeg' | 'digitPattern' | 'strokeScale'>,
  precomputedContours?: [
    Array<{ x: number; y: number; radius: number }>,
    Array<{ x: number; y: number; radius: number }>,
    Array<{ x: number; y: number; radius: number }>,
  ]
): WritingCamSolution {
  const K = WRITING_CAM_KINEMATICS;
  const { U, V, N, shoulderPos } = WRITING_FRAME;
  const driverAngleRad = shaftAngleRad + degToRad(module.phaseDeg);
  const cycleProgress01 = normalizeAngleRad(driverAngleRad) / (Math.PI * 2);
  const contours =
    precomputedContours ?? sampleWritingCamContours(module.digitPattern, module.strokeScale);

  const cy0 = evaluateWritingRollerCenterY(contours[0], driverAngleRad);
  const cy1 = evaluateWritingRollerCenterY(contours[1], driverAngleRad);
  const cy2 = evaluateWritingRollerCenterY(contours[2], driverAngleRad);

  const pitchBase = K.camBaseRadius + K.rollerRadius;
  const rod0Y = K.rodMinDeckY + Math.max(0, Math.min(K.xyStrokeSpan, cy0 - pitchBase));
  const rod1Y = K.rodMinDeckY + Math.max(0, Math.min(K.xyStrokeSpan, cy1 - pitchBase));
  const liftDisp = Math.max(0, Math.min(K.liftStrokeSpan, cy2 - pitchBase));
  const rod2Y = K.rodMinDeckY + liftDisp;

  const bellCrankX = solveWritingXYBellCrank(rod0Y, K.bell0);
  const bellCrankY = solveWritingXYBellCrank(rod1Y, K.bell1);
  const bellCrankLift = solveWritingLiftBellCrank(rod2Y);

  const theta1 = bellCrankX.jointAngle;
  const phi2 = bellCrankY.jointAngle;
  const penLift01 = liftDisp / K.liftStrokeSpan;
  const liftCollarOffset = penLift01 * 0.055;
  const penClearance = penLift01 * K.penMaxLift;
  const penPitchRad = penLift01 * K.penMaxPitchRad;

  const c1 = Math.cos(theta1);
  const s1 = Math.sin(theta1);
  const c2 = Math.cos(phi2);
  const s2 = Math.sin(phi2);

  const elbowPos = {
    x: shoulderPos.x + K.upperArmLen * (c1 * U.x + s1 * V.x),
    y: shoulderPos.y + K.upperArmLen * (c1 * U.y + s1 * V.y),
    z: shoulderPos.z + K.upperArmLen * (c1 * U.z + s1 * V.z),
  };

  // Parallel forearm control pin offset perpendicular to forearm by 0.075 in arm plane
  const drumOffsetR = 0.075;
  const perpAng = phi2 + Math.PI * 0.5;
  const pxOff = drumOffsetR * (Math.cos(perpAng) * U.x + Math.sin(perpAng) * V.x);
  const pyOff = drumOffsetR * (Math.cos(perpAng) * U.y + Math.sin(perpAng) * V.y);
  const pzOff = drumOffsetR * (Math.cos(perpAng) * U.z + Math.sin(perpAng) * V.z);

  const shoulderDrumPin = {
    x: shoulderPos.x + pxOff - 0.03 * N.x,
    y: shoulderPos.y + pyOff - 0.03 * N.y,
    z: shoulderPos.z + pzOff - 0.03 * N.z,
  };
  const elbowHornPin = {
    x: elbowPos.x + pxOff - 0.03 * N.x,
    y: elbowPos.y + pyOff - 0.03 * N.y,
    z: elbowPos.z + pzOff - 0.03 * N.z,
  };

  // Forearm remains strictly rigid in the arm plane; pen slides cleanly along +N inside the wrist guide bushing
  const wristPos = {
    x: elbowPos.x + K.forearmLen * (c2 * U.x + s2 * V.x),
    y: elbowPos.y + K.forearmLen * (c2 * U.y + s2 * V.y),
    z: elbowPos.z + K.forearmLen * (c2 * U.z + s2 * V.z),
  };

  const pu = K.shoulderLocal.u + K.upperArmLen * c1 + K.forearmLen * c2;
  const pv = K.shoulderLocal.v + K.upperArmLen * s1 + K.forearmLen * s2;
  const paperU = pu / K.paperDrawW + 0.5;
  const paperV = pv / K.paperDrawH + 0.5;

  const penTipPos = {
    x: K.paperCenter.x + pu * U.x + pv * V.x + penClearance * N.x,
    y: K.paperCenter.y + pu * U.y + pv * V.y + penClearance * N.y,
    z: K.paperCenter.z + pu * U.z + pv * V.z + penClearance * N.z,
  };
  const penTopPos = {
    x: penTipPos.x + K.penLength * N.x,
    y: penTipPos.y + K.penLength * N.y,
    z: penTipPos.z + K.penLength * N.z,
  };

  const trajInfo = evaluateWritingTrajectory(cycleProgress01, module.digitPattern, module.strokeScale);

  return {
    driverAngleRad,
    cycleProgress01,
    camRollerCenterY: [cy0, cy1, cy2],
    rodDeckY: [rod0Y, rod1Y, rod2Y],
    bellCrankX,
    bellCrankY,
    bellCrankLift,
    liftCollarOffset,
    shoulderAngleRad: theta1,
    forearmAngleRad: phi2,
    shoulderPos,
    elbowPos,
    wristPos,
    shoulderDrumPin,
    elbowHornPin,
    penTopPos,
    penTipPos,
    paperU,
    paperV,
    penLift01,
    penClearance,
    penPitchRad,
    strokeState: trajInfo.strokeState,
  };
}






