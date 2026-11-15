export type MaterialFinishId = 'beech-brass' | 'walnut-copper' | 'folk-painted';

export type CamProfileType = 'eccentric' | 'pear' | 'snail' | 'heart';

export type CircularCamShape = 'circle' | 'triangle' | 'square';

export type TopperType = 'seagull' | 'woodpecker' | 'boatman' | 'windmill' | 'propeller' | 'arrow' | 'none';

export type ModuleType =
  | 'cam-follower'
  | 'four-bar-linkage'
  | 'bevel-gear'
  | 'pin-lantern'
  | 'ratchet'
  | 'circular-cam'
  | 'dual-eccentric'
  | 'semi-circular-cam';

export type CameraPresetId = 'three-quarter' | 'front' | 'side' | 'top';

export interface BaseAutomataModule {
  id: string;
  type: ModuleType;
  label: string;
  /** Position along the main horizontal camshaft (world X), valid range [-2.55, 2.55] */
  slotX: number;
  /** Rotational phase offset on the camshaft in degrees [0, 360) */
  phaseDeg: number;
  /** Articulated wooden sculpture mounted on the output shaft/pushrod */
  topper: TopperType;
  /** Scale multiplier for topper figure [0.7, 1.3] */
  topperScale: number;
}

export interface CamFollowerModule extends BaseAutomataModule {
  type: 'cam-follower';
  camType: CamProfileType;
  /** Base circular radius of the cam */
  baseRadius: number;
  /** Radial lobe lift / eccentricity */
  lift: number;
  /** Horizontal foot radius for flat-faced follower */
  footRadius: number;
}

export interface FourBarLinkageModule extends BaseAutomataModule {
  type: 'four-bar-linkage';
  /** Crank arm pin radius from main shaft */
  crankRadius: number;
  /** Coupler rod length connecting crank pin to rocker joint */
  couplerLength: number;
  /** Rocker arm length from fixed pegboard pivot to coupler joint */
  rockerLength: number;
  /** Fixed fulcrum pivot Z offset from shaft axis (on box pegboard plane) */
  pivotZ: number;
  /** Fixed fulcrum pivot Y offset above main shaft axis */
  pivotY: number;
  /** Output pushrod arm offset along rocker from fulcrum */
  outputArmRatio: number;
}

export interface BevelGearModule extends BaseAutomataModule {
  type: 'bevel-gear';
  /** Driven-to-driver speed ratio (e.g. 1.0, 1.5, 2.0) */
  gearRatio: number;
  /** Tooth count on driver gear */
  teethCount: number;
}

export interface PinLanternModule extends BaseAutomataModule {
  type: 'pin-lantern';
  /** Driven-to-driver speed ratio (e.g. 0.5, 1.0, 1.5, 2.0, 2.5) */
  gearRatio: number;
  /** Number of axial drive pegs on the wooden face wheel */
  pegCount: number;
}

export interface RatchetModule extends BaseAutomataModule {
  type: 'ratchet';
  /** Sawtooth count on the ratchet wheel (8..16) */
  teethCount: number;
  /** Teeth advanced per crank cycle (1..3) */
  stepTeeth: number;
  /** One-way stepping direction: 1 = forward step & reverse lock, -1 = reverse step & forward lock */
  lockDirection: 1 | -1;
}

export interface CircularCamModule extends BaseAutomataModule {
  type: 'circular-cam';
  /** Driven-to-driver speed ratio (0.5..3.0) */
  gearRatio: number;
  /** Circumradius of the vertical cam wheel on the main shaft (0.28..0.48) */
  driverRadius: number;
  /** Horizontal offset ratio of the vertical cam wheel under the horizontal turntable [-1 = left, 0 = center, +1 = right] */
  camOffsetRatio: number;
  /** Vertical cam profile shape: regular circle, equilateral triangle, or square */
  camShape?: CircularCamShape;
}

export interface DualEccentricModule extends BaseAutomataModule {
  type: 'dual-eccentric';
  /** Driven-to-driver alternating speed multiplier (0.5..2.5) */
  gearRatio: number;
  /** Radius of each eccentric circular cam disc on the main shaft (0.28..0.40) */
  camRadius: number;
  /** Eccentric offset of each cam disc center from main camshaft axis (0.04..0.12) */
  eccentricity: number;
}

export interface SemiCircularCamModule extends BaseAutomataModule {
  type: 'semi-circular-cam';
  /** Active contact speed ratio (0.5..2.5); 1.0 = 180 deg turn per crank rev */
  gearRatio: number;
  /** Radius of the vertical semicircular cam arc on the main shaft (0.28..0.46) */
  driverRadius: number;
  /** Horizontal offset ratio under the horizontal turntable [-1 = left, 0 = center, +1 = right] */
  camOffsetRatio: number;
}

export type AutomataModule =
  | CamFollowerModule
  | FourBarLinkageModule
  | BevelGearModule
  | PinLanternModule
  | RatchetModule
  | CircularCamModule
  | DualEccentricModule
  | SemiCircularCamModule;

export interface AutomataBlueprint {
  id: string;
  name: string;
  styleCredit: string;
  description: string;
  boxWidth: number;
  boxHeight: number;
  boxDepth: number;
  /** Y coordinate of main horizontal crankshaft above workbench floor */
  shaftY: number;
  /** Y coordinate of top stage deck surface */
  stageDeckY: number;
  finish: MaterialFinishId;
  modules: AutomataModule[];
}

export interface CamFollowerSolution {
  /** Height of follower bottom face relative to shaft center (Y) */
  followerBottomY: number;
  /** Vertical displacement relative to minimum rest height [0, maxLift] */
  liftDisplacement: number;
  /** Normalized displacement [0, 1] */
  normalizedLift: number;
  /** Current contact point X offset on the follower foot */
  contactOffsetZ: number;
}

export interface FourBarSolution {
  /** Crank pin position in local YZ plane relative to shaft (z, y) */
  crankPin: { z: number; y: number };
  /** Rocker-coupler joint position in local YZ plane relative to shaft (z, y) */
  couplerJoint: { z: number; y: number };
  /** Fixed fulcrum position in local YZ plane relative to shaft (z, y) */
  fulcrum: { z: number; y: number };
  /** Angle of crank pin in radians */
  crankAngleRad: number;
  /** Angle of coupler rod in YZ plane */
  couplerAngleRad: number;
  /** Angle of rocker arm from fulcrum to coupler joint */
  rockerAngleRad: number;
  /** Vertical displacement of pushrod driven by rocker */
  pushrodLift: number;
  /** Normalized swing [-1, 1] */
  normalizedSwing: number;
  /** True if link lengths cannot close cleanly without clamping */
  isOverStretched: boolean;
}

export interface BevelGearSolution {
  driverAngleRad: number;
  drivenAngleRad: number;
}

export interface PinLanternSolution {
  driverAngleRad: number;
  drivenAngleRad: number;
}

export interface RatchetSolution {
  crankPin: { z: number; y: number };
  rockerPin: { z: number; y: number };
  crankAngleRad: number;
  couplerAngleRad: number;
  armAngleRad: number;
  ratchetWheelAngleRad: number;
  drivenOutputAngleRad: number;
  drivePawlPivot: { z: number; y: number };
  drivePawlAngleRad: number;
  holdPawlPivot: { z: number; y: number };
  holdPawlAngleRad: number;
  isLocked: boolean;
}

export interface CircularCamSolution {
  driverAngleRad: number;
  drivenAngleRad: number;
  trayBottomY: number;
  liftDisplacement: number;
}

export interface DualEccentricSolution {
  leftCamAngleRad: number;
  rightCamAngleRad: number;
  leftCamTopY: number;
  rightCamTopY: number;
  trayBottomY: number;
  leftGap: number;
  rightGap: number;
  drivenAngleRad: number;
  activeCam: 'left' | 'right' | 'handoff';
}

export interface SemiCircularCamSolution {
  driverAngleRad: number;
  drivenAngleRad: number;
  isEngaged: boolean;
  /** Vertical clearance gap between cam top surface and tray underside (>= 0) */
  gap: number;
}
