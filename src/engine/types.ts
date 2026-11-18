export type MaterialFinishId = 'beech-brass' | 'walnut-copper' | 'folk-painted';

export type CamProfileType = 'eccentric' | 'pear' | 'snail' | 'heart';

export type CircularCamShape = 'circle' | 'triangle' | 'square';

export type TopperType =
  | 'seagull'
  | 'woodpecker'
  | 'boatman'
  | 'windmill'
  | 'propeller'
  | 'arrow'
  | 'sawyers'
  | 'carpenter'
  | 'loom'
  | 'bellows'
  | 'hockey'
  | 'double-doors'
  | 'clapping'
  | 'duck-egg'
  | 'rattle-drum'
  | 'donkey-mill'
  | 'digital-display'
  | 'writing-arm'
  | 'none';

export type ModuleType =
  | 'cam-follower'
  | 'four-bar-linkage'
  | 'bevel-gear'
  | 'pin-lantern'
  | 'ratchet'
  | 'circular-cam'
  | 'dual-eccentric'
  | 'semi-circular-cam'
  | 'scotch-yoke'
  | 'bell-crank'
  | 'digital-numbers'
  | 'writing-cam';

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

export interface ScotchYokeModule extends BaseAutomataModule {
  type: 'scotch-yoke';
  /** Eccentric crank pin radius from offset gear center [0.18, 0.36]; SHM stroke = 2 * crankRadius */
  crankRadius: number;
  /** Speed multiplier of the overhead crank gear relative to the main camshaft [0.75, 1.5] */
  gearRatio: number;
}

export interface BellCrankModule extends BaseAutomataModule {
  type: 'bell-crank';
  /** Eccentric crank pin radius on offset crank gear [0.10, 0.16] */
  crankRadius: number;
  /** Speed multiplier of the overhead crank gear relative to the main camshaft [0.75, 1.5] */
  gearRatio: number;
}

export interface DigitalNumbersModule extends BaseAutomataModule {
  type: 'digital-numbers';
  /** Fraction of each 36-deg digit sector spent holding steady on a digit [0.40, 0.70] */
  dwellRatio: number;
  /** Counting direction per forward shaft turn: 1 = 0->9, -1 = 9->0 */
  countDirection: 1 | -1;
}

export type WritingDigitPattern = '12' | '1' | '2';

export interface WritingCamModule extends BaseAutomataModule {
  type: 'writing-cam';
  /** Programmed trajectory pattern encoded across the 3 coaxial cams */
  digitPattern: WritingDigitPattern;
  /** Stroke scale factor on the drawing pad [0.75, 1.15] */
  strokeScale: number;
}

export type AutomataModule =
  | CamFollowerModule
  | FourBarLinkageModule
  | BevelGearModule
  | PinLanternModule
  | RatchetModule
  | CircularCamModule
  | DualEccentricModule
  | SemiCircularCamModule
  | ScotchYokeModule
  | BellCrankModule
  | DigitalNumbersModule
  | WritingCamModule;

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
  /** Offset crank gear axle center in local YZ plane relative to main shaft (z, y) */
  crankCenter: { z: number; y: number };
  /** Crank pin position in local YZ plane relative to main shaft (z, y) */
  crankPin: { z: number; y: number };
  /** Rocker-coupler joint position in local YZ plane relative to main shaft (z, y) */
  couplerJoint: { z: number; y: number };
  /** Fixed fulcrum position in local YZ plane relative to main shaft (z, y) */
  fulcrum: { z: number; y: number };
  /** Angle of main-shaft 1:1 driver spur gear in radians */
  driverAngleRad: number;
  /** Angle of offset crank pin in radians (counter-rotating 1:1 spur gear) */
  crankAngleRad: number;
  /** Angle of coupler rod in YZ plane */
  couplerAngleRad: number;
  /** Angle of rocker arm from fulcrum to coupler joint */
  rockerAngleRad: number;
  /** Pitch radius of each 1:1 meshing spur gear */
  gearPitchR: number;
  /** Tooth count of each 1:1 meshing spur gear */
  gearTeeth: number;
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
  angularVelNorm: number;
  angularAccNorm: number;
  activeCam: 'left' | 'right' | 'handoff';
}

export interface SemiCircularCamSolution {
  driverAngleRad: number;
  drivenAngleRad: number;
  isEngaged: boolean;
  /** Vertical clearance gap between cam top surface and tray underside (>= 0) */
  gap: number;
}

export interface ScotchYokeSolution {
  driverAngleRad: number;
  crankAngleRad: number;
  crankCenter: { z: number; y: number };
  crankPin: { z: number; y: number };
  /** Pure SHM horizontal linear reciprocation along Z: crankRadius * cos(crankAngleRad) */
  sliderZ: number;
  /** Vertical coordinate of the bronze sliding die block inside the yoke slot (relative to main shaft Y) */
  slotBlockY: number;
  /** Normalized horizontal shuttle position [0, 1] */
  normalizedSlide: number;
  driverTeeth: number;
  drivenTeeth: number;
  driverPitchR: number;
  drivenPitchR: number;
  slotHalfHeight: number;
  guideRailY: number;
}

export interface BellCrankSolution {
  driverAngleRad: number;
  crankAngleRad: number;
  crankCenter: { z: number; y: number };
  crankPin: { z: number; y: number };
  crossheadY: number;
  leftFulcrum: { x: number; y: number };
  rightFulcrum: { x: number; y: number };
  leftInPin: { x: number; y: number };
  rightInPin: { x: number; y: number };
  leftOutPin: { x: number; y: number };
  rightOutPin: { x: number; y: number };
  /** Symmetric horizontal deck carriage offset (|x| from module centerline) */
  sliderX: number;
  /** Normalized opening fraction [0 = closed/clapped, 1 = wide open] */
  open01: number;
  driverTeeth: number;
  drivenTeeth: number;
  driverPitchR: number;
  drivenPitchR: number;
}

export type DigitalSegmentId = 'a' | 'f' | 'e' | 'd' | 'c' | 'g' | 'b';

export interface DigitalNumbersSegmentSolution {
  slotIndex: number;
  segmentId: DigitalSegmentId;
  camX: number;
  /** Height of cam top contact surface relative to shaftY */
  camTopY: number;
  /** Pushrod clevis pin height above stageDeckY */
  rodTopDeckY: number;
  /** Collar crank pin position in local (z, y) relative to (0, stageDeckY) */
  crankPin: { z: number; y: number };
  /** Rigid rotation around +X of the collar + bent wire + segment (0 = upright ON, -PI/2 = tilted back OFF) */
  armTiltRad: number;
  /** Normalized activation [0 = OFF, 1 = ON] */
  on01: number;
}

export interface DigitalNumbersSolution {
  driverAngleRad: number;
  activeDigit: number;
  isTransitioning: boolean;
  segments: DigitalNumbersSegmentSolution[];
}

export interface WritingCamBellCrankSolution {
  pivot: { x: number; y: number; z: number };
  rodClevis: { x: number; y: number; z: number };
  inPin: { x: number; y: number; z: number };
  outPin: { x: number; y: number; z: number };
  hornPin: { x: number; y: number; z: number };
  hornCenter: { x: number; y: number; z: number };
}

export interface WritingCamSolution {
  driverAngleRad: number;
  cycleProgress01: number;
  /** Roller follower center Y above shaftY for [camX, camY, camLift] */
  camRollerCenterY: [number, number, number];
  /** Pushrod clevis Y above stageDeckY for [rodX, rodY, rodLift] */
  rodDeckY: [number, number, number];
  bellCrankX: WritingCamBellCrankSolution;
  bellCrankY: WritingCamBellCrankSolution;
  bellCrankLift: WritingCamBellCrankSolution;
  /** Shoulder column lift collar offset along paper normal +N */
  liftCollarOffset: number;
  /** Shoulder & elbow angles in paper (U, V) plane */
  shoulderAngleRad: number;
  forearmAngleRad: number;
  /** Joint positions in stage-deck local space (origin at (slotX, stageDeckY, 0)) */
  shoulderPos: { x: number; y: number; z: number };
  elbowPos: { x: number; y: number; z: number };
  wristPos: { x: number; y: number; z: number };
  /** Parallel forearm linkage pins in stage-deck local space */
  shoulderDrumPin: { x: number; y: number; z: number };
  elbowHornPin: { x: number; y: number; z: number };
  /** Pen holder world-deck points */
  penTopPos: { x: number; y: number; z: number };
  penTipPos: { x: number; y: number; z: number };
  /** Paper coordinates u,v in [0, 1] and pen lift amount [0 = touching paper, 1 = raised] */
  paperU: number;
  paperV: number;
  penLift01: number;
  penClearance: number;
  penPitchRad: number;
  strokeState: 'writing-1' | 'pen-lift-jump' | 'writing-2' | 'pen-lift-return';
}


