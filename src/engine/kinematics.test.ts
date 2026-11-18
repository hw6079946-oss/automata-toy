import test from 'node:test';
import assert from 'node:assert/strict';
import {
  BOATMAN_KINEMATICS,
  CAM_BEVEL_RADIAL,
  clampCircularCamOffset,
  clampFourBarPivot,
  clampModuleSlotX,
  computeFourBarPushrodConfig,
  evaluateCamRadius,
  evaluateRatchetWheelRadius,
  findNearestFreeSlotX,
  MIN_MODULE_SPACING,
  sampleCamContour,
  sampleCycleCurve,
  sampleRatchetContour,
  solveBevelGear,
  solveBevelGearGeometry,
  solveCamFollower,
  solveCircularCam,
  solveCircularCamGeometry,
  solveDualEccentric,
  solveDualEccentricGeometry,
  solveFourBarLinkage,
  solveFourBarPushrod,
  solvePinLantern,
  solvePinLanternGeometry,
  BELLOWS_KINEMATICS,
  BELL_CRANK_KINEMATICS,
  CARPENTER_KINEMATICS,
  CLAPPING_KINEMATICS,
  DIGITAL_NUMBERS_KINEMATICS,
  DIGITAL_SEGMENT_PATTERNS,
  DONKEY_MILL_KINEMATICS,
  DOUBLE_DOORS_KINEMATICS,
  DUCK_EGG_KINEMATICS,
  evaluateDigitalNumbersCamRadius,
  HOCKEY_KINEMATICS,
  LOOM_KINEMATICS,
  RATTLE_DRUM_KINEMATICS,
  SAWYERS_KINEMATICS,
  SCOTCH_YOKE_CENTER_DIST,
  solveBellCrank,
  solveBellCrankGeometry,
  solveDigitalNumbers,
  solveRatchet,
  solveRatchetGeometry,
  solveScotchYoke,
  solveScotchYokeGeometry,
  solveSemiCircularCam,
  solveSemiCircularCamGeometry,
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
  WRITING_CAM_KINEMATICS,
  evaluateWritingTrajectory,
  sampleWritingCamContours,
  solveWritingCam,
} from './kinematics';
import { CLASSIC_PRESETS, createDefaultModule } from './presets';
import type { CamProfileType, RatchetModule, WritingDigitPattern } from './types';

test('cam contour evaluation produces positive finite radii across all profiles', () => {
  const profiles: CamProfileType[] = ['eccentric', 'pear', 'snail', 'heart'];
  for (const profile of profiles) {
    const contour = sampleCamContour(profile, 0.55, 0.45, 72);
    assert.equal(contour.length, 72);
    for (const pt of contour) {
      assert.equal(Number.isFinite(pt.radius), true);
      assert.ok(pt.radius >= 0.4, `${profile} radius should be >= 0.4`);
    }
  }
});

test('snail cam rises monotonically then drops sharply', () => {
  const rEarly = evaluateCamRadius('snail', 0.2, 0.5, 0.5);
  const rMid = evaluateCamRadius('snail', Math.PI, 0.5, 0.5);
  const rPeak = evaluateCamRadius('snail', Math.PI * 2 * 0.92, 0.5, 0.5);
  assert.ok(rEarly < rMid && rMid < rPeak, 'snail cam should rise steadily');
});

test('cam follower never penetrates beveled cam surface and reaches near full lift', () => {
  const profiles: CamProfileType[] = ['eccentric', 'pear', 'snail', 'heart'];
  const baseRadius = 0.55;
  const lift = 0.5;
  const footRadius = 0.34;
  for (const profile of profiles) {
    let minLift = Infinity;
    let maxLift = -Infinity;
    for (let deg = 0; deg < 360; deg += 10) {
      const theta = (deg * Math.PI) / 180;
      const sol = solveCamFollower(profile, theta, 0, baseRadius, lift, footRadius);
      assert.equal(Number.isFinite(sol.followerBottomY), true);
      assert.ok(
        sol.followerBottomY >= baseRadius + CAM_BEVEL_RADIAL - 0.01,
        `${profile} follower must stay above beveled baseRadius`
      );
      // Verify no point on beveled contour under foot penetrates above followerBottomY
      for (let i = 0; i < 144; i++) {
        const alpha = (i / 144) * Math.PI * 2;
        const r = evaluateCamRadius(profile, alpha, baseRadius, lift) + CAM_BEVEL_RADIAL;
        const pz = r * Math.cos(alpha + theta);
        const py = r * Math.sin(alpha + theta);
        if (Math.abs(pz) <= footRadius) {
          assert.ok(
            py <= sol.followerBottomY + 1e-6,
            `${profile} cam point (${py}) penetrated follower (${sol.followerBottomY})`
          );
        }
      }
      minLift = Math.min(minLift, sol.normalizedLift);
      maxLift = Math.max(maxLift, sol.normalizedLift);
    }
    assert.ok(maxLift > 0.85, `${profile} follower should achieve high lift (${maxLift})`);
    assert.ok(minLift < 0.25, `${profile} follower should return near rest (${minLift})`);
  }
});

test('4-bar linkage solver preserves coupler and rocker link lengths in reachable range', () => {
  const params = {
    phaseDeg: 0,
    crankRadius: 0.42,
    couplerLength: 1.45,
    rockerLength: 1.08,
    pivotZ: 0.92,
    pivotY: 1.15,
    outputArmRatio: 0.85,
  };

  for (let deg = 0; deg < 360; deg += 15) {
    const sol = solveFourBarLinkage((deg * Math.PI) / 180, params);
    assert.equal(sol.isOverStretched, false, `Should not overstretch at ${deg} deg`);

    const couplerDist = Math.hypot(
      sol.couplerJoint.z - sol.crankPin.z,
      sol.couplerJoint.y - sol.crankPin.y
    );
    const rockerDist = Math.hypot(
      sol.couplerJoint.z - sol.fulcrum.z,
      sol.couplerJoint.y - sol.fulcrum.y
    );

    assert.ok(
      Math.abs(couplerDist - params.couplerLength) < 1e-4,
      `Coupler length mismatch: ${couplerDist}`
    );
    assert.ok(
      Math.abs(rockerDist - params.rockerLength) < 1e-4,
      `Rocker length mismatch: ${rockerDist}`
    );
  }
});

test('4-bar guided vertical pushrod keeps constant pitman link length and zero sideways clipping', () => {
  const params = {
    phaseDeg: 0,
    crankRadius: 0.42,
    couplerLength: 1.45,
    rockerLength: 1.08,
    pivotZ: 0.92,
    pivotY: 1.15,
    outputArmRatio: 0.85,
  };
  const cfg = computeFourBarPushrodConfig(params);
  for (let deg = 0; deg < 360; deg += 15) {
    const pushSol = solveFourBarPushrod((deg * Math.PI) / 180, params, cfg);
    const linkDist = Math.hypot(
      cfg.outputStageZ - pushSol.clevisZ,
      pushSol.pushrodBottomY - pushSol.clevisY
    );
    assert.ok(
      Math.abs(linkDist - cfg.linkLength) < 1e-4,
      `Pitman link length mismatch: ${linkDist} vs ${cfg.linkLength}`
    );
    assert.ok(pushSol.liftAboveMin >= -1e-6);
  }
});

test('4-bar linkage handles over-stretched geometry gracefully without NaN', () => {
  const sol = solveFourBarLinkage(0.5, {
    phaseDeg: 0,
    crankRadius: 0.5,
    couplerLength: 0.6,
    rockerLength: 0.6,
    pivotZ: 2.5,
    pivotY: 2.5,
    outputArmRatio: 0.8,
  });
  assert.equal(sol.isOverStretched, true);
  assert.equal(Number.isFinite(sol.couplerJoint.z), true);
  assert.equal(Number.isFinite(sol.couplerJoint.y), true);
});

function segDistToOrigin(az: number, ay: number, bz: number, by: number): number {
  const dz = bz - az;
  const dy = by - ay;
  const lenSq = dz * dz + dy * dy;
  if (lenSq < 1e-12) return Math.hypot(az, ay);
  const t = Math.max(0, Math.min(1, -(az * dz + ay * dy) / lenSq));
  return Math.hypot(az + t * dz, ay + t * dy);
}

test('4-bar offset-gear linkage maintains zero camshaft collision and stays inside box bounds across 360 deg', () => {
  const SHAFT_RADIUS = 0.085;
  const BAR_HALF_WIDTH = 0.082; // width 0.14 + 2*0.012 bevel = 0.164 -> half = 0.082
  const MIN_CLEARANCE = SHAFT_RADIUS + BAR_HALF_WIDTH + 0.04; // >= 0.207

  const cases: Array<{
    name: string;
    phaseDeg: number;
    crankRadius: number;
    couplerLength: number;
    rockerLength: number;
    pivotZ: number;
    pivotY: number;
    outputArmRatio: number;
  }> = [];

  for (const preset of CLASSIC_PRESETS) {
    for (const mod of preset.modules) {
      if (mod.type === 'four-bar-linkage') {
        cases.push({ name: `${preset.id}:${mod.id}`, ...mod });
      }
    }
  }
  const defMod = createDefaultModule('four-bar-linkage', 0, 1);
  if (defMod.type === 'four-bar-linkage') {
    cases.push({ name: 'default-4bar', ...defMod });
    // Mirror negative-Z dragged pivot and corner extremes
    for (const [pz, py] of [
      [-0.92, 1.15],
      [0.55, 0.65],
      [0.55, 1.3],
      [1.2, 0.65],
      [1.2, 1.3],
    ]) {
      const clamped = clampFourBarPivot(pz, py);
      cases.push({
        name: `pivot(${clamped.pivotZ},${clamped.pivotY})`,
        ...defMod,
        ...clamped,
      });
    }
  }

  for (const c of cases) {
    const pushCfg = computeFourBarPushrodConfig(c);
    for (let deg = 0; deg < 360; deg += 5) {
      const theta = (deg * Math.PI) / 180;
      const sol = solveFourBarLinkage(theta, c);
      const pushSol = solveFourBarPushrod(theta, c, pushCfg, sol);

      const pinDist = Math.hypot(sol.crankPin.z, sol.crankPin.y);
      const couplerDist = segDistToOrigin(
        sol.crankPin.z,
        sol.crankPin.y,
        sol.couplerJoint.z,
        sol.couplerJoint.y
      );
      const rockerDist = segDistToOrigin(
        sol.fulcrum.z,
        sol.fulcrum.y,
        sol.couplerJoint.z,
        sol.couplerJoint.y
      );
      const pitmanDist = segDistToOrigin(
        pushSol.clevisZ,
        pushSol.clevisY,
        pushCfg.outputStageZ,
        pushSol.pushrodBottomY
      );

      assert.ok(
        pinDist >= MIN_CLEARANCE,
        `${c.name} crankPin collided with shaft at ${deg} deg (dist=${pinDist.toFixed(3)})`
      );
      assert.ok(
        couplerDist >= MIN_CLEARANCE,
        `${c.name} coupler collided with shaft at ${deg} deg (dist=${couplerDist.toFixed(3)})`
      );
      assert.ok(
        rockerDist >= MIN_CLEARANCE,
        `${c.name} rocker collided with shaft at ${deg} deg (dist=${rockerDist.toFixed(3)})`
      );
      assert.ok(
        pitmanDist >= MIN_CLEARANCE,
        `${c.name} pitman collided with shaft at ${deg} deg (dist=${pitmanDist.toFixed(3)})`
      );
      assert.ok(
        Math.abs(sol.crankPin.z) <= 1.32 && Math.abs(sol.couplerJoint.z) <= 1.32,
        `${c.name} exceeded glass depth at ${deg} deg`
      );
      assert.ok(
        pushSol.pushrodBottomY <= 1.92,
        `${c.name} pushrod bottom exceeded stage deck underside at ${deg} deg (${pushSol.pushrodBottomY})`
      );
    }
  }
});

test('bevel gear ratio and cone geometry remain bounded and non-overlapping across ratios', () => {
  const sol = solveBevelGear(Math.PI, { phaseDeg: 90, gearRatio: 2.0, teethCount: 16 });
  assert.ok(Math.abs(sol.driverAngleRad - Math.PI * 1.5) < 1e-6);
  assert.ok(Math.abs(sol.drivenAngleRad - Math.PI * 3.0) < 1e-6);

  for (const teeth of [12, 14, 16, 20, 24]) {
    for (const ratio of [0.5, 1.0, 1.5, 2.0, 3.0]) {
      const spec = solveBevelGearGeometry(ratio, teeth);
      assert.ok(
        Math.abs(spec.driverPitchR / spec.drivenPitchR - spec.effectiveRatio) < 1e-6,
        `Pitch radii ratio must equal tooth ratio (${spec.driverPitchR}/${spec.drivenPitchR} vs ${spec.effectiveRatio})`
      );
      assert.ok(
        Math.abs(spec.coneAngleDriver + spec.coneAngleDriven - Math.PI / 2) < 1e-6,
        'Complementary 90-deg bevel pitch cones'
      );
      assert.ok(spec.driverPitchR <= 0.6 && spec.drivenPitchR <= 0.55);
      assert.ok(spec.driverX - 0.1 > -0.65);
      assert.ok(spec.drivenX + spec.drivenPitchR < 0.85);
    }
  }
});

test('pin-lantern wheel and cantilevered pinion maintain exact ratio and zero peg-stave clipping', () => {
  const sol = solvePinLantern(Math.PI, { phaseDeg: 90, gearRatio: 1.5, pegCount: 12 });
  assert.ok(Math.abs(sol.driverAngleRad - Math.PI * 1.5) < 1e-6);
  assert.ok(Math.abs(sol.drivenAngleRad - Math.PI * 2.25) < 1e-6);

  for (const pegs of [8, 10, 12, 16]) {
    for (const ratio of [0.5, 0.75, 1.0, 1.5, 2.0, 2.5]) {
      const spec = solvePinLanternGeometry(ratio, pegs);
      assert.ok(
        Math.abs(spec.driverPitchR / spec.lanternPitchR - spec.effectiveRatio) < 1e-6,
        `Pitch ratio mismatch for pegs=${pegs}, ratio=${ratio}`
      );
      // Driver disc face must stay safely clear of lantern staves
      assert.ok(spec.drivenX - spec.lanternPitchR - spec.staveRadius > spec.driverDiscFaceX + 0.01);

      // Check 3D capsule-like clearance across full driver revolution
      let minClearance = Infinity;
      for (let deg = 0; deg < 360; deg += 4) {
        const theta = (deg * Math.PI) / 180;
        const phi = theta * spec.effectiveRatio;
        for (let i = 0; i < spec.driverPegs; i++) {
          const a = (i / spec.driverPegs) * Math.PI * 2;
          const py0 = spec.driverPitchR * Math.cos(a);
          const pz0 = spec.driverPitchR * Math.sin(a);
          const py = py0 * Math.cos(theta) + pz0 * Math.sin(theta);
          const pz = -py0 * Math.sin(theta) + pz0 * Math.cos(theta);
          // Peg Y must be inside stave vertical range to interact with staves
          if (py + spec.pegRadius < spec.lanternYOffset + spec.staveBottomY) continue;
          if (py - spec.pegRadius > spec.lanternYOffset + spec.staveTopY) continue;

          for (let j = 0; j < spec.lanternStaves; j++) {
            const b = Math.PI + spec.lanternPhaseOffsetRad + (j / spec.lanternStaves) * Math.PI * 2;
            const sx0 = spec.lanternPitchR * Math.cos(b);
            const sz0 = -spec.lanternPitchR * Math.sin(b);
            const sx = spec.drivenX + sx0 * Math.cos(phi) - sz0 * Math.sin(phi);
            const sz = sx0 * Math.sin(phi) + sz0 * Math.cos(phi);
            if (sx <= spec.pegTipX && sx >= spec.driverDiscFaceX) {
              const distYZ = Math.hypot(py - spec.lanternYOffset, pz - sz);
              const dZ = Math.abs(pz - sz);
              const clearance = dZ - (spec.pegRadius + spec.staveRadius);
              // Outside stave vertical segment, clearance isn't negative in YZ; inside segment X/Z must stay non-penetrating
              if (sx <= spec.pegTipX - 0.002 && distYZ < 0.25) {
                minClearance = Math.min(minClearance, clearance);
              }
            }
          }
        }
      }
      assert.ok(
        minClearance > 0.008,
        `Peg-stave clearance too tight (${minClearance}) for pegs=${pegs}, ratio=${ratio}`
      );
    }
  }
});

test('topper rigid linkage solvers maintain exact link lengths across full vertical stroke', () => {
  for (let t = 0; t <= 10; t++) {
    const pinY = 0.16 + (t / 10) * 0.42;
    const gull = solveTopperSeagullLinkage(pinY);
    const gullWingDist = Math.hypot(gull.hornX - 0.14, gull.hornY - 0.96);
    const gullLinkDist = Math.hypot(gull.hornX - 0.18, gull.hornY - pinY);
    assert.ok(Math.abs(gullWingDist - 0.3) < 1e-4);
    assert.ok(Math.abs(gullLinkDist - 0.6) < 1e-4);

    const wp = solveTopperWoodpeckerLinkage(pinY);
    const wpLeverDist = Math.hypot(wp.leverPinZ - -0.22, wp.leverPinY - 0.72);
    const wpLinkDist = Math.hypot(wp.leverPinZ - 0, wp.leverPinY - pinY);
    assert.ok(Math.abs(wpLeverDist - 0.27) < 1e-4);
    assert.ok(Math.abs(wpLinkDist - 0.39) < 1e-4);
  }

  const K = BOATMAN_KINEMATICS;
  let prevSailorPitch = Infinity;
  for (let t = 0; t <= 12; t++) {
    const pinY = 0.12 + (t / 12) * 0.6;
    const bm = solveTopperBoatmanLinkage(pinY);
    const keelDist = Math.hypot(bm.keelPinZ - K.boatFulcrumZ, bm.keelPinY - K.boatFulcrumY);
    const pushDist = Math.hypot(bm.keelPinZ - 0, bm.keelPinY - pinY);
    assert.ok(Math.abs(keelDist - K.boatLeverLen) < 1e-4, `keelDist ${keelDist}`);
    assert.ok(Math.abs(pushDist - K.boatLinkLen) < 1e-4, `pushDist ${pushDist}`);

    const hornDist = Math.hypot(bm.dorsalHornZ - bm.sailorSeatZ, bm.dorsalHornY - bm.sailorSeatY);
    const dorsalLinkDist = Math.hypot(bm.dorsalHornZ - K.anchorZ, bm.dorsalHornY - K.anchorY);
    assert.ok(Math.abs(hornDist - K.sailorHornLen) < 1e-4, `hornDist ${hornDist}`);
    assert.ok(Math.abs(dorsalLinkDist - K.sailorLinkLen) < 1e-4, `dorsalLinkDist ${dorsalLinkDist}`);

    const armDist = Math.hypot(bm.handLocalZ - bm.shoulderLocalZ, bm.handLocalY - bm.shoulderLocalY);
    const oarInboardDist = Math.hypot(K.oarlockLocalZ - bm.handLocalZ, K.oarlockLocalY - bm.handLocalY);
    assert.ok(Math.abs(armDist - K.armLenYZ) < 1e-4, `armDist ${armDist}`);
    assert.ok(Math.abs(oarInboardDist - K.inboardOarLenYZ) < 1e-4, `oarInboardDist ${oarInboardDist}`);
    assert.ok(bm.handLocalZ < K.oarlockLocalZ - 0.08, `Oar must never flip across oarlock: ${bm.handLocalZ}`);

    assert.ok(bm.sailorPitchRad < prevSailorPitch, 'Sailor pulls back monotonically as pushrod rises');
    prevSailorPitch = bm.sailorPitchRad;
  }
});

test('slot clamping prevents overlapping modules along camshaft', () => {
  const existing = CLASSIC_PRESETS[0].modules;
  const clamped = clampModuleSlotX(-0.7, 'mod-buoy-cam', existing);
  assert.ok(Math.abs(clamped - -0.75) >= MIN_MODULE_SPACING - 0.01);
  const freeSlot = findNearestFreeSlotX(0, existing);
  assert.ok(existing.every((m) => Math.abs(m.slotX - freeSlot) >= MIN_MODULE_SPACING - 0.01));
});

test('4-bar pivot clamping keeps fulcrum away from shaft center', () => {
  const clamped = clampFourBarPivot(0.1, -0.5);
  assert.ok(Math.abs(clamped.pivotZ) >= 0.55);
  assert.ok(clamped.pivotY >= 0.55);
});

test('all classic presets solve cleanly for a full revolution', () => {
  for (const preset of CLASSIC_PRESETS) {
    for (const mod of preset.modules) {
      const curve = sampleCycleCurve(mod, 36);
      assert.equal(curve.length, 36);
      for (const v of curve) {
        assert.equal(Number.isFinite(v), true);
      }
    }
  }
});

test('ratchet contour and crank-rocker linkage preserve rigid link lengths and sharp sawteeth', () => {
  for (const lockDirection of [1, -1] as const) {
    for (const teethCount of [8, 10, 12, 16]) {
      const contour = sampleRatchetContour(teethCount, lockDirection);
      assert.equal(contour.length, teethCount * 4);
      for (const pt of contour) {
        assert.equal(Number.isFinite(pt.x) && Number.isFinite(pt.y), true);
        const r = Math.hypot(pt.x, pt.y);
        assert.ok(r >= 0.33 && r <= 0.45, `Unexpected tooth radius ${r}`);
      }
      const rVal = evaluateRatchetWheelRadius(0.1, teethCount, lockDirection);
      assert.ok(rVal >= 0.339 && rVal <= 0.441);

      for (const stepTeeth of [1, 2, 3]) {
        const spec = solveRatchetGeometry(teethCount, stepTeeth, lockDirection);
        for (let deg = 0; deg < 360; deg += 12) {
          const sol = solveRatchet((deg * Math.PI) / 180, {
            phaseDeg: 0,
            teethCount,
            stepTeeth,
            lockDirection,
          });
          const couplerDist = Math.hypot(
            sol.rockerPin.z - sol.crankPin.z,
            sol.rockerPin.y - sol.crankPin.y
          );
          const armDist = Math.hypot(
            sol.rockerPin.z,
            sol.rockerPin.y - spec.axleYOffset
          );
          assert.ok(
            Math.abs(couplerDist - spec.couplerLength) < 1e-4,
            `Coupler mismatch for N=${teethCount}, S=${stepTeeth}: ${couplerDist} vs ${spec.couplerLength}`
          );
          assert.ok(
            Math.abs(armDist - spec.armLinkR) < 1e-4,
            `Arm mismatch for N=${teethCount}, S=${stepTeeth}: ${armDist} vs ${spec.armLinkR}`
          );

          // Both driving and holding pawls maintain exact pawlLength from pivot to wheel surface
          const driveTipZ = sol.drivePawlPivot.z + spec.pawlLength * Math.cos(sol.drivePawlAngleRad);
          const driveTipY =
            sol.drivePawlPivot.y + spec.pawlLength * Math.sin(sol.drivePawlAngleRad) - spec.axleYOffset;
          const driveTipR = Math.hypot(driveTipZ, driveTipY);
          assert.ok(
            driveTipR >= spec.wheelRootR && driveTipR <= spec.wheelTipR + 0.03,
            `Drive pawl tip radius out of bounds: ${driveTipR}`
          );

          const holdTipZ = sol.holdPawlPivot.z + spec.pawlLength * Math.cos(sol.holdPawlAngleRad);
          const holdTipY =
            sol.holdPawlPivot.y + spec.pawlLength * Math.sin(sol.holdPawlAngleRad) - spec.axleYOffset;
          const holdTipR = Math.hypot(holdTipZ, holdTipY);
          assert.ok(
            holdTipR >= spec.wheelRootR && holdTipR <= spec.wheelTipR + 0.03,
            `Hold pawl tip radius out of bounds: ${holdTipR}`
          );
        }
      }
    }
  }
});

test('ratchet accumulates one-way stepped motion across 3 revolutions with dwell lock phase', () => {
  for (const lockDirection of [1, -1] as const) {
    const mod: Pick<RatchetModule, 'phaseDeg' | 'teethCount' | 'stepTeeth' | 'lockDirection'> = {
      phaseDeg: 0,
      teethCount: 12,
      stepTeeth: 2,
      lockDirection,
    };
    const spec = solveRatchetGeometry(mod.teethCount, mod.stepTeeth, mod.lockDirection);
    let prevAngle = solveRatchet(0, mod).ratchetWheelAngleRad;
    let wasLocked = solveRatchet(0, mod).isLocked;
    const startAngle = prevAngle;
    let lockedSamples = 0;
    let totalSamples = 0;

    for (let deg = 5; deg <= 1080; deg += 5) {
      const sol = solveRatchet((deg * Math.PI) / 180, mod);
      if (lockDirection === 1) {
        assert.ok(
          sol.ratchetWheelAngleRad >= prevAngle - 1e-6,
          `Forward ratchet must never slip backward (${sol.ratchetWheelAngleRad} < ${prevAngle} at ${deg} deg)`
        );
      } else {
        assert.ok(
          sol.ratchetWheelAngleRad <= prevAngle + 1e-6,
          `Reverse ratchet must never slip forward (${sol.ratchetWheelAngleRad} > ${prevAngle} at ${deg} deg)`
        );
      }
      if (sol.isLocked) {
        lockedSamples++;
        if (wasLocked) {
          assert.ok(
            Math.abs(sol.ratchetWheelAngleRad - prevAngle) < 1e-6,
            `Locked ratchet wheel moved at ${deg} deg`
          );
        }
      }
      totalSamples++;
      wasLocked = sol.isLocked;
      prevAngle = sol.ratchetWheelAngleRad;
    }

    const totalDelta = prevAngle - startAngle;
    const expectedTotal = lockDirection * 3 * spec.stepAngleRad;
    assert.ok(
      Math.abs(totalDelta - expectedTotal) < 1e-4,
      `3-rev ratchet delta mismatch: ${totalDelta} vs ${expectedTotal}`
    );
    assert.ok(
      lockedSamples / totalSamples >= 0.4,
      `Expected >= 40% dwell-lock fraction, got ${lockedSamples}/${totalSamples}`
    );
  }

  const defMod = createDefaultModule('ratchet', 0, 1);
  const curve = sampleCycleCurve(defMod, 72);
  assert.equal(curve.length, 72);
  for (const v of curve) {
    assert.ok(Number.isFinite(v) && v >= 0 && v <= 1);
  }
});

test('circular-cam 90-deg orthogonal rotary drive adjusts wheel offset independently (left / center idle / right reverse)', () => {
  for (const ratio of [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]) {
    for (const r of [0.28, 0.38, 0.48]) {
      const specLeft = solveCircularCamGeometry(ratio, r, -1);
      const specCenter = solveCircularCamGeometry(ratio, r, 0);
      const specRight = solveCircularCamGeometry(ratio, r, 1);

      // Horizontal turntable stays centered at drivenX = 0 while ONLY the vertical cam slides left/center/right
      assert.equal(specLeft.drivenX, 0);
      assert.equal(specCenter.drivenX, 0);
      assert.equal(specRight.drivenX, 0);
      assert.ok(specLeft.driverX < 0 && Math.abs(specLeft.driverX + specLeft.maxOffset) < 1e-6);
      assert.equal(specCenter.driverX, 0);
      assert.ok(specRight.driverX > 0 && Math.abs(specRight.driverX - specRight.maxOffset) < 1e-6);

      // Driver disc stays inside tray footprint across full [-1, +1] travel
      assert.ok(
        Math.abs(specLeft.driverX) + specLeft.driverThickness * 0.5 < specLeft.drivenDiscRadius,
        'Driver wheel must stay within horizontal tray radius'
      );

      // Left = forward (+ratio), Center = stopped (0), Right = reverse (-ratio)
      assert.ok(Math.abs(specLeft.effectiveRatio - ratio) < 1e-6);
      assert.equal(specCenter.effectiveRatio, 0);
      assert.ok(Math.abs(specRight.effectiveRatio + ratio) < 1e-6);

      const solLeft = solveCircularCam(Math.PI, {
        phaseDeg: 90,
        gearRatio: ratio,
        driverRadius: r,
        camOffsetRatio: -1,
      });
      const solCenter = solveCircularCam(Math.PI, {
        phaseDeg: 90,
        gearRatio: ratio,
        driverRadius: r,
        camOffsetRatio: 0,
      });
      const solRight = solveCircularCam(Math.PI, {
        phaseDeg: 90,
        gearRatio: ratio,
        driverRadius: r,
        camOffsetRatio: 1,
      });

      assert.ok(Math.abs(solLeft.driverAngleRad - Math.PI * 1.5) < 1e-6);
      assert.ok(Math.abs(solLeft.drivenAngleRad - Math.PI * 1.5 * ratio) < 1e-6);
      assert.equal(solCenter.drivenAngleRad, 0);
      assert.ok(Math.abs(solRight.drivenAngleRad + Math.PI * 1.5 * ratio) < 1e-6);
    }
  }

  assert.equal(clampCircularCamOffset(-1.4), -1);
  assert.equal(clampCircularCamOffset(1.4), 1);
  assert.equal(clampCircularCamOffset(0.05, 0.09), 0);
  assert.equal(clampCircularCamOffset(-0.93, 0.09), -1);
  assert.equal(clampCircularCamOffset(0.94, 0.09), 1);

  const defCam = createDefaultModule('circular-cam', 0, 1);
  assert.equal(defCam.topper, 'propeller');
  if (defCam.type === 'circular-cam') {
    assert.equal(defCam.camOffsetRatio, -1);
    assert.equal(defCam.camShape, 'circle');
    for (const shape of ['circle', 'triangle', 'square'] as const) {
      for (const pos of [-1, 0, 1]) {
        const curve = sampleCycleCurve(
          { ...defCam, camOffsetRatio: pos, camShape: shape },
          72
        );
        assert.equal(curve.length, 72);
        for (const v of curve) {
          assert.ok(Number.isFinite(v) && v >= 0 && v <= 1);
        }
        if (pos === 0) {
          assert.ok(curve.every((v) => v === 0), 'Center idle should produce flat zero curve');
        }
      }

      const spec = solveCircularCamGeometry(1.5, 0.38, -1, shape);
      assert.ok(spec.inradius >= 0.14, `${shape} inradius must enclose steel camshaft`);

      let prevDriven = solveCircularCam(0, {
        phaseDeg: 0,
        gearRatio: 1.5,
        driverRadius: 0.38,
        camOffsetRatio: -1,
        camShape: shape,
      }).drivenAngleRad;
      let minTrayY = Infinity;
      let maxTrayY = -Infinity;

      for (let deg = 5; deg <= 360; deg += 5) {
        const sol = solveCircularCam((deg * Math.PI) / 180, {
          phaseDeg: 0,
          gearRatio: 1.5,
          driverRadius: 0.38,
          camOffsetRatio: -1,
          camShape: shape,
        });
        assert.ok(sol.trayBottomY >= spec.inradius - 1e-6);
        assert.ok(sol.trayBottomY <= spec.driverRadius + 1e-6);
        assert.ok(sol.drivenAngleRad > prevDriven, `${shape} forward drive must advance`);
        minTrayY = Math.min(minTrayY, sol.trayBottomY);
        maxTrayY = Math.max(maxTrayY, sol.trayBottomY);
        prevDriven = sol.drivenAngleRad;
      }

      if (shape === 'circle') {
        assert.ok(Math.abs(maxTrayY - minTrayY) < 1e-6);
      } else {
        assert.ok(maxTrayY - minTrayY > 0.08, `${shape} cam must modulate tray height`);
      }
    }
  }
});

test('dual-eccentric pair alternates contact under one tray with zero penetration and forward/reverse strokes', () => {
  for (const r of [0.28, 0.33, 0.4]) {
    for (const e of [0.04, 0.08, 0.12]) {
      for (const ratio of [0.5, 1.25, 2.5]) {
        const spec = solveDualEccentricGeometry(r, e, ratio);
        // Both eccentric cams stay cleanly beneath the tray footprint and enclose the camshaft
        assert.ok(spec.camHalfSpacing + spec.camThickness * 0.5 < spec.drivenDiscRadius);
        assert.ok(spec.camRadius - spec.eccentricity >= 0.14);

        let prevDriven = solveDualEccentric(0, {
          phaseDeg: 0,
          camRadius: r,
          eccentricity: e,
          gearRatio: ratio,
        }).drivenAngleRad;
        let sawLeftActive = false;
        let sawRightActive = false;

        for (let deg = 5; deg <= 360; deg += 5) {
          const theta = (deg * Math.PI) / 180;
          const sol = solveDualEccentric(theta, {
            phaseDeg: 0,
            camRadius: r,
            eccentricity: e,
            gearRatio: ratio,
          });
          // Neither cam ever penetrates the horizontal tray bottom face
          assert.ok(sol.leftCamTopY <= sol.trayBottomY + 1e-9);
          assert.ok(sol.rightCamTopY <= sol.trayBottomY + 1e-9);
          assert.ok(sol.leftGap >= 0 && sol.rightGap >= 0);
          assert.ok(Math.min(sol.leftGap, sol.rightGap) < 1e-9, 'At least one cam contacts tray');

          if (Math.sin(theta) > 0.08) {
            sawLeftActive = true;
            assert.equal(sol.activeCam, 'left');
            assert.ok(sol.leftGap < 1e-9 && sol.rightGap > 0);
            assert.ok(sol.drivenAngleRad > prevDriven - 1e-9, 'Left cam stroke drives forward');
          } else if (Math.sin(theta) < -0.08) {
            sawRightActive = true;
            assert.equal(sol.activeCam, 'right');
            assert.ok(sol.rightGap < 1e-9 && sol.leftGap > 0);
            assert.ok(sol.drivenAngleRad < prevDriven + 1e-9, 'Right cam stroke drives reverse');
          }
          prevDriven = sol.drivenAngleRad;
        }
        assert.ok(sawLeftActive && sawRightActive, 'Both eccentric cams must alternate per revolution');
      }
    }
  }

  const defDual = createDefaultModule('dual-eccentric', 0, 1);
  assert.equal(defDual.type, 'dual-eccentric');
  assert.equal(defDual.topper, 'propeller');
  const curve = sampleCycleCurve(defDual, 72);
  assert.equal(curve.length, 72);
  for (const v of curve) {
    assert.ok(Number.isFinite(v) && v >= -1e-6 && v <= 1 + 1e-6);
  }
});

test('semi-circular-cam drives intermittent half-cycle steps with rock-solid dwell and arrow topper', () => {
  for (const offset of [-1, 0, 1]) {
    for (const r of [0.28, 0.38, 0.46]) {
      for (const ratio of [0.5, 1.0, 2.0]) {
        const spec = solveSemiCircularCamGeometry(ratio, r, offset);
        assert.ok(spec.hubRadius > 0.072 && spec.hubRadius < spec.driverRadius);
        assert.ok(Math.abs(spec.driverX) < spec.drivenDiscRadius);

        let prevDriven = solveSemiCircularCam(0, {
          phaseDeg: 0,
          gearRatio: ratio,
          driverRadius: r,
          camOffsetRatio: offset,
        }).drivenAngleRad;
        let dwellCount = 0;
        let activeCount = 0;

        for (let deg = 5; deg <= 1080; deg += 5) {
          const theta = (deg * Math.PI) / 180;
          const sol = solveSemiCircularCam(theta, {
            phaseDeg: 0,
            gearRatio: ratio,
            driverRadius: r,
            camOffsetRatio: offset,
          });
          assert.ok(sol.gap >= 0, 'Semicircular cam must never penetrate tray underside');

          if (sol.isEngaged) {
            activeCount++;
            assert.equal(sol.gap, 0);
            if (offset < 0) {
              assert.ok(sol.drivenAngleRad >= prevDriven - 1e-9);
            } else if (offset > 0) {
              assert.ok(sol.drivenAngleRad <= prevDriven + 1e-9);
            } else {
              assert.equal(sol.drivenAngleRad, 0);
            }
          } else {
            dwellCount++;
            assert.ok(sol.gap > 0, 'Cutaway flat side must stay clear of tray during dwell');
            assert.ok(
              Math.abs(sol.drivenAngleRad - prevDriven) < 1e-9,
              `Arrow must hold stationary during dwell at ${deg} deg`
            );
          }
          prevDriven = sol.drivenAngleRad;
        }

        assert.ok(activeCount > 90 && dwellCount > 90);
        const expectedTotal = 3 * Math.PI * spec.effectiveRatio;
        assert.ok(
          Math.abs(prevDriven - expectedTotal) < 1e-6,
          `3-rev intermittent angle ${prevDriven} vs expected ${expectedTotal}`
        );
      }
    }
  }

  const defSemi = createDefaultModule('semi-circular-cam', 0, 1);
  assert.equal(defSemi.type, 'semi-circular-cam');
  assert.equal(defSemi.topper, 'arrow');
  const curve = sampleCycleCurve(defSemi, 72);
  assert.equal(curve.length, 72);
  assert.ok(curve[0] === 0);
  assert.ok(curve[18] > 0.45 && curve[18] < 0.55);
  assert.ok(curve[54] === 1, 'Second half of cycle must dwell at plateau 1.0');
});

test('scotch-yoke produces pure SHM horizontal shuttle motion with zero camshaft collision', () => {
  const shaftRadius = 0.085;
  for (const crankRadius of [0.18, 0.28, 0.36]) {
    for (const gearRatio of [0.75, 1.0, 1.25, 1.5]) {
      const geo = solveScotchYokeGeometry(crankRadius, gearRatio);
      assert.ok(
        Math.abs(geo.driverPitchR + geo.drivenPitchR - SCOTCH_YOKE_CENTER_DIST) < 1e-9,
        'Spur gear pitch radii must sum strictly to center distance'
      );

      const yokeLowestY = geo.crankCenter.y - geo.slotHalfHeight - 0.055;
      assert.ok(
        yokeLowestY - shaftRadius >= 0.2,
        `Lowest yoke frame Y (${yokeLowestY}) must clear main shaft (${shaftRadius}) by >= 0.20`
      );

      for (let deg = 0; deg < 720; deg += 5) {
        const theta = (deg * Math.PI) / 180;
        const sol = solveScotchYoke(theta, { phaseDeg: 0, crankRadius, gearRatio });
        assert.ok(Number.isFinite(sol.sliderZ) && Number.isFinite(sol.slotBlockY));
        assert.ok(
          Math.abs(sol.sliderZ - crankRadius * Math.cos(sol.crankAngleRad)) < 1e-9,
          'Horizontal slider must follow pure cosine SHM'
        );
        assert.ok(
          Math.abs(sol.slotBlockY - sol.crankCenter.y) <= sol.slotHalfHeight - 0.05,
          'Sliding bronze block must stay cleanly inside vertical yoke slot'
        );
        assert.ok(sol.normalizedSlide >= 0 && sol.normalizedSlide <= 1);
      }
    }
  }

  const defScotch = createDefaultModule('scotch-yoke', 0, 1);
  assert.equal(defScotch.type, 'scotch-yoke');
  assert.equal(defScotch.topper, 'sawyers');
  const curve = sampleCycleCurve(defScotch, 72);
  assert.equal(curve.length, 72);
  assert.ok(Math.abs(curve[0] - 1) < 1e-6);
  assert.ok(Math.abs(curve[18] - 0.5) < 1e-6);
  assert.ok(Math.abs(curve[36] - 0) < 1e-6);
});

test('sawyers two-man crosscut linkage preserves exact upperArm and forearm lengths across full stroke', () => {
  const K = SAWYERS_KINEMATICS;
  for (let sz = -0.36; sz <= 0.361; sz += 0.02) {
    const sol = solveTopperSawyersLinkage(sz);
    assert.ok(Math.abs(sol.sliderZ - sz) < 1e-9);
    for (const branch of [sol.rear, sol.front]) {
      const dUpper = Math.hypot(branch.elbowZ - branch.shoulderZ, branch.elbowY - branch.shoulderY);
      const dFore = Math.hypot(branch.handleZ - branch.elbowZ, branch.handleY - branch.elbowY);
      assert.ok(
        Math.abs(dUpper - K.upperArmLen) < 1e-4,
        `Upper arm length ${dUpper} drifted from ${K.upperArmLen} at sliderZ=${sz}`
      );
      assert.ok(
        Math.abs(dFore - K.forearmLen) < 1e-4,
        `Forearm length ${dFore} drifted from ${K.forearmLen} at sliderZ=${sz}`
      );
      assert.ok(
        Math.abs(branch.handleZ) >= 0.155,
        `Saw handle post at ${branch.handleZ} must always clear central log radius 0.13`
      );
    }
  }
});

test('carpenter, loom, and bellows horizontal SHM toppers preserve rigid kinematics and bounds', () => {
  const CK = CARPENTER_KINEMATICS;
  const LK = LOOM_KINEMATICS;
  const BK = BELLOWS_KINEMATICS;

  for (let deg = 0; deg < 360; deg += 10) {
    const ang = (deg * Math.PI) / 180;
    const sz = 0.36 * Math.cos(ang);

    // Carpenter 2-link arm IK check
    const carp = solveTopperCarpenterLinkage(sz, ang);
    const dUpper = Math.hypot(carp.arm.elbowZ - carp.arm.shoulderZ, carp.arm.elbowY - carp.arm.shoulderY);
    const dFore = Math.hypot(carp.arm.handleZ - carp.arm.elbowZ, carp.arm.handleY - carp.arm.elbowY);
    assert.ok(Math.abs(dUpper - CK.upperArmLen) < 1e-4, `Carpenter upper arm drifted: ${dUpper}`);
    assert.ok(Math.abs(dFore - CK.forearmLen) < 1e-4, `Carpenter forearm drifted: ${dFore}`);
    assert.ok(Number.isFinite(carp.shavingCurlAngle) && carp.shavingCurlAngle > 0);

    // Loom symmetric heddle shed & picker kick check
    const loom = solveTopperLoomLinkage(sz, ang);
    assert.ok(Math.abs(loom.heddleAOffsetY + loom.heddleBOffsetY) < 1e-9);
    assert.ok(Math.abs(loom.heddleAOffsetY) <= LK.heddleMaxOffset + 1e-9);
    assert.ok(loom.rearPickerAngleRad <= 0 && loom.frontPickerAngleRad >= 0);

    // Bellows strictly ordered non-inverting pleat compression check
    const bel = solveTopperBellowsLinkage(sz);
    assert.ok(bel.gap >= 0.1, `Bellows minimum clearance gap violated: ${bel.gap}`);
    assert.equal(bel.pleatZs.length, BK.pleatCount);
    let prevZ = bel.rearBoardZ;
    for (const pz of bel.pleatZs) {
      assert.ok(pz > prevZ, 'Bellows pleats must stay strictly monotonic from rear to front');
      prevZ = pz;
    }
    assert.ok(BK.frontHeaderZ > prevZ);
    assert.ok(bel.emberGlow01 >= 0 && bel.emberGlow01 <= 1);
  }
});

test('hockey tabletop duel closed-chain cam-rocker linkage preserves all link lengths and clears puck & ice', () => {
  const HK = HOCKEY_KINEMATICS;
  const minCenterDist = HK.puckRadius + HK.bladeHalfThick;

  for (const crankR of [0.18, 0.28, 0.36]) {
    for (let deg = 0; deg < 360; deg += 10) {
      const ang = (deg * Math.PI) / 180;
      const sz = crankR * Math.cos(ang);
      const sol = solveTopperHockeyLinkage(sz);

      assert.ok(
        sol.puckZ - sol.rear.bladeZ >= minCenterDist - 1e-9,
        `Rear hockey blade (${sol.rear.bladeZ}) penetrated puck (${sol.puckZ})`
      );
      assert.ok(
        sol.front.bladeZ - sol.puckZ >= minCenterDist - 1e-9,
        `Front hockey blade (${sol.front.bladeZ}) penetrated puck (${sol.puckZ})`
      );

      for (const player of [sol.rear, sol.front]) {
        const dRoller = Math.hypot(
          player.rollerZ - player.rockerPivotZ,
          player.rollerY - player.rockerPivotY
        );
        const dMid = Math.hypot(
          player.midPinZ - player.rockerPivotZ,
          player.midPinY - player.rockerPivotY
        );
        const dOuter = Math.hypot(
          player.outerPinZ - player.rockerPivotZ,
          player.outerPinY - player.rockerPivotY
        );
        const dTorsoRod = Math.hypot(
          player.waistPinZ - player.midPinZ,
          player.waistPinY - player.midPinY
        );
        const dVertRod = Math.hypot(
          player.hornPinZ - player.outerPinZ,
          player.hornPinY - player.outerPinY
        );
        const dWaistHorn = Math.hypot(
          player.waistPinZ - player.arm.hipZ,
          player.waistPinY - player.arm.hipY
        );
        const dStickHorn = Math.hypot(
          player.hornPinZ - player.gripZ,
          player.hornPinY - player.gripY
        );
        const dBlade = Math.hypot(player.bladeZ - player.gripZ, player.bladeY - player.gripY);
        const dUpper = Math.hypot(
          player.arm.elbowZ - player.arm.shoulderZ,
          player.arm.elbowY - player.arm.shoulderY
        );
        const dFore = Math.hypot(
          player.arm.handleZ - player.arm.elbowZ,
          player.arm.handleY - player.arm.elbowY
        );

        assert.ok(Math.abs(dRoller - HK.rollerArmLen) < 1e-6);
        assert.ok(Math.abs(dMid - HK.midArmLen) < 1e-6);
        assert.ok(Math.abs(dOuter - HK.outerArmLen) < 1e-6);
        assert.ok(Math.abs(dTorsoRod - player.torsoRodLen) < 1e-6);
        assert.ok(Math.abs(dVertRod - player.vertRodLen) < 1e-6);
        assert.ok(Math.abs(dWaistHorn - HK.waistHornR) < 1e-6);
        assert.ok(Math.abs(dStickHorn - HK.stickHornR) < 1e-6);
        assert.ok(Math.abs(dBlade - HK.bladeLen) < 1e-6);
        assert.ok(Math.abs(dUpper - HK.upperArmLen) < 1e-4);
        assert.ok(Math.abs(dFore - HK.forearmLen) < 1e-4);
        assert.ok(
          player.bladeY - 0.018 >= HK.iceTopY,
          `Stick blade bottom (${player.bladeY - 0.018}) clipped ice surface (${HK.iceTopY})`
        );
      }
    }
  }
});

test('bell-crank twin 90-deg clamshell preserves rigid L-arms, gear clearance, and symmetric deck opening', () => {
  const K = BELL_CRANK_KINEMATICS;
  for (const crankRadius of [0.1, 0.13, 0.16]) {
    for (const gearRatio of [0.75, 1.0, 1.25, 1.5]) {
      const geo = solveBellCrankGeometry(crankRadius, gearRatio);
      assert.ok(
        Math.abs(geo.driverPitchR + geo.drivenPitchR - K.centerDist) < 1e-9,
        'Gear pitch radii must sum strictly to centerDist'
      );

      const drivenOuterR = geo.drivenPitchR + (2 * geo.drivenPitchR) / geo.drivenTeeth * 0.68;
      const minCrossheadBottomY = K.centerDist - crankRadius + K.pitmanLen - 0.034;
      assert.ok(
        minCrossheadBottomY > K.centerDist + drivenOuterR + 0.02,
        `Lowest crosshead (${minCrossheadBottomY}) must clear driven gear teeth (${K.centerDist + drivenOuterR})`
      );

      for (let deg = 0; deg < 720; deg += 5) {
        const theta = (deg * Math.PI) / 180;
        const sol = solveBellCrank(theta, { phaseDeg: 0, crankRadius, gearRatio });
        assert.ok(sol.open01 >= 0 && sol.open01 <= 1);
        assert.ok(sol.sliderX >= K.sliderMinBoundX && sol.sliderX <= K.sliderMaxBoundX);

        const dPitman = Math.hypot(sol.crossheadY - sol.crankPin.y, sol.crankPin.z);
        const dInArmL = Math.hypot(sol.leftInPin.x - sol.leftFulcrum.x, sol.leftInPin.y - sol.leftFulcrum.y);
        const dOutArmL = Math.hypot(sol.leftOutPin.x - sol.leftFulcrum.x, sol.leftOutPin.y - sol.leftFulcrum.y);
        const dCouplerL = Math.hypot(sol.leftInPin.x - -K.crossheadEarX, sol.leftInPin.y - sol.crossheadY);
        const dTopLinkL = Math.hypot(-sol.sliderX - sol.leftOutPin.x, K.deckRailY - sol.leftOutPin.y);

        assert.ok(Math.abs(dPitman - K.pitmanLen) < 1e-6);
        assert.ok(Math.abs(dInArmL - K.inArmLen) < 1e-6);
        assert.ok(Math.abs(dOutArmL - K.outArmLen) < 1e-6);
        assert.ok(Math.abs(dCouplerL - K.couplerLen) < 1e-6);
        assert.ok(Math.abs(dTopLinkL - K.topLinkLen) < 1e-6);

        // Verify exact 90-degree rigid angle between input and output arms of bell-crank
        const dotL =
          (sol.leftInPin.x - sol.leftFulcrum.x) * (sol.leftOutPin.x - sol.leftFulcrum.x) +
          (sol.leftInPin.y - sol.leftFulcrum.y) * (sol.leftOutPin.y - sol.leftFulcrum.y);
        assert.ok(Math.abs(dotL) < 1e-6, 'Bell-crank L-arms must stay orthogonal (90 deg)');

        // Exact mirror symmetry between Left & Right branches
        assert.ok(Math.abs(sol.leftInPin.x + sol.rightInPin.x) < 1e-9);
        assert.ok(Math.abs(sol.leftInPin.y - sol.rightInPin.y) < 1e-9);
        assert.ok(Math.abs(sol.leftOutPin.x + sol.rightOutPin.x) < 1e-9);
        assert.ok(Math.abs(sol.leftOutPin.y - sol.rightOutPin.y) < 1e-9);
      }
    }
  }

  const defBell = createDefaultModule('bell-crank', 0, 1);
  assert.equal(defBell.type, 'bell-crank');
  assert.equal(defBell.topper, 'double-doors');
  const curve = sampleCycleCurve(defBell, 72);
  assert.equal(curve.length, 72);
  assert.ok(Math.max(...curve) > 0.99 && Math.min(...curve) < 0.01);
});

test('double-doors and clapping toppers maintain rigid closed-chain links and zero interpenetration', () => {
  const DK = DOUBLE_DOORS_KINEMATICS;
  const CK = CLAPPING_KINEMATICS;
  const BK = BELL_CRANK_KINEMATICS;

  const expectedArmLen = Math.hypot(DK.doorClosedOuterX - DK.pivotX, DK.doorClosedZ - DK.pivotZ);
  const expectedLugDist = Math.hypot(DK.doorLugX - DK.doorClosedOuterX, DK.doorLugZ - DK.doorClosedZ);
  const expectedCoupledLen = Math.hypot(DK.doorLugX - DK.linkPivotX, DK.doorLugZ - DK.linkPivotZ);
  assert.ok(DK.pivotX > DK.doorClosedOuterX && DK.pivotX > DK.baseOuterX, 'Main pivot must be offset outward from door edge and base side');
  assert.ok(DK.linkPivotX > DK.baseOuterX, 'Coupled swing-link pivot must sit outboard of base side');

  let maxDoorOpen = 0;
  let minDoorOpen = Infinity;
  let minBirdBeakZ = Infinity;
  let maxBirdBeakZ = -Infinity;
  let maxOpenOuterBackX = -Infinity;
  let minPalmGap = Infinity;
  let maxArmSpread = -Infinity;

  for (let i = 0; i <= 40; i++) {
    const sx = BK.sliderMinBoundX + (i / 40) * (BK.sliderMaxBoundX - BK.sliderMinBoundX);

    const door = solveTopperDoubleDoorsLinkage(sx);
    const dHornL = Math.hypot(door.leftHornX - -DK.pivotX, door.leftHornZ - DK.pivotZ);
    const dLinkL = Math.hypot(door.leftHornX - -door.sliderX, door.leftHornZ);
    const dMainArmR = Math.hypot(door.rightHingeX - DK.pivotX, door.rightHingeZ - DK.pivotZ);
    const dLugSpanR = Math.hypot(door.rightLugX - door.rightHingeX, door.rightLugZ - door.rightHingeZ);
    const dCoupledArmR = Math.hypot(door.rightLugX - DK.linkPivotX, door.rightLugZ - DK.linkPivotZ);
    const dBirdToggle = Math.hypot(door.sliderX, door.birdCarriageZ);
    assert.ok(Math.abs(dHornL - DK.hornR) < 1e-6);
    assert.ok(Math.abs(dLinkL - DK.linkLen) < 1e-6);
    assert.ok(Math.abs(dMainArmR - expectedArmLen) < 1e-6);
    assert.ok(Math.abs(dLugSpanR - expectedLugDist) < 1e-6);
    assert.ok(Math.abs(dCoupledArmR - expectedCoupledLen) < 1e-6);
    assert.ok(Math.abs(dBirdToggle - DK.birdToggleLen) < 1e-6);
    assert.ok(door.doorOpenRad >= 0 && door.doorOpenRad <= Math.PI * 0.56);
    assert.ok(door.rightLugX > DK.baseOuterX, `Coupled door lug (${door.rightLugX}) must clear base side (${DK.baseOuterX})`);
    // Check all 4 swept door leaf corners against jamb front face (z <= DK.jambFrontZ)
    const c = Math.cos(door.doorOpenRad);
    const s = Math.sin(door.doorOpenRad);
    for (const lx of [-DK.doorWidth, 0]) {
      for (const lz of [-DK.doorThick * 0.5, DK.doorThick * 0.5]) {
        const worldX = door.rightHingeX + lx * c + lz * s;
        const worldZ = door.rightHingeZ - lx * s + lz * c;
        assert.ok(
          worldZ > DK.jambFrontZ,
          `Door corner (lx=${lx}, lz=${lz}) at openRad=${door.doorOpenRad} swept to z=${worldZ} <= jambFrontZ (${DK.jambFrontZ})`
        );
        if (lx === 0 && lz === -DK.doorThick * 0.5) {
          maxOpenOuterBackX = Math.max(maxOpenOuterBackX, worldX);
        }
      }
    }
    minDoorOpen = Math.min(minDoorOpen, door.doorOpenRad);
    maxDoorOpen = Math.max(maxDoorOpen, door.doorOpenRad);
    minBirdBeakZ = Math.min(minBirdBeakZ, door.birdBeakZ);
    maxBirdBeakZ = Math.max(maxBirdBeakZ, door.birdBeakZ);

    const clap = solveTopperClappingLinkage(sx);
    for (const sideSign of [-1, 1] as const) {
      const branch = sideSign === -1 ? clap.left : clap.right;
      const dHorn = Math.hypot(branch.hornX - branch.shoulderX, branch.hornY - branch.shoulderY);
      const dRod = Math.hypot(branch.hornX - sideSign * clap.sliderX, branch.hornY - CK.pinY);
      const dArm = Math.hypot(branch.wristX - branch.shoulderX, branch.wristY - branch.shoulderY);
      assert.ok(Math.abs(dHorn - CK.armHornR) < 1e-6);
      assert.ok(Math.abs(dRod - CK.armRodLen) < 1e-6);
      assert.ok(Math.abs(dArm - CK.armLen) < 1e-6);
    }
    // Rear pushrod plane (z = -0.145, r = 0.009) must clear arms (z >= -0.119) & torso (z >= -0.0825)
    assert.ok(CK.rodPlaneZ + 0.009 < -0.119, 'Clapping pushrod plane must stay strictly behind arms and torso');
    // Deck outrigger (|x| <= sliderX + 0.012) must stay strictly inside puppet plinths (|x| >= hipX - 0.055)
    assert.ok(clap.sliderX + 0.012 < CK.hipX - 0.055, 'Slider outrigger must clear puppet plinths');

    const palmInnerEdgeL = clap.left.wristX + 0.006 + CK.palmHalfThick;
    assert.ok(palmInnerEdgeL <= -1e-4, `Clapping palms must never cross x=0 (${palmInnerEdgeL})`);
    minPalmGap = Math.min(minPalmGap, -2 * palmInnerEdgeL);
    maxArmSpread = Math.max(maxArmSpread, clap.right.wristX - clap.left.wristX);
  }

  assert.ok(minDoorOpen < 1e-6, 'Doors must close flush at minimum sliderX');
  assert.ok(maxDoorOpen >= 1.4, `Doors must swing wide open (~90 deg), got ${maxDoorOpen} rad`);
  assert.ok(
    maxOpenOuterBackX > DK.baseOuterX,
    `Open door back face (${maxOpenOuterBackX}) must swing strictly outside chalet base side (${DK.baseOuterX})`
  );
  assert.ok(
    minBirdBeakZ < DK.doorClosedZ - 0.04,
    `Bird beak (${minBirdBeakZ}) must retract cleanly behind closed door (${DK.doorClosedZ})`
  );
  assert.ok(
    maxBirdBeakZ > DK.doorClosedZ + 0.2,
    `Bird beak (${maxBirdBeakZ}) must pop visibly forward past open doorway (${DK.doorClosedZ})`
  );
  assert.ok(minPalmGap < 0.025, 'Clapping palms must nearly kiss at clap extreme');
  assert.ok(maxArmSpread > 0.5, 'Clapping arms must open wide outward');
});

test('duck-egg Harada Decoy linkage preserves rigid rocker/pitman/egg arms, collar clearance, and trapdoor-before-egg timing', () => {
  const K = DUCK_EGG_KINEMATICS;
  assert.ok(Math.abs(K.eggPivotX) > 0.16, 'Egg arm pivot must clear central brass collar (r=0.16)');
  assert.ok(K.hipX > 0.16 && K.rockerPivotX > 0.16, 'Duck hip and rocker pivot must clear central collar');
  assert.ok(K.rockerPivotY < K.hipY - 0.15, 'Under-belly rocker stanchion must stay strictly below duck belly underside');

  let prevRockerAng = -Infinity;
  for (let i = 0; i <= 40; i++) {
    const pinY = K.minPinY + (i / 40) * (K.maxPinY - K.minPinY);
    const sol = solveTopperDuckEggLinkage(pinY);

    assert.ok(Number.isFinite(sol.rockerPinX) && Number.isFinite(sol.rockerPinY));
    assert.ok(Number.isFinite(sol.eggCenterX) && Number.isFinite(sol.eggCenterY));

    const dRocker = Math.hypot(sol.rockerPinX - K.rockerPivotX, sol.rockerPinY - K.rockerPivotY);
    const dPitman = Math.hypot(sol.rockerPinX - 0, sol.rockerPinY - sol.pinY);
    const dEggArm = Math.hypot(sol.eggCenterX - K.eggPivotX, sol.eggCenterY - K.eggPivotY);
    const dEggLug = Math.hypot(sol.eggLugX - K.eggPivotX, sol.eggLugY - K.eggPivotY);

    assert.ok(Math.abs(dRocker - K.rockerArmLen) < 1e-6, `Rocker arm drifted: ${dRocker}`);
    assert.ok(Math.abs(dPitman - K.pitmanLen) < 1e-6, `Pitman rod drifted: ${dPitman}`);
    assert.ok(Math.abs(dEggArm - K.eggArmLen) < 1e-6, `Egg arm drifted: ${dEggArm}`);
    assert.ok(Math.abs(dEggLug - K.eggLugRadius) < 1e-6, `Egg lug drifted: ${dEggLug}`);
    assert.ok(sol.rockerPinX <= 0.205, `Rocker pin (${sol.rockerPinX}) must clear front breast (x>=0.225)`);
    assert.ok(sol.rockerPinY <= 0.72, `Rocker pin (${sol.rockerPinY}) must stay below saddleback (y>=0.78)`);

    assert.ok(sol.rockerAngleRad > prevRockerAng, 'Rocker angle must advance strictly monotonically with pinY');
    prevRockerAng = sol.rockerAngleRad;

    if (sol.egg01 > 0.05) {
      assert.ok(
        sol.flap01 > 0.85,
        `Rear trapdoor must swing wide open (flap01=${sol.flap01}) before egg exits (egg01=${sol.egg01})`
      );
    }
  }

  const rest = solveTopperDuckEggLinkage(K.minPinY);
  assert.equal(rest.flap01, 0);
  assert.equal(rest.egg01, 0);
  assert.equal(rest.quack01, 0);
  assert.ok(rest.eggCenterX + K.eggRadiusX < -0.04, 'Concealed egg must clear central pushrod clevis (x=0)');
  assert.ok(
    rest.eggCenterX - K.eggRadiusX > K.hipX + K.flapPivotLocalX + 0.012,
    'Concealed egg rear must clear closed tail trapdoor inner face'
  );

  const laid = solveTopperDuckEggLinkage(0.68);
  assert.equal(laid.flap01, 1);
  assert.equal(laid.egg01, 1);
  assert.equal(laid.quack01, 1);
  assert.ok(Math.abs(laid.eggCenterX - K.nestCenterX) < 0.01, 'Laid egg must settle in straw nest center');

  const haradaPreset = CLASSIC_PRESETS.find((p) => p.id === 'harada-duck-egg');
  assert.ok(haradaPreset, 'harada-duck-egg preset must exist');
  assert.equal(haradaPreset.modules[0].type, 'cam-follower');
  if (haradaPreset.modules[0].type === 'cam-follower') {
    assert.equal(haradaPreset.modules[0].camType, 'pear');
    assert.equal(haradaPreset.modules[0].topper, 'duck-egg');
  }
});

test('rattle-drum (拨浪鼓) driven by dual-eccentric alternating dynamics preserves cord length, drum clearance, and opposite-face strikes', () => {
  const RK = RATTLE_DRUM_KINEMATICS;
  let maxFrontPulse = 0;
  let maxBackPulse = 0;
  let minVel = Infinity;
  let maxVel = -Infinity;
  let minAcc = Infinity;
  let maxAcc = -Infinity;

  for (let deg = 0; deg < 360; deg += 4) {
    const theta = (deg * Math.PI) / 180;
    const dualSol = solveDualEccentric(theta, {
      phaseDeg: 0,
      camRadius: 0.35,
      eccentricity: 0.1,
      gearRatio: 1.5,
    });
    assert.ok(Number.isFinite(dualSol.angularVelNorm));
    assert.ok(Number.isFinite(dualSol.angularAccNorm));
    minVel = Math.min(minVel, dualSol.angularVelNorm);
    maxVel = Math.max(maxVel, dualSol.angularVelNorm);
    minAcc = Math.min(minAcc, dualSol.angularAccNorm);
    maxAcc = Math.max(maxAcc, dualSol.angularAccNorm);

    const drumSol = solveTopperRattleDrum(
      dualSol.drivenAngleRad,
      dualSol.angularVelNorm,
      dualSol.angularAccNorm
    );
    assert.ok(Number.isFinite(drumSol.swingNorm));
    assert.ok(drumSol.centrifugal01 >= 0 && drumSol.centrifugal01 <= 1);
    maxFrontPulse = Math.max(maxFrontPulse, drumSol.frontStrikePulse01);
    maxBackPulse = Math.max(maxBackPulse, drumSol.backStrikePulse01);

    for (const cord of [drumSol.leftCord, drumSol.rightCord]) {
      assert.ok(
        Math.abs(cord.totalCordLength - RK.cordLength) < 0.025,
        `Cord length (${cord.totalCordLength}) drifted from ${RK.cordLength} at deg=${deg}`
      );
      assert.ok(
        Math.abs(cord.elbow.x) > RK.drumRadius,
        `Cord elbow (x=${cord.elbow.x}) must clear drum barrel radius (${RK.drumRadius})`
      );
      const rFace = Math.hypot(cord.bead.x, cord.bead.y - RK.drumCenterY);
      const insideBarrelCylinder =
        rFace < RK.drumRadius + RK.beadRadius - 1e-3 &&
        Math.abs(cord.bead.z) < RK.drumHalfDepth + RK.beadRadius - 1e-3;
      assert.equal(
        insideBarrelCylinder,
        false,
        `Bead at (${cord.bead.x}, ${cord.bead.y}, ${cord.bead.z}) penetrated drum cylinder at deg=${deg}`
      );
    }

    assert.equal(
      drumSol.leftCord.strikeSide,
      -drumSol.rightCord.strikeSide,
      `Left & right beads must whip to opposite Z faces at deg=${deg}`
    );
  }

  assert.ok(minVel < -0.5 && maxVel > 0.5, 'Dual-eccentric angular velocity must reverse each half turn');
  assert.ok(minAcc < -0.5 && maxAcc > 0.5, 'Dual-eccentric angular acceleration must reverse at dead centers');
  assert.ok(maxFrontPulse > 0.7 && maxBackPulse > 0.7, 'Both front (+Z) and back (-Z) drumheads must receive clear strikes');

  const rattlePreset = CLASSIC_PRESETS.find((p) => p.id === 'folk-rattle-drum');
  assert.ok(rattlePreset, 'folk-rattle-drum preset must exist');
  assert.equal(rattlePreset.modules[0].type, 'dual-eccentric');
  if (rattlePreset.modules[0].type === 'dual-eccentric') {
    assert.equal(rattlePreset.modules[0].topper, 'rattle-drum');
  }
});

test('donkey-mill (毛驴拉磨) with vintage roller skates maintains rigid 2-link IK, flush track rolling, zero penetration, and forward wheel spin', () => {
  const K = DONKEY_MILL_KINEMATICS;

  let minClearance = Infinity;
  let maxClearance = -Infinity;
  let glideSamples = 0;
  let maxStrideZ = 0;
  let prevSpinFL = -Infinity;

  for (let deg = 0; deg <= 720; deg += 3) {
    const angleRad = (deg * Math.PI) / 180;
    const sol = solveTopperDonkeyMill(angleRad);

    assert.ok(Number.isFinite(sol.millAngleRad));
    assert.ok(Number.isFinite(sol.gaitPhaseRad));
    assert.ok(Number.isFinite(sol.donkeyOrbitAngleRad));

    const orbitRadiusActual = Math.hypot(sol.donkeyPos.x, sol.donkeyPos.z);
    assert.ok(
      Math.abs(orbitRadiusActual - K.orbitRadius) < 1e-6,
      `Donkey orbit radius drifted: ${orbitRadiusActual}`
    );

    const hitchRadius = Math.hypot(sol.hitchTipWorld.x, sol.hitchTipWorld.z);
    assert.ok(
      Math.abs(hitchRadius - K.hitchBoomLen) < 1e-6,
      `Hitch boom length drifted: ${hitchRadius}`
    );

    const legs = [sol.frontLeftLeg, sol.frontRightLeg, sol.rearLeftLeg, sol.rearRightLeg];
    for (const leg of legs) {
      const dThigh = Math.hypot(leg.knee.y - leg.hip.y, leg.knee.z - leg.hip.z);
      const dShank = Math.hypot(leg.ankle.y - leg.knee.y, leg.ankle.z - leg.knee.z);

      assert.ok(
        Math.abs(dThigh - K.thighLen) < 1e-6,
        `Thigh link length (${dThigh}) drifted from ${K.thighLen} at deg=${deg}`
      );
      assert.ok(
        Math.abs(dShank - K.shankLen) < 1e-6,
        `Shank link length (${dShank}) drifted from ${K.shankLen} at deg=${deg}`
      );

      // Nested Three.js rotation telescope invariant: world skate rotX === -skatePitchRad
      const netSkateRotX =
        sol.torsoPitchRad + leg.thighLocalRotX + leg.shankLocalRotX + leg.skateLocalRotX;
      assert.ok(
        Math.abs(netSkateRotX - -leg.skatePitchRad) < 1e-9,
        `Nested leg rotations must telescope to -skatePitchRad (${netSkateRotX} vs ${-leg.skatePitchRad})`
      );

      // Verify zero brass-track penetration
      assert.ok(
        leg.hoof.y >= K.deckTopY - 1e-9,
        `Roller wheel (${leg.hoof.y}) penetrated track surface (${K.deckTopY}) at deg=${deg}`
      );
      assert.ok(leg.groundClearance >= -1e-9);

      if (leg.isGliding) {
        glideSamples++;
        assert.ok(
          Math.abs(leg.groundClearance) < 1e-9,
          `Gliding roller skate must roll flush on track (got clearance=${leg.groundClearance})`
        );
        assert.equal(leg.skatePitchRad, 0);
      }

      minClearance = Math.min(minClearance, leg.groundClearance);
      maxClearance = Math.max(maxClearance, leg.groundClearance);
      maxStrideZ = Math.max(maxStrideZ, Math.abs(leg.ankle.z - (leg.hip.z >= 0 ? K.shoulderZ : K.hipZ)));
    }

    assert.ok(
      sol.frontLeftLeg.wheelSpinRad > prevSpinFL,
      `Roller wheels must spin monotonically forward at deg=${deg}`
    );
    prevSpinFL = sol.frontLeftLeg.wheelSpinRad;

    // Diagonal skating pairing: FL & RR share glide/swing phase, opposite FR & RL
    assert.equal(sol.frontLeftLeg.isGliding, sol.rearRightLeg.isGliding);
    assert.equal(sol.frontRightLeg.isGliding, sol.rearLeftLeg.isGliding);
  }

  assert.ok(glideSamples > 100, 'Skates must roll flush on track across glide stroke');
  assert.ok(minClearance >= -1e-9, 'All wheels must maintain non-negative clearance to brass track');
  assert.ok(maxClearance > 0.015, 'Skates must lift cleanly during forward recovery swing');
  assert.ok(maxStrideZ > 0.055, 'Skates must sweep a visible forward-backward glide stride');

  const donkeyPreset = CLASSIC_PRESETS.find((p) => p.id === 'folk-donkey-mill');
  assert.ok(donkeyPreset, 'folk-donkey-mill preset must exist');
  assert.equal(donkeyPreset.modules[0].type, 'pin-lantern');
  if (donkeyPreset.modules[0].type === 'pin-lantern') {
    assert.equal(donkeyPreset.modules[0].topper, 'donkey-mill');
  }
});

test('digital-numbers (7-cam 7-segment display) cycles accurately 0..9 and 9..0 with rigid 4-bar crank-rockers and zero cam penetration', () => {
  const K = DIGITAL_NUMBERS_KINEMATICS;

  // 1. Verify all 10 digits hold crisp exact ON (armTilt = 0) and OFF (armTilt = -PI/2) states at 36-deg centers
  for (const dir of [1, -1] as const) {
    for (let step = 0; step < 10; step++) {
      const deg = step * 36;
      const theta = (deg * Math.PI) / 180;
      const expectedDigit = dir === 1 ? step : (10 - step) % 10;
      const sol = solveDigitalNumbers(theta, {
        phaseDeg: 0,
        dwellRatio: 0.55,
        countDirection: dir,
      });
      assert.equal(sol.activeDigit, expectedDigit, `Expected digit ${expectedDigit} at deg=${deg}`);
      assert.equal(sol.isTransitioning, false, `Sector center deg=${deg} must be inside dwell plateau`);

      const pattern = DIGITAL_SEGMENT_PATTERNS[expectedDigit];
      for (let i = 0; i < K.slotCount; i++) {
        const seg = sol.segments[i];
        if (pattern[i] === 1) {
          assert.ok(seg.on01 > 0.99, `Segment ${seg.segmentId} should be ON (1) for digit ${expectedDigit}`);
          assert.ok(Math.abs(seg.armTiltRad) < 0.01, `ON segment ${seg.segmentId} must sit upright at 0 rad`);
        } else {
          assert.ok(seg.on01 < 0.01, `Segment ${seg.segmentId} should be OFF (0) for digit ${expectedDigit}`);
          assert.ok(
            Math.abs(seg.armTiltRad + Math.PI * 0.5) < 0.01,
            `OFF segment ${seg.segmentId} must fold back -90 deg (got ${seg.armTiltRad})`
          );
        }
      }
    }
  }

  // 2. Sweep full 0..720 deg at 2-deg increments verifying rigid link lengths & cam non-penetration
  for (let deg = 0; deg <= 720; deg += 2) {
    const theta = (deg * Math.PI) / 180;
    const sol = solveDigitalNumbers(theta, {
      phaseDeg: 0,
      dwellRatio: 0.55,
      countDirection: 1,
    });
    for (let i = 0; i < K.slotCount; i++) {
      const seg = sol.segments[i];
      const crankDist = Math.hypot(seg.crankPin.z - K.pivotBarZ, seg.crankPin.y - K.pivotBarY);
      const couplerDist = Math.hypot(seg.crankPin.z - 0, seg.crankPin.y - seg.rodTopDeckY);
      assert.ok(
        Math.abs(crankDist - K.crankLen) < 1e-6,
        `Crank horn length drifted (${crankDist} vs ${K.crankLen}) at deg=${deg}, slot=${i}`
      );
      assert.ok(
        Math.abs(couplerDist - K.couplerLen) < 1e-6,
        `Coupler length drifted (${couplerDist} vs ${K.couplerLen}) at deg=${deg}, slot=${i}`
      );
      assert.ok(
        seg.armTiltRad >= -Math.PI * 0.5 - 1e-4 && seg.armTiltRad <= 1e-4,
        `armTiltRad out of [-PI/2, 0] at deg=${deg}, slot=${i}: ${seg.armTiltRad}`
      );
      const alphaAtTop = Math.PI * 0.5 - theta;
      const expectedLocalR = evaluateDigitalNumbersCamRadius(i, alphaAtTop, 0.55, 1) + K.camBevelRadial;
      assert.ok(
        seg.camTopY >= expectedLocalR - 1e-3,
        `Follower penetrated cam ${i} at deg=${deg}: footY=${seg.camTopY} < camR=${expectedLocalR}`
      );
    }
  }

  // 3. Verify preset & factory wiring
  const digitalPreset = CLASSIC_PRESETS.find((p) => p.id === 'yh246801-digital-numbers');
  assert.ok(digitalPreset, 'yh246801-digital-numbers preset must exist');
  assert.equal(digitalPreset.modules[0].type, 'digital-numbers');
  if (digitalPreset.modules[0].type === 'digital-numbers') {
    assert.equal(digitalPreset.modules[0].topper, 'digital-display');
  }
});

test('writing-cam (3-cam programmable drawing automaton) writes legible 1 2 with zero cusps and exact rigid linkage', () => {
  const K = WRITING_CAM_KINEMATICS;
  const dist3 = (
    a: { x: number; y: number; z: number },
    b: { x: number; y: number; z: number }
  ) => Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);

  const patterns: WritingDigitPattern[] = ['12', '1', '2'];
  const scales = [0.75, 1.0, 1.15];

  for (const pattern of patterns) {
    for (const scale of scales) {
      const contours = sampleWritingCamContours(pattern, scale);
      assert.equal(contours.length, 3);
      for (let c = 0; c < 3; c++) {
        const pts = contours[c];
        assert.equal(pts.length, K.contourSamples);
        // Star-shaped contour simplicity check: cross(p_i, p_{i+1}) > 0 (since contour winds CW, p_{i+1} x p_i > 0)
        let minCross = Infinity;
        for (let i = 0; i < pts.length; i++) {
          const p = pts[i];
          const q = pts[(i + 1) % pts.length];
          assert.ok(Number.isFinite(p.radius) && p.radius > K.camBaseRadius - 0.02);
          const cross = q.x * p.y - q.y * p.x;
          if (cross < minCross) minCross = cross;
        }
        assert.ok(
          minCross > 0,
          `Cam ${c} contour must have zero cusps/self-intersections (${pattern}, scale=${scale}, minCross=${minCross})`
        );
      }

      let maxXYErr = 0;
      let maxLiftErr = 0;
      let sawWrite1 = false;
      let sawWrite2 = false;
      let sawJumpLift = false;
      let minWrite1U = Infinity;
      let maxWrite1U = -Infinity;
      let minWrite2U = Infinity;
      let maxWrite2U = -Infinity;

      for (let deg = 0; deg <= 720; deg += 4) {
        const theta = (deg * Math.PI) / 180;
        const sol = solveWritingCam(
          theta,
          { phaseDeg: 0, digitPattern: pattern, strokeScale: scale },
          contours
        );

        assert.ok(Number.isFinite(sol.paperU) && Number.isFinite(sol.paperV));
        assert.ok(Number.isFinite(sol.penLift01) && sol.penLift01 >= -1e-6 && sol.penLift01 <= 1 + 1e-6);

        // Rigid arm links + parallel 4-bar forearm rod
        const upperLen = dist3(sol.shoulderPos, sol.elbowPos);
        const forearmLen = dist3(sol.elbowPos, sol.wristPos);
        const parallelLen = dist3(sol.shoulderDrumPin, sol.elbowHornPin);
        assert.ok(Math.abs(upperLen - K.upperArmLen) < 1e-6, `Upper arm length drifted: ${upperLen}`);
        assert.ok(Math.abs(forearmLen - K.forearmLen) < 1e-6, `Forearm length drifted: ${forearmLen}`);
        assert.ok(Math.abs(parallelLen - K.upperArmLen) < 1e-6, `Parallel tie rod drifted: ${parallelLen}`);

        // Bell-crank rigid link constraints
        const checks = [
          { b: sol.bellCrankX, cfg: K.bell0 },
          { b: sol.bellCrankY, cfg: K.bell1 },
          { b: sol.bellCrankLift, cfg: K.bell2 },
        ] as const;
        for (const { b, cfg } of checks) {
          assert.ok(Math.abs(dist3(b.pivot, b.inPin) - cfg.inLen) < 1e-5);
          assert.ok(Math.abs(dist3(b.pivot, b.outPin) - cfg.outLen) < 1e-5);
          const xyCoupler = Math.hypot(b.rodClevis.x - b.inPin.x, b.rodClevis.y - b.inPin.y);
          assert.ok(Math.abs(xyCoupler - cfg.coupLen) < 1e-5);
          assert.ok(Math.abs(dist3(b.rodClevis, b.inPin) - Math.hypot(cfg.coupLen, cfg.pivot.z)) < 1e-5);
          assert.ok(Math.abs(dist3(b.outPin, b.hornPin) - cfg.tieLen) < 1e-5);
          assert.ok(Math.abs(dist3(b.hornCenter, b.hornPin) - cfg.hornR) < 1e-5);
        }

        // Trajectory reproduction fidelity
        const expected = evaluateWritingTrajectory(sol.cycleProgress01, pattern, scale);
        maxXYErr = Math.max(maxXYErr, Math.hypot(sol.paperU - expected.u, sol.paperV - expected.v));
        maxLiftErr = Math.max(maxLiftErr, Math.abs(sol.penLift01 - expected.penLift));

        if (expected.penLift === 0 && expected.strokeState === 'writing-1') {
          sawWrite1 = true;
          assert.ok(sol.penClearance < 0.002, `Pen must touch paper during writing-1 (${sol.penClearance})`);
          minWrite1U = Math.min(minWrite1U, sol.paperU);
          maxWrite1U = Math.max(maxWrite1U, sol.paperU);
        }
        if (expected.penLift === 0 && expected.strokeState === 'writing-2') {
          sawWrite2 = true;
          assert.ok(sol.penClearance < 0.002, `Pen must touch paper during writing-2 (${sol.penClearance})`);
          minWrite2U = Math.min(minWrite2U, sol.paperU);
          maxWrite2U = Math.max(maxWrite2U, sol.paperU);
        }
        if (expected.strokeState === 'pen-lift-jump' && expected.penLift > 0.85) {
          sawJumpLift = true;
          assert.ok(sol.penClearance >= 0.055, `Pen must lift cleanly during jump (${sol.penClearance})`);
        }
      }

      assert.ok(maxXYErr < 0.008, `Max XY cam-tracking error too high (${maxXYErr})`);
      assert.ok(maxLiftErr < 0.03, `Max lift cam-tracking error too high (${maxLiftErr})`);
      if (pattern === '12') {
        assert.ok(sawWrite1 && sawWrite2 && sawJumpLift, 'Mode 12 must visit write-1, jump-lift, and write-2');
        assert.ok(maxWrite1U < minWrite2U - 0.08, `Digit 1 (${maxWrite1U}) must stay cleanly left of Digit 2 (${minWrite2U})`);
      } else if (pattern === '1') {
        assert.ok(sawWrite1 && !sawWrite2);
      } else {
        assert.ok(!sawWrite1 && sawWrite2);
      }
    }
  }

  // Preset, module factory, and cycle curve integration checks
  const writingPreset = CLASSIC_PRESETS.find((p) => p.id === 'waseda-writing-12');
  assert.ok(writingPreset, 'waseda-writing-12 preset must exist');
  assert.equal(writingPreset.modules[0].type, 'writing-cam');
  if (writingPreset.modules[0].type === 'writing-cam') {
    assert.equal(writingPreset.modules[0].topper, 'writing-arm');
    assert.equal(writingPreset.modules[0].digitPattern, '12');
  }

  const defaultMod = createDefaultModule('writing-cam', 0, 1);
  assert.equal(defaultMod.type, 'writing-cam');
  const curve = sampleCycleCurve(defaultMod, 60);
  assert.equal(curve.length, 60);
  assert.ok(curve.every((val) => Number.isFinite(val)));
});





