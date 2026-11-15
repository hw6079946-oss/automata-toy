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
  solveRatchet,
  solveRatchetGeometry,
  solveSemiCircularCam,
  solveSemiCircularCamGeometry,
  solveTopperBoatmanLinkage,
  solveTopperSeagullLinkage,
  solveTopperWoodpeckerLinkage,
} from './kinematics';
import { CLASSIC_PRESETS, createDefaultModule } from './presets';
import type { CamProfileType, RatchetModule } from './types';

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
