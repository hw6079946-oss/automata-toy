---
name: automata-kinematics
description: Complete mechanical kinematics, 3D geometry invariants, anti-penetration rules, and stage-topper linkage patterns for wooden Automata toys. Use when designing, implementing, debugging, or extending mechanical automata drives (cams, 4-bar linkages, bevel gears, pin-lantern gears, ratchets, circular/polygon orthogonal friction cams, dual-eccentric alternating drives, semicircular intermittent cams) or articulated stage toppers.
---

# Wooden Automata Toy Kinematics & 3D Rigging Skill

This skill codifies the exact mathematical solvers, geometric clearance invariants, 3D coordinate conventions, and articulated topper coupling patterns used across **Atelier Automaton**.

---

## 1. Global Coordinate & Rotation Conventions

- **World Frame (Right-Handed)**:
  - `+X`: Horizontal camshaft axis (left `-2.55` to right `+2.55`, hand crank at `+X`).
  - `+Y`: Vertical output axis (workbench `0` -> camshaft `shaftY = 1.65` -> stage deck `stageDeckY = 3.82`).
  - `+Z`: Depth axis (`+Z` toward viewer, `-Z` toward rear pegboard wall).
- **Primary Camshaft Sign**:
  - Increasing `shaftAngleRad` rotates camshaft meshes by `rotation.x = -shaftAngleRad`.
  - Top rim (`+Y`) moves toward `-Z` (rear wall).
- **Orthogonal Vertical Output Sign**:
  - When a vertical cam at `driverX < 0` (left of spindle) pushes the tray's left side toward `-Z`, the turntable spins **clockwise from above**, which is `rotation.y = -drivenAngleRad` for `drivenAngleRad > 0`.
- **Module Slot Spacing**:
  - Clamp `slotX` to `[-2.55, 2.55]` with minimum inter-module clearance `MIN_MODULE_SPACING = 1.18` so adjacent cams/trays never collide.

---

## 2. Core Drive Mechanisms (8 Modular Archetypes)

### 2.1 Cam & Flat-Faced Follower (`cam-follower`)
- **Polar Profiles** (`evaluateCamRadius(camType, alpha, baseRadius, lift)`):
  - `eccentric`: Offset circle ($e = \text{lift}/2$, $r = \text{base} + e$), smooth sinusoidal SHM, zero dwell.
  - `pear`: Lower half rests at `baseRadius`; upper half rises via $\sin^{1.65}(\alpha)$ — classic CMT **half-cycle dwell + graceful flap**.
  - `snail`: Archimedean spiral over $94\%$ of revolution, sharp $6\%$ drop step — **slow build + sudden snap**.
  - `heart`: $0.5(1 - \cos 2\alpha)$ — **double-frequency two-beat cycle** per crank turn.
- **Anti-Penetration Envelope Solver**:
  - When extruding 2D contours with bevel (`CAM_BEVEL_RADIAL = 0.012`), inflate sampled radius by `CAM_BEVEL_RADIAL` and solve:
    $$y_{\text{foot}} = \max_{|z(\alpha)| \le \text{footRadius}} (r(\alpha) + \text{CAM\_BEVEL\_RADIAL})\sin(\alpha + \theta)$$
  - Keep `footRadius` $\ge$ max tangential contact offset so the cam never slips off the edge of the flat follower foot.

### 2.2 Planar 4-Bar Crank-Rocker (`four-bar-linkage`)
- **YZ Circle-Circle Closure** (`solveFourBarLinkage`):
  - Crank pin $P_1 = (r_c \cos\theta, r_c \sin\theta)$, fixed pegboard fulcrum $P_3 = (\text{pivotZ}, \text{pivotY})$.
  - Soft-clamp $d = |P_3 - P_1|$ into $[|L_1 - L_2| + 0.02,\, L_1 + L_2 - 0.02]$ so live user dragging of the fulcrum (`pivotZ`, `pivotY`) never produces `NaN` or broken links.
- **Coaxial Guided Vertical Pushrod** (`computeFourBarPushrodConfig` + `solveFourBarPushrod`):
  - Sample 36 angles over $[0, 2\pi)$ to place the deck thrust collar at the horizontal midpoint `outputStageZ = 0.5 * (minZ + maxZ)` of the rocker clevis arc.
  - Size pitman link length `linkLength = maxDz + 0.24` so the vertical pushrod slides **strictly vertically** inside the brass deck collar with zero sideways deck clipping.

### 2.3 90° Conical Bevel Gear (`bevel-gear`)
- **Co-Apex Pitch Cone Invariant** (`solveBevelGearGeometry`):
  - Shared tooth scale `unitR = min(0.58 / N1, 0.52 / N2, 0.86 / (N1 + N2))` guarantees identical circular pitch on driver ($R_1 = N_1 \cdot \text{unitR}$) and driven ($R_2 = N_2 \cdot \text{unitR}$).
  - Axes intersect at `(drivenX, shaftY, 0)` with `driverX = drivenX - R_2`, `drivenYOffset = R_1`, and complementary cone angles $\operatorname{atan2}(R_1, R_2)$ / $\operatorname{atan2}(R_2, R_1)$.
  - Offset driven tooth phase by `PI / N2` so teeth mesh into valleys cleanly without root clipping.

### 2.4 Face Pin-Wheel & Cantilevered Lantern Pinion (`pin-lantern`)
- **Traditional Bevel Alternative** (`solvePinLanternGeometry`):
  - Face disc with axial brass pegs drives an **open-bottom cantilevered top-crown lantern pinion** (no lower flange!). Eliminates bottom-flange collision across all ratios (`0.5x..2.5x`) and tolerates slight shaft offsets.
  - Shared `unitR = min(0.52 / N_p, 0.44 / N_s, 0.78 / (N_p + N_s))`, peg radius $0.55 \cdot \text{unitR}$, stave radius $0.55 \cdot \text{unitR}$, phase offset `PI / N_s`.

### 2.5 Ratchet & Pawl One-Way Stepper (`ratchet`)
- **Crank-Rocker + Dual-Pawl Locking** (`solveRatchetGeometry`, `solveRatchet`):
  - Solve exact rocker dead centers (`thetaAtMin`, `thetaAtMax`) over $[0, 2\pi)$ to align forward drive stroke with `stepTeeth * (2*PI / N)` plus `0.16 * toothPitch` backlash slack.
  - Active **drive pawl** advances the sawtooth wheel during forward swing; frame-mounted **hold pawl** locks the wheel root during return stroke (producing clean step-and-dwell motion).
  - Iteratively solve `solvePawlTipContact` against `evaluateRatchetWheelRadius` so both pawls ride exact tooth flanks in both `lockDirection: 1` (forward) and `-1` (reverse).
  - Requires **unwrapped** `shaftAngleRad` so multi-revolution stepped progress accumulates seamlessly.

### 2.6 90° Orthogonal Cam Rotary Drive: Circle / Triangle / Square (`circular-cam`)
- **Independent Cam Offset (`camOffsetRatio ∈ [-1, +1]`)**:
  - `-1` (left of spindle): forward rotation (`effectiveRatio = +gearRatio`).
  - `0` (centered under spindle): `effectiveRatio = 0` (no rotation spin; polygons still pump axially).
  - `+1` (right of spindle): reverse rotation (`effectiveRatio = -gearRatio`).
- **Cam Shape Kinematics** (`solveCircularCam`):
  - `circle`: constant `trayBottomY = R`, `liftDisplacement = 0`, `drivenAngle = driverAngle * effectiveRatio`.
  - `triangle` ($N=3$) & `square` ($N=4$): inradius $r_{\text{in}} = R \cos(\pi / N)$. Over each sector $[0, 2\pi/N)$ with half-sector $h = \pi / N$:
    $$y_{\text{top}}(s) = R \cos\min(s,\, 2h - s), \quad \text{lift} = y_{\text{top}} - r_{\text{in}}$$
    $$\Phi_{\text{sector}}(s) = \begin{cases} \sin(s), & s \le h \\ 2\sin(h) - \sin(2h - s), & s > h \end{cases}$$
    Integrating instantaneous horizontal contact radius $y_{\text{top}}(s)/R$ gives smooth speed modulation (peaks at corners, eases on flats) synchronized with $N$-pulse vertical tray bobbing.
- **Critical Coupling Rules**:
  - **Hub & Drag-Ring Clearance**: Clamp hub radius ($\le 0.68\,r_{\text{in}}$) and crimson drag rings ($\le 0.70\,r_{\text{in}}$) so flat triangle edges never push hub hardware into the tray underside even at minimum `driverRadius = 0.28`.
  - **Axial Lift Propagation**: Pass `sol.liftDisplacement` to `topperRig.update(..., axialLiftWorld)` and extend rotary masts downward into the deck bushing so propellers, arrows, and rotary converters **rise and fall synchronously** with the tray & vertical shaft without leaving air gaps.
  - **Parameter-Switch Accumulator Bias**: When `camOffsetRatio`, `gearRatio`, or `camShape` changes live, record `bias = prevDisplayed - newSolved` so the spinning figure never snaps discontinuously.

### 2.7 Twin Eccentric Alternating Cam Drive (`dual-eccentric`)
- **180° Out-of-Phase Pair Under Shared Tray** (`solveDualEccentric`):
  - Left cam ($x = -0.25$, angle $\theta$) and Right cam ($x = +0.25$, angle $\theta + \pi$) share one horizontal turntable tray:
    $$y_{\text{left}} = r + e\sin\theta, \quad y_{\text{right}} = r - e\sin\theta, \quad y_{\text{tray}} = \max(y_{\text{left}}, y_{\text{right}}) = r + e|\sin\theta|$$
  - During $\sin\theta > 0$, Left Cam lifts & friction-drives forward while Right Cam clears (`rightGap > 0`); during $\sin\theta < 0$, Right Cam takes over in reverse (`leftGap > 0`).
  - Smooth $C^\infty$ alternating swing integral: $\Phi(\theta) = \frac{K}{2}(1 - \cos\theta)$ with `leftGap >= 0, rightGap >= 0` strictly guaranteed.

### 2.8 Semicircular Cam Intermittent Drive (`semi-circular-cam`)
- **180° Active Arc + 180° Relief Dwell** (`solveSemiCircularCam`):
  - Half-moon vertical cam with active circular arc radius $R$ ($0 \le \text{phase} \le \pi$) and recessed shaft hub $r_{\text{hub}} = 0.145$ ($\pi < \text{phase} < 2\pi$).
  - Engaged half-turn: `gap = 0`, turntable & arrow turret advance smoothly by $\pi \cdot \text{effectiveRatio}$.
  - Relief half-turn: `gap > 0` (`R - max(hubRadius, R*|cos(phase)|)`), turntable pauses cleanly in rock-solid dwell.

---

## 3. Articulated Stage Toppers & Rigging Rules

- **Direct Rotary Spindles (`propeller`, `arrow`, `none`)**:
  - Keep rosette `base` and brass `bushing` anchored flush to `stageDeckY`.
  - Apply `-continuousAngleRad` to `rotation.y` and `+liftLocal` (`axialLiftWorld / max(0.5, scale)`) to mast and rotor head; ensure mast bottom extends below `-0.20` in local space so full lift strokes at minimum `topperScale = 0.7` never reveal a gap above the bushing.
- **Closed-Chain Reciprocating Characters (`seagull`, `woodpecker`, `boatman`)**:
  - **Seagull**: Symmetric XY circle-circle closure from vertical yoke crosshead to under-wing control horns (`solveTopperSeagullLinkage`).
  - **Woodpecker**: YZ triangle closure from pushrod clevis pin to belly crank lever (`solveTopperWoodpeckerLinkage`), striking a forward vertical trunk.
  - **Boatman**: 3-stage rigid-body solver (`solveTopperBoatmanLinkage`) — (1) keel lever -> boat pitch, (2) rear stanchion anchor -> sailor torso lean, (3) shoulder + oarlock closure -> hands stay locked onto oar handles across full stroke.
  - In `rotary` mode, attach a vertical steel stem beneath the top brass converter wheel and add `axialLiftLocal` so polygon cam strokes actively animate even reciprocating toppers.
- **Windmill Belfry (`windmill`)**:
  - Open timber-frame gallery exposing a horizontal-to-vertical bevel gear train in `rotary` mode or eccentric crank-rocker in `linear` mode.

---

## 4. Verification & Craft Checklist

1. **Zero Z-Fighting**: Never stack coplanar box/bearing faces; offset overlapping layers or keep inner/outer wall thicknesses cleanly separated.
2. **Spindle Engagement**: Check both **lowest drop** (spindle top must stay inside deck collar) and **highest lift** (spindle top must remain tucked inside topper bushing).
3. **Pure Math Decoupling & Unit Tests**: Keep all solvers pure in a headless module tested across full $360^\circ$ & multi-revolution sweeps (`node:test`) asserting boundedness, zero `NaN`, rigid link length preservation, and `gap >= 0`.
4. **Visual Multi-Angle Inspection**: Verify dead-center extremes ($0^\circ$, $45^\circ$, $60^\circ$, $90^\circ$, $180^\circ$, $270^\circ$) in a real browser to confirm contact tangency, dwell stillness, and topper synchronization.

For classic Automata artistic & storytelling design principles (CMT, Paul Spooner, Keith Newstead, Karakuri), see [references/classic-automata-techniques.md](references/classic-automata-techniques.md).
