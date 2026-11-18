---
name: automata-kinematics
description: Complete mechanical kinematics, 3D geometry invariants, anti-penetration rules, and stage-topper linkage patterns for wooden Automata toys. Use when designing, implementing, debugging, or extending mechanical automata drives (cams, 4-bar linkages, bevel gears, pin-lantern gears, ratchets, circular/polygon orthogonal friction cams, dual-eccentric alternating drives, semicircular intermittent cams, scotch-yoke shuttles, twin 90-deg bell-crank clamshells) or articulated stage toppers.
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

## 2. Core Drive Mechanisms (10 Modular Archetypes)

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
- **Zero Shaft-Collision Architecture (`solveFourBarGearSpec`, `solveFourBarLinkage`)**:
  - A crank pin orbiting `(0, 0)` on a continuous unbroken drive shaft topologically forces any coupler reaching a rocker above the shaft to sweep straight across the shaft once per turn.
  - To remain 100% physically realizable, the main shaft carries a **16T driver spur gear** at `(0, 0)` meshing 1:1 with an **offset 16T crank spur gear** mounted on an independent bearing subframe at `crankCenter = (-sign(pivotZ) * 0.80, +0.26)` (`gearPitchR ≈ 0.421`, `crankAngleRad = -driverAngleRad`).
  - Choosing the overhead intersection branch (`branchSign = sign(pivotZ)`) keeps the crank pin, coupler bar `P1 -> P2`, rocker arm `P3 -> P2`, pitman link, and subframe strictly clear of the horizontal drive shaft (`minDist >= 0.22 >> 0.085 + barHalfWidth`) across all 360°.
  - Soft-clamp $d = |P_3 - P_1|$ into $[|L_1 - L_2| + 0.02,\, L_1 + L_2 - 0.02]$ and separate parallel X planes (`subframe -0.145`, `gears ±0.044`, `pushrod ±0.036`, `pitman +0.068`, `rocker +0.132`, `coupler +0.205`) so zero links collide in X or YZ.
- **Coaxial Guided Vertical Pushrod** (`computeFourBarPushrodConfig` + `solveFourBarPushrod`):
  - Sample 36 angles over $[0, 2\pi)$ to place the deck thrust collar at the horizontal midpoint `outputStageZ = 0.5 * (minZ + maxZ)` of the rocker clevis arc.
  - Size pitman link length `max(0.30, maxDz + 0.16)` so the vertical pushrod slides **strictly vertically** inside the brass deck collar with zero sideways deck clipping and stays below `stageDeckY`.

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

### 2.9 Offset-Gear Scotch Yoke Horizontal Shuttle (`scotch-yoke`)
- **Pure Simple Harmonic Linear Reciprocation** (`solveScotchYoke`):
  - Main-shaft 16T spur gear meshes vertically upward ($d = 0.76$) with an overhead crank spur gear ($R_1 + R_2 \equiv d$) carrying a brass crank pin at radius $r_c \in [0.18, 0.36]$ inside a vertical-slot carriage riding on twin horizontal steel guide rails ($y = 1.38$).
  - Guarantees generous main-shaft clearance ($y_{\text{slotBottom}} \ge 0.28 > r_{\text{shaft}} = 0.085$) while outputting distortion-free SHM horizontal shuttle displacement:
    $$z_{\text{slider}}(\theta) = r_c \cos(\theta_{\text{crank}}), \quad y_{\text{block}}(\theta) = y_{\text{crank}} + r_c \sin(\theta_{\text{crank}})$$

### 2.10 Twin 90° Bell-Crank / Clamshell Symmetric Opening & Closing (`bell-crank`)
- **Vertical Crosshead -> Opposing Horizontal X Sliders** (`solveBellCrankGeometry`, `solveBellCrank`):
  - Main-shaft 16T spur gear meshes vertically (`centerDist = 0.62`) with an overhead crank spur gear driving a pitman connecting rod (`L = 0.64`) into a twin-rail guided vertical crosshead (`z = -0.07`, clearing gear tooth tips `crossheadBottom > centerDist + drivenOuterR + 0.02`).
  - Symmetric coupler rods (`z = +0.035`, decoupled in Z from the rear guide rails) pull/push the horizontal input arms of Left & Right monolithic 90° L-shaped bell-cranks pivoted at `(±0.46, 1.62)`, redirecting vertical stroke into symmetric opposing horizontal motion along an upper steel deck rail (`sliderX ∈ [0.11, 0.455]` at `z = 0`).

### 2.11 7-Cam Digital Numbers Display (`digital-numbers`)
- **7 Coaxial Programmed Cams -> Guided Pushrods -> Upper Pivot-Bar Rocker Linkage** (`DIGITAL_NUMBERS_KINEMATICS`, `evaluateDigitalNumbersCamRadius`, `solveDigitalNumbers`):
  - Seven coaxial brass cams (`camSpacingX = 0.19`, slots `0..6` mapped left-to-right to segments `['a', 'f', 'e', 'd', 'c', 'g', 'b']`) encode digits `0..9` in ten $36^\circ$ sectors (`dwellRatio ∈ [0.40, 0.70]`, $C^1$ cosine transition ramps).
  - Narrow follower shoes (`followerFootRadius = 0.032`) ride cleanly on beveled cam rims (`r_base = 0.42`, `lift = 0.15`, `camBevelRadial = 0.008`) with zero neighboring-lobe interference: low cam valley drops the vertical steel pushrod (`rodMinDeckY = 0.04`) to pull the lower crank horn upright (`ON`, `0 rad`), while high cam lobe lifts the pushrod (`rodMaxDeckY = 0.19`) to swing the horn `+90°` forward and fold the segment `-90°` backward (`OFF`).
  - Exact lower-horn 4-bar crank-rocker closure (`pivotBar = (0.20, 0.35)`, `crankLen = 0.069`, `couplerLen = 0.308`, `span = 89.996°`) swings each collar + bent steel actuator wire + beveled emerald-green hexagonal 7-segment bar strictly rigidly around `+X` between `0 rad` (`ON`, upright facing `+Z`) and `-PI/2 rad` (`OFF`, folded flat backward into horizontal `XZ` plane behind the bar with zero self-collision).

### 2.12 3-Cam Programmable Writing Automaton (`writing-cam`)
- **3 Coaxial Roller-Envelope Cams -> Bell-Cranks -> Parallelogram 2-Link Writing Arm + Pen Lift** (`WRITING_CAM_KINEMATICS`, `evaluateWritingTrajectory`, `sampleWritingCamContours`, `solveWritingCam`):
  - Three coaxial brass cams (`camSpacingX = 0.24`, slots `-0.24, 0, +0.24`) encode `(theta1, phi2, penLift)` via exact inward roller-follower envelope offset (`rollerRadius = 0.015`, `camBevelRadial = 0.005`), guaranteeing simple star-shaped contours (`minCross > 0`) and zero tracking error (`< 1e-4`) across modes `'12'`, `'1'`, and `'2'` (`strokeScale ∈ [0.75, 1.15]`).
  - Vertical steel pushrods carry rolling follower wheels (`r = 0.015`) driving 3 planar Z-axis bell-cranks inside an open-front expressive automaton chassis overlooking an inclined walnut easel (`paperTiltRad = 0.32`).
  - Rigid 3D tie-rods turn coaxial shoulder/elbow-drum horns driving a flat 2-link arm (`upperArmLen = 0.54`, `forearmLen = 0.52`) parallel to the easel plus a sliding wrist pen chuck (`penClearance ∈ [0, 0.072]`) that draws `"1"`, lifts cleanly during the jump, draws `"2"`, and renders live instanced ink segments on the cream paper sheet.

---

## 3. Articulated Stage Toppers & Rigging Rules

- **Direct Rotary Spindles (`propeller`, `arrow`, `none`)**:
  - Keep rosette `base` and brass `bushing` anchored flush to `stageDeckY`.
  - Apply `-continuousAngleRad` to `rotation.y` and `+liftLocal` (`axialLiftWorld / max(0.5, scale)`) to mast and rotor head; ensure mast bottom extends below `-0.20` in local space so full lift strokes at minimum `topperScale = 0.7` never reveal a gap above the bushing.
- **Closed-Chain Reciprocating Characters (`seagull`, `woodpecker`, `boatman`, `sawyers`)**:
  - **Seagull**: Symmetric XY circle-circle closure from vertical yoke crosshead to under-wing control horns (`solveTopperSeagullLinkage`).
  - **Woodpecker**: YZ triangle closure from pushrod clevis pin to belly crank lever (`solveTopperWoodpeckerLinkage`), striking a forward vertical trunk.
  - **Boatman**: 3-stage rigid-body solver (`solveTopperBoatmanLinkage`) — (1) keel lever -> boat pitch, (2) rear stanchion anchor -> sailor torso lean, (3) shoulder + oarlock closure -> hands stay locked onto oar handles across full stroke.
  - **Twin Sawyers (`sawyers`)**: Synchronized 2-worker torso lean + 2-link shoulder-elbow-hand IK (`solveTopperSawyersLinkage`, $L_u = L_f = 0.35$) locking both lumberjacks' hands onto the horizontally reciprocating crosscut saw handles (`handleOffset = 0.52 > logRadius = 0.13`) as the teeth ride inside the felled log's center kerf.
  - **Carpenter Planing (`carpenter`)**: Single woodworker (`hipZ = -0.56`) pushing a wooden hand plane (`solveTopperCarpenterLinkage`, $L_u = L_f = 0.40$) along a slotted workbench plank while an articulated gold shaving curls up on forward strokes.
  - **Weaving Loom (`loom`)**: Flying wooden shuttle gliding along Z while twin heddle frames (`solveTopperLoomLinkage`) alternate shed height ($\pm 0.055\sin\theta$) and end picker levers rebound at stroke extremes.
  - **Forge Bellows (`bellows`)**: 5-pleat accordion bellows (`solveTopperBellowsLinkage`) driven by a horizontal plunger rod pumping air into a glowing charcoal hearth & anvil.
  - **Tabletop Hockey (`hockey`)**: Twin profiled shuttle-mounted cam wedges (`evaluateHockeyCamLift`) drive under-ice brass-roller seesaw rockers (`solveTopperHockeyLinkage`, held against cam/dwell pads by rear helical return springs) whose rear pins push rigid torso-waist rods and vertical stick-horn rods so both opposing skaters (`hipZ = ±0.82`) lean and swing slanted sticks cleanly above the ivory ice (`bladeBottom >= iceTopY`) without puck overlap.
  - **Double Doors (`double-doors`)**: Deepened timber chalet (`z ∈ [-0.63, -0.245]`, base outer side `|x| = 0.460`) supported on rear piers with a raised slotted threshold sill (`sillTopY = 0.125`); hinged wooden doors sit flush on the sill (`y = 0.128..0.964`) driven by an outboard offset-pivot 4-bar coupled swing-arm linkage whose primary shafts (`±0.510, -0.195`) and secondary coupled swing-link pivots (`±0.568, -0.164`) sit strictly outside the door edges (`±0.386`) and base sides (`±0.460`), so both main swing arms and coupled swing links swing the doors outward and forward (`0..93°`, `worldX_back >= 0.470 > 0.460`, `worldZ > -0.245`) with zero jamb/base collision while under-floor crank horns (`y = 0.04, r = 0.24, L = 0.28`) and V-toggle rods (`L = 0.465, y = 0.072`) synchronize the pop-out songbird.
  - **Twin Clapping Puppets (`clapping`)**: Symmetric wooden puppets standing at `hipX = ±0.54` (`|x| >= 0.485`, strictly outside `maxSliderX = 0.455`) whose brass deck sliders carry rear outrigger lugs to `rodPlaneZ = -0.145`; steel pushrods (`L = 0.65`) and rear shoulder crank horns (`r = 0.16`) operate cleanly inside `z = -0.145` (strictly behind legs, torso, and rear arms `z >= -0.119`) so twin arms & palms swing up to kiss at `x = 0` with zero body clipping.
  - **Egg-Laying Duck (`duck-egg`)**: Kazuaki Harada *Decoy*-inspired side-profile duck (`+X` facing) with hollowed rear belly tunnel (`z = ±0.095` flanks, upper saddle back `y >= 0.760`, front breast `x >= 0.225`, rear hinged `tailFlap` driven via upper-tunnel brass crank horn), front webbed-foot pedestal (`x >= 0.165`, clearing `r = 0.16` deck collar), external front throat quack pull-wire, and `-X` slotted twin-cheek chute leading into a dark walnut & brass-straw nest (`nestCenterX = -0.493`); `solveTopperDuckEggLinkage` places the rocker stanchion `(0.19, 0.45)` strictly below the belly (`r = 0.25`, `pitman = 0.18`) so no rod or bracket ever intersects the duck's body while `tailFlap` opens before `eggCarrierArm` (`(-0.20, 0.18)`, `r = 0.30`) lowers the ivory egg into the nest.
  - **Rattle Drum / Pellet Drum (`rattle-drum`)**: Folk crimson-lacquered wooden rattle drum (`RATTLE_DRUM_KINEMATICS`: `drumRadius = 0.24`, `drumHalfDepth = 0.068`, `eyeletOffsetX = 0.256`, `cordLength = 0.245`, `beadRadius = 0.046`) mounted atop a reciprocating spindle; `solveTopperRattleDrum` takes `drumAngleRad` plus analytical `angularVelNorm = K * sin(theta)` and `angularAccNorm = K * cos(theta)` from `solveDualEccentric` (or rotational drive) to drive 2-segment braided cords (`eyelet -> elbow -> bead`) wrapping around `(±0.274, 0.775, ±0.096)` onto opposing parchment drumheads (`±0.116`) with zero barrel penetration and pulsing central gold/crimson medallions on strike.
  - **Donkey Grinding Mill (`donkey-mill`)**: Traditional folk Chinese donkey mill (`DONKEY_MILL_KINEMATICS`: `millRadius = 0.28`, `runnerRadius = 0.21`, `orbitRadius = 0.58`, `thighLen = 0.125`, `shankLen = 0.125`, `skateDrop = 0.036`, `wheelRadius = 0.0145`, `deckTopY = 0.036`) driven by 90-degree pin-lantern gear (`pin-lantern`); stationary circular wooden courtyard plinth (`courtyardTopY = 0.032`) with inlaid brass skating track ring (`r ∈ [0.495, 0.665]`), lower bedstone with flour collection groove, tucked inner flour tub (`r <= 0.451`), and outer grain sack (`r >= 0.68`); rotating upper runner stone with grain hopper and elevated overhead sweeptree yoke (`y = 0.44`) with twin brass harness drop-traces clearing the donkey's torso; harnessed, red-blindfolded wooden donkey wears twin-wheel vintage brass/wood roller skates on all 4 hooves solved via 2-link analytical skating IK (`solveTopperDonkeyMill`) so skates roll flush on the brass track (`groundClearance === 0`) during the 60% glide stroke, lift cleanly during recovery swing, and spin all 8 wheels monotonically forward while staying rigidly parented to `torsoPivot`.
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
