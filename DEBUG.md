# DEBUG / Known Issues

Working notes on open bugs, regressions, behavioural questions, and
design decisions for the ADAS stack. Clean usage docs live in
[README.md](README.md); this file is the engineering record — what
broke, why, and what we changed.

Items marked **[FIXED]** have an applied patch; the description is
kept so we remember what was wrong and why we changed it. **[DONE]**
is a deliberate change (feature add, refactor). **[DECIDED]** is a
non-code architectural call. **[KNOWN]** is an open limitation with
mitigation. **[PLANNED]** is scoped but not started.

> **Before every run after pulling code changes, rebuild:**
> ```
> cd ROS_ADAS_Stack
> colcon build --packages-select perception controller
> source install/setup.bash
> ```
> The most common silent failure mode is forgetting this and getting
> `No executable found` for `debug_image_fusion_node`, or, more
> insidiously, running stale stanley / controller binaries whose log
> output doesn't match the code in `src/` (e.g. HOLD printing
> `steer=+0.000` instead of `steer=+nan`).

---

## Reading guide

Two views over the same content:

1. **Chronological dev log** — sections §1 onward, in the order
   issues were encountered and resolved. Captures the *iteration
   history*: a fix in §11 is later refined by §12, the junction
   handling in §5 is superseded by §9 and §15, etc. Use this view
   to understand "how we got here".
2. **Thematic index** — below. Groups entries by subsystem.
   Useful for thesis writing or anyone reading the document for
   the first time.

Every entry follows an evolving template that maps cleanly onto a
thesis Objective / Methods / Results structure:

| Section in entry            | Thesis correspondence            |
|-----------------------------|----------------------------------|
| Symptom / Background        | **Objective** — the problem      |
| Root cause                  | **Analysis** — what we found     |
| Applied fix / changes       | **Methods** — what we did        |
| Caveats / follow-ups        | **Results / Discussion**         |

---

## Thematic index

### Chapter 1 — System infrastructure & bridge
- [§10](#10-bridge-hard-coded-for-one-scenario--now-fully-argparse-driven-fixed) Bridge — argparse-driven scenario configuration
- [§4](#4-carla-graphics-flicker--ego-bonnet--npc-lods-unstable-partial--sync-mode-regressed-motion-now-opt-in) CARLA graphics flicker — sync vs. async investigation
- [§12](#12-ego-still-stalls-at-full-acc-throttle--bridge-jpeg-encoder-starves-the-ros-executor-at-1920×1080-fixed-with-caveat) JPEG encoder starves the ROS executor at 1920×1080
- [§17](#17-synchronous-mode-ui-control-removed-done) Synchronous-mode UI control removed
- [§14](#14-bonnet-flicker-is-worse-in-town10hd-than-town03-known-mitigation-only) Bonnet flicker worse in Town10HD than Town03

### Chapter 2 — Perception
- [§2](#2-no-on-screen-indication-that-acc--lkas-are-running-fixed) On-screen indication that ACC / LKAS are running
- [§6](#6-uncontrolled-acceleration--acc-ignores-closing-leads-ego-rear-ends-them-fixed) ACC distance-filter and gap tuning
- [§7](#7-combined-yolo--ufld-topic-does-not-run-fixed--build-dependency--load-tuning) Combined YOLO + UFLD topic — build + load tuning
- [§8](#8-combined-yolo--ufld-view-looks-like-two-overlapping-video-sequences-fixed) Combined view fusion — timestamp-matched overlays
- [§16](#16-acc-lane-roi-via-ufld-vehicle-frame-ipm-done) ACC lane ROI via UFLD vehicle-frame IPM
- [§20](#20-junction-lane-mapping--approaches-and-trade-offs-planned) Junction-lane mapping — approaches and trade-offs [PLANNED]

### Chapter 3 — Control
- [§1](#1-npc-traffic-does-not-follow-the-road--drives-straight-and-crashes-fixed) NPC traffic via TrafficManager autopilot
- [§3](#3-fallback-behaviour-when-acc-or-lkas-is-off-decided) Fallback behaviour when ACC or LKAS is off
- [§11](#11-ego-stalls-at-full-acc-throttle--pp--bridge-race-on-apply_control-fixed) PP / bridge `apply_control` race
- [§15](#15-ufld-inference-paused-in-junction--stanleypp-cmd_steer-race-fixed) Stanley / PP cmd_steer race + UFLD pause

### Chapter 4 — Junction policy (evolution over time)
- [§5](#5-junction-policy--ufld-lane-drops-out-stanley-says-hold-car-still-steered-fixed-with-caveat) Stanley HOLD heuristic (v1)
- [§5b](#5b-junction-policy-did-not-visibly-engage-in-testing--diagnostic-logging-added) Diagnostic logging
- [§9](#9-junction-policy--map-based-supersedes-5--5b-fixed) Map-based junction policy (v2 — current)
- [§13](#13-junction-policy-is-now-a-ui-choice-pure-pursuit--hold-straight-done) UI control: Pure-pursuit vs. Hold-straight
- §15 (cross-listed under Chapter 3 — completes the handoff)

### Chapter 5 — Tooling & visualisation
- [§18](#18-foxglove-studio-integration-done) Foxglove Studio integration
- [§19](#19-ipm-birds-eye-view--ipm_view_node-done) IPM bird's-eye view (`ipm_view_node`)

---

## 1. NPC traffic does not follow the road — drives straight and crashes [FIXED]

**Symptom.** NPC vehicles spawned by the CARLA bridge
([carlaaccsim/carlaAccSimTown.py](../../carlaaccsim/carlaAccSimTown.py)) drive
straight from their spawn point, ignore lane geometry, and crash into the
first wall or curb. No routing, no lane keeping, no intersection handling.

**Root cause (suspected).** The bridge spawns the lead vehicle with
`world.try_spawn_actor(...)` but does **not** hand it to CARLA's
TrafficManager. Movement is driven by `run_pure_escape(lead_vehicle,
lead_route, ...)`, which walks a precomputed `lead_route` of waypoints at a
2 m step (carlaAccSimTown.py:79–86). If that thread isn't started, isn't
ticking, or runs out of route, the actor receives no control and physics
carries it straight until impact.

**Reference (working pattern).** `lkas_validate_0.9.16.py:346–393` in
[00_Lane_Assistant/02_UFLD_V2](../../00_Lane_Assistant/02_UFLD_V2/lkas_validate_0.9.16.py)
uses CARLA's TrafficManager:

```python
tm = client.get_trafficmanager()
tm.set_synchronous_mode(True)
tm_port = tm.get_port()
...
actor = world.try_spawn_actor(bp, sp)
actor.set_autopilot(True, tm_port)
```

TM owns routing, lane-following, traffic-light response, and collision
avoidance for every actor registered to it. NPCs spawned this way "just
work" without a hand-rolled route.

**Applied fix.**
- `carlaAccSimTown.py` now hands the lead vehicle to TrafficManager:
  `tm = client.get_trafficmanager(); lead.set_autopilot(True, tm.get_port())`.
- `lead_route` and the `run_pure_escape` thread were removed — TM owns
  routing now. The scripted route stays available in
  `pure_pursuit_controller.run_pure_escape` for the AEBS scenarios.
- Lead is capped at 60 % of the lane speed limit
  (`tm.vehicle_percentage_speed_difference(lead, 40.0)`) so the ego ACC can
  catch up and engage.
- The UI's "Spawn Traffic" button already used TM autopilot; no change
  needed there.

---

## 2. No on-screen indication that ACC / LKAS are running [FIXED]

**Symptom.** The UI camera feed shows the raw bridge image only. There is no
YOLO bounding box on the lead vehicle, no UFLD lane overlay, and no
indicator showing whether ACC or LKAS is currently controlling the car.

**Current state of the code.**
- `perception_node.py:131` already draws YOLO bounding boxes and publishes
  the annotated image on `/ACC/perception/debug_image`.
- `lane_detection_node.py:233` (`annotate(...)`) already draws the UFLD
  ego-left / ego-right polylines and publishes on
  `/LKAS/perception/debug_image`.
- [UI.py](UI.py) only subscribes to `/Car_1/camera/front/compressed`
  (the raw bridge feed). The debug topics are never displayed.

So the visualisations exist — the UI just isn't wired to show them.

**Applied fix.**
- UI.py replaced the single-topic `CameraView` with `TelemetryView`, which
  subscribes to the three image topics + `/Car_1/cmd_vel` +
  `/Car_1/cmd_steer`. JPEGs are stashed raw; only the active source gets
  decoded each render tick.
- The camera widget now has a "Source" combobox: **Raw / ACC (YOLO) /
  LKAS (UFLD)**. Switching is instant — the same camera subscription set
  is always live, the renderer just picks which JPEG to decode.
- Two status labels next to the ACC and LKAS feature buttons read from the
  heartbeat timestamps (1.5 s window):
  - ACC: `● active` when both `/ACC/perception/debug_image` and
    `/Car_1/cmd_vel` are publishing; `◐ partial` when only one is;
    `○ idle` when neither.
  - LKAS: same logic against `/LKAS/perception/debug_image` and
    `/Car_1/cmd_steer`.

---

## 3. Fallback behaviour when ACC or LKAS is off [DECIDED]

### 3a. ACC off → car coasts (current behaviour, kept) [DECIDED]

Killing `controller_node` (what the UI's "ACC: OFF" does today) stops
`/Car_1/cmd_vel` publication. The bridge holds its last-seen throttle/brake
(0/0 on startup), so the car coasts — drag and friction bleed off speed.
No constant-speed cruise.

The only speed target in the system is
[controller_node.py:65](src/controller/controller/controller_node.py#L65)
(`self.target_speed = 20 / 3.6  # m/s`, i.e. 20 km/h), and it only applies
while `controller_node` is running.

**Decision.** Keep the coast behaviour. It's honest about what "ACC off"
means, requires no extra code, and matches the user's mental model. If we
ever want a real-time ACC enable/disable without process restart, the
follow-up work is to add a parameter or topic on `controller_node` and
gate output on it — out of scope for now.

### 3b. LKAS off → ego steers via pure-pursuit fallback [FIXED]

The bridge previously held the last `/Car_1/cmd_steer` value (0 on
startup), so ACC-only mode could not stay in lane on any curve.

**Applied fix.** Re-enabled the pure-pursuit controller that already
existed in [carlaaccsim/pure_pursuit_controller.py](../../carlaaccsim/pure_pursuit_controller.py)
as the steer fallback. The bridge now:
- Precomputes an `ego_route` (~800 m of forward waypoints in the ego's
  starting lane) in [carlaaccsim/carlaAccSimTown.py](../../carlaaccsim/carlaAccSimTown.py).
- Runs `run_pure_pursuit(hero, ego_route, world, should_apply=...)` in a
  background thread, always on.
- The `should_apply=lambda: not avt_node.is_steer_fresh()` gate is the key:
  `CarlaAVT.is_steer_fresh()` returns True for `STEER_FRESH_WINDOW_S` (0.5 s)
  after the last `/Car_1/cmd_steer` message. While LKAS is publishing the
  fallback yields each tick; the moment LKAS goes silent the fallback owns
  steer and the ego stays in lane.

Longitudinal control is unchanged — pure pursuit's `_ego_speed_policy`
reads back the current `VehicleControl.throttle/brake` (set by ACC's
`/Car_1/cmd_vel`) and replays it, so the ACC controller still owns speed.

**Caveats / follow-ups.**
- `ego_route` walks `wp.next(2.0)[0]` (first successor only). It will not
  initiate lane changes; if the ego drifts to a parallel lane the
  precomputed route still aims it back to the original lane.
- The route is finite (~800 m). When the ego runs past the end the
  fallback aims at the last waypoint, which is fine for short demos but
  not for long autonomous runs. A streaming route refresher is the obvious
  next step.

---

## 4. CARLA graphics flicker — ego bonnet + NPC LODs unstable [PARTIAL — sync mode regressed motion, now opt-in]

**Symptom.** When running the full ROS stack (CARLA + bridge +
perception/controller nodes + UI), the ego car's bonnet flickers between
two states each frame, and NPC vehicles' high-LOD geometry pops in and
out. The non-ROS validator
[00_Lane_Assistant/02_UFLD_V2/lkas_validate_0.9.16.py](../../00_Lane_Assistant/02_UFLD_V2/lkas_validate_0.9.16.py)
does not have this problem against the same CARLA install.

**Likely cause (working hypothesis).** The two setups differ in how they
drive CARLA's actor / render pipeline:

- `lkas_validate_0.9.16.py` runs as a single Python process, holds the
  spectator on the ego, and ticks the world directly (sync or async mode
  decided once at startup). One client, one tick source, stable LOD
  selection per frame.
- The ROS stack has at least three concurrent CARLA clients:
  `carlaAccSimTown.py` (bridge), the TrafficManager (lead vehicle
  autopilot — same TM port but separate client session for any extra UI
  NPCs), and incidental clients started by the UI snippets
  (weather/traffic). The bridge currently calls `world.wait_for_tick()` —
  no `world.apply_settings(...)` — so the world is in **async mode**, and
  CARLA's LOD picker sees a non-deterministic frame cadence from each
  client. The bonnet "two-state flicker" is the classic symptom of
  competing client commits between Unreal frames.

**Applied fix.** Mirrored `lkas_validate_0.9.16.py`'s synchronous setup
in [carlaaccsim/carlaAccSimTown.py](../../carlaaccsim/carlaAccSimTown.py):

1. Snapshot `original_settings = world.get_settings()` at startup.
2. Do all spawns (ego, camera, lead) and TM autopilot binding in
   async mode — same order as the validator, since spawn-then-sync is
   the configuration that's known to work.
3. Just before the main loop, flip to sync 20 Hz:
   `settings.synchronous_mode = True;
   settings.fixed_delta_seconds = 0.05; world.apply_settings(settings)`.
4. Set `tm.set_synchronous_mode(True)` so TrafficManager ticks in step
   with the world; otherwise NPC autopilots freeze when we tick.
5. Replace `world.wait_for_tick()` with `world.tick()` in the loop so
   the bridge owns the cadence.
6. Restore `original_settings` (and TM async) in `finally:` so a later
   `lkas_validate` run isn't stuck with our sync configuration.

This also fixes the secondary worry from issue #3 in this list — frame
content was unstable across ticks, which can only have hurt YOLO + UFLD.

**Caveats.**
- All CARLA Python clients touching this world (the bridge, UI snippets,
  TrafficManager) now share one tick source. The UI snippets are
  short-lived and that's still fine, but if anything else opens a
  long-lived client it must avoid `world.tick()` (only one client may
  drive ticks in sync mode).

**Regression observed in field test.** With sync mode forced on, the ego
stalled at 0 m/s — Stanley still entered STANLEY mode with a real lane
error (`e_lat=-0.95 m`), but `vehicle.get_velocity()` stayed at 0 and no
forward motion happened. The exact failure mode (sensor settling? sub-tick
apply_control queueing? interaction with the PP-thread `time.sleep`?) is
not yet diagnosed.

**Mitigation (applied).** Sync mode is now gated behind the
`BRIDGE_SYNC_MODE` env var. Default OFF restores the original
`world.wait_for_tick()` async behaviour; the bridge prints
`sync_mode = False` on startup so it's obvious which path is live. Set
`BRIDGE_SYNC_MODE=1` to opt back into sync mode for flicker debugging.
Stays open until we have a sync setup that keeps the ego moving.

Also added `flush=True` to the bridge's startup `print()` calls — the
prior run produced zero `[bridge]` lines in the UI log because Python
buffers stdout when stdout isn't a TTY (the UI's `subprocess.PIPE`
qualifies). Without those prints flushing, the bridge looked dead even
when it was running.

---

## 5. Junction policy — UFLD lane drops out, Stanley says HOLD, car still steered [FIXED, with caveat]

**Symptom.** Inside a junction the log line shows `[   HOLD]` (Stanley
gave up on UFLD lanes), but the car still steers left or right rather
than holding straight. We had no clean turn behaviour through
intersections.

**What HOLD actually meant in the old code.** In
[stanley_node.py](src/controller/controller/stanley_node.py):

```python
if lookahead is None:
    steer = 0.0
    mode = 'HOLD'
…
self.steer_pub.publish(out)   # always published, even in HOLD
```

So Stanley *did* publish `steer=0.0` every HOLD tick, the bridge applied
it, and the car *should* have driven straight. The "it still steered"
symptom was the bridge's prior `_cmd_steer` value carrying over for a
few frames while Stanley's publication rate caught up, *or* an
intermittent bridge-side stale-state retention — not a Stanley bug per
se, but Stanley's "0.0 is my answer" reply was actively suppressing the
new pure-pursuit fallback (which always sees `is_steer_fresh()` =
True while Stanley is publishing zeros).

**Applied fix.** Stanley no longer publishes during HOLD. The bridge's
`is_steer_fresh()` then goes False after `STEER_FRESH_WINDOW_S`
(0.5 s) and the pure-pursuit fallback (see [3b](#3b-lkas-off--ego-steers-via-pure-pursuit-fallback-fixed))
owns steer through the junction. Stanley resumes the moment UFLD
recovers a lane centre on the far side and the fallback yields back.

This matches the user's proposal: *use pure pursuit at junctions, then
hand back to UFLD on the exit.*

**Caveats / follow-ups.**
- Pure pursuit follows the precomputed `ego_route` from the ego's
  starting lane. At a junction it takes the *first* successor
  (`wp.next(2.0)[0]`), so the chosen turn direction is fixed at
  bridge startup. To pick a turn dynamically per junction we'd need a
  route refresher / decision policy.
- The `[HOLD]` log line now prints `steer=+nan` to make it obvious in
  the log that Stanley deliberately yielded rather than published zero.
- During HOLD Stanley still emits an INFO log every second so the
  operator sees that the fallback is engaged, not that Stanley crashed.

---

## 5b. Junction policy did not visibly engage in testing — diagnostic logging added

The earlier fix in §5 (Stanley stops publishing during HOLD →
pure-pursuit fallback drives the junction) was reported as not working in
the field. Two changes to make the failure mode actually diagnosable on
the next run:

**1. Shorter handoff window.** `STEER_FRESH_WINDOW_S` lowered from 0.5 s
to 0.2 s in [carlaaccsim/custom_ROS_pub_sub.py](../../carlaaccsim/custom_ROS_pub_sub.py).
At 20 Hz Stanley, the bridge now lets PP take over after ~4 missed
publishes instead of ~10. Previously it was plausible the ego had
already traversed enough of a small junction in 0.5 s for the late
handoff to do nothing visible.

**2. Edge-triggered logs on both ends.**
- Stanley now WARN-logs `HOLD — no lane centre at lookahead=… m` on
  every HOLD entry (with `left_pts`/`right_pts` counts so we can tell
  whether UFLD is dropping the polylines entirely or just losing the
  lookahead row), and INFO-logs `STANLEY re-engaged` on exit.
- `pure_pursuit_controller._run_controller` prints
  `[pure_pursuit] ENGAGED (LKAS cmd_steer stale → owning steer)` or
  `[pure_pursuit] YIELDING (LKAS cmd_steer fresh → letting LKAS drive)`
  on every transition.

If after the next run the bridge stdout shows no `[pure_pursuit]`
transitions and Stanley shows no `HOLD` warning at a junction, then
UFLD is not actually dropping the lanes inside junctions — the real
problem is upstream and we need to look at lane_detection_node /
UFLD's confidence floor instead of the Stanley → PP handoff.

---

## 6. Uncontrolled acceleration — ACC ignores closing leads, ego rear-ends them [FIXED]

**Symptom.** With ACC engaged and a lead vehicle clearly visible in the
YOLO debug view (bounding box + distance label drawn each frame), the
ego accelerates as if the road were clear and bumps into the lead at
cruise speed. No EMERGENCY-brake intervention either.

**Root cause.** [controller_node.py:79](src/controller/controller/controller_node.py#L79)
ran the distance filter at `ALPHA = 0.01`:

```python
self.d_lead_filtered = self.ALPHA * d + (1 - self.ALPHA) * self.d_lead_filtered
```

That's a ~100-sample memory. At 5–10 Hz (perception_node's YOLO
inference is slower than the camera publish rate), the filter takes
double-digit seconds to track a real change in distance. Result: when
a lead enters the scene at 30 m and the ego starts closing, the filter
shows the original 30 m for many seconds; the controller computes
`distance_error = d_filtered − d_desired = +large`, requests full
positive acceleration, and the throttle rate limit (1 s ramp to full)
doesn't save us either. EMERGENCY mode only fires below 3 m of *filtered*
distance — by which time we're already through the actual 3 m gap.

**Applied fix.** Raised `ALPHA` to `0.4`. That's a ~2.5-sample memory
(~250–500 ms at typical YOLO Hz), still smoothing single-frame
bounding-box jitter but actually tracking when the lead closes.

**Follow-ups (not done).**
- The pinhole distance estimate in
  [perception_node.py:124](src/perception/perception/perception_node.py#L124)
  uses fixed `OBJECT_HEIGHTS` per class and the YOLO bounding-box
  height. Both numbers are wrong for partially-occluded boxes
  (e.g. when the lead's roof is clipped by the top of the frame at
  close range), and the result is a *systematic over-estimate* of
  distance at close range. A more robust estimate would use the
  bounding-box bottom plus a ground-plane projection (same trick UFLD
  uses for lanes). Today's fix narrows the worst case but doesn't
  eliminate it.

**Tuning follow-up (applied).** Field test showed ACC braking the
moment YOLO acquired a lead. With `d0 = 5 m` and `T_gap = 1.5 s`, the
desired gap formula `d_desired = d0 + T_gap * v_ego` evaluates to
13.4 m at 20 km/h cruise — so any detection inside ~13 m fed a
negative `distance_error` into the PD loop and ACC braked. The
formula is correct (matches the conventional time-headway model);
the values were just too cautious for the demo. Dropped `T_gap` to
**0.5 s**, which gives `d_desired ≈ 7.8 m` at cruise and settles to
`d0 = 5 m` at standstill — matching the user-expected "follow at
roughly 5 m" behaviour. `d0` itself was correct and stays at 5 m.
- `MIN_CONFIDENCE = 0.1` is permissive; spurious low-confidence detections
  on roadside objects could still feed garbage distances into the filter.
  Raise once we know what classes the model produces at confidence
  > 0.3 in our maps.

---

## 7. Combined YOLO + UFLD topic does not run [FIXED — build dependency + load tuning]

**Symptom.** Selecting the new `ADAS (YOLO+UFLD)` source in the UI
shows the placeholder ("waiting for first camera frame…") — the
`/ADAS/perception/debug_image` topic appears to have no publisher.

**Root cause.** Two plausible factors, neither fatal once addressed:

1. **`colcon build` not re-run.** `debug_image_fusion_node` is a *new*
   entry point in `src/perception/setup.py`. Without a fresh build the
   installed `perception` package doesn't know about it, and
   `ros2 run perception debug_image_fusion_node` (invoked from
   `start_acc.sh`) fails immediately with `executable not found`. The
   `start_acc.sh` background runner doesn't surface that failure
   prominently, so the symptom is "topic silently absent".
2. **CPU competition.** Even when launched, the node decodes 2 JPEGs
   and encodes 1 each tick. Running alongside YOLO + UFLD on a single
   GPU/CPU box, the original 12 Hz / quality 85 settings made it the
   most expensive non-inference node in the stack.

**Applied fix.**
- Node lowered to `PUB_HZ = 8` and `JPEG_QUALITY = 75`. Plenty for a UI
  preview; cheap enough not to compete with perception.
- Added a startup INFO log ("Debug-image fusion node started …") so it's
  visible in the start_acc.sh stdout that the entry point loaded.
- Added a 5-second WARN heartbeat that names the source topic(s) that
  haven't produced a frame yet (`/ACC/perception/debug_image`,
  `/LKAS/perception/debug_image`). Tells the operator immediately when
  the fusion node is alive but starved of inputs — i.e. one of the
  upstream perception nodes didn't launch — vs. when fusion itself is
  the missing piece.
- Added a one-shot INFO log on the first successful publish so success
  is visible too.

**Operational note.** After pulling these changes, run
`colcon build --packages-select perception && source install/setup.bash`
before relaunching `start_acc.sh`. The other entry points
(`perception_node`, `lane_detection_node`) survive without a rebuild;
only the new one needs it.

---

## 8. Combined YOLO + UFLD view looks like two overlapping video sequences [FIXED]

**Symptom.** With both ACC and LKAS running, the new combined source in
the UI (`/ADAS/perception/debug_image`) showed a clearly double-exposed
background — as though two video tracks were laid on top of each other.
Felt "unsmoothing and laggy" in real time.

**Root cause.** The first implementation just did
`cv2.max(acc_debug, lkas_debug)`. That's fine *for the bright overlay
pixels* (YOLO greens, UFLD circles), which dominate the max. But the
two debug images came from *different camera frames* — YOLO inference
runs at ~5–10 Hz and UFLD at ~10–15 Hz, so the two perception nodes
publish from camera frames that differ by 50–200 ms. The road,
horizon, and buildings in the BACKGROUND of the two debug images
disagree by exactly that camera-motion offset, and per-pixel max picks
the brighter of the two non-overlay scenes at every pixel — producing
the double-exposed look.

**Applied fix.** Rewrote
[debug_image_fusion_node.py](src/perception/perception/debug_image_fusion_node.py)
to do timestamp-matched mask extraction:

1. Subscribe to the raw camera (`/Car_1/camera/front/compressed`) as a
   third input. Keep a 30-frame ring buffer keyed by `header.stamp`.
2. When an ACC or LKAS debug image arrives, look up the raw frame it
   was computed from (matched by `header.stamp` — both perception
   nodes preserve the original camera header). Compute the overlay
   pixels as
   `mask = max(|debug_bgr − raw_bgr|, axis=-1) > OVERLAY_THRESHOLD`.
3. On every publish tick, paint the latest cached masks onto the
   LATEST raw frame.

So the background is always the most-recent raw frame (smooth, no
ghosting), and the overlays sit at the pixel positions the perception
nodes drew them at. Some lag is unavoidable for the overlays — at
higher speeds the YOLO box may trail the actual lead by a frame or two
— but that's a much milder artifact than the dual-frame ghosting.

`OVERLAY_THRESHOLD = 25` was tuned to reject the JPEG re-encoding noise
between the bridge's q=95 raw and the perception nodes' q=85 debug
outputs while still catching the overlay colours. `MATCH_TOLERANCE_NS =
150 ms` rejects timestamp-matches when the raw buffer hasn't caught up
yet (the perception debug is then skipped that cycle rather than
fused against a stale background).

**Caveats / follow-ups.**
- A "real" fix would have the perception nodes publish their raw
  detection data on separate topics (bounding boxes for ACC, pixel
  polylines for LKAS) and the fusion node would draw fresh onto the
  latest raw. That eliminates the overlay lag entirely. The current
  fix avoids the API changes by recovering the overlay from a pixel
  diff — cheap, but at the cost of small overlay lag.

---

## 9. Junction policy — map-based, supersedes §5 / §5b [FIXED]

§5 and §5b above are the previous "Stanley yields on HOLD → fallback
engages after 0.2 s" approach. It works when UFLD actually loses the
lane inside a junction, but in practice UFLD often keeps producing
*something* (curb lines, crosswalk markings) and Stanley stays in
STANLEY mode through the intersection, steering against whatever
garbage it sees.

**Applied fix.** Lifted the CARLA-map-based junction detection from
`00_Lane_Assistant/02_UFLD_V2/lkas_validate_0.9.16.py:junction_steer`
and wired it into the bridge:

1. `carlaAccSimTown.py` runs a `junction_monitor` thread at 10 Hz that
   queries `world.get_map().get_waypoint(ego.location)` and decides
   whether the ego is in (or approaching) a junction zone, using the
   same `JUNCTION_ENTRY_LOOKAHEAD_M = 2.0` /
   `JUNCTION_EXIT_LOOKAHEAD_M = 6.0` thresholds as lkas_validate.
2. When in-junction, the monitor calls `avt_node.set_in_junction(True)`.
3. `custom_ROS_pub_sub.CarlaAVT.is_steer_fresh()` now returns False
   *unconditionally* while `_in_junction` is True — Stanley keeps
   publishing, but its authority is revoked.
4. The existing pure-pursuit fallback (via `should_apply=lambda: not
   avt_node.is_steer_fresh()`) takes steer through the intersection
   along `ego_route`. UFLD and Stanley continue to run; their cmd_steer
   is just discarded for the junction window.
5. `build_ego_route()` now picks the lane successor whose yaw is
   closest to the current heading at each fork — i.e. "drive straight
   through" by default — instead of the old `wp.next(2.0)[0]`
   arbitrary-first-successor walker.

This is the user's literal ask: *"use junction detection, switch off
UFLD, use pure_pursuit.py as long as in junction"*. UFLD isn't
literally killed — but its output is ignored — which is the same
operational effect and avoids the complexity of stopping/starting a
heavyweight inference node mid-run.

The bridge prints `[junction] ENTER` / `EXIT` events to its stdout, so
the operator can see the handoffs in the UI log.

Switchable: `carlaAccSimTown.py --junction-policy none` reverts to the
§5/5b heuristic (Stanley yields only when it actually enters HOLD).

**Follow-ups (not done).**
- Heading-aligned route picks "straight through" — to take a specific
  turn at a specific junction we'd need a route planner on top of
  `build_ego_route`, or a higher-level routing client.
- The 2 m entry / 6 m exit constants are taken straight from
  lkas_validate; if junctions in Town01 feel premature/late we can
  expose those as flags too.

---

## 10. Bridge hard-coded for one scenario — now fully argparse-driven [FIXED]

**Symptom.** Town, weather, vehicle blueprint, traffic count, camera
resolution, spawn index were all module constants in
`carlaaccsim/carlaAccSimTown.py`. The non-ROS validator
`lkas_validate_0.9.16.py` had been argparse-driven from the start and
ran cross-town / cross-weather smoothly; the bridge couldn't.

**Applied fix.** Mirrored lkas_validate's argparse interface in the
bridge. New flags (see `carlaaccsim/README.md` for the full table):

- `--port`, `--town`, `--vehicle`, `--weather`, `--traffic`
- `--lead-speed-pct`, `--lead-gap-m`
- `--spawn-index`, `--list-spawns`
- `--cam-width`, `--cam-height`, `--cam-fov`, `--cam-tick`
- `--ego-route-len`
- `--junction-policy {pp-takeover,none}`

Defaults:
- Camera bumped from **1280×720 → 1920×1080** to match the validator.
  This roughly doubles the pixel count of a far-car bounding box and
  improves YOLO distance accuracy at range, which contributed to the
  Town03 early-brake behaviour we were seeing (§6 follow-up).
- All other defaults preserve the previous hard-coded behaviour, so
  running the bridge with no flags is the closest behavioural match to
  the old script.

**Related downstream fix.**
`src/perception/perception/perception_node.py` was hard-coded for
1280-wide imagery (`FOCAL_LENGTH = 640.0`). The distance formula now
computes the focal length per frame from the actual image width and a
ROS parameter `camera_fov_deg` (default 90), so distances stay
correct at 1080p or any other resolution the bridge ships.

**Follow-up (not done).**
- The UI's "Start Bridge" button launches the bridge with no flags.
  Threading the new flags through the UI (town selector → `--town`,
  weather → `--weather`, traffic count → `--traffic`, junction
  toggle → `--junction-policy`) is the next step.

---

## 11. Ego stalls at full ACC throttle — PP / bridge race on `apply_control` [FIXED]

**Symptom.** With everything launched from the UI, `ros2 topic` showed
ACC commanding `/Car_1/cmd_vel.linear.x = 1.0` (full throttle) at a
steady 20 Hz, `/Car_1/vehicle/speed` publishing fine, no spin-thread
exception in the bridge — but the ego sat at 0 m/s. New behaviour
appeared after §9's map-based junction policy went live, but the root
cause was an older latent bug exposed by it.

**Root cause.** The pure-pursuit fallback's speed policy in
[carlaaccsim/pure_pursuit_controller.py](../../carlaaccsim/pure_pursuit_controller.py)
was reading throttle/brake back from CARLA, then re-applying them
alongside its computed steer:

```python
def _ego_speed_policy(vehicle, speed, min_index):
    ctrl = vehicle.get_control()    # ← effective control from PREVIOUS tick
    return ctrl.throttle, ctrl.brake
```

`carla.Actor.get_control()` returns the effective control from the
last physics step, not the latest queued `apply_control` call. Between
ticks the bridge would receive ACC's `cmd_vel(throttle=1.0)` and
queue a fresh `apply_control(throttle=1.0)`; PP would then tick a
millisecond later, read `get_control().throttle = 0` (the still-effective
prior-tick value), and queue `apply_control(throttle=0)`. The later
queue entry wins per actor → CARLA applied throttle=0 for the next
physics step. Car never moved.

This race had always existed, but PP only engages when
`is_steer_fresh()` is False — previously that meant just "LKAS isn't
publishing", which was rare during normal demos. §9's junction monitor
now flips `_in_junction=True` whenever the ego is in a junction zone,
so PP started engaging at startup (Town03's `spawn_points[0]` happens
to sit in a junction). PP then ran constantly and the throttle never
got through.

**Applied fix.** PP now reads throttle/brake from the *bridge's
authoritative state* (the values most recently received on
`/Car_1/cmd_vel`) instead of from `vehicle.get_control()`. Both threads
agree on the same value, so even though PP queues its own
`apply_control` after the bridge's, the throttle written is identical
to what ACC commanded — the bridge's value survives. PP only owns
steer in practice.

The plumbing:
- `custom_ROS_pub_sub.CarlaAVT.current_throttle_brake() → (throttle, brake)`
  returns the most recent `cmd_vel` values.
- `pure_pursuit_controller.run_pure_pursuit(..., throttle_brake_provider=callable)`
  accepts an optional provider; when supplied, the speed policy calls
  it instead of `get_control()`. `_ego_speed_policy` was refactored
  into `_make_ego_speed_policy(throttle_brake_provider)`.
- `carlaAccSimTown.py` passes
  `throttle_brake_provider=avt_node.current_throttle_brake` when
  starting the PP thread.

`run_pure_escape` is unaffected — the lead vehicle has its own escape
speed policy, no ROS handoff involved.

**Caveat / follow-up.** This fix assumes the bridge's ROS callbacks
(spin thread) are alive. If the spin thread dies for any reason,
`current_throttle_brake()` returns the last value it saw before the
crash — possibly stale 0. The earlier spin-exception hunt (still
intermittent in heavy-traffic Town03 runs) remains an open thread
above this in the stack.

---

## 12. Ego still stalls at full ACC throttle — bridge JPEG encoder starves the ROS executor at 1920×1080 [FIXED, with caveat]

**Symptom.** After §11's fix went in we hit the same end-user symptom
again: ACC publishing `/Car_1/cmd_vel.linear.x = 1.0` at a steady 20 Hz,
Stanley publishing fresh `cmd_steer`, but the ego sat at 0 m/s. Visual
camera feed showed no motion. Two diagnostic signatures gave the cause
away:

- `ros2 topic hz /Car_1/vehicle/speed` and `…/ACC/lead_vehicle_distance`
  **timed out** even though `ros2 topic info` showed the bridge as the
  publisher.
- The bridge process was sitting at **117 % CPU** with no ADAS load.

`/Car_1/cmd_vel` was alive and steady (ACC was healthy), but none of the
bridge's *own* topics — speed, distance, the camera — were actually
making it out to the wire. The bridge's spin thread was running but
its callbacks weren't getting CPU.

**Root cause.** `custom_ROS_pub_sub.CarlaAVT` uses a single-threaded
ROS executor (`rclpy.executors.SingleThreadedExecutor`). Every
subscription callback and every timer callback runs on the same
thread. With the §10 camera bump to **1920 × 1080 at JPEG quality 95**,
`_publish_camera` became expensive enough (~30–50 ms per frame; the
inner `while not self.image_queue.empty()` drains any backlog in one
call) that it monopolised the executor. `_cmd_vel_cb` then fired
late or not at all, so the bridge's `self._throttle` stayed at its
initial `0.0`. The pure-pursuit fallback's speed policy read that
stale 0 via `current_throttle_brake()` (§11) and dutifully applied
`apply_control(throttle=0, …)` to CARLA at 20 Hz. Car never moved.

The reason §11's fix held under the validator but broke under the ROS
stack is exactly the same load asymmetry: the validator runs as one
process at low quality presets, the ROS stack runs the bridge alongside
two GPU-heavy perception nodes that steal cycles and make the encoder
fall further behind.

**Applied fix.** Lowered the bridge's default camera resolution from
1920×1080 to **1280×720** in
[carlaaccsim/carlaAccSimTown.py:113-118](../../carlaaccsim/carlaAccSimTown.py#L113-L118).
Encoding cost drops ~2.25× and the single-threaded executor regains
enough headroom that `_cmd_vel_cb` fires on every message. The ego
now moves the moment ACC commands throttle, confirmed end-to-end.

**Caveats / follow-ups.**
- The real fix is a multi-threaded executor in the bridge (or moving
  JPEG encoding off the executor thread), so subscription callbacks
  can run in parallel with image publishing. The resolution drop just
  raises the load ceiling — at full traffic + perception load 1280p
  may still creep up on it.
- 1280p loses pixel area for YOLO at range. §6's follow-up about
  pinhole distance accuracy at long range gets slightly worse.
  Acceptable for now; the alternative was a non-moving car.
- The bridge can still be run at 1920×1080 explicitly with
  `--cam-width 1920 --cam-height 1080` — useful for offline dataset
  capture where ACC throttle isn't in the loop.

---

## 13. Junction policy is now a UI choice (Pure pursuit / Hold straight) [DONE]

**Background.** §9 added CARLA-map junction detection with one
behaviour: inside a junction zone, the bridge sets `_in_junction =
True`, `is_steer_fresh()` returns False, and the pure-pursuit fallback
follows the precomputed `ego_route` through the intersection. That's
fine when the ego is following a well-formed route, but for an
X-junction where "drive straight through" is the right answer, holding
`steer = 0` is simpler, needs no route, and avoids PP's wheel-overshoot
on tight corners. The non-ROS validator at
`00_Lane_Assistant/02_UFLD_V2/lkas_validate_0.9.16.py:320` already
supported both via `--policy {hold-straight, map-follow, pure-pursuit,
none}`, exposed in the validator's UI by a Combobox.

**Applied changes.**

- **Bridge.** `--junction-policy` choices extended from
  `{none, pp-takeover}` to `{none, pp-takeover, hold-straight}` in
  [carlaaccsim/carlaAccSimTown.py](../../carlaaccsim/carlaAccSimTown.py).
  The junction monitor now passes the policy through to
  `CarlaAVT.set_in_junction(in_junc, policy=…)`.
- **CarlaAVT.** New `_junction_policy` field in
  [carlaaccsim/custom_ROS_pub_sub.py](../../carlaaccsim/custom_ROS_pub_sub.py).
  `is_steer_fresh()` returns True inside a junction under
  `hold-straight` (so the PP fallback yields), False under
  `pp-takeover` (so PP engages). `_apply_control()` clamps `steer = 0`
  when in-junction under `hold-straight` — that overrides whatever
  Stanley most recently published.
- **UI.** New "Junction policy" combobox in
  [UI.py](UI.py) Processes section with the two labels
  *Pure pursuit* (maps to `pp-takeover`) and
  *Hold straight* (maps to `hold-straight`). The chosen policy is
  passed to the bridge as `--junction-policy <value>` at Start Bridge.
  Switching mid-run has no effect — restart the bridge to apply.

**Why no UI option for `none`.** The user's only ask was the two
operational policies. `none` (LKAS keeps steer authority through
junctions) is available on the command line for debugging but isn't
useful in normal driving — it's the exact behaviour §5/§9 fixed by
*not* letting LKAS steer against curbs and crosswalk markings.

**Caveats / follow-ups.**
- Like the sync-mode checkbox, the policy choice is baked into the
  bridge subprocess at launch. A future improvement is a runtime ROS
  parameter on `CarlaAVT` so the operator can hot-swap policies; the
  scaffolding (`set_in_junction(policy=…)`) is in place for it.
- `hold-straight` is genuinely "go straight" — at a T-junction where
  the road bends, the car will drive into the kerb. Pick `pp-takeover`
  in maps with frequent T-junctions; `hold-straight` shines on X-grid
  towns (Town01/Town03 cores).

---

## 14. Bonnet flicker is worse in Town10HD than Town03 [KNOWN, mitigation only]

Operator-confirmed during demo: the §4 bonnet flicker visibly worsens
when switching from Town03 to Town10HD. Same code, same bridge config,
same camera resolution — only the map changes.

**Why.** Town10HD is the high-density urban map and has roughly 2–3×
the environment-object count of Town03. The flicker is Unreal's LOD
picker resolving inconsistent state across multi-client commits
between frames (see §4); the more meshes near the camera, the more
likely the bad picks land on geometry that's visually prominent (and
the bonnet is *right* in front of the camera). Town10HD also takes
longer per frame on the same GPU, which widens the window during
which the bridge / TrafficManager / UI snippets can race on
`apply_*` calls — more race window means more inconsistent frames.

**Mitigations (no fix, sync mode would have addressed it but
regressed motion per §4).**
- For demos that care about visual quality, default to **Town03**.
- For Town10HD specifically: set **Quality: Low** in the UI dropdown
  before Start CARLA. Low quality removes a lot of LOD tiers so
  there's less to flicker between, and shortens frame time.
- Combine Low quality with the 1280×720 bridge default (§12) for the
  most stable look.
- Avoid concurrent UI helper actions (Apply Weather, List spawns,
  Spawn Traffic) while recording or screenshotting — each adds a
  client that races on commits.

---

## 15. UFLD inference paused in junction + Stanley/PP cmd_steer race [FIXED]

**Symptom.** With the §9/§13 junction policy active and
`pp-takeover` selected, the operator could see the pure-pursuit
fallback "kick" steer about once a second instead of the smooth 20 Hz
control it produces outside junctions. UFLD continued running on
junction frames (curbs, crosswalk markings) and Stanley kept
publishing `/Car_1/cmd_steer` against that garbage, which felt wrong
even though §9's `is_steer_fresh()` was supposed to override LKAS.

**Two root causes**, both in the bridge's
[carlaaccsim/custom_ROS_pub_sub.py](../../carlaaccsim/custom_ROS_pub_sub.py):

1. **Bridge / PP `apply_control` race.** `_cmd_steer_cb` calls
   `_apply_control()` on every Stanley publish (~20 Hz), writing
   `(throttle, brake, Stanley_steer)` to CARLA. The pure-pursuit
   thread runs its own 20 Hz loop and writes
   `(throttle, brake, PP_steer)`. Whichever call lands closest to
   the next physics tick wins. Roughly half the ticks Stanley's
   stale steer beat PP's fresh value — that's the "1 Hz" feel.
2. **Lane data was still flowing from junction frames.** UFLD ran on
   every camera frame, including inside the junction box. Stanley
   stayed in `STANLEY` mode (not HOLD) and kept publishing
   `cmd_steer` against curbs/crosswalk lines — feeding the race
   above with bogus values.

**Applied fix.**

- **Bridge.** `_apply_control()` early-returns when
  `_in_junction and _junction_policy == 'pp-takeover'`. PP owns
  apply_control exclusively in that window — it already writes
  throttle/brake (via `current_throttle_brake()` per §11) and steer.
  The Stanley write is dropped on the floor for the duration.
- **Bridge.** New publisher `/Car_1/in_junction`
  (`std_msgs/Bool`, depth 1) — published on every `set_in_junction`
  call so downstream subscribers see ENTER/EXIT immediately.
- **lane_detection_node.** Subscribes to `/Car_1/in_junction`. While
  True, skips UFLD inference entirely; emits empty Paths on
  `/LKAS/ego_lane_left` and `/LKAS/ego_lane_right` (Stanley reads
  empty Paths as HOLD and stops publishing `cmd_steer`); publishes
  the raw camera frame with a `JUNCTION (UFLD paused)` overlay on
  `/LKAS/perception/debug_image` so the operator sees the pause.

Net result: in a junction, PP is the only writer to
`vehicle.apply_control`, Stanley is silent (HOLD), UFLD doesn't burn
GPU cycles on frames it can't reason about, and the user-visible
steer trace is a clean 20 Hz curve instead of a 1 Hz step.

**Caveats / follow-ups.**
- The Stanley HOLD message stops firing in the log while UFLD is
  paused — Stanley simply sees no Paths. If you want a positive
  "Stanley paused for junction" log signal, it has to come from
  Stanley reacting to the same `/Car_1/in_junction` topic.

---

## 16. ACC lane ROI via UFLD vehicle-frame IPM [DONE]

**Background.** Pre-existing ACC perception filtered YOLO detections
to "within 20 % of image-centre horizontal", which is a crude
substitute for "is this car in my lane". Cars in the adjacent lane
near the image centre passed; cars in the ego lane at the periphery
of the image (curving road) failed. With UFLD already producing the
ego lane polylines, we can do better.

**First attempt (replaced).** lane_detection published image-space
polylines as `Float32MultiArray` on
`/LKAS/ego_lane_{left,right}_px`; perception built a closed polygon
and called `cv2.pointPolygonTest` on each detection's
bottom-centre. Worked, but introduced a second coordinate frame
(image space) for the same lane data Stanley was already consuming
in vehicle frame.

**Applied design.** Use the IPM that lane_detection already uses for
Stanley. perception subscribes to `/LKAS/ego_lane_left` and
`/LKAS/ego_lane_right` (`nav_msgs/Path`, vehicle frame, REP 103) and
runs the same `ipm_pixel_to_vehicle` to ground-project each
detection's bottom-centre into the road plane. A detection is
in-lane iff its `Y_left` lies between the interpolated left and
right lane Y at its `X_forward`. Same projection model, same
coordinate frame, two consumers — no duplicated topics.

- Files:
  [src/perception/perception/perception_node.py](src/perception/perception/perception_node.py),
  [src/perception/perception/lane_detection_node.py](src/perception/perception/lane_detection_node.py).
- Camera extrinsics added as ROS parameters
  `cam_height_m` (1.35) and `cam_x_offset` (0.6), matching the
  bridge rig and lane_detection_node's existing defaults.
- The image-space `_px` topics were removed — clean redesign.
- Fallback: if either lane Path is empty (UFLD warm-up, junction
  pause, or detection X beyond the polyline range), perception
  falls back to the legacy centre-strip filter so ACC isn't blind.
- The lane polygon was previously drawn on the ACC debug image; the
  user asked for it to be removed so the ACC view stays focused on
  YOLO boxes — the lane is already shown on the LKAS source.

**Caveats / follow-ups.**
- Both nodes hard-code the 1.35 m / 0.6 m camera rig. If the bridge
  ever ships a different mount, both nodes' ROS parameters need to
  be updated together. A bridge-published `/camera_info`-style
  topic would centralise this.
- IPM assumes a flat ground plane. On steep grade or speed bumps,
  the projected (X, Y) gets noisy; we're not seeing this in CARLA
  but real-world deployment would want a tilt-corrected variant.

---

## 17. Synchronous-mode UI control removed [DONE]

§4's sync-mode opt-in was exposed in the UI as a checkbox. After
§12's resolution fix landed and the ego stopped stalling in async
mode, the operator never enabled sync mode in practice — flicker is
tolerated in exchange for guaranteed motion. To reduce surface area
and confusion, the UI control was retired:

- Removed the `Bridge: synchronous mode` Checkbutton and its
  `bridge_sync_var` BooleanVar from [UI.py](UI.py).
- Removed the `BRIDGE_SYNC_MODE=1` env-var injection in
  `start_bridge`.

The bridge still honours `BRIDGE_SYNC_MODE=1` if set on the command
line — kept as an escape hatch for flicker-only investigations. The
UI just doesn't surface it any more. §4's post-mortem stays as-is;
the feature isn't deleted, just hidden behind a CLI knob.

---

## 18. Foxglove Studio integration [DONE]

The team uses Foxglove Studio for live telemetry visualisation.
`foxglove_bridge` is a separate ROS 2 node that opens
`ws://localhost:8765` for the Studio app to connect to — it's not
auto-started by anything in the stack, so the operator had to
remember to launch it manually each session and the saved layout
would silently fail to connect.

**Applied changes.**

- Added **Start Foxglove** / **Stop Foxglove** buttons to the UI's
  Processes group ([UI.py](UI.py)). Behind the scenes:
  `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`, streamed
  into the UI log with the `[foxglove]` prefix.
- The Foxglove process is independent of CARLA / Bridge / ADAS — it
  can be started before any of them and stays up for rosbag playback
  after the stack is torn down. Window-close also tears it down.

**Recommended starter layout.**
- 3-series Plot panel for cmd_vel.linear.x (throttle),
  cmd_vel.linear.y (brake), cmd_steer.data (steer). Put throttle on
  a separate Y-axis from velocity or it'll be crushed at the bottom
  of a shared 0–5 m/s scale.
- Indicator panel on `/Car_1/in_junction` — big colored block that
  lights up on PP / hold-straight handoff.
- Log panel filtered on `/rosout` — replaces grepping the UI log
  for HOLD warnings, junction ENTER/EXIT prints, etc.
- Image panels on `/ACC/perception/debug_image` and
  `/LKAS/perception/debug_image` (and `/ADAS/perception/debug_image`
  for the fused view if launched).

**Caveats / follow-ups.**
- Stanley logs `e_lat`, `e_head`, and STANLEY/HOLD mode as text
  only — none of those are publishable today, so they can't be
  plotted in Foxglove. Adding `/LKAS/stanley/e_lat`,
  `/LKAS/stanley/e_head` (Float32) and `/LKAS/stanley/mode` (String
  or enum) is ~5 lines in `stanley_node.py` and would give full
  closed-loop lateral diagnostics from Foxglove.

---

## 19. IPM bird's-eye view — `ipm_view_node` [DONE]

**Background.** The LKAS perception-debug image shows the lanes in
*pixel space* of the forward camera: convenient for visual cross-
check against the road, but hard to read distances off ("is that
lane 30 m or 10 m away?"). We needed a top-down, metric view of the
same lane geometry — both as a sanity check on the IPM the
controllers consume, and as a foundation for future work that needs
ground-plane reasoning (junction-lane mapping, lead detection in BEV,
etc).

**v1 — blank canvas with lane dots.** First pass at
[src/perception/perception/ipm_view_node.py](src/perception/perception/ipm_view_node.py)
just drew the `/LKAS/ego_lane_left` / `_right` polylines on a black
canvas using `veh_to_bev` (the inverse of UFLD's IPM). Useful for
showing *what UFLD believes the lanes are*, but couldn't show whether
those beliefs matched the actual road — "straight UFLD line on
curving road" and "straight UFLD line on straight road" looked the
same.

**v2 — warped camera + lane overlay [current].** Same node now
warps the live `/Car_1/camera/front/compressed` frame to the BEV
canvas using a fixed homography, then draws the UFLD polylines on
top of the warped asphalt.

- The homography is computed once per camera resolution from four
  ground control points — a trapezoid at 5 m / 25 m forward × ±3 m
  laterally — projected forward into the image with CARLA's pinhole
  + the canonical extrinsics (camera height 1.35 m, x-offset 0.6 m,
  FOV 90°). Same numbers as `lane_detection_node`'s IPM so the warp
  and the polylines share a coordinate frame.
- Re-computed automatically if the camera resolution changes (the
  bridge can run at 720p or 1080p; §12).
- Published on `/ADAS/ipm/debug_image` at 10 Hz so a Foxglove Image
  panel can show it alongside the LKAS / ACC views.
- The warp dims the road texture multiplicatively so the
  blue/green UFLD overlays read clearly against the asphalt.

**Why this is useful.**

1. **Camera-calibration sanity check.** On straight road the warped
   lane paint runs vertically (parallel to the image columns). If it
   diverges with distance, the camera height / x-offset / FOV
   constants are wrong — and the same constants drive ACC's lane-ROI
   filter and Stanley's lateral error. Easier to spot it here than
   to back it out of controller behaviour.
2. **UFLD honesty check.** If UFLD's blue/green dots don't track the
   *real* lane paint on the warped image, UFLD is hallucinating.
3. **Foundation for junction-lane mapping.** Same homography can
   project any vehicle-frame polyline — including `carla.Map`
   junction waypoints — onto the same canvas, making it cheap to
   visualise *every* drivable lane through a junction, not just the
   one UFLD currently follows.

**Caveats / follow-ups.**
- IPM beyond ~25 m is unreliable. The ground-plane assumption breaks
  on grades and the image-pixel quantisation amplifies (1 px maps
  to many metres of ground at the horizon). The far-row control
  point sits at 25 m for that reason; pushing it further would make
  the near texture look correct but the far texture nonsensically
  stretched.
- The node is not wired into `setup.py` or `start_acc.sh` yet — run
  directly with
  `python3 src/perception/perception/ipm_view_node.py`. Add an
  entry point if it becomes part of the regular launch.
- `cv2.warpPerspective` at 10 Hz is cheap on the CPU side
  (~ms-scale at the published 1280×720) and decouples cleanly from
  the GPU-bound perception nodes. No load issue.

---

## 20. Junction-lane mapping — approaches and trade-offs [PLANNED]

**Background.** The current junction stack ([§5](#5-junction-policy--ufld-lane-drops-out-stanley-says-hold-car-still-steered-fixed-with-caveat) →
[§9](#9-junction-policy--map-based-supersedes-5--5b-fixed) →
[§13](#13-junction-policy-is-now-a-ui-choice-pure-pursuit--hold-straight-done) →
[§15](#15-ufld-inference-paused-in-junction--stanleypp-cmd_steer-race-fixed))
suppresses UFLD inside junction zones and either holds steer = 0 or
runs pure-pursuit along a *single* precomputed `ego_route`. This
works *operationally* — the ego crosses an X-junction without
straying — but it doesn't actually *map* the junction topology: we
can't see every possible exit, can't pick a turn dynamically from a
route plan, and can't verify post-junction that the ego ended up in
a legal exit lane.

To upgrade beyond a single hard-coded route we need a representation
of **every drivable lane through the junction** in the ego's vehicle
frame, refreshed online. Three families of approaches exist, in
roughly increasing order of effort and decreasing reliance on prior
information:

### Approach A — CARLA Map API (sim-only, ground truth)

CARLA exposes the full lane topology of the loaded town through its
Python API: `world.get_map().get_topology()` returns every connected
`(start_wp, end_wp)` lane pair in the map. For a junction
specifically, `wp.get_junction()` retrieves the junction object and
`junction.get_waypoints(carla.LaneType.Driving)` returns *every
entry-exit waypoint pair through that junction* — left turn, right
turn, straight, and any extra connectors.

- **Method.** Detect the upcoming junction (the existing
  `junction_monitor` already does this). At ENTER, query
  `get_waypoints` for every entry-exit pair, walk each pair at 2 m
  resolution to obtain polylines in *world* coordinates, transform
  to *vehicle* frame using `ego.get_transform()`, hand the
  polylines to the IPM node ([§19](#19-ipm-birds-eye-view--ipm_view_node-done))
  for rendering — each path in a different colour.
- **Effort.** ~50 lines, mostly in the bridge.
- **Result.** Perfect lane topology in sim. Doesn't generalise to
  real-world (no equivalent API).
- **Right next step here** — it builds directly on what we already
  have and shows immediately whether *visualising* every exit is
  useful in the first place.

### Approach B — Online camera-based BEV lane networks

Train (or fine-tune) a neural network that takes the forward camera
(or a multi-camera surround view) and outputs lane geometry directly
in BEV. Modern state of the art:

- **StreamMapNet** (Yuan et al., 2024) — transformer-based
  encoder, temporal stream of BEV features, outputs **vector lanes**
  with type labels (divider, boundary, centreline) in real time. The
  user's intended thesis target.
- **MapTR / MapTRv2** (Liao et al., 2022/2024) — earlier vector
  lane networks; MapTRv2 added multi-class instances and is the
  reference baseline.
- **HDMapNet** (Li et al., 2022) — rasterized BEV semantic maps
  + post-hoc vectorisation. Simpler but less direct.
- **Lift-Splat-Shoot** (Philion & Fidler, 2020) — the lifting
  backbone many BEV networks build on; gives a top-down feature map
  from N cameras via per-pixel depth estimation.

- **Method.** Replace `lane_detection_node`'s single-lane
  UFLD output with a multi-lane vector head. In sim: train on
  nuScenes / Argoverse-2 / Waymo Open Map for transfer, or
  synthesise CARLA ground truth from Approach A's
  `get_waypoints` calls (CARLA-native dataset, no domain gap, but
  no real-world generalisation either).
- **Effort.** Substantial — model architecture, training pipeline,
  evaluation against ground truth, integration into the ROS stack.
  Thesis-scope work.
- **Result.** Generalises beyond CARLA (depending on training data),
  no reliance on a pre-built map. State of the art for *online* HD-
  map prediction.

### Approach C — Pre-built HD maps (production reality)

The map is recorded *offline* with a dedicated survey vehicle
(LiDAR + GNSS-INS), aligned to centimetre scale, and shipped in the
car. Online perception then mostly **localises in the map** and
**confirms it is still valid** (construction, snow, repainted
markings).

- **Method.** Pre-record lane geometry for every junction the car
  is allowed to operate in. At runtime, localise with high accuracy
  (RTK-GNSS + IMU + LiDAR/vision feature matching) and look up the
  junction's lane topology from the on-board HD map.
- **Effort.** Lowest *online* compute, highest *offline* logistics:
  survey vehicles, map storage, change-detection pipeline,
  geographic restriction of the operational domain (ODD).
- **Result.** Highest reliability, lowest ODD breadth. Not suitable
  for a research project in CARLA, but it's what makes
  Level-3-certified consumer systems (see below) possible today.

### What current OEM ADAS systems use

- **Tesla (FSD / AP, vision-only).** Single neural backbone
  ("HydraNet") with multiple heads — among them vector lane
  prediction, object detection, traffic-light state, drivable
  space, and the more recent occupancy network for arbitrary 3D
  obstacles. Eight cameras → BEV transformer → vector lanes. No
  LiDAR, no radar, no HD map. Public direction is increasingly
  end-to-end neural planning (cameras → control). Mobileye and
  Wayve are pursuing similar.
- **Mercedes Drive Pilot (Level 3, S-Class / EQS).** The opposite
  philosophy: **HD map + LiDAR + radar + cameras + ultrasonic +
  high-precision GNSS/IMU.** Operates *only* on pre-mapped highway
  segments (Germany, Nevada, California) at up to 95 km/h. The HD
  map provides lane topology; onboard perception confirms presence
  of lane lines and vehicles and localises within centimetres.
  Conservative ODD is the trade-off for Mercedes taking legal
  liability while engaged.
- **Waymo / Cruise / Mobileye Chauffeur.** Closer to Mercedes' end
  — LiDAR + HD map + multi-modal perception, with neural BEV
  networks layered on top for redundancy. Mobileye additionally
  crowd-builds a thin HD map ("Road Experience Management") from
  production-vehicle camera feeds while the car also runs
  vision-only perception.

The field is bifurcating into "vision-only, big-data, big-model" on
one side (Tesla, Wayve, Mobileye SuperVision) and "HD-map + sensor-
fusion, narrow ODD, certified" on the other (Mercedes, Waymo).
Junction-lane mapping is where the two diverge most visibly:
vision-only systems must *predict* it online; HD-map systems can
*look it up*.

### Recommended sequencing for this stack

1. **Approach A first** (immediate, ~50 lines). Renders every
   junction lane on the existing IPM BEV canvas. Validates the
   visualisation + IPM math against ground truth before any
   neural component is involved.
2. **Approach B (StreamMapNet) for thesis novelty.** Synthesise
   CARLA training data using Approach A as the labeller, train
   StreamMapNet, replace UFLD's single-lane output with multi-lane
   vector predictions. Compare against Approach A ground truth
   inside CARLA; evaluate generalisation by additionally running
   on a real-world dataset (nuScenes mini, OpenLane).
3. **Approach C is out of scope** for a CARLA-only research
   project, but worth a half-page in the thesis discussion as the
   industrial reference point.

**Open questions / decisions.**

- *Training-data realism.* CARLA's camera and lane geometry have a
  domain gap to real-world driving (lighting, weather variety,
  marking deterioration). A CARLA-only-trained model may not
  transfer. The mitigation is mixing CARLA + a real-world dataset,
  or pre-training on real and fine-tuning on CARLA.
- *Evaluation metric.* MapTR-family papers use Chamfer distance and
  AP-by-class against vectorised ground truth. CARLA's
  `get_waypoints` gives us perfect ground truth for free — no
  manual labelling.
- *Latency target.* If the StreamMapNet output feeds the same
  Stanley / pure-pursuit hand-off as today's UFLD does, it needs to
  meet ≥10 Hz with bounded latency. Real-time inference budget on
  the dev GPU is the gating constraint.
