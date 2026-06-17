# DEBUG / Known Issues

Working notes on open bugs, regressions, and behavioural questions for the
ADAS stack. Clean usage docs live in [README.md](README.md); this file is for
"what's broken and what we still need to fix" only.

Items marked **[FIXED]** have an applied patch; the description is kept so we
remember what was wrong and why we changed it.

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
