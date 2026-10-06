<!-- pico-cli:plugin-context:pico-spatial-agentic-tools:start -->
## Plugin Context

Also read `./PICO-SPATIAL-AGENTIC-TOOLS.AGENTS.md` for PICO Spatial plugin guidance.
<!-- pico-cli:plugin-context:pico-spatial-agentic-tools:end -->

## Project Handoff

- The project and launcher label are `Spatial Chopper Toy`; the Android application ID and Kotlin
  namespace are `com.pico.spatial.choppertoy`.
- The application is a Full Space `DefaultStage` PICO Spatial SDK app.
- `HomeStage.kt` owns the always-on tracking lifecycle, first-install tutorial coordination, palm
  instrument cluster, and Spatial drag gesture.
- `HelicopterSceneController.kt` owns the helicopter entity, its attached launch panel, plane
  colliders, physics, palm input, tutorial motion, safe-landing classification, entity-bound
  spatial audio, and the two-state flight lifecycle.
- AnyController and the RC Drone mode are removed. The app starts directly with palm, HMD, and
  plane tracking; there is no launch-time mode chooser.
- The helicopter state machine is `NOT_RUNNING -> RUNNING -> NOT_RUNNING`, with a separate
  `NONE`/`CRASHED`/`LANDED` end reason. There is intentionally no Stop, Exit, or alert-dialog flow.
- START/RESTART is enabled only when a palm is ready. In `NOT_RUNNING`, the helicopter is kinematic,
  gravity-free, and draggable.
  In `RUNNING`, the panel and drag interaction are disabled and the combined lift/gravity terminal-
  velocity solver is active.
- START/RESTART calibrates the current filtered palm pitch/roll/yaw as zero, preserves current yaw,
  and levels pitch/roll. The same calibrated deltas drive the attitude/heading instruments and are
  post-composed in the helicopter's local space; pitch/roll invert the hand-tracking sign at `3/40`
  gain and yaw remains one-to-one. The palm height at the click is neutral collective. Ending or
  canceling a placement drag levels pitch/roll and points the helicopter's tail toward the current
  HMD position.
- Flight motion composes pitch and roll into one local body-up lift direction, adds local gravity,
  solves each net-force component for terminal velocity using linear-plus-quadratic air resistance,
  and rotates that velocity into Stage space using heading. The rigid body remains dynamic for
  collisions but SDK gravity is disabled because gravity is already present in the solver.
- `FlightTutorialPreferences` persists completion of a right-hand-only seven-step tutorial. Each
  lesson recalibrates neutral, uses production control math, exposes one DoF, gates Next on the
  required gesture, and loops kinematic motion at 20 cm. The viewer-facing tutorial panel is a
  helicopter child at the same offset as START/RESTART; there is no tutorial hand model. Pitch
  lessons gate the same longitudinal motion signs as the production solver: forward first, then
  backward.
- Tracked-plane collisions use the plane anchor's scene-space surface center and normal rather than
  collision-box-local contact coordinates. A safe landing requires Stage-up surface alignment of at
  least `0.82`, the surface center below the helicopter center, support-normal/helicopter-up
  alignment of at least `0.70`, plane-normal speed no greater than `0.70 m/s`, tangential speed no
  greater than `0.75 m/s`, and total relative speed no greater than `0.95 m/s`. Every decision logs
  geometry-condition booleans, alignments, vertical separation, and the speed breakdown. Landing
  stops the rotor without crash audio; every other active-flight contact is a crash.
- Separate `MainRotorAudioEmitter` and `CrashAudioEmitter` children use `ObjectAudioComponent`. The rotor loops only in `RUNNING`; the crash one-shot plays only on a crash-caused `RUNNING -> NOT_RUNNING` transition. Audio controllers and resources are released in `destroy()`.
- The launcher keeps PICO's two-layer spatial icon contract. `design/icons/handycopter-layer-0.svg`
  is the opaque attitude-indicator/palm base and `handycopter-layer-1.svg` is the transparent toy-
  helicopter foreground. Their 1024 px PNGs, matching grayscale distance fields, and flattened
  compatibility icon live under `app/src/main/res/`; `drawables_3d.xml` orders foreground before
  background.
- `LookAtComponent.setViewerAsTarget()` must be called only after the component has been added to its entity; calling it inside the pre-attachment `apply` block produces a native `Entity nullptr` error on device.
- Formatting and static analysis mirror SpatialAI: run `./gradlew spotlessApply` to format, and use `./gradlew spotlessCheck detekt` as the code-quality gate. Detekt is strict (`maxIssues: 0`) and has no baseline.
- Build and test with `GRADLE_USER_HOME=/data00/home/yunhao.liu/.gradle ./gradlew spotlessCheck detekt testDebugUnitTest assembleDebug lintDebug`.
- Debug APK: `app/build/outputs/apk/debug/app-debug.apk`; package/activity: `com.pico.spatial.choppertoy/.platform.LaunchActivity`.
- Current 2026-10-05 device baseline: PICO CLI 0.5.0 cleanly reinstalled and launched the debug APK
  on `PB311XKGL4160042B` after the anchor-surface landing and tutorial pitch-gate fixes. The process
  stayed live, helicopter GLB and both spatial-audio resources loaded, the crash buffer was empty,
  and a bounded 10-second crash watcher found no crash. Host gates passed the SpatialUI verifier,
  Spotless, Detekt, 43 unit tests, APK assembly, and Android Lint. Physical landing classification
  still needs a worn-headset interaction pass; use the logged horizontal/below/aligned/speed fields.
