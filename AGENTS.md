<!-- pico-cli:plugin-context:pico-spatial-agentic-tools:start -->
## Plugin Context

Also read `./PICO-SPATIAL-AGENTIC-TOOLS.AGENTS.md` for PICO Spatial plugin guidance.
<!-- pico-cli:plugin-context:pico-spatial-agentic-tools:end -->

## Project Handoff

- The application is a Full Space `DefaultStage` PICO Spatial SDK app.
- `HomeStage.kt` owns launch-time mode selection, mode-specific tracking lifecycles, the palm
  instrument cluster, the AnyController overlay, and the Spatial drag gesture.
- `HelicopterSceneController.kt` owns the helicopter entity, its attached launch panel, plane
  colliders, physics, entity-bound spatial audio, both input adapters, and the two-state flight
  lifecycle.
- `:anycontroller` is a vendored reusable Android library from internal upstream commit `591666d`.
  The upstream demo app is intentionally excluded. Its overlay shows plane-discovery wireframes
  only before surface selection, while calibrated pads, axes, and fingertip guidance remain.
- The helicopter state machine is `NOT_RUNNING -> RUNNING -> NOT_RUNNING` on crash. There is intentionally no Stop, Exit, or alert-dialog flow.
- The launch panel appears after mode selection. START/RESTART is enabled only when the chosen
  controller is ready. In `NOT_RUNNING`, the helicopter is kinematic, gravity-free, and draggable.
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
- RC mode follows AnyController's guided surface/left-circle/left-up/right-circle calibration.
  Its Mode-2 mapping is left Y = collective, left X = local yaw rate, right Y = pitch, and right X =
  visible-direction roll. Inactive or lost fingertips neutralize their respective pad.
- Every tracked-plane contact during `RUNNING` is a crash in both modes. The transition freezes the
  helicopter, restores drag interaction and the attached RESTART panel, and retains RC calibration.
- Separate `MainRotorAudioEmitter` and `CrashAudioEmitter` children use `ObjectAudioComponent`. The rotor loops only in `RUNNING`; the crash one-shot plays only on a crash-caused `RUNNING -> NOT_RUNNING` transition. Audio controllers and resources are released in `destroy()`.
- `LookAtComponent.setViewerAsTarget()` must be called only after the component has been added to its entity; calling it inside the pre-attachment `apply` block produces a native `Entity nullptr` error on device.
- Formatting and static analysis mirror SpatialAI: run `./gradlew spotlessApply` to format, and use `./gradlew spotlessCheck detekt` as the code-quality gate. Detekt is strict (`maxIssues: 0`) and has no baseline.
- Build and test with `GRADLE_USER_HOME=/data00/home/yunhao.liu/.gradle ./gradlew spotlessCheck detekt testDebugUnitTest assembleDebug lintDebug`.
- Debug APK: `app/build/outputs/apk/debug/app-debug.apk`; package/activity: `com.pico.spatial.handycopter/.platform.LaunchActivity`.
- Latest device check: installed and launched successfully on `PB311XKGL4160042B`; Full Space, tracking, attachment panels, helicopter loading, and both spatial-audio preparations were observed with an empty crash buffer. A headset START/crash cycle logged spatialized 48 kHz rotor playback, rotor stop on crash, then one spatialized 44.1 kHz crash playback through completion. Perceived sound direction and attenuation remain headset-listening checks because ADB cannot capture spatial audio.
