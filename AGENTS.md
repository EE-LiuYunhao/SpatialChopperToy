<!-- pico-cli:plugin-context:pico-spatial-agentic-tools:start -->
## Plugin Context

Also read `./PICO-SPATIAL-AGENTIC-TOOLS.AGENTS.md` for PICO Spatial plugin guidance.
<!-- pico-cli:plugin-context:pico-spatial-agentic-tools:end -->

## Project Handoff

- The application is a Full Space `DefaultStage` PICO Spatial SDK app.
- `HomeStage.kt` owns hand/HMD/plane tracking, the palm instrument cluster, and the Spatial drag gesture.
- `HelicopterSceneController.kt` owns the helicopter entity, its attached launch panel, plane colliders, physics, entity-bound spatial audio, and the two-state flight lifecycle.
- The helicopter state machine is `NOT_RUNNING -> RUNNING -> NOT_RUNNING` on crash. There is intentionally no Stop, Exit, or alert-dialog flow.
- In `NOT_RUNNING`, the helicopter is kinematic, gravity-free, draggable, and displays its attached START or RESTART panel. In `RUNNING`, the panel and drag interaction are disabled and gravity plus palm-controlled lift are active.
- START/RESTART calibrates the current filtered palm pitch/roll/yaw as zero and preserves the helicopter's current quaternion as the baseline. The same calibrated deltas drive the attitude/heading instruments and are post-composed in the helicopter's local space; pitch/roll are direction-preserving at `1/20` gain and yaw remains one-to-one. The palm height at the click is neutral collective. Ending or canceling a placement drag levels pitch/roll and points the helicopter's tail toward the current HMD position.
- Flight force is rotor lift along the helicopter's body-up axis plus explicit velocity-dependent linear and quadratic aerodynamic drag; stage gravity remains owned by `PhysicsWorldComponent`.
- Separate `MainRotorAudioEmitter` and `CrashAudioEmitter` children use `ObjectAudioComponent`. The rotor loops only in `RUNNING`; the crash one-shot plays only on a crash-caused `RUNNING -> NOT_RUNNING` transition. Audio controllers and resources are released in `destroy()`.
- `LookAtComponent.setViewerAsTarget()` must be called only after the component has been added to its entity; calling it inside the pre-attachment `apply` block produces a native `Entity nullptr` error on device.
- Formatting and static analysis mirror SpatialAI: run `./gradlew spotlessApply` to format, and use `./gradlew spotlessCheck detekt` as the code-quality gate. Detekt is strict (`maxIssues: 0`) and has no baseline.
- Build and test with `GRADLE_USER_HOME=/data00/home/yunhao.liu/.gradle ./gradlew spotlessCheck detekt testDebugUnitTest assembleDebug lintDebug`.
- Debug APK: `app/build/outputs/apk/debug/app-debug.apk`; package/activity: `com.pico.spatial.handycopter/.platform.LaunchActivity`.
- Latest device check: installed and launched successfully on `PB311XKGL4160042B`; Full Space, tracking, attachment panels, helicopter loading, and both spatial-audio preparations were observed with an empty crash buffer. A headset START/crash cycle logged spatialized 48 kHz rotor playback, rotor stop on crash, then one spatialized 44.1 kHz crash playback through completion. Perceived sound direction and attenuation remain headset-listening checks because ADB cannot capture spatial audio.
