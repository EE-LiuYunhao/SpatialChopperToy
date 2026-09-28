# HandyCopter

HandyCopter is a PICO Spatial SDK Full Space application that lets the user position and fly a
small helicopter with either realistic palm controls or a dual virtual-pad RC controller.

## Interaction model

At launch, a viewer-facing SpatialUI panel asks the user to choose one control mode:

- **Realistic helicopter** uses one tracked palm as the cyclic, anti-torque, and collective input.
- **RC drone** uses the vendored AnyController library to turn two circles drawn on a detected
  plane into Mode-2-style virtual touchpads.

The helicopter has exactly two runtime states:

- `NOT_RUNNING`: the helicopter is kinematic, has no gravity or lift, ignores flight controls, and
  can be dragged in three dimensions. Its attached SpatialUI panel exposes START only after the
  selected controller is ready. After a crash the panel displays the recovery message and RESTART.
- `RUNNING`: drag interaction and the launch panel are disabled. Gravity is enabled, rotor lift
  acts along the helicopter's body-up axis, and the selected controller drives local aircraft
  attitude and collective. A crash freezes the helicopter at the impact site and returns it to
  `NOT_RUNNING`.

There is deliberately no Stop button, Exit button, stop transition, or alert dialog.

## Spatial behavior

- The helicopter initially appears approximately one meter in front of and 25 cm below the HMD.
- The visual model and box collider share the same doubled scale.
- Every tracked plane receives a collider. Gentle bottom contact with an upward horizontal plane is a landing; side contacts, non-horizontal contacts, and fast landings are crashes.
- The launch/recovery panel is a child of the helicopter and faces the viewer.
- The mode/setup panel is placed in front of the HMD at launch. In RC mode it remains visible with
  step-specific calibration guidance until both pads are ready.
- Dragging uses a Spatial gesture targeted to the helicopter entity. The entity is interactable only in `NOT_RUNNING`.
- Releasing or canceling a placement drag levels pitch and roll, then points the helicopter's tail toward the viewer from its new position.
- Pressing START or RESTART captures the current filtered palm pitch, roll, and yaw as zero. The attitude ball and heading panel then display movement relative to that pose, eliminating the tracked hand's static roll bias. The helicopter preserves its current orientation as a quaternion baseline and composes the same palm deltas in aircraft-local space.
- Filtered palm pitch and roll preserve the palm's direction and use `1/20` gain for both the attitude ball and helicopter cyclic control. Calibrated yaw remains one-to-one.
- The palm height at START or RESTART is the neutral collective point. At that height, a level helicopter receives `mass * gravity` rotor lift. Collective gain is `0.11` lift-multiplier units per meter, which is one twentieth of the previous `2.2` mapping.
- RC setup first renders detected planes. The left index fingertip selects a plane, after which all
  plane-discovery outlines are hidden. The user draws and lifts to size the left circle, touches its
  forward direction, then draws and lifts to size the right circle.
- RC input is neutral at each pad center. Left-pad Y maps linearly from zero to twice hover lift,
  left-pad X commands a local yaw rate of up to 90 degrees per second, right-pad Y commands pitch,
  and right-pad X commands roll. Pitch and roll remain bounded to 30 degrees. Leaving a pad or
  losing tracking immediately neutralizes that pad instead of holding stale commands.
- Rotor lift is never converted into an arbitrary world-space translation. It points along body-up, so its vertical component fights gravity and its horizontal component accelerates the helicopter in the tilt direction. Explicit linear rotor damping plus quadratic parasitic air drag oppose velocity and keep a small sustained tilt from producing unbounded speed.

## Spatial audio

- Two `ObjectAudioComponent` child entities are parented to the helicopter, so their HRTF position and inverse-square distance attenuation follow the aircraft automatically.
- The looping rotor sound starts only on `NOT_RUNNING -> RUNNING` and stops immediately when flight ends.
- A one-shot crash sound plays only on a crash-caused `RUNNING -> NOT_RUNNING` transition. Repeated collision updates in `NOT_RUNNING` cannot retrigger it.
- Both clips are preloaded from uncompressed APK assets and their player controllers/resources are released with the Stage lifecycle.

## Build and device validation

```bash
# Format Kotlin, Java, Gradle Kotlin, Markdown, XML, and YAML sources.
./gradlew spotlessApply

# Verify formatting and run strict Kotlin static analysis.
./gradlew spotlessCheck detekt

# Run the complete local verification gate.
GRADLE_USER_HOME=/data00/home/yunhao.liu/.gradle ./gradlew spotlessCheck detekt testDebugUnitTest assembleDebug lintDebug

adb -s <device> install -r app/build/outputs/apk/debug/app-debug.apk
adb -s <device> shell am start -W -n com.pico.spatial.handycopter/.platform.LaunchActivity
```

Spotless and Detekt mirror the SpatialAI repository configuration. Detekt uses
`config/detekt/detekt.yml`, fails on any finding, and analyzes both the app and reusable
`:anycontroller` module main/unit-test Kotlin sources without a baseline.

The debug build was installed and launched on PICO device `PB311XKGL4160042B`. Device logs confirmed Full Space creation, both attachment panels, hand/HMD/plane tracking, successful `helicopter.glb` loading, and preparation of both spatial-audio resources. A headset START/crash cycle then showed the 48 kHz rotor source start as spatialized audio, stop at the crash transition, and the 44.1 kHz crash source start and complete once. The Android crash buffer remained empty. ADB screenshots are black for this Spatial compositor path, so perceived sound direction and attenuation still require listening validation in the headset.

The reusable controller source is vendored under `anycontroller/` from internal upstream commit
`591666d`; provenance and local changes are recorded in `anycontroller/UPSTREAM.md`. The upstream
demo application is not included.

The packaged helicopter asset is `app/src/main/assets/helicopter.glb`; its attribution is recorded
in `helicopter.LICENSE.txt`. Audio attribution and source URLs are recorded in
`audio.LICENSE.md`.
