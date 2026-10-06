# Spatial Chopper Toy

Spatial Chopper Toy is a PICO Spatial SDK Full Space application for flying a small helicopter in
the user's real environment with palm gestures. Pitch and roll act as cyclic control, palm height
acts as collective, and palm heading acts as anti-torque/yaw input.

## First-install tutorial

Android `SharedPreferences` records completion of a seven-lesson tutorial. A fresh installation
requires the tracked right hand and teaches, in order:

1. pitch forward;
2. pitch backward;
3. roll left;
4. roll right;
5. raise the palm to climb;
6. lower the palm to descend;
7. turn the palm to yaw.

The tutorial panel is attached to the helicopter body at the same viewer-facing offset as the
START/RESTART panel; there is no separate 3D hand guide. Each lesson recalibrates the current right
palm pose as neutral and projects the production control mapping onto exactly one degree of freedom.
The attitude indicator and helicopter use the same `3/40` cyclic mapping, yaw delta, collective
gain, attitude controller, and lift solver as flight. The helicopter stays kinematic and
gravity-free: tutorial velocity is projected onto only the lesson axis, slowed, and integrated for
20 cm before snapping back to the lesson origin. Next remains disabled until the required
directional gesture has been demonstrated. Forward/backward pitch and left/right roll gating follow
the production motion signs, preventing a lesson label from accepting the opposite gesture.

## Interaction and flight model

There is no launch-time control-mode choice and no virtual RC controller. Hand, HMD, and plane
tracking start with the Stage.

The helicopter has two runtime modes plus an end reason:

- `NOT_RUNNING`: kinematic, gravity-free, draggable, and controlled by its attached START/RESTART
  panel. START is enabled once a valid palm is tracked.
- `RUNNING`: dynamic for collision response, while SDK gravity stays disabled because gravity is
  included in the app's solver. Drag and the launch panel are disabled.
- The previous flight end reason is `NONE`, `CRASHED`, or `LANDED`, allowing the recovery panel to
  distinguish a safe landing from a crash.

There is deliberately no Stop button, Exit button, stop transition, or alert dialog.

- The helicopter initially appears approximately one meter in front of and 25 cm below the HMD.
- The launch/recovery panel is attached to the helicopter and faces the viewer.
- Drag release or cancellation levels pitch and roll and points the tail toward the viewer.
- START/RESTART captures filtered palm pitch, roll, yaw, and height as the session zero. Pitch and
  roll invert the tracking sign and use `3/40` gain; yaw is one-to-one. Attitude deltas are composed
  in aircraft-local space after the preserved level heading.
- Palm height uses a collective gain of `0.11` lift-multiplier units per meter. At the captured
  height, level lift equals `mass * gravity`.
- Pitch and roll rotate one body-up lift vector. The solver adds gravity, solves each force component
  for terminal velocity with linear-plus-quadratic drag, and rotates velocity into Stage space by
  heading. Tilting therefore trades vertical lift for horizontal motion.

## Collision, landing, and audio

Every tracked plane has a detailed collider. Landing classification uses the tracked plane anchor's
scene-space surface center and normal instead of inferring a helicopter-box face from collision
contact coordinates. A contact during `RUNNING` is a safe landing only when:

- the collided plane is approximately horizontal, with its support normal aligned to Stage up by at
  least `0.82`;
- the tracked surface center is below the helicopter center in Stage space;
- the support normal aligns with helicopter local up by at least `cos⁻¹(0.70)`;
- plane-normal relative speed is at most `0.70 m/s`;
- plane-tangential drift is at most `0.75 m/s`; and
- total relative speed is at most `0.95 m/s`.

Splitting normal and tangential speed avoids rejecting a gentle descent merely because it also has
modest horizontal drift. Each collision logs horizontal alignment, below-body separation,
helicopter-up alignment, contact count, surface geometry, and all three measured speeds for device
diagnosis. All other tracked-plane contacts remain crashes. Either outcome freezes the helicopter
at the contact site, stops the looping rotor, restores drag, and shows RESTART. A safe landing does
not play the crash sound and displays exactly:

> Congratulations! You have landed the helicopter successfully.

Two `ObjectAudioComponent` children follow the helicopter. The rotor loop starts only on entry to
`RUNNING`; the crash one-shot plays only for a crash-caused exit. Audio controllers and resources
are released with the Stage lifecycle.

## Layered launcher icon

The PICO launcher icon keeps the platform's two-layer spatial presentation. The opaque base layer
uses the attitude indicator's blue-sky/brown-ground palette and a foreshortened horizontal palm;
the transparent foreground layer places a cartoon toy helicopter above the palm so it appears to
be supported by the hand. Editable 1024×1024 SVG sources are in `design/icons/`.

The packaged layer order remains foreground `icon_3d_layer_1` followed by background
`icon_3d_layer_0`. Matching grayscale distance fields drive the system highlight and edge effects,
and `ic_spatial_launcher.png` is the flattened compatibility icon.

## Build and validation

```bash
./gradlew spotlessApply
GRADLE_USER_HOME=/data00/home/yunhao.liu/.gradle \
  ./gradlew spotlessCheck detekt testDebugUnitTest assembleDebug lintDebug

adb -s <device> install -r app/build/outputs/apk/debug/app-debug.apk
adb -s <device> shell am start -W \
  -n com.pico.spatial.choppertoy/.platform.LaunchActivity
```

Spotless and Detekt mirror the SpatialAI repository configuration. Detekt has no baseline and fails
on any finding. The debug APK is `app/build/outputs/apk/debug/app-debug.apk`.

The 2026-10-05 gate passed the SpatialUI verifier (0 errors, 0 warnings), Spotless, Detekt, 43 unit
tests, APK assembly, and Android Lint. PICO CLI 0.5.0 then cleanly reinstalled and launched the APK
on `PB311XKGL4160042B`; the process stayed live, the helicopter and both spatial-audio resources
loaded, the crash buffer remained empty, and a bounded 10-second watcher observed no crash. A
physical safe-landing attempt still requires a worn-headset interaction pass; each collision now
logs the horizontal-surface, below-body, alignment, and speed evidence needed for that pass.

The packaged helicopter asset is `app/src/main/assets/helicopter.glb`; its attribution is in
`helicopter.LICENSE.txt`. Audio attribution and source URLs are in `audio.LICENSE.md`.
