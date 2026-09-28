package com.pico.spatial.handycopter.content

import com.pico.spatial.core.math.EulerAngles
import com.pico.spatial.core.math.Quat
import com.pico.spatial.core.math.Vector3
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt

/** Simulated helicopter mass used by its rigid-body mass properties. */
internal const val HELICOPTER_MASS_KILOGRAMS = 0.35f

/** Stage gravity magnitude in meters per second squared. */
internal const val GRAVITY_METERS_PER_SECOND_SQUARED = 9.81f

/** Rotor force that exactly balances gravity at the configured mass. */
internal const val NEUTRAL_LIFT_NEWTONS =
    HELICOPTER_MASS_KILOGRAMS * GRAVITY_METERS_PER_SECOND_SQUARED

/** Maximum downward impact speed accepted as a safe landing. */
internal const val SAFE_LANDING_SPEED_METERS_PER_SECOND = 1.25f

/** Half of the helicopter collision body's vertical extent. */
internal const val HELICOPTER_HALF_HEIGHT_METERS = 0.075f

/** Maximum pitch or roll command accepted from the palm cyclic input. */
internal const val MAX_CYCLIC_TILT_DEGREES = 30f

/** Maximum rate at which cyclic pitch or roll can change. */
internal const val CYCLIC_SLEW_RATE_DEGREES_PER_SECOND = 75f

/** Low-speed horizontal rotor and airframe drag in newton-seconds per meter. */
internal const val HORIZONTAL_LINEAR_DRAG_NEWTON_SECONDS_PER_METER = 0.32f

/** High-speed horizontal parasitic-drag coefficient in newton-seconds squared per meter squared. */
internal const val HORIZONTAL_QUADRATIC_DRAG_NEWTON_SECONDS_SQUARED_PER_METER_SQUARED = 0.16f

/** Low-speed vertical rotor and airframe drag in newton-seconds per meter. */
internal const val VERTICAL_LINEAR_DRAG_NEWTON_SECONDS_PER_METER = 0.18f

/** High-speed vertical parasitic-drag coefficient in newton-seconds squared per meter squared. */
internal const val VERTICAL_QUADRATIC_DRAG_NEWTON_SECONDS_SQUARED_PER_METER_SQUARED = 0.08f

/** The complete runtime state machine for the helicopter. */
internal enum class FlightMode {
    NOT_RUNNING,
    RUNNING,
}

/** Playback commands emitted only at meaningful helicopter state transitions. */
internal enum class FlightAudioCue {
    START_ROTOR,
    STOP_ROTOR,
    PLAY_CRASH,
}

/**
 * Maps a flight-state transition to deterministic audio commands.
 *
 * The crash cue is emitted only for a crash-caused `RUNNING -> NOT_RUNNING` transition, while the
 * rotor starts and stops strictly on entry to and exit from [FlightMode.RUNNING].
 */
internal fun flightAudioCuesForTransition(
    previousMode: FlightMode,
    currentMode: FlightMode,
    causedByCrash: Boolean,
): List<FlightAudioCue> = buildList {
    if (previousMode == currentMode) return@buildList
    if (previousMode == FlightMode.RUNNING) add(FlightAudioCue.STOP_ROTOR)
    if (currentMode == FlightMode.RUNNING) add(FlightAudioCue.START_ROTOR)
    if (
        causedByCrash && previousMode == FlightMode.RUNNING && currentMode == FlightMode.NOT_RUNNING
    ) {
        add(FlightAudioCue.PLAY_CRASH)
    }
}

/** UI-facing snapshot of the helicopter state and its launch-panel presentation. */
internal data class FlightUiState(
    val helicopterReady: Boolean = false,
    val mode: FlightMode = FlightMode.NOT_RUNNING,
    val crashed: Boolean = false,
) {
    val running: Boolean
        get() = mode == FlightMode.RUNNING
}

/** Filtered but uncalibrated palm channels sampled in stage space. */
internal data class PalmFlightControl(
    val palmHeightMeters: Float,
    val pitchDegrees: Float,
    val rollDegrees: Float,
    val headingDegrees: Float,
)

/** Bounded local-space attitude delta applied relative to the flight-start orientation. */
internal data class HelicopterAttitudeCommand(
    val pitchDegrees: Float,
    val rollDegrees: Float,
    val headingDegrees: Float,
)

/** Calibrated commands shared by the cockpit instruments and helicopter controller. */
internal data class CalibratedPalmFlightControl(
    val instrumentReadout: InstrumentReadout,
    val helicopterControl: PalmFlightControl,
)

/**
 * Converts absolute palm tracking into start-relative cockpit and helicopter commands.
 *
 * START or RESTART captures the current palm pose as zero. Subsequent palm pitch, roll, and yaw are
 * shortest-path deltas from that sample; cyclic deltas use the shared one-twentieth control gain,
 * while yaw remains one-to-one. The scene controller composes these deltas after the helicopter's
 * flight-start quaternion so every control axis is local to the aircraft.
 */
internal class PalmFlightCalibration {
    private var palmAtStart: PalmFlightControl? = null

    /** True after a valid palm sample has been captured for the current flight session. */
    val isCalibrated: Boolean
        get() = palmAtStart != null

    /** Captures the palm zero point at START or RESTART. */
    fun calibrate(palm: PalmFlightControl) {
        palmAtStart = palm
    }

    /**
     * Resolves a raw palm sample into zero-relative instruments and aircraft-local flight input.
     */
    fun resolve(palm: PalmFlightControl?): CalibratedPalmFlightControl? {
        val reference = palmAtStart ?: return null
        palm ?: return null
        val pitchDelta =
            shortestSignedAngleDelta(reference.pitchDegrees, palm.pitchDegrees) *
                PALM_ATTITUDE_CONTROL_GAIN
        val rollDelta =
            shortestSignedAngleDelta(reference.rollDegrees, palm.rollDegrees) *
                PALM_ATTITUDE_CONTROL_GAIN
        val headingDelta = shortestSignedAngleDelta(reference.headingDegrees, palm.headingDegrees)

        return CalibratedPalmFlightControl(
            instrumentReadout =
                InstrumentReadout(
                    pitchDegrees = pitchDelta,
                    rollDegrees = rollDelta,
                    headingDegrees = normalizeHeadingDegrees(headingDelta),
                ),
            helicopterControl =
                PalmFlightControl(
                    palmHeightMeters = palm.palmHeightMeters,
                    pitchDegrees = pitchDelta,
                    rollDegrees = rollDelta,
                    headingDegrees = headingDelta,
                ),
        )
    }
}

/** Composes a bounded control delta after [baseline], keeping every axis aircraft-local. */
internal fun composeLocalAttitude(baseline: Quat, localAttitude: HelicopterAttitudeCommand): Quat =
    (baseline *
            EulerAngles(
                    pitch = localAttitude.pitchDegrees,
                    yaw = localAttitude.headingDegrees,
                    roll = localAttitude.rollDegrees,
                )
                .toQuat())
        .normalize()

/**
 * Returns the level yaw whose forward axis points from the viewer toward the helicopter.
 *
 * The helicopter model's tail is opposite that axis, so the resulting level pose keeps the tail
 * facing the viewer after placement. [fallbackHeadingDegrees] is retained when both points have the
 * same horizontal coordinates.
 */
internal fun tailTowardViewerHeadingDegrees(
    helicopterPosition: Vector3,
    viewerPosition: Vector3,
    fallbackHeadingDegrees: Float,
): Float {
    val awayFromViewer =
        Vector3(
            helicopterPosition.x - viewerPosition.x,
            0f,
            helicopterPosition.z - viewerPosition.z,
        )
    if (vectorLength(awayFromViewer) <= 1e-6f) return fallbackHeadingDegrees
    return normalizeHeadingDegrees(
        Math.toDegrees(atan2(awayFromViewer.x.toDouble(), awayFromViewer.z.toDouble())).toFloat()
    )
}

/**
 * Converts the filtered palm attitude into a safe helicopter attitude command.
 *
 * Cyclic tilt is bounded to a realistic bank angle and slew-limited so a tracking burst cannot
 * teleport the dynamic collider into an inverted pose. Heading remains unsmoothed after the
 * start-relative calibration so it follows the palm's yaw delta exactly.
 */
internal class HelicopterAttitudeController(
    private val maximumCyclicTiltDegrees: Float = MAX_CYCLIC_TILT_DEGREES,
    private val cyclicSlewRateDegreesPerSecond: Float = CYCLIC_SLEW_RATE_DEGREES_PER_SECOND,
) {
    private var pitchDegrees = 0f
    private var rollDegrees = 0f
    private var headingDegrees = 0f

    init {
        require(maximumCyclicTiltDegrees in 0f..89f)
        require(cyclicSlewRateDegreesPerSecond > 0f)
    }

    /** Resets the controller to the helicopter attitude captured at flight start. */
    fun reset(attitude: HelicopterAttitudeCommand) {
        pitchDegrees = attitude.pitchDegrees
        rollDegrees = attitude.rollDegrees
        headingDegrees = attitude.headingDegrees
    }

    /** Advances the bounded cyclic command while applying yaw without smoothing. */
    fun step(
        control: PalmFlightControl?,
        cyclicEnabled: Boolean,
        deltaSeconds: Float,
    ): HelicopterAttitudeCommand {
        if (control != null) headingDegrees = control.headingDegrees
        val targetPitch =
            if (cyclicEnabled && control != null) {
                control.pitchDegrees.coerceIn(-maximumCyclicTiltDegrees, maximumCyclicTiltDegrees)
            } else {
                0f
            }
        val targetRoll =
            if (cyclicEnabled && control != null) {
                control.rollDegrees.coerceIn(-maximumCyclicTiltDegrees, maximumCyclicTiltDegrees)
            } else {
                0f
            }
        val maximumStep = cyclicSlewRateDegreesPerSecond * deltaSeconds.coerceIn(0f, 1f / 30f)
        pitchDegrees = moveTowards(pitchDegrees, targetPitch, maximumStep)
        rollDegrees = moveTowards(rollDegrees, targetRoll, maximumStep)
        return HelicopterAttitudeCommand(pitchDegrees, rollDegrees, headingDegrees)
    }

    private fun moveTowards(current: Float, target: Float, maximumStep: Float): Float =
        current + (target - current).coerceIn(-maximumStep, maximumStep)
}

/**
 * Calculates the non-gravity force applied to the helicopter rigid body.
 *
 * Rotor lift always follows body-up. Linear rotor damping dominates near hover and quadratic
 * parasitic drag grows with airspeed, preventing a small sustained tilt from accelerating forever.
 * The Spatial physics world applies gravity separately.
 */
internal fun calculateHelicopterAerodynamicForce(
    bodyUp: Vector3,
    liftNewtons: Float,
    velocityMetersPerSecond: Vector3,
    horizontalLinearDrag: Float = HORIZONTAL_LINEAR_DRAG_NEWTON_SECONDS_PER_METER,
    horizontalQuadraticDrag: Float =
        HORIZONTAL_QUADRATIC_DRAG_NEWTON_SECONDS_SQUARED_PER_METER_SQUARED,
    verticalLinearDrag: Float = VERTICAL_LINEAR_DRAG_NEWTON_SECONDS_PER_METER,
    verticalQuadraticDrag: Float = VERTICAL_QUADRATIC_DRAG_NEWTON_SECONDS_SQUARED_PER_METER_SQUARED,
): Vector3 {
    require(liftNewtons >= 0f)
    require(horizontalLinearDrag >= 0f && horizontalQuadraticDrag >= 0f)
    require(verticalLinearDrag >= 0f && verticalQuadraticDrag >= 0f)

    val bodyUpLength = vectorLength(bodyUp)
    val liftDirection = if (bodyUpLength > 1e-6f) bodyUp * (1f / bodyUpLength) else Vector3.UP
    val horizontalVelocity = Vector3(velocityMetersPerSecond.x, 0f, velocityMetersPerSecond.z)
    val horizontalSpeed = vectorLength(horizontalVelocity)
    val horizontalDrag =
        horizontalVelocity * -(horizontalLinearDrag + horizontalQuadraticDrag * horizontalSpeed)
    val verticalSpeed = velocityMetersPerSecond.y
    val verticalDrag =
        Vector3.UP *
            (-verticalSpeed * (verticalLinearDrag + verticalQuadraticDrag * abs(verticalSpeed)))
    return liftDirection * liftNewtons + horizontalDrag + verticalDrag
}

/** Pure control-state model kept separate from the SDK physics objects for deterministic tests. */
internal class HelicopterFlightModel(
    private val neutralLiftNewtons: Float = NEUTRAL_LIFT_NEWTONS,
    private val collectiveGainPerMeter: Float = 0.11f,
    private val minimumLiftMultiplier: Float = 0f,
    private val maximumLiftMultiplier: Float = 2.25f,
) {
    var mode: FlightMode = FlightMode.NOT_RUNNING
        private set

    var crashed: Boolean = false
        private set

    private var neutralPalmHeightMeters: Float? = null

    /** Enters RUNNING and returns the initial lift that balances gravity. */
    fun start(currentPalmHeightMeters: Float?): Float {
        neutralPalmHeightMeters = currentPalmHeightMeters
        mode = FlightMode.RUNNING
        crashed = false
        return neutralLiftNewtons
    }

    /** Maps palm-height displacement from the start height to collective lift. */
    fun liftForPalmHeight(palmHeightMeters: Float?): Float {
        if (mode != FlightMode.RUNNING) return 0f
        val palmHeight = palmHeightMeters ?: return neutralLiftNewtons
        val neutralHeight =
            neutralPalmHeightMeters ?: palmHeight.also { neutralPalmHeightMeters = it }
        val multiplier =
            (1f + (palmHeight - neutralHeight) * collectiveGainPerMeter).coerceIn(
                minimumLiftMultiplier,
                maximumLiftMultiplier,
            )
        return neutralLiftNewtons * multiplier
    }

    /** Returns to NOT_RUNNING while retaining crash presentation for the restart panel. */
    fun crash() {
        mode = FlightMode.NOT_RUNNING
        crashed = true
        neutralPalmHeightMeters = null
    }
}

/** Classifies whether all reported contacts describe a sufficiently gentle bottom landing. */
internal fun isSafeLandingContact(
    planeIsHorizontalUpward: Boolean,
    localContactHeightsMeters: List<Float>,
    downwardSpeedMetersPerSecond: Float,
    halfHeightMeters: Float = HELICOPTER_HALF_HEIGHT_METERS,
    bottomContactToleranceMeters: Float = 0.012f,
    safeLandingSpeedMetersPerSecond: Float = SAFE_LANDING_SPEED_METERS_PER_SECOND,
): Boolean {
    if (!planeIsHorizontalUpward || localContactHeightsMeters.isEmpty()) return false
    val allContactsAtBottom =
        localContactHeightsMeters.all { contactHeight ->
            abs(contactHeight + halfHeightMeters) <= bottomContactToleranceMeters
        }
    return allContactsAtBottom && downwardSpeedMetersPerSecond <= safeLandingSpeedMetersPerSecond
}

/** Returns the shortest signed angular delta from [fromDegrees] to [toDegrees]. */
internal fun shortestSignedAngleDelta(fromDegrees: Float, toDegrees: Float): Float {
    var delta = (toDegrees - fromDegrees) % 360f
    if (delta > 180f) delta -= 360f
    if (delta < -180f) delta += 360f
    return delta
}

internal fun normalizeHeadingDegrees(degrees: Float): Float = ((degrees % 360f) + 360f) % 360f

private fun vectorLength(vector: Vector3): Float =
    sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
