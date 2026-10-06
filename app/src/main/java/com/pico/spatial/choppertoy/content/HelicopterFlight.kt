package com.pico.spatial.choppertoy.content

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

/** Maximum pitch or roll command accepted from the palm cyclic input. */
internal const val MAX_CYCLIC_TILT_DEGREES = 30f

/** Maximum rate at which cyclic pitch or roll can change. */
internal const val CYCLIC_SLEW_RATE_DEGREES_PER_SECOND = 75f

/** Lift-multiplier change applied per meter of palm-height displacement. */
internal const val PALM_COLLECTIVE_GAIN_PER_METER = 0.11f

/** Low-speed horizontal air-resistance coefficient in newton-seconds per meter. */
internal const val HORIZONTAL_LINEAR_DRAG_NEWTON_SECONDS_PER_METER = 0.32f

/** High-speed horizontal air-resistance coefficient in newton-seconds squared per meter squared. */
internal const val HORIZONTAL_QUADRATIC_DRAG_NEWTON_SECONDS_SQUARED_PER_METER_SQUARED = 0.16f

/** Low-speed vertical air-resistance coefficient in newton-seconds per meter. */
internal const val VERTICAL_LINEAR_DRAG_NEWTON_SECONDS_PER_METER = 0.18f

/** High-speed vertical air-resistance coefficient in newton-seconds squared per meter squared. */
internal const val VERTICAL_QUADRATIC_DRAG_NEWTON_SECONDS_SQUARED_PER_METER_SQUARED = 0.08f

/** The complete runtime state machine for the helicopter. */
internal enum class FlightMode {
    NOT_RUNNING,
    RUNNING,
}

/** How the most recent flight session ended. */
internal enum class FlightEndReason {
    NONE,
    CRASHED,
    LANDED,
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
    val endReason: FlightEndReason = FlightEndReason.NONE,
    val controlReady: Boolean = false,
) {
    val running: Boolean
        get() = mode == FlightMode.RUNNING

    val crashed: Boolean
        get() = endReason == FlightEndReason.CRASHED

    val landed: Boolean
        get() = endReason == FlightEndReason.LANDED
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

/** Trigonometric lift decomposition and drag-limited velocity for one controller frame. */
internal data class HelicopterMotionSolution(
    val localLiftDirection: Vector3,
    val localNetForceNewtons: Vector3,
    val localVelocityMetersPerSecond: Vector3,
    val worldVelocityMetersPerSecond: Vector3,
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
 * shortest-path deltas from that sample. Palm pitch/roll use the aircraft-facing inverse sign and
 * the shared three-fortieths control gain, while yaw remains one-to-one. The scene controller
 * composes these deltas after the helicopter's level flight-start heading so every control axis is
 * local to the aircraft.
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
            mapPalmCyclicAngle(shortestSignedAngleDelta(reference.pitchDegrees, palm.pitchDegrees))
        val rollDelta =
            mapPalmCyclicAngle(shortestSignedAngleDelta(reference.rollDegrees, palm.rollDegrees))
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
 * Resolves one attitude and collective frame into a drag-limited world-space velocity.
 *
 * Pitch and roll are composed once to rotate local body-up, naturally decomposing rotor lift into
 * lateral, vertical, and longitudinal components. Local gravity is then added to the lift force.
 * Rather than integrating acceleration, each net-force component is converted to its terminal
 * velocity by solving `quadraticDrag * v² + linearDrag * v = |force|`. Finally, the aircraft's
 * level heading rotates that local velocity into Stage space; translation is intentionally not
 * applied because velocity is a vector, not a point.
 */
internal fun solveHelicopterMotion(
    attitude: HelicopterAttitudeCommand,
    liftNewtons: Float,
    flightStartHeadingDegrees: Float,
    massKilograms: Float = HELICOPTER_MASS_KILOGRAMS,
    gravityMetersPerSecondSquared: Float = GRAVITY_METERS_PER_SECOND_SQUARED,
    horizontalLinearDrag: Float = HORIZONTAL_LINEAR_DRAG_NEWTON_SECONDS_PER_METER,
    horizontalQuadraticDrag: Float =
        HORIZONTAL_QUADRATIC_DRAG_NEWTON_SECONDS_SQUARED_PER_METER_SQUARED,
    verticalLinearDrag: Float = VERTICAL_LINEAR_DRAG_NEWTON_SECONDS_PER_METER,
    verticalQuadraticDrag: Float = VERTICAL_QUADRATIC_DRAG_NEWTON_SECONDS_SQUARED_PER_METER_SQUARED,
): HelicopterMotionSolution {
    require(liftNewtons >= 0f)
    require(massKilograms > 0f)
    require(gravityMetersPerSecondSquared >= 0f)
    require(horizontalLinearDrag >= 0f && horizontalQuadraticDrag >= 0f)
    require(verticalLinearDrag >= 0f && verticalQuadraticDrag >= 0f)
    require(horizontalLinearDrag > 0f || horizontalQuadraticDrag > 0f)
    require(verticalLinearDrag > 0f || verticalQuadraticDrag > 0f)

    val localLiftDirection =
        EulerAngles(pitch = attitude.pitchDegrees, roll = attitude.rollDegrees)
            .toQuat()
            .rotateVector(Vector3.UP)
            .normalizedOrUp()
    val localGravityForce = Vector3(0f, -massKilograms * gravityMetersPerSecondSquared, 0f)
    val localNetForce = localLiftDirection * liftNewtons + localGravityForce
    val localVelocity =
        Vector3(
            terminalVelocityForForce(
                localNetForce.x,
                horizontalLinearDrag,
                horizontalQuadraticDrag,
            ),
            terminalVelocityForForce(localNetForce.y, verticalLinearDrag, verticalQuadraticDrag),
            terminalVelocityForForce(localNetForce.z, horizontalLinearDrag, horizontalQuadraticDrag),
        )
    val worldHeading = flightStartHeadingDegrees + attitude.headingDegrees
    val worldVelocity = EulerAngles(yaw = worldHeading).toQuat().rotateVector(localVelocity)
    return HelicopterMotionSolution(
        localLiftDirection = localLiftDirection,
        localNetForceNewtons = localNetForce,
        localVelocityMetersPerSecond = localVelocity,
        worldVelocityMetersPerSecond = worldVelocity,
    )
}

/** Pure control-state model kept separate from the SDK physics objects for deterministic tests. */
internal class HelicopterFlightModel(
    private val neutralLiftNewtons: Float = NEUTRAL_LIFT_NEWTONS,
    private val collectiveGainPerMeter: Float = PALM_COLLECTIVE_GAIN_PER_METER,
    private val minimumLiftMultiplier: Float = 0f,
    private val maximumLiftMultiplier: Float = 2.25f,
) {
    var mode: FlightMode = FlightMode.NOT_RUNNING
        private set

    var endReason: FlightEndReason = FlightEndReason.NONE
        private set

    private var neutralPalmHeightMeters: Float? = null

    /** Enters RUNNING and returns the initial lift that balances gravity. */
    fun start(currentPalmHeightMeters: Float?): Float {
        neutralPalmHeightMeters = currentPalmHeightMeters
        mode = FlightMode.RUNNING
        endReason = FlightEndReason.NONE
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
        stop(FlightEndReason.CRASHED)
    }

    /** Returns to NOT_RUNNING after a qualified low-speed support-surface contact. */
    fun land() {
        stop(FlightEndReason.LANDED)
    }

    private fun stop(reason: FlightEndReason) {
        require(reason != FlightEndReason.NONE)
        mode = FlightMode.NOT_RUNNING
        endReason = reason
        neutralPalmHeightMeters = null
    }
}

/** Scene-space geometry of the tracked plane involved in a helicopter collision. */
internal data class LandingSurfaceSample(
    val positionInScene: Vector3,
    val supportNormalInScene: Vector3,
    val normalAlignmentWithHelicopterUp: Float,
)

/** Diagnostic breakdown of one tracked-plane collision decision. */
internal data class TrackedPlaneCollisionAssessment(
    val result: TrackedPlaneCollisionResult,
    val hasHorizontalSurface: Boolean,
    val isSurfaceBelowHelicopter: Boolean,
    val hasAlignedSupportSurface: Boolean,
    val surfaceHorizontalAlignment: Float?,
    val surfaceVerticalSeparationMeters: Float?,
    val totalSpeedMetersPerSecond: Float,
    val normalSpeedMetersPerSecond: Float?,
    val tangentialSpeedMetersPerSecond: Float?,
)

private data class LandingRelativeSpeeds(
    val total: Float,
    val normal: Float?,
    val tangential: Float?,
)

/** Result of classifying contact between the helicopter and a tracked physical plane. */
internal enum class TrackedPlaneCollisionResult {
    IGNORE,
    CRASH,
    LAND,
}

/** Maximum total plane-relative speed permitted for a safe landing. */
internal const val SAFE_LANDING_MAX_RELATIVE_SPEED_METERS_PER_SECOND = 0.95f

/** Maximum plane-normal speed permitted for a safe landing. */
internal const val SAFE_LANDING_MAX_NORMAL_SPEED_METERS_PER_SECOND = 0.70f

/** Maximum plane-tangential drift speed permitted for a safe landing. */
internal const val SAFE_LANDING_MAX_TANGENTIAL_SPEED_METERS_PER_SECOND = 0.75f

/** Minimum cosine alignment between a landing surface normal and Stage up. */
internal const val LANDING_MINIMUM_SURFACE_HORIZONTAL_ALIGNMENT = 0.82f

/** Minimum cosine alignment between the plane support normal and helicopter local up. */
internal const val LANDING_MINIMUM_NORMAL_ALIGNMENT = 0.70f

/** Box-collider dimensions used by helicopter flight physics. */
internal val HELICOPTER_COLLISION_SIZE_METERS = Vector3(0.380f, 0.150f, 0.500f)

/**
 * Classifies a tracked-plane collision as ignored, a crash, or a safe landing.
 *
 * A safe landing requires a tracked collision plane that is approximately horizontal, whose surface
 * is below the helicopter center, and whose support normal points toward helicopter local up.
 * Total, plane-normal, and plane-tangential speeds are bounded independently so gentle descent with
 * modest drift is not rejected merely because both components contribute to vector length. Any
 * other active-flight contact is a crash.
 */
internal fun classifyTrackedPlaneCollision(
    mode: FlightMode,
    collidedWithTrackedPlane: Boolean,
    surface: LandingSurfaceSample?,
    helicopterCenterInScene: Vector3,
    relativeVelocityMetersPerSecond: Vector3,
    maximumRelativeSpeedMetersPerSecond: Float = SAFE_LANDING_MAX_RELATIVE_SPEED_METERS_PER_SECOND,
    maximumNormalSpeedMetersPerSecond: Float = SAFE_LANDING_MAX_NORMAL_SPEED_METERS_PER_SECOND,
    maximumTangentialSpeedMetersPerSecond: Float =
        SAFE_LANDING_MAX_TANGENTIAL_SPEED_METERS_PER_SECOND,
    minimumSurfaceHorizontalAlignment: Float = LANDING_MINIMUM_SURFACE_HORIZONTAL_ALIGNMENT,
    minimumNormalAlignment: Float = LANDING_MINIMUM_NORMAL_ALIGNMENT,
): TrackedPlaneCollisionResult =
    assessTrackedPlaneCollision(
            mode = mode,
            collidedWithTrackedPlane = collidedWithTrackedPlane,
            surface = surface,
            helicopterCenterInScene = helicopterCenterInScene,
            relativeVelocityMetersPerSecond = relativeVelocityMetersPerSecond,
            maximumRelativeSpeedMetersPerSecond = maximumRelativeSpeedMetersPerSecond,
            maximumNormalSpeedMetersPerSecond = maximumNormalSpeedMetersPerSecond,
            maximumTangentialSpeedMetersPerSecond = maximumTangentialSpeedMetersPerSecond,
            minimumSurfaceHorizontalAlignment = minimumSurfaceHorizontalAlignment,
            minimumNormalAlignment = minimumNormalAlignment,
        )
        .result

/** Returns the safe-landing decision plus surface and speed evidence for device diagnostics. */
internal fun assessTrackedPlaneCollision(
    mode: FlightMode,
    collidedWithTrackedPlane: Boolean,
    surface: LandingSurfaceSample?,
    helicopterCenterInScene: Vector3,
    relativeVelocityMetersPerSecond: Vector3,
    maximumRelativeSpeedMetersPerSecond: Float = SAFE_LANDING_MAX_RELATIVE_SPEED_METERS_PER_SECOND,
    maximumNormalSpeedMetersPerSecond: Float = SAFE_LANDING_MAX_NORMAL_SPEED_METERS_PER_SECOND,
    maximumTangentialSpeedMetersPerSecond: Float =
        SAFE_LANDING_MAX_TANGENTIAL_SPEED_METERS_PER_SECOND,
    minimumSurfaceHorizontalAlignment: Float = LANDING_MINIMUM_SURFACE_HORIZONTAL_ALIGNMENT,
    minimumNormalAlignment: Float = LANDING_MINIMUM_NORMAL_ALIGNMENT,
): TrackedPlaneCollisionAssessment {
    val totalSpeed = relativeVelocityMetersPerSecond.length()
    if (mode != FlightMode.RUNNING || !collidedWithTrackedPlane) {
        return TrackedPlaneCollisionAssessment(
            result = TrackedPlaneCollisionResult.IGNORE,
            hasHorizontalSurface = false,
            isSurfaceBelowHelicopter = false,
            hasAlignedSupportSurface = false,
            surfaceHorizontalAlignment = null,
            surfaceVerticalSeparationMeters = null,
            totalSpeedMetersPerSecond = totalSpeed,
            normalSpeedMetersPerSecond = null,
            tangentialSpeedMetersPerSecond = null,
        )
    }
    require(maximumRelativeSpeedMetersPerSecond >= 0f)
    require(maximumNormalSpeedMetersPerSecond >= 0f)
    require(maximumTangentialSpeedMetersPerSecond >= 0f)
    require(minimumSurfaceHorizontalAlignment in -1f..1f)
    require(minimumNormalAlignment in -1f..1f)

    val supportNormal = surface?.supportNormalInScene?.takeIf { it.length() > 1e-6f }?.normalize()
    val horizontalAlignment = supportNormal?.dot(Vector3.UP)
    val verticalSeparation =
        surface?.let { (helicopterCenterInScene - it.positionInScene).dot(Vector3.UP) }
    val normalAlignmentWithHelicopterUp = surface?.normalAlignmentWithHelicopterUp
    val hasHorizontalSurface =
        horizontalAlignment != null && horizontalAlignment >= minimumSurfaceHorizontalAlignment
    val isSurfaceBelowHelicopter = verticalSeparation != null && verticalSeparation >= 0f
    val hasAlignedSupportSurface =
        hasHorizontalSurface &&
            isSurfaceBelowHelicopter &&
            normalAlignmentWithHelicopterUp != null &&
            normalAlignmentWithHelicopterUp >= minimumNormalAlignment
    val qualifiedSupportNormal = supportNormal.takeIf { hasAlignedSupportSurface }
    val speeds = landingRelativeSpeeds(relativeVelocityMetersPerSecond, qualifiedSupportNormal)
    val speedIsSafe =
        speeds.areWithin(
            maximumTotal = maximumRelativeSpeedMetersPerSecond,
            maximumNormal = maximumNormalSpeedMetersPerSecond,
            maximumTangential = maximumTangentialSpeedMetersPerSecond,
        )
    return TrackedPlaneCollisionAssessment(
        result =
            if (hasAlignedSupportSurface && speedIsSafe) {
                TrackedPlaneCollisionResult.LAND
            } else {
                TrackedPlaneCollisionResult.CRASH
            },
        hasHorizontalSurface = hasHorizontalSurface,
        isSurfaceBelowHelicopter = isSurfaceBelowHelicopter,
        hasAlignedSupportSurface = hasAlignedSupportSurface,
        surfaceHorizontalAlignment = horizontalAlignment,
        surfaceVerticalSeparationMeters = verticalSeparation,
        totalSpeedMetersPerSecond = speeds.total,
        normalSpeedMetersPerSecond = speeds.normal,
        tangentialSpeedMetersPerSecond = speeds.tangential,
    )
}

private fun Vector3.dot(other: Vector3): Float = x * other.x + y * other.y + z * other.z

private fun landingRelativeSpeeds(
    velocity: Vector3,
    supportNormal: Vector3?,
): LandingRelativeSpeeds {
    val total = velocity.length()
    supportNormal ?: return LandingRelativeSpeeds(total, null, null)
    val signedNormalVelocity =
        velocity.x * supportNormal.x + velocity.y * supportNormal.y + velocity.z * supportNormal.z
    return LandingRelativeSpeeds(
        total = total,
        normal = abs(signedNormalVelocity),
        tangential = (velocity - supportNormal * signedNormalVelocity).length(),
    )
}

private fun LandingRelativeSpeeds.areWithin(
    maximumTotal: Float,
    maximumNormal: Float,
    maximumTangential: Float,
): Boolean =
    total <= maximumTotal &&
        normal != null &&
        normal <= maximumNormal &&
        tangential != null &&
        tangential <= maximumTangential

/** Returns the shortest signed angular delta from [fromDegrees] to [toDegrees]. */
internal fun shortestSignedAngleDelta(fromDegrees: Float, toDegrees: Float): Float {
    var delta = (toDegrees - fromDegrees) % 360f
    if (delta > 180f) delta -= 360f
    if (delta < -180f) delta += 360f
    return delta
}

internal fun normalizeHeadingDegrees(degrees: Float): Float = ((degrees % 360f) + 360f) % 360f

/** Normalizes an angle to the shortest signed representation from -180 through 180 degrees. */
internal fun normalizeSignedAngleDegrees(degrees: Float): Float {
    val normalized = normalizeHeadingDegrees(degrees)
    return if (normalized > 180f) normalized - 360f else normalized
}

private fun vectorLength(vector: Vector3): Float =
    sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)

private fun Vector3.normalizedOrUp(): Vector3 {
    val length = vectorLength(this)
    return if (length > 1e-6f) this * (1f / length) else Vector3.UP
}

private fun terminalVelocityForForce(
    forceNewtons: Float,
    linearDrag: Float,
    quadraticDrag: Float,
): Float {
    val magnitude = abs(forceNewtons)
    if (magnitude <= 1e-6f) return 0f
    val speed =
        if (quadraticDrag <= 1e-6f) {
            magnitude / linearDrag
        } else {
            (sqrt(linearDrag * linearDrag + 4f * quadraticDrag * magnitude) - linearDrag) /
                (2f * quadraticDrag)
        }
    return if (forceNewtons < 0f) -speed else speed
}
