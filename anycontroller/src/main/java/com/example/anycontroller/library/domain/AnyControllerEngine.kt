package com.example.anycontroller.library.domain

import com.example.anycontroller.library.model.AnyControllerConfig
import com.example.anycontroller.library.model.AnyControllerFrame
import com.example.anycontroller.library.model.CalibrationState
import com.example.anycontroller.library.model.ControllerLayout
import com.example.anycontroller.library.model.ControllerSurface
import com.example.anycontroller.library.model.PadCoordinateFrame
import com.example.anycontroller.library.model.PadGeometry
import com.example.anycontroller.library.model.PadState
import com.example.anycontroller.library.model.Point2
import com.example.anycontroller.library.model.Point3
import kotlin.math.abs
import kotlin.math.sqrt

/** Platform-neutral calibration and input-normalization state machine. */
class AnyControllerEngine(private val config: AnyControllerConfig = AnyControllerConfig()) {
    /** Current calibration step. */
    var calibrationState: CalibrationState = CalibrationState.STOPPED
        private set

    /** Immutable snapshot of the selected surface and pad geometry. */
    var layout: ControllerLayout = ControllerLayout()
        private set

    private var sequence = 0L
    private var previousLeft = PadState()
    private var previousRight = PadState()
    private var leftBoundaryRadiusCandidate: Float? = null
    private var rightBoundaryRadiusCandidate: Float? = null

    /** Starts a fresh calibration session. */
    fun start() {
        calibrationState = CalibrationState.WAITING_FOR_SURFACE
        layout = ControllerLayout()
        previousLeft = PadState()
        previousRight = PadState()
        resetBoundaryCandidates()
    }

    /** Stops the session, clears calibration, and returns a neutral frame. */
    fun stop(): AnyControllerFrame {
        calibrationState = CalibrationState.STOPPED
        layout = ControllerLayout()
        previousLeft = PadState()
        previousRight = PadState()
        resetBoundaryCandidates()
        return nextNeutral(System.nanoTime())
    }

    /** Selects [surface] as the immutable calibration coordinate frame. */
    fun selectSurface(surface: ControllerSurface) {
        layout = ControllerLayout(surface = surface)
        previousLeft = PadState()
        previousRight = PadState()
        resetBoundaryCandidates()
        calibrationState = CalibrationState.PLACE_LEFT_CENTER
    }

    /** Clears pad geometry while retaining the currently selected surface. */
    fun recalibrate() {
        val surface = layout.surface
        layout = ControllerLayout(surface = surface)
        previousLeft = PadState()
        previousRight = PadState()
        resetBoundaryCandidates()
        calibrationState =
            if (surface == null) CalibrationState.WAITING_FOR_SURFACE
            else CalibrationState.PLACE_LEFT_CENTER
    }

    /** Advances calibration or produces normalized pad input for the current fingertip samples. */
    fun update(
        leftIndexTip: Point3?,
        rightIndexTip: Point3?,
        timestampNanos: Long,
    ): AnyControllerFrame {
        val surface = layout.surface ?: return nextNeutral(timestampNanos)
        when (calibrationState) {
            CalibrationState.PLACE_LEFT_CENTER -> {
                contactPoint(surface, leftIndexTip)?.let { center ->
                    layout = layout.copy(leftPad = PadGeometry(center, 0f))
                    calibrationState = CalibrationState.PLACE_LEFT_BOUNDARY
                }
            }
            CalibrationState.PLACE_LEFT_BOUNDARY ->
                updateRadiusFromBoundaryGesture(
                    surface = surface,
                    fingertip = leftIndexTip,
                    isLeft = true,
                )
            CalibrationState.CALIBRATE_LEFT_UP -> calibrateLeftUp(surface, leftIndexTip)
            CalibrationState.PLACE_RIGHT_CENTER -> {
                contactPoint(surface, rightIndexTip)?.let { center ->
                    layout =
                        layout.copy(
                            rightPad =
                                PadGeometry(
                                    center = center,
                                    radiusMeters = 0f,
                                    frame = layout.leftPad?.frame,
                                )
                        )
                    calibrationState = CalibrationState.PLACE_RIGHT_BOUNDARY
                }
            }
            CalibrationState.PLACE_RIGHT_BOUNDARY ->
                updateRadiusFromBoundaryGesture(
                    surface = surface,
                    fingertip = rightIndexTip,
                    isLeft = false,
                )
            else -> Unit
        }

        if (calibrationState != CalibrationState.READY) return nextNeutral(timestampNanos)

        val left = padState(surface, layout.leftPad, leftIndexTip, previousLeft)
        val right = padState(surface, layout.rightPad, rightIndexTip, previousRight)
        previousLeft = left
        previousRight = right
        return AnyControllerFrame(
            sequence = ++sequence,
            timestampNanos = timestampNanos,
            leftPad = left,
            rightPad = right,
            trackingValid = leftIndexTip != null || rightIndexTip != null,
            calibrationState = calibrationState,
        )
    }

    @Suppress("ReturnCount")
    private fun updateRadiusFromBoundaryGesture(
        surface: ControllerSurface,
        fingertip: Point3?,
        isLeft: Boolean,
    ) {
        val pad = if (isLeft) layout.leftPad else layout.rightPad
        if (pad == null) return
        // A missing fingertip is tracking loss, not evidence that the user deliberately lifted it.
        if (fingertip == null) return
        val local = surface.worldToLocal(fingertip)
        val normalDistance = abs(local.z)

        // Contact and lift use separate thresholds so normal tracking jitter cannot accidentally
        // finish calibration. The radius follows the last valid surface-contact sample.
        if (normalDistance <= config.contactDistanceMeters && surface.contains(local)) {
            val radius = distance2D(local, pad.center).coerceAtMost(config.maximumPadRadiusMeters)
            if (isLeft) {
                leftBoundaryRadiusCandidate = radius
                layout = layout.copy(leftPad = pad.copy(radiusMeters = radius))
            } else {
                rightBoundaryRadiusCandidate = radius
                layout = layout.copy(rightPad = pad.copy(radiusMeters = radius))
            }
            return
        }

        val candidate = if (isLeft) leftBoundaryRadiusCandidate else rightBoundaryRadiusCandidate
        if (candidate == null || normalDistance < config.calibrationLiftDistanceMeters) return

        if (isLeft) leftBoundaryRadiusCandidate = null else rightBoundaryRadiusCandidate = null
        if (candidate < config.minimumPadRadiusMeters) {
            if (isLeft) {
                layout = layout.copy(leftPad = pad.copy(radiusMeters = 0f))
            } else {
                layout = layout.copy(rightPad = pad.copy(radiusMeters = 0f))
            }
            return
        }

        val committed = pad.copy(radiusMeters = candidate)
        if (isLeft) {
            layout = layout.copy(leftPad = committed)
            calibrationState = CalibrationState.CALIBRATE_LEFT_UP
        } else {
            layout = layout.copy(rightPad = committed)
            calibrationState = CalibrationState.READY
        }
    }

    private fun resetBoundaryCandidates() {
        leftBoundaryRadiusCandidate = null
        rightBoundaryRadiusCandidate = null
    }

    private fun calibrateLeftUp(surface: ControllerSurface, fingertip: Point3?) {
        val leftPad = layout.leftPad ?: return
        val contact = contactPoint(surface, fingertip) ?: return
        val dx = contact.x - leftPad.center.x
        val dy = contact.y - leftPad.center.y
        val length = sqrt(dx * dx + dy * dy)
        if (length < config.minimumAxisCalibrationDistanceMeters) return

        val yAxis = Point2(dx / length, dy / length)
        // Preserve a right-handed surface basis: identity +Y produces identity +X.
        val frame = PadCoordinateFrame(xAxis = Point2(yAxis.y, -yAxis.x), yAxis = yAxis)
        layout = layout.copy(leftPad = leftPad.copy(frame = frame))
        calibrationState = CalibrationState.PLACE_RIGHT_CENTER
    }

    private fun contactPoint(surface: ControllerSurface, fingertip: Point3?): Point2? {
        if (fingertip == null) return null
        val local = surface.worldToLocal(fingertip)
        return if (abs(local.z) <= config.contactDistanceMeters && surface.contains(local)) {
            Point2(local.x, local.y)
        } else null
    }

    private fun padState(
        surface: ControllerSurface,
        geometry: PadGeometry?,
        fingertip: Point3?,
        previous: PadState,
    ): PadState {
        val frame = geometry?.frame
        if (geometry == null || frame == null || fingertip == null || geometry.radiusMeters <= 0f) {
            return PadState()
        }
        val local = surface.worldToLocal(fingertip)
        if (abs(local.z) > config.contactDistanceMeters) return PadState()
        val displacementX = local.x - geometry.center.x
        val displacementY = local.y - geometry.center.y
        // The visible/calibrated radius defines normalization. A second, twice-as-large circle
        // acts as the release boundary so a hand can overshoot while retaining a clamped command.
        val releaseRadius = geometry.radiusMeters * config.inputReleaseRadiusMultiplier
        if (distance2D(local, geometry.center) > releaseRadius) return PadState()
        val rawX =
            ((displacementX * frame.xAxis.x + displacementY * frame.xAxis.y) /
                    geometry.radiusMeters)
                .coerceIn(-1f, 1f)
        val rawY =
            ((displacementX * frame.yAxis.x + displacementY * frame.yAxis.y) /
                    geometry.radiusMeters)
                .coerceIn(-1f, 1f)
        val filteredX = applyDeadZone(rawX)
        val filteredY = applyDeadZone(rawY)
        val alpha = config.smoothingFactor.coerceIn(0f, 1f)
        return PadState(
            x = previous.x + (filteredX - previous.x) * alpha,
            y = previous.y + (filteredY - previous.y) * alpha,
            active = true,
        )
    }

    private fun applyDeadZone(value: Float): Float {
        val magnitude = abs(value)
        if (magnitude <= config.deadZoneFraction) return 0f
        val scaled = (magnitude - config.deadZoneFraction) / (1f - config.deadZoneFraction)
        return if (value < 0f) -scaled else scaled
    }

    private fun distance2D(point: Point3, center: Point2): Float {
        val x = point.x - center.x
        val y = point.y - center.y
        return sqrt(x * x + y * y)
    }

    private fun nextNeutral(timestampNanos: Long) =
        AnyControllerFrame(
            sequence = ++sequence,
            timestampNanos = timestampNanos,
            calibrationState = calibrationState,
        )
}
