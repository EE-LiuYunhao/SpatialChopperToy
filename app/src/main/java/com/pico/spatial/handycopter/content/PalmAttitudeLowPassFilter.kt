package com.pico.spatial.handycopter.content

import kotlin.math.exp

/**
 * Time-based low-pass filter for the palm attitude channels that drive the 3D attitude ball.
 *
 * Roll uses the shortest path across the -180/180 boundary, while the slew-rate limit prevents a
 * single bad tracking sample from producing a visibly abrupt rotation.
 */
internal class PalmAttitudeLowPassFilter(
    private val timeConstantSeconds: Float,
    private val maxAngularSpeedDegreesPerSecond: Float,
) {
    private var initialized = false
    private var filteredPitchDegrees = 0f
    private var filteredRollDegrees = 0f

    init {
        require(timeConstantSeconds > 0f)
        require(maxAngularSpeedDegreesPerSecond > 0f)
    }

    fun step(raw: InstrumentReadout, deltaSeconds: Float): InstrumentReadout {
        if (!initialized) {
            filteredPitchDegrees = raw.pitchDegrees
            filteredRollDegrees = raw.rollDegrees
            initialized = true
            return raw
        }

        val dt = deltaSeconds.coerceIn(MIN_DELTA_SECONDS, MAX_DELTA_SECONDS)
        val alpha = 1f - exp((-dt / timeConstantSeconds).toDouble()).toFloat()
        val maximumStep = maxAngularSpeedDegreesPerSecond * dt

        filteredPitchDegrees =
            lowPassStep(
                current = filteredPitchDegrees,
                delta = raw.pitchDegrees - filteredPitchDegrees,
                alpha = alpha,
                maximumStep = maximumStep,
            )
        filteredRollDegrees =
            lowPassStep(
                current = filteredRollDegrees,
                delta = shortestSignedAngleDelta(filteredRollDegrees, raw.rollDegrees),
                alpha = alpha,
                maximumStep = maximumStep,
            )

        return raw.copy(pitchDegrees = filteredPitchDegrees, rollDegrees = filteredRollDegrees)
    }

    fun reset() {
        initialized = false
        filteredPitchDegrees = 0f
        filteredRollDegrees = 0f
    }

    private fun lowPassStep(current: Float, delta: Float, alpha: Float, maximumStep: Float): Float =
        current + (delta * alpha).coerceIn(-maximumStep, maximumStep)

    private companion object {
        const val MIN_DELTA_SECONDS = 1f / 240f
        const val MAX_DELTA_SECONDS = 1f / 30f
    }
}
