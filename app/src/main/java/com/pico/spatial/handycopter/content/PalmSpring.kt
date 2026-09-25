package com.pico.spatial.handycopter.content

import com.pico.spatial.core.math.Vector3
import kotlin.math.sqrt

internal enum class TrackedHand {
    LEFT,
    RIGHT,
}

/** Selects the right hand whenever available and falls back to the left hand. */
internal class RightPreferredHandSelector {
    var selected: TrackedHand? = null
        private set

    fun select(leftTracked: Boolean, rightTracked: Boolean): TrackedHand? {
        selected =
            when {
                rightTracked -> TrackedHand.RIGHT
                leftTracked -> TrackedHand.LEFT
                else -> null
            }
        return selected
    }
}

/** A stable, semi-implicit Euler integration of a unit-mass damped spring. */
internal class SpringFollower3D(private val stiffness: Float, dampingRatio: Float) {
    private val damping = 2f * dampingRatio * sqrt(stiffness)
    private var initialized = false
    private var position = Vector3.ZERO
    private var velocity = Vector3.ZERO

    init {
        require(stiffness > 0f)
        require(dampingRatio >= 0f)
    }

    fun snapTo(target: Vector3) {
        position = target
        velocity = Vector3.ZERO
        initialized = true
    }

    fun step(target: Vector3, deltaSeconds: Float): Vector3 {
        if (!initialized) {
            snapTo(target)
            return position
        }

        // Clamp update gaps so lifecycle stalls cannot inject unstable spring energy.
        val dt = deltaSeconds.coerceIn(MIN_DELTA_SECONDS, MAX_DELTA_SECONDS)
        val acceleration = (target - position) * stiffness - velocity * damping
        velocity += acceleration * dt
        position += velocity * dt
        return position
    }

    fun clear() {
        initialized = false
        position = Vector3.ZERO
        velocity = Vector3.ZERO
    }

    private companion object {
        const val MIN_DELTA_SECONDS = 1f / 240f
        const val MAX_DELTA_SECONDS = 1f / 30f
    }
}
