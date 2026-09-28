package com.pico.spatial.handycopter.content

import com.pico.spatial.core.math.Vector3
import com.pico.spatial.tracking.hand.HandJoint
import com.pico.spatial.tracking.hand.HandPose
import kotlin.math.atan2
import kotlin.math.sqrt

/** Palm pitch/roll gain shared by the attitude indicator and helicopter cyclic control. */
internal const val PALM_ATTITUDE_CONTROL_GAIN = 1f / 20f

internal data class InstrumentReadout(
    val pitchDegrees: Float,
    val rollDegrees: Float,
    val headingDegrees: Float,
) {
    companion object {
        val ZERO = InstrumentReadout(0f, 0f, 0f)
    }
}

internal data class PalmFrame(
    val fingerForward: Vector3,
    val horizontalForward: Vector3,
    val readout: InstrumentReadout,
)

/**
 * Builds a palm frame from joints whose positions are defined by the SDK in global stage space.
 * Palm-to-middle-tip supplies the user-facing "where the fingers point" direction. The index/little
 * metacarpals supply palm roll without relying on an undocumented joint-local axis.
 */
internal fun HandPose.palmFrame(hand: TrackedHand): PalmFrame? {
    val palm = validJoint(HandJoint.Index.PALM) ?: return null
    val middleTip = validJoint(HandJoint.Index.MIDDLE_TIP) ?: return null
    val indexMetacarpal = validJoint(HandJoint.Index.INDEX_METACARPAL) ?: return null
    val littleMetacarpal = validJoint(HandJoint.Index.LITTLE_METACARPAL) ?: return null

    val forward = normalizedOrNull(middleTip.position - palm.position) ?: return null
    val horizontalForward = normalizedOrNull(Vector3(forward.x, 0f, forward.z)) ?: return null
    // This makes the across-palm vector point to stage-right for either hand in a neutral pose.
    val acrossPalm =
        when (hand) {
            TrackedHand.RIGHT -> littleMetacarpal.position - indexMetacarpal.position
            TrackedHand.LEFT -> indexMetacarpal.position - littleMetacarpal.position
        }
    val actualRight =
        normalizedOrNull(acrossPalm - forward * dot(acrossPalm, forward)) ?: return null

    return PalmFrame(
        fingerForward = forward,
        horizontalForward = horizontalForward,
        readout = calculateInstrumentReadout(forward, actualRight),
    )
}

internal fun calculateInstrumentReadout(forward: Vector3, actualRight: Vector3): InstrumentReadout {
    val normalizedForward = normalizedOrNull(forward) ?: return InstrumentReadout.ZERO
    val normalizedRight = normalizedOrNull(actualRight) ?: return InstrumentReadout.ZERO
    val horizontalLength =
        sqrt(normalizedForward.x * normalizedForward.x + normalizedForward.z * normalizedForward.z)
    val heading =
        normalizeDegrees(
            Math.toDegrees(atan2(normalizedForward.x.toDouble(), normalizedForward.z.toDouble()))
                .toFloat()
        )
    val pitch =
        Math.toDegrees(atan2(normalizedForward.y.toDouble(), horizontalLength.toDouble())).toFloat()

    val referenceRight = normalizedOrNull(Vector3(normalizedForward.z, 0f, -normalizedForward.x))
    val roll =
        if (referenceRight == null) {
            0f
        } else {
            val cross = cross(referenceRight, normalizedRight)
            -Math.toDegrees(
                    atan2(
                        dot(cross, normalizedForward).toDouble(),
                        dot(referenceRight, normalizedRight).toDouble(),
                    )
                )
                .toFloat()
        }

    return InstrumentReadout(pitch, roll, heading)
}

/**
 * Converts filtered palm attitude into the display and flight-control convention.
 *
 * Both axes preserve the measured palm direction and are attenuated to one twentieth of the
 * measured angle. Absolute heading is not modified.
 */
internal fun InstrumentReadout.toControlledAttitude(): InstrumentReadout =
    copy(
        pitchDegrees = pitchDegrees * PALM_ATTITUDE_CONTROL_GAIN,
        rollDegrees = rollDegrees * PALM_ATTITUDE_CONTROL_GAIN,
    )

private fun HandPose.validJoint(index: HandJoint.Index): HandJoint? {
    val joint = handJoints.firstOrNull { it.index == index } ?: return null
    val position = joint.position
    return joint.takeIf {
        position.x.isFinite() &&
            position.y.isFinite() &&
            position.z.isFinite() &&
            position != Vector3.ZERO
    }
}

private fun normalizedOrNull(vector: Vector3): Vector3? {
    val magnitudeSquared = dot(vector, vector)
    if (!magnitudeSquared.isFinite() || magnitudeSquared < 1e-8f) return null
    return vector * (1f / sqrt(magnitudeSquared))
}

private fun dot(left: Vector3, right: Vector3): Float =
    left.x * right.x + left.y * right.y + left.z * right.z

private fun cross(left: Vector3, right: Vector3): Vector3 =
    Vector3(
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x,
    )

private fun normalizeDegrees(degrees: Float): Float = ((degrees % 360f) + 360f) % 360f
