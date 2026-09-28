package com.pico.spatial.handycopter.content

import com.pico.spatial.core.math.Vector3
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class PalmSpringTest {
    @Test
    fun rightHandIsPreferredWhenBothHandsAreTracked() {
        val selector = RightPreferredHandSelector()

        assertEquals(TrackedHand.RIGHT, selector.select(leftTracked = true, rightTracked = true))
    }

    @Test
    fun rightHandTakesPriorityAfterLeftHandWasSelected() {
        val selector = RightPreferredHandSelector()

        assertEquals(TrackedHand.LEFT, selector.select(leftTracked = true, rightTracked = false))
        assertEquals(TrackedHand.RIGHT, selector.select(leftTracked = true, rightTracked = true))
    }

    @Test
    fun leftHandIsUsedOnlyWhenRightHandIsNotTracked() {
        val selector = RightPreferredHandSelector()

        assertEquals(TrackedHand.RIGHT, selector.select(leftTracked = true, rightTracked = true))
        assertEquals(TrackedHand.LEFT, selector.select(leftTracked = true, rightTracked = false))
        assertNull(selector.select(leftTracked = false, rightTracked = false))
    }

    @Test
    fun springLagsThenConvergesToItsTarget() {
        val spring = SpringFollower3D(stiffness = 58f, dampingRatio = 0.72f)
        spring.snapTo(Vector3.ZERO)

        val firstStep = spring.step(Vector3(1f, 0f, 0f), 1f / 90f)
        assertTrue(firstStep.x > 0f)
        assertTrue(firstStep.x < 1f)

        var settled = firstStep
        repeat(360) { settled = spring.step(Vector3(1f, 0f, 0f), 1f / 90f) }
        assertEquals(1f, settled.x, 0.001f)
        assertEquals(0f, settled.y, 0.001f)
        assertEquals(0f, settled.z, 0.001f)
    }

    @Test
    fun headingUsesStagePositiveZAsZeroAndPositiveXAsNinety() {
        val north =
            calculateInstrumentReadout(
                forward = Vector3(0f, 0f, 1f),
                actualRight = Vector3(1f, 0f, 0f),
            )
        val east =
            calculateInstrumentReadout(
                forward = Vector3(1f, 0f, 0f),
                actualRight = Vector3(0f, 0f, -1f),
            )
        val south =
            calculateInstrumentReadout(
                forward = Vector3(0f, 0f, -1f),
                actualRight = Vector3(-1f, 0f, 0f),
            )
        val west =
            calculateInstrumentReadout(
                forward = Vector3(-1f, 0f, 0f),
                actualRight = Vector3(0f, 0f, 1f),
            )

        assertEquals(0f, north.headingDegrees, 0.001f)
        assertEquals(90f, east.headingDegrees, 0.001f)
        assertEquals(180f, south.headingDegrees, 0.001f)
        assertEquals(270f, west.headingDegrees, 0.001f)
    }

    @Test
    fun attitudeReportsPitchAndRollInDegrees() {
        val fortyFiveDegrees = Math.sqrt(0.5).toFloat()
        val pitched =
            calculateInstrumentReadout(
                forward = Vector3(0f, fortyFiveDegrees, fortyFiveDegrees),
                actualRight = Vector3(1f, 0f, 0f),
            )
        val leftBank =
            calculateInstrumentReadout(
                forward = Vector3(0f, 0f, 1f),
                actualRight = Vector3(fortyFiveDegrees, fortyFiveDegrees, 0f),
            )
        val rightBank =
            calculateInstrumentReadout(
                forward = Vector3(0f, 0f, 1f),
                actualRight = Vector3(fortyFiveDegrees, -fortyFiveDegrees, 0f),
            )

        assertEquals(45f, pitched.pitchDegrees, 0.001f)
        assertEquals(0f, pitched.rollDegrees, 0.001f)
        assertEquals(-45f, leftBank.rollDegrees, 0.001f)
        assertEquals(45f, rightBank.rollDegrees, 0.001f)
    }

    @Test
    fun controlledAttitudePreservesDirectionAndUsesOneTwentiethGain() {
        val controlled =
            InstrumentReadout(pitchDegrees = 20f, rollDegrees = -40f, headingDegrees = 123f)
                .toControlledAttitude()

        assertEquals(1f, controlled.pitchDegrees, 0.001f)
        assertEquals(-2f, controlled.rollDegrees, 0.001f)
        assertEquals(123f, controlled.headingDegrees, 0.001f)
    }

    @Test
    fun attitudeBallUsesControlledPitchAndRollWithoutYaw() {
        val controlled =
            InstrumentReadout(pitchDegrees = 20f, rollDegrees = -35f, headingDegrees = 120f)
                .toControlledAttitude()
        val rotation = attitudeBallRotation(controlled)

        assertEquals(1f, rotation.pitch, 0.001f)
        assertEquals(0f, rotation.yaw, 0.001f)
        assertEquals(-1.75f, rotation.roll, 0.001f)
    }
}
