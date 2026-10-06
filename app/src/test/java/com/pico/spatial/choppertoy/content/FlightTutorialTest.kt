package com.pico.spatial.choppertoy.content

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class FlightTutorialTest {
    @Test
    fun incorrectGestureCannotAdvanceAndOnlyLessonAxisReachesOutput() {
        val session = FlightTutorialSession()
        session.begin(palm(height = 1f))

        val wrongDirection = requireNotNull(session.update(palm(height = 1.2f, pitch = 30f)))

        assertEquals(FlightTutorialStep.PITCH_FORWARD, wrongDirection.step)
        assertFalse(wrongDirection.canAdvance)
        assertEquals(0f, wrongDirection.attitude.rollDegrees, 0f)
        assertEquals(0f, wrongDirection.attitude.headingDegrees, 0f)
        assertEquals(NEUTRAL_LIFT_NEWTONS, wrongDirection.liftNewtons, 0.0001f)
        assertEquals(
            FlightTutorialAdvanceResult.BLOCKED,
            session.advance(palm(height = 1.2f, pitch = 30f)),
        )
    }

    @Test
    fun successfulGestureStaysLatchedUntilNext() {
        val session = FlightTutorialSession()
        val neutral = palm(height = 1f)
        session.begin(neutral)

        assertTrue(requireNotNull(session.update(palm(height = 1f, pitch = -20f))).canAdvance)
        val returnedToNeutral = requireNotNull(session.update(neutral))

        assertFalse(returnedToNeutral.gestureCurrentlyCorrect)
        assertTrue(returnedToNeutral.canAdvance)
    }

    @Test
    fun sevenLessonsAdvanceInOrderAndRecalibrateEachNeutral() {
        val session = FlightTutorialSession()
        var current = palm(height = 1f)
        session.begin(current)

        current = palm(height = 1f, pitch = -20f)
        assertGestureAndAdvance(session, current, FlightTutorialStep.PITCH_FORWARD)
        current = palm(height = 1f, pitch = 20f)
        assertGestureAndAdvance(session, current, FlightTutorialStep.PITCH_BACKWARD)
        current = palm(height = 1f, pitch = 20f, roll = 20f)
        assertGestureAndAdvance(session, current, FlightTutorialStep.ROLL_LEFT)
        current = palm(height = 1f, pitch = 20f, roll = -20f)
        assertGestureAndAdvance(session, current, FlightTutorialStep.ROLL_RIGHT)
        current = palm(height = 1.08f, pitch = 20f, roll = -20f)
        assertGestureAndAdvance(session, current, FlightTutorialStep.ASCEND)
        current = palm(height = 1f, pitch = 20f, roll = -20f)
        assertGestureAndAdvance(session, current, FlightTutorialStep.DESCEND)
        current = palm(height = 1f, pitch = 20f, roll = -20f, heading = 20f)
        val yawFrame = requireNotNull(session.update(current))

        assertEquals(FlightTutorialStep.YAW, yawFrame.step)
        assertEquals(0f, yawFrame.attitude.pitchDegrees, 0f)
        assertEquals(0f, yawFrame.attitude.rollDegrees, 0f)
        assertEquals(20f, yawFrame.attitude.headingDegrees, 0.001f)
        assertEquals(FlightTutorialAdvanceResult.COMPLETED, session.advance(current))
    }

    @Test
    fun collectiveLessonsUseProductionGainWithNoAttitudeFreedom() {
        val session = FlightTutorialSession()
        var current = palm(height = 1f)
        session.begin(current)
        current = palm(height = 1f, pitch = -20f)
        assertGestureAndAdvance(session, current, FlightTutorialStep.PITCH_FORWARD)
        current = palm(height = 1f, pitch = 20f)
        assertGestureAndAdvance(session, current, FlightTutorialStep.PITCH_BACKWARD)
        current = palm(height = 1f, pitch = 20f, roll = 20f)
        assertGestureAndAdvance(session, current, FlightTutorialStep.ROLL_LEFT)
        current = palm(height = 1f, pitch = 20f, roll = -20f)
        assertGestureAndAdvance(session, current, FlightTutorialStep.ROLL_RIGHT)

        val ascend =
            requireNotNull(
                session.update(palm(height = 1.1f, pitch = 80f, roll = -80f, heading = 90f))
            )

        assertEquals(HelicopterAttitudeCommand(0f, 0f, 0f), ascend.attitude)
        assertEquals(
            NEUTRAL_LIFT_NEWTONS * (1f + 0.1f * PALM_COLLECTIVE_GAIN_PER_METER),
            ascend.liftNewtons,
            0.0001f,
        )
    }

    @Test
    fun pitchLessonsGateTheSameLongitudinalSignsAsProductionFlight() {
        val session = FlightTutorialSession()
        var current = palm(height = 1f)
        session.begin(current)

        current = palm(height = 1f, pitch = -30f)
        val forward = requireNotNull(session.update(current))
        val forwardMotion =
            solveHelicopterMotion(
                attitude = forward.attitude,
                liftNewtons = forward.liftNewtons,
                flightStartHeadingDegrees = 0f,
            )

        assertTrue(forward.gestureCurrentlyCorrect)
        assertTrue(forward.attitude.pitchDegrees > 0f)
        assertTrue(forwardMotion.localVelocityMetersPerSecond.z > 0f)
        assertEquals(FlightTutorialAdvanceResult.ADVANCED, session.advance(current))

        current = palm(height = 1f, pitch = 10f)
        val backward = requireNotNull(session.update(current))
        val backwardMotion =
            solveHelicopterMotion(
                attitude = backward.attitude,
                liftNewtons = backward.liftNewtons,
                flightStartHeadingDegrees = 0f,
            )

        assertTrue(backward.gestureCurrentlyCorrect)
        assertTrue(backward.attitude.pitchDegrees < 0f)
        assertTrue(backwardMotion.localVelocityMetersPerSecond.z < 0f)
    }

    private fun assertGestureAndAdvance(
        session: FlightTutorialSession,
        palm: PalmFlightControl,
        expectedStep: FlightTutorialStep,
    ) {
        val frame = requireNotNull(session.update(palm))
        assertEquals(expectedStep, frame.step)
        assertTrue(frame.gestureCurrentlyCorrect)
        assertEquals(FlightTutorialAdvanceResult.ADVANCED, session.advance(palm))
    }

    private fun palm(height: Float, pitch: Float = 0f, roll: Float = 0f, heading: Float = 0f) =
        PalmFlightControl(height, pitch, roll, heading)
}
