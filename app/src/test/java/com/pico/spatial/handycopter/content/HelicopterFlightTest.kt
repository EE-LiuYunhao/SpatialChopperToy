package com.pico.spatial.handycopter.content

import com.pico.spatial.core.math.EulerAngles
import com.pico.spatial.core.math.Vector3
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class HelicopterFlightTest {
    @Test
    fun audioStartsOnlyWhenEnteringRunning() {
        assertEquals(
            listOf(FlightAudioCue.START_ROTOR),
            flightAudioCuesForTransition(
                previousMode = FlightMode.NOT_RUNNING,
                currentMode = FlightMode.RUNNING,
                causedByCrash = false,
            ),
        )
        assertTrue(
            flightAudioCuesForTransition(
                    previousMode = FlightMode.RUNNING,
                    currentMode = FlightMode.RUNNING,
                    causedByCrash = false,
                )
                .isEmpty()
        )
    }

    @Test
    fun crashTransitionStopsRotorAndPlaysCrashOnce() {
        assertEquals(
            listOf(FlightAudioCue.STOP_ROTOR, FlightAudioCue.PLAY_CRASH),
            flightAudioCuesForTransition(
                previousMode = FlightMode.RUNNING,
                currentMode = FlightMode.NOT_RUNNING,
                causedByCrash = true,
            ),
        )
        assertTrue(
            flightAudioCuesForTransition(
                    previousMode = FlightMode.NOT_RUNNING,
                    currentMode = FlightMode.NOT_RUNNING,
                    causedByCrash = true,
                )
                .isEmpty()
        )
    }

    @Test
    fun startAppliesExactlyNeutralLift() {
        val model = HelicopterFlightModel()

        val lift = model.start(currentPalmHeightMeters = 1.1f)

        assertEquals(FlightMode.RUNNING, model.mode)
        assertEquals(NEUTRAL_LIFT_NEWTONS, lift, 0.0001f)
    }

    @Test
    fun collectiveHeightRaisesAndLowersLiftAroundStartHeight() {
        val model = HelicopterFlightModel()
        model.start(currentPalmHeightMeters = 1.0f)

        val lowered = model.liftForPalmHeight(0.8f)
        val neutral = model.liftForPalmHeight(1.0f)
        val raised = model.liftForPalmHeight(1.2f)

        assertTrue(lowered < NEUTRAL_LIFT_NEWTONS)
        assertEquals(NEUTRAL_LIFT_NEWTONS, neutral, 0.0001f)
        assertTrue(raised > NEUTRAL_LIFT_NEWTONS)
        assertEquals(NEUTRAL_LIFT_NEWTONS * 0.978f, lowered, 0.0001f)
        assertEquals(NEUTRAL_LIFT_NEWTONS * 1.022f, raised, 0.0001f)
    }

    @Test
    fun crashRemovesLiftAndReturnsToNotRunning() {
        val model = HelicopterFlightModel()
        model.start(currentPalmHeightMeters = 1f)

        model.crash()
        assertEquals(FlightMode.NOT_RUNNING, model.mode)
        assertTrue(model.crashed)
        assertEquals(0f, model.liftForPalmHeight(2f), 0f)
    }

    @Test
    fun startAfterCrashBeginsAFreshRunningSession() {
        val model = HelicopterFlightModel()
        model.start(currentPalmHeightMeters = 1f)
        model.crash()

        val lift = model.start(currentPalmHeightMeters = 1.2f)

        assertEquals(FlightMode.RUNNING, model.mode)
        assertFalse(model.crashed)
        assertEquals(NEUTRAL_LIFT_NEWTONS, lift, 0.0001f)
    }

    @Test
    fun gentleBottomContactOnUpwardPlaneIsSafe() {
        assertTrue(
            isSafeLandingContact(
                planeIsHorizontalUpward = true,
                localContactHeightsMeters = listOf(-0.074f, -0.078f),
                downwardSpeedMetersPerSecond = 0.8f,
            )
        )
    }

    @Test
    fun sideContactIsCrash() {
        assertFalse(
            isSafeLandingContact(
                planeIsHorizontalUpward = true,
                localContactHeightsMeters = listOf(0f),
                downwardSpeedMetersPerSecond = 0.2f,
            )
        )
    }

    @Test
    fun fastBottomContactIsCrash() {
        assertFalse(
            isSafeLandingContact(
                planeIsHorizontalUpward = true,
                localContactHeightsMeters = listOf(-HELICOPTER_HALF_HEIGHT_METERS),
                downwardSpeedMetersPerSecond = SAFE_LANDING_SPEED_METERS_PER_SECOND + 0.01f,
            )
        )
    }

    @Test
    fun contactWithWallIsCrashEvenAtBottom() {
        assertFalse(
            isSafeLandingContact(
                planeIsHorizontalUpward = false,
                localContactHeightsMeters = listOf(-HELICOPTER_HALF_HEIGHT_METERS),
                downwardSpeedMetersPerSecond = 0.1f,
            )
        )
    }

    @Test
    fun cyclicStaysLevelUntilHelicopterHasLaunchClearance() {
        val controller = HelicopterAttitudeController()
        controller.reset(HelicopterAttitudeCommand(0f, 0f, 180f))

        val attitude =
            controller.step(
                control =
                    PalmFlightControl(
                        palmHeightMeters = 1f,
                        pitchDegrees = 25f,
                        rollDegrees = -20f,
                        headingDegrees = 210f,
                    ),
                cyclicEnabled = false,
                deltaSeconds = 1f / 90f,
            )

        assertEquals(0f, attitude.pitchDegrees, 0f)
        assertEquals(0f, attitude.rollDegrees, 0f)
        assertEquals(210f, attitude.headingDegrees, 0f)
    }

    @Test
    fun cyclicIsBoundedAndSlewLimitedWhileYawRemainsExact() {
        val controller = HelicopterAttitudeController()
        controller.reset(HelicopterAttitudeCommand(0f, 0f, 0f))
        val control =
            PalmFlightControl(
                palmHeightMeters = 1f,
                pitchDegrees = 180f,
                rollDegrees = -180f,
                headingDegrees = 271.5f,
            )

        val first = controller.step(control, cyclicEnabled = true, deltaSeconds = 1f / 30f)
        var settled = first
        repeat(60) {
            settled = controller.step(control, cyclicEnabled = true, deltaSeconds = 1f / 30f)
        }

        assertEquals(2.5f, first.pitchDegrees, 0.001f)
        assertEquals(-2.5f, first.rollDegrees, 0.001f)
        assertEquals(271.5f, first.headingDegrees, 0f)
        assertEquals(MAX_CYCLIC_TILT_DEGREES, settled.pitchDegrees, 0.001f)
        assertEquals(-MAX_CYCLIC_TILT_DEGREES, settled.rollDegrees, 0.001f)
        assertEquals(271.5f, settled.headingDegrees, 0f)
    }

    @Test
    fun calibrationMakesStartPalmPoseZeroWithoutChangingHelicopterPose() {
        val calibration = PalmFlightCalibration()
        val palmAtStart =
            PalmFlightControl(
                palmHeightMeters = 1.17f,
                pitchDegrees = 12f,
                rollDegrees = -9f,
                headingDegrees = 347f,
            )
        calibration.calibrate(palmAtStart)
        val result = requireNotNull(calibration.resolve(palmAtStart))

        assertEquals(InstrumentReadout.ZERO, result.instrumentReadout)
        assertEquals(0f, result.helicopterControl.pitchDegrees, 0.001f)
        assertEquals(0f, result.helicopterControl.rollDegrees, 0.001f)
        assertEquals(0f, result.helicopterControl.headingDegrees, 0.001f)
        assertEquals(1.17f, result.helicopterControl.palmHeightMeters, 0.001f)
    }

    @Test
    fun calibrationUsesOneTwentiethCyclicAndShortestPathYawDeltas() {
        val calibration = PalmFlightCalibration()
        calibration.calibrate(palm = PalmFlightControl(1f, 10f, 175f, 350f))

        val result =
            requireNotNull(
                calibration.resolve(
                    PalmFlightControl(
                        palmHeightMeters = 1.2f,
                        pitchDegrees = 20f,
                        rollDegrees = -175f,
                        headingDegrees = 10f,
                    )
                )
            )

        assertEquals(0.5f, result.instrumentReadout.pitchDegrees, 0.001f)
        assertEquals(0.5f, result.instrumentReadout.rollDegrees, 0.001f)
        assertEquals(20f, result.instrumentReadout.headingDegrees, 0.001f)
        assertEquals(0.5f, result.helicopterControl.pitchDegrees, 0.001f)
        assertEquals(0.5f, result.helicopterControl.rollDegrees, 0.001f)
        assertEquals(20f, result.helicopterControl.headingDegrees, 0.001f)
    }

    @Test
    fun flightAttitudeIsComposedAfterTheAircraftBaseline() {
        val baseline = EulerAngles(pitch = 11f, yaw = 73f, roll = -8f).toQuat()
        val localDelta =
            HelicopterAttitudeCommand(pitchDegrees = 6f, rollDegrees = -4f, headingDegrees = 19f)

        val composed = composeLocalAttitude(baseline, localDelta)
        val expected =
            baseline *
                EulerAngles(
                        pitch = localDelta.pitchDegrees,
                        yaw = localDelta.headingDegrees,
                        roll = localDelta.rollDegrees,
                    )
                    .toQuat()
        val stageSpaceEulerAddition = EulerAngles(pitch = 17f, yaw = 92f, roll = -12f).toQuat()

        assertTrue(composed.equivalentCheck(expected))
        assertFalse(composed.equivalentCheck(stageSpaceEulerAddition))
    }

    @Test
    fun placementHeadingPointsTheTailBackTowardTheViewer() {
        assertEquals(
            180f,
            tailTowardViewerHeadingDegrees(
                helicopterPosition = Vector3(0f, 1f, -1f),
                viewerPosition = Vector3.ZERO,
                fallbackHeadingDegrees = 27f,
            ),
            0.001f,
        )
        assertEquals(
            90f,
            tailTowardViewerHeadingDegrees(
                helicopterPosition = Vector3(1f, 1f, 0f),
                viewerPosition = Vector3.ZERO,
                fallbackHeadingDegrees = 27f,
            ),
            0.001f,
        )
    }

    @Test
    fun placementHeadingRetainsYawWhenViewerAndHelicopterOverlapHorizontally() {
        assertEquals(
            27f,
            tailTowardViewerHeadingDegrees(
                helicopterPosition = Vector3(1f, 2f, 3f),
                viewerPosition = Vector3(1f, 1f, 3f),
                fallbackHeadingDegrees = 27f,
            ),
            0.001f,
        )
    }

    @Test
    fun levelNeutralLiftExactlyOpposesConfiguredGravity() {
        val force =
            calculateHelicopterAerodynamicForce(
                bodyUp = Vector3.UP,
                liftNewtons = NEUTRAL_LIFT_NEWTONS,
                velocityMetersPerSecond = Vector3.ZERO,
            )

        assertEquals(0f, force.x, 0.0001f)
        assertEquals(NEUTRAL_LIFT_NEWTONS, force.y, 0.0001f)
        assertEquals(0f, force.z, 0.0001f)
    }

    @Test
    fun tiltedLiftNaturallySplitsIntoVerticalAndHorizontalComponents() {
        val sixtyDegrees = Math.sqrt(0.75).toFloat()
        val force =
            calculateHelicopterAerodynamicForce(
                bodyUp = Vector3(0.5f, sixtyDegrees, 0f),
                liftNewtons = NEUTRAL_LIFT_NEWTONS,
                velocityMetersPerSecond = Vector3.ZERO,
            )

        assertEquals(NEUTRAL_LIFT_NEWTONS * 0.5f, force.x, 0.0001f)
        assertEquals(NEUTRAL_LIFT_NEWTONS * sixtyDegrees, force.y, 0.0001f)
        assertEquals(0f, force.z, 0.0001f)
    }

    @Test
    fun aerodynamicDragOpposesHorizontalAndVerticalVelocity() {
        val force =
            calculateHelicopterAerodynamicForce(
                bodyUp = Vector3.UP,
                liftNewtons = 0f,
                velocityMetersPerSecond = Vector3(2f, -1f, -1f),
            )

        assertTrue(force.x < 0f)
        assertTrue(force.y > 0f)
        assertTrue(force.z > 0f)
    }
}
