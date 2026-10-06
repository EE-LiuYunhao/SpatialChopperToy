package com.pico.spatial.choppertoy.content

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
        assertEquals(FlightEndReason.CRASHED, model.endReason)
        assertEquals(0f, model.liftForPalmHeight(2f), 0f)
    }

    @Test
    fun startAfterCrashBeginsAFreshRunningSession() {
        val model = HelicopterFlightModel()
        model.start(currentPalmHeightMeters = 1f)
        model.crash()

        val lift = model.start(currentPalmHeightMeters = 1.2f)

        assertEquals(FlightMode.RUNNING, model.mode)
        assertEquals(FlightEndReason.NONE, model.endReason)
        assertEquals(NEUTRAL_LIFT_NEWTONS, lift, 0.0001f)
    }

    @Test
    fun safeLandingStopsRotorWithoutCrashAudio() {
        assertEquals(
            listOf(FlightAudioCue.STOP_ROTOR),
            flightAudioCuesForTransition(
                previousMode = FlightMode.RUNNING,
                currentMode = FlightMode.NOT_RUNNING,
                causedByCrash = false,
            ),
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
    fun calibrationInvertsCyclicAtThreeFortiethsAndUsesShortestPathYaw() {
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

        assertEquals(-0.75f, result.instrumentReadout.pitchDegrees, 0.001f)
        assertEquals(-0.75f, result.instrumentReadout.rollDegrees, 0.001f)
        assertEquals(20f, result.instrumentReadout.headingDegrees, 0.001f)
        assertEquals(-0.75f, result.helicopterControl.pitchDegrees, 0.001f)
        assertEquals(-0.75f, result.helicopterControl.rollDegrees, 0.001f)
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
    fun lowSpeedHorizontalSurfaceBelowHelicopterIsSafeLanding() {
        val result =
            classifyTrackedPlaneCollision(
                mode = FlightMode.RUNNING,
                collidedWithTrackedPlane = true,
                surface =
                    LandingSurfaceSample(
                        positionInScene = Vector3(0.02f, 0f, 0.03f),
                        supportNormalInScene = Vector3.UP,
                        normalAlignmentWithHelicopterUp = 0.99f,
                    ),
                helicopterCenterInScene = Vector3(0f, 0.10f, 0f),
                relativeVelocityMetersPerSecond = Vector3(0.08f, -0.18f, 0.04f),
            )

        assertEquals(TrackedPlaneCollisionResult.LAND, result)
    }

    @Test
    fun horizontalSurfaceAboveNormalLandingSpeedIsCrash() {
        val result =
            classifyTrackedPlaneCollision(
                mode = FlightMode.RUNNING,
                collidedWithTrackedPlane = true,
                surface =
                    LandingSurfaceSample(
                        positionInScene = Vector3.ZERO,
                        supportNormalInScene = Vector3.UP,
                        normalAlignmentWithHelicopterUp = 1f,
                    ),
                helicopterCenterInScene = Vector3(0f, 0.10f, 0f),
                relativeVelocityMetersPerSecond = Vector3(0f, -0.71f, 0f),
            )

        assertEquals(TrackedPlaneCollisionResult.CRASH, result)
    }

    @Test
    fun gentleDescentWithModestTangentialDriftIsSafeLanding() {
        val assessment =
            assessTrackedPlaneCollision(
                mode = FlightMode.RUNNING,
                collidedWithTrackedPlane = true,
                surface =
                    LandingSurfaceSample(
                        positionInScene = Vector3(0.02f, 0f, 0.03f),
                        supportNormalInScene = Vector3.UP,
                        normalAlignmentWithHelicopterUp = 0.72f,
                    ),
                helicopterCenterInScene = Vector3(0f, 0.10f, 0f),
                relativeVelocityMetersPerSecond = Vector3(0.65f, -0.55f, 0f),
            )

        assertEquals(TrackedPlaneCollisionResult.LAND, assessment.result)
        assertTrue(assessment.hasHorizontalSurface)
        assertTrue(assessment.isSurfaceBelowHelicopter)
        assertTrue(assessment.hasAlignedSupportSurface)
        assertEquals(0.55f, requireNotNull(assessment.normalSpeedMetersPerSecond), 0.001f)
        assertEquals(0.65f, requireNotNull(assessment.tangentialSpeedMetersPerSecond), 0.001f)
    }

    @Test
    fun fastTangentialDriftRemainsCrash() {
        val result =
            classifyTrackedPlaneCollision(
                mode = FlightMode.RUNNING,
                collidedWithTrackedPlane = true,
                surface =
                    LandingSurfaceSample(
                        positionInScene = Vector3.ZERO,
                        supportNormalInScene = Vector3.UP,
                        normalAlignmentWithHelicopterUp = 1f,
                    ),
                helicopterCenterInScene = Vector3(0f, 0.10f, 0f),
                relativeVelocityMetersPerSecond = Vector3(0.76f, -0.1f, 0f),
            )

        assertEquals(TrackedPlaneCollisionResult.CRASH, result)
    }

    @Test
    fun nonHorizontalAboveOrUnsupportedSurfaceIsCrash() {
        val nonHorizontal =
            LandingSurfaceSample(
                positionInScene = Vector3.ZERO,
                supportNormalInScene = Vector3(1f, 0f, 0f),
                normalAlignmentWithHelicopterUp = 1f,
            )
        val aboveHelicopter =
            LandingSurfaceSample(
                positionInScene = Vector3(0f, 0.11f, 0f),
                supportNormalInScene = Vector3.UP,
                normalAlignmentWithHelicopterUp = 1f,
            )
        val unsupported =
            LandingSurfaceSample(
                positionInScene = Vector3.ZERO,
                supportNormalInScene = Vector3.UP,
                normalAlignmentWithHelicopterUp = 0.69f,
            )

        listOf(nonHorizontal, aboveHelicopter, unsupported).forEach { surface ->
            assertEquals(
                TrackedPlaneCollisionResult.CRASH,
                classifyTrackedPlaneCollision(
                    mode = FlightMode.RUNNING,
                    collidedWithTrackedPlane = true,
                    surface = surface,
                    helicopterCenterInScene = Vector3(0f, 0.10f, 0f),
                    relativeVelocityMetersPerSecond = Vector3.ZERO,
                ),
            )
        }
    }

    @Test
    fun untrackedOrInactiveContactIsIgnored() {
        val surface =
            LandingSurfaceSample(
                positionInScene = Vector3.ZERO,
                supportNormalInScene = Vector3.UP,
                normalAlignmentWithHelicopterUp = 1f,
            )

        assertEquals(
            TrackedPlaneCollisionResult.IGNORE,
            classifyTrackedPlaneCollision(
                mode = FlightMode.NOT_RUNNING,
                collidedWithTrackedPlane = true,
                surface = surface,
                helicopterCenterInScene = Vector3(0f, 0.10f, 0f),
                relativeVelocityMetersPerSecond = Vector3.ZERO,
            ),
        )
        assertEquals(
            TrackedPlaneCollisionResult.IGNORE,
            classifyTrackedPlaneCollision(
                mode = FlightMode.RUNNING,
                collidedWithTrackedPlane = false,
                surface = surface,
                helicopterCenterInScene = Vector3(0f, 0.10f, 0f),
                relativeVelocityMetersPerSecond = Vector3.ZERO,
            ),
        )
    }

    @Test
    fun landingSetsDedicatedEndReasonAndNextStartClearsIt() {
        val model = HelicopterFlightModel()
        model.start(currentPalmHeightMeters = 1f)

        model.land()
        assertEquals(FlightMode.NOT_RUNNING, model.mode)
        assertEquals(FlightEndReason.LANDED, model.endReason)

        model.start(currentPalmHeightMeters = 1f)
        assertEquals(FlightEndReason.NONE, model.endReason)
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
    fun levelNeutralLiftProducesNoNetForceOrVelocity() {
        val motion =
            solveHelicopterMotion(
                attitude = HelicopterAttitudeCommand(0f, 0f, 0f),
                liftNewtons = NEUTRAL_LIFT_NEWTONS,
                flightStartHeadingDegrees = 37f,
            )

        assertVectorEquals(Vector3.UP, motion.localLiftDirection)
        assertVectorEquals(Vector3.ZERO, motion.localNetForceNewtons)
        assertVectorEquals(Vector3.ZERO, motion.localVelocityMetersPerSecond)
        assertVectorEquals(Vector3.ZERO, motion.worldVelocityMetersPerSecond)
    }

    @Test
    fun pitchAndRollAreCombinedIntoOneThreeDimensionalLiftDirection() {
        val motion =
            solveHelicopterMotion(
                attitude = HelicopterAttitudeCommand(30f, -30f, 0f),
                liftNewtons = NEUTRAL_LIFT_NEWTONS,
                flightStartHeadingDegrees = 0f,
            )

        assertEquals(1f, motion.localLiftDirection.length(), 0.0001f)
        assertTrue(motion.localLiftDirection.x > 0f)
        assertTrue(motion.localLiftDirection.y > 0f)
        assertTrue(motion.localLiftDirection.z > 0f)
        assertTrue(motion.localVelocityMetersPerSecond.x > 0f)
        assertTrue(motion.localVelocityMetersPerSecond.y < 0f)
        assertTrue(motion.localVelocityMetersPerSecond.z > 0f)
    }

    @Test
    fun collectiveChangesVerticalVelocityAroundNeutralHover() {
        val raised =
            solveHelicopterMotion(
                attitude = HelicopterAttitudeCommand(0f, 0f, 0f),
                liftNewtons = NEUTRAL_LIFT_NEWTONS * 1.2f,
                flightStartHeadingDegrees = 0f,
            )
        val lowered =
            solveHelicopterMotion(
                attitude = HelicopterAttitudeCommand(0f, 0f, 0f),
                liftNewtons = NEUTRAL_LIFT_NEWTONS * 0.8f,
                flightStartHeadingDegrees = 0f,
            )

        assertTrue(raised.worldVelocityMetersPerSecond.y > 0f)
        assertTrue(lowered.worldVelocityMetersPerSecond.y < 0f)
    }

    @Test
    fun yawRotatesLocalFlightVelocityIntoStageSpace() {
        val motion =
            solveHelicopterMotion(
                attitude = HelicopterAttitudeCommand(30f, 0f, 90f),
                liftNewtons = NEUTRAL_LIFT_NEWTONS,
                flightStartHeadingDegrees = 0f,
            )

        assertEquals(0f, motion.localVelocityMetersPerSecond.x, 0.0001f)
        assertTrue(motion.localVelocityMetersPerSecond.z > 0f)
        assertTrue(motion.worldVelocityMetersPerSecond.x > 0f)
        assertEquals(0f, motion.worldVelocityMetersPerSecond.z, 0.0001f)
    }

    private fun assertVectorEquals(expected: Vector3, actual: Vector3, tolerance: Float = 0.0001f) {
        assertEquals(expected.x, actual.x, tolerance)
        assertEquals(expected.y, actual.y, tolerance)
        assertEquals(expected.z, actual.z, tolerance)
    }
}
