package com.pico.spatial.choppertoy.content

import kotlin.math.abs

/** Ordered single-degree-of-freedom lessons in the first-install palm-control tutorial. */
internal enum class FlightTutorialStep {
    PITCH_FORWARD,
    PITCH_BACKWARD,
    ROLL_LEFT,
    ROLL_RIGHT,
    ASCEND,
    DESCEND,
    YAW,
}

/** UI-facing state for the tutorial panel attached to the helicopter body. */
internal data class FlightTutorialUiState(
    val step: FlightTutorialStep = FlightTutorialStep.PITCH_FORWARD,
    val stepNumber: Int = 1,
    val stepCount: Int = FlightTutorialStep.entries.size,
    val canAdvance: Boolean = false,
    val waitingForRightHand: Boolean = true,
    val preparingHelicopter: Boolean = true,
)

/** One projected tutorial frame using the same calibrated mapping as normal flight. */
internal data class FlightTutorialFrame(
    val step: FlightTutorialStep,
    val attitude: HelicopterAttitudeCommand,
    val liftNewtons: Float,
    val instrumentReadout: InstrumentReadout,
    val gestureCurrentlyCorrect: Boolean,
    val canAdvance: Boolean,
)

/** Result of advancing the tutorial after the required gesture has been demonstrated. */
internal enum class FlightTutorialAdvanceResult {
    BLOCKED,
    ADVANCED,
    COMPLETED,
}

/**
 * Calibrates and gates the seven-step tutorial without introducing a second control mapping.
 *
 * Each lesson captures the current palm as its neutral pose, resolves shortest-path deltas through
 * the production pitch/roll gain and yaw convention, and projects the result onto exactly one
 * control degree of freedom. A successful gesture remains latched until Next is pressed.
 */
internal class FlightTutorialSession {
    private data class TutorialSignals(
        val pitchDegrees: Float,
        val rollDegrees: Float,
        val yawDegrees: Float,
        val heightDeltaMeters: Float,
    )

    private var stepIndex = 0
    private var palmAtStepStart: PalmFlightControl? = null
    private var gestureSatisfied = false

    /** Current lesson in the fixed tutorial order. */
    val step: FlightTutorialStep
        get() = FlightTutorialStep.entries[stepIndex]

    /** Whether the current directional gesture has been demonstrated. */
    val canAdvance: Boolean
        get() = gestureSatisfied

    /** Captures the first lesson's neutral palm pose. */
    fun begin(palm: PalmFlightControl) {
        stepIndex = 0
        palmAtStepStart = palm
        gestureSatisfied = false
    }

    /** Resolves one raw palm sample into the current single-DoF tutorial command. */
    fun update(palm: PalmFlightControl): FlightTutorialFrame? {
        val baseline = palmAtStepStart ?: return null
        val signals = resolveSignals(baseline, palm)
        val currentStep = step
        val attitude = projectAttitude(currentStep, signals)
        val liftNewtons = projectLift(currentStep, signals.heightDeltaMeters)
        val currentlyCorrect = requiredGestureIsPresent(currentStep, signals)
        gestureSatisfied = gestureSatisfied || currentlyCorrect
        return FlightTutorialFrame(
            step = currentStep,
            attitude = attitude,
            liftNewtons = liftNewtons,
            instrumentReadout =
                InstrumentReadout(
                    pitchDegrees = attitude.pitchDegrees,
                    rollDegrees = attitude.rollDegrees,
                    headingDegrees = normalizeHeadingDegrees(attitude.headingDegrees),
                ),
            gestureCurrentlyCorrect = currentlyCorrect,
            canAdvance = gestureSatisfied,
        )
    }

    /** Advances to the next lesson and recalibrates neutral, or completes the final lesson. */
    fun advance(currentPalm: PalmFlightControl): FlightTutorialAdvanceResult {
        if (!gestureSatisfied) return FlightTutorialAdvanceResult.BLOCKED
        if (stepIndex == FlightTutorialStep.entries.lastIndex) {
            return FlightTutorialAdvanceResult.COMPLETED
        }
        stepIndex += 1
        palmAtStepStart = currentPalm
        gestureSatisfied = false
        return FlightTutorialAdvanceResult.ADVANCED
    }

    private fun resolveSignals(
        baseline: PalmFlightControl,
        palm: PalmFlightControl,
    ): TutorialSignals =
        TutorialSignals(
            pitchDegrees =
                mapPalmCyclicAngle(
                    shortestSignedAngleDelta(baseline.pitchDegrees, palm.pitchDegrees)
                ),
            rollDegrees =
                mapPalmCyclicAngle(
                    shortestSignedAngleDelta(baseline.rollDegrees, palm.rollDegrees)
                ),
            yawDegrees = shortestSignedAngleDelta(baseline.headingDegrees, palm.headingDegrees),
            heightDeltaMeters = palm.palmHeightMeters - baseline.palmHeightMeters,
        )

    private fun projectAttitude(
        tutorialStep: FlightTutorialStep,
        signals: TutorialSignals,
    ): HelicopterAttitudeCommand =
        when (tutorialStep) {
            FlightTutorialStep.PITCH_FORWARD,
            FlightTutorialStep.PITCH_BACKWARD ->
                HelicopterAttitudeCommand(signals.pitchDegrees, 0f, 0f)
            FlightTutorialStep.ROLL_LEFT,
            FlightTutorialStep.ROLL_RIGHT -> HelicopterAttitudeCommand(0f, signals.rollDegrees, 0f)
            FlightTutorialStep.ASCEND,
            FlightTutorialStep.DESCEND -> HelicopterAttitudeCommand(0f, 0f, 0f)
            FlightTutorialStep.YAW -> HelicopterAttitudeCommand(0f, 0f, signals.yawDegrees)
        }

    private fun projectLift(tutorialStep: FlightTutorialStep, heightDeltaMeters: Float): Float {
        val isCollectiveLesson =
            tutorialStep == FlightTutorialStep.ASCEND || tutorialStep == FlightTutorialStep.DESCEND
        if (!isCollectiveLesson) return NEUTRAL_LIFT_NEWTONS
        val multiplier =
            (1f + heightDeltaMeters * PALM_COLLECTIVE_GAIN_PER_METER).coerceIn(0f, 2.25f)
        return NEUTRAL_LIFT_NEWTONS * multiplier
    }

    private fun requiredGestureIsPresent(
        tutorialStep: FlightTutorialStep,
        signals: TutorialSignals,
    ): Boolean =
        when (tutorialStep) {
            FlightTutorialStep.PITCH_FORWARD ->
                signals.pitchDegrees >= CYCLIC_GESTURE_THRESHOLD_DEGREES
            FlightTutorialStep.PITCH_BACKWARD ->
                signals.pitchDegrees <= -CYCLIC_GESTURE_THRESHOLD_DEGREES
            FlightTutorialStep.ROLL_LEFT -> signals.rollDegrees <= -CYCLIC_GESTURE_THRESHOLD_DEGREES
            FlightTutorialStep.ROLL_RIGHT -> signals.rollDegrees >= CYCLIC_GESTURE_THRESHOLD_DEGREES
            FlightTutorialStep.ASCEND ->
                signals.heightDeltaMeters >= COLLECTIVE_GESTURE_THRESHOLD_METERS
            FlightTutorialStep.DESCEND ->
                signals.heightDeltaMeters <= -COLLECTIVE_GESTURE_THRESHOLD_METERS
            FlightTutorialStep.YAW -> abs(signals.yawDegrees) >= YAW_GESTURE_THRESHOLD_DEGREES
        }

    private companion object {
        const val CYCLIC_GESTURE_THRESHOLD_DEGREES = 1.5f
        const val COLLECTIVE_GESTURE_THRESHOLD_METERS = 0.06f
        const val YAW_GESTURE_THRESHOLD_DEGREES = 15f
    }
}
