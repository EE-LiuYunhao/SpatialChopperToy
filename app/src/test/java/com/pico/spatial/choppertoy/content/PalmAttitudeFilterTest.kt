package com.pico.spatial.choppertoy.content

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class PalmAttitudeFilterTest {
    private val filter =
        PalmAttitudeLowPassFilter(
            timeConstantSeconds = 0.12f,
            maxAngularSpeedDegreesPerSecond = 180f,
        )

    @Test
    fun abruptMeasurementIsLowPassFilteredAndSlewLimited() {
        filter.step(InstrumentReadout.ZERO, FRAME_SECONDS)

        val filtered =
            filter.step(
                InstrumentReadout(pitchDegrees = 80f, rollDegrees = -120f, headingDegrees = 123f),
                FRAME_SECONDS,
            )

        assertEquals(2f, filtered.pitchDegrees, 0.001f)
        assertEquals(-2f, filtered.rollDegrees, 0.001f)
        assertEquals(123f, filtered.headingDegrees, 0.001f)
    }

    @Test
    fun stableMeasurementConvergesWithoutPermanentBias() {
        filter.step(InstrumentReadout.ZERO, FRAME_SECONDS)
        var filtered = InstrumentReadout.ZERO

        repeat(360) {
            filtered =
                filter.step(
                    InstrumentReadout(pitchDegrees = 35f, rollDegrees = -50f, headingDegrees = 0f),
                    FRAME_SECONDS,
                )
        }

        assertEquals(35f, filtered.pitchDegrees, 0.001f)
        assertEquals(-50f, filtered.rollDegrees, 0.001f)
    }

    @Test
    fun rollRemainsContinuousAcrossSignedAngleBoundary() {
        filter.step(
            InstrumentReadout(pitchDegrees = 0f, rollDegrees = 179f, headingDegrees = 0f),
            FRAME_SECONDS,
        )

        val filtered =
            filter.step(
                InstrumentReadout(pitchDegrees = 0f, rollDegrees = -179f, headingDegrees = 0f),
                FRAME_SECONDS,
            )

        assertTrue(filtered.rollDegrees > 179f)
        assertTrue(filtered.rollDegrees < 180f)
    }

    @Test
    fun resetSnapsToNewlyAcquiredHandInsteadOfBlendingOldPose() {
        filter.step(InstrumentReadout.ZERO, FRAME_SECONDS)
        filter.step(
            InstrumentReadout(pitchDegrees = 60f, rollDegrees = 60f, headingDegrees = 0f),
            FRAME_SECONDS,
        )

        filter.reset()
        val reacquired =
            filter.step(
                InstrumentReadout(pitchDegrees = -25f, rollDegrees = -40f, headingDegrees = 90f),
                FRAME_SECONDS,
            )

        assertEquals(-25f, reacquired.pitchDegrees, 0.001f)
        assertEquals(-40f, reacquired.rollDegrees, 0.001f)
        assertEquals(90f, reacquired.headingDegrees, 0.001f)
    }

    private companion object {
        const val FRAME_SECONDS = 1f / 90f
    }
}
