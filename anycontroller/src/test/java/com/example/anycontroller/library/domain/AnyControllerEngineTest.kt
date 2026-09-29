package com.example.anycontroller.library.domain

import com.example.anycontroller.library.model.AnyControllerConfig
import com.example.anycontroller.library.model.CalibrationState
import com.example.anycontroller.library.model.ControllerSurface
import com.example.anycontroller.library.model.Point2
import com.example.anycontroller.library.model.Point3
import com.example.anycontroller.library.model.RotationValue
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class AnyControllerEngineTest {
    private val surface =
        ControllerSurface(
            origin = Point3(0f, 0f, 0f),
            right = Point3(1f, 0f, 0f),
            up = Point3(0f, 1f, 0f),
            normal = Point3(0f, 0f, 1f),
            rotation = RotationValue(0f, 0f, 0f, 1f),
            halfWidthMeters = 1f,
            halfHeightMeters = 1f,
        )

    @Test
    fun calibrationUsesIndexForCentersLiveBoundaryPreviewAndLiftConfirmation() {
        val engine = AnyControllerEngine()
        engine.start()
        engine.selectSurface(surface)

        engine.update(Point3(-0.2f, 0f, 0f), null, 1)
        assertEquals(CalibrationState.PLACE_LEFT_BOUNDARY, engine.calibrationState)
        assertEquals(0f, engine.layout.leftPad!!.radiusMeters, 0f)
        engine.update(
            leftIndexTip = Point3(-0.1f, 0f, 0f),
            rightIndexTip = null,
            timestampNanos = 2,
        )
        assertEquals(CalibrationState.PLACE_LEFT_BOUNDARY, engine.calibrationState)
        assertEquals(0.1f, engine.layout.leftPad!!.radiusMeters, 0.0001f)
        engine.update(
            leftIndexTip = Point3(-0.1f, 0f, 0.05f),
            rightIndexTip = null,
            timestampNanos = 3,
        )
        assertEquals(CalibrationState.CALIBRATE_LEFT_UP, engine.calibrationState)

        engine.update(Point3(-0.2f, 0.1f, 0f), null, 4)
        assertEquals(CalibrationState.PLACE_RIGHT_CENTER, engine.calibrationState)

        engine.update(null, Point3(0.2f, 0f, 0f), 5)
        engine.update(leftIndexTip = null, rightIndexTip = Point3(0.3f, 0f, 0f), timestampNanos = 6)
        assertEquals(CalibrationState.PLACE_RIGHT_BOUNDARY, engine.calibrationState)
        val ready =
            engine.update(
                leftIndexTip = null,
                rightIndexTip = Point3(0.3f, 0f, 0.05f),
                timestampNanos = 7,
            )

        assertEquals(CalibrationState.READY, ready.calibrationState)
        assertEquals(0.1f, engine.layout.leftPad!!.radiusMeters, 0.0001f)
        assertEquals(0.1f, engine.layout.rightPad!!.radiusMeters, 0.0001f)
    }

    @Test
    fun boundaryTouchCloserThanMinimumRadiusIsIgnored() {
        val engine = AnyControllerEngine()
        engine.start()
        engine.selectSurface(surface)

        engine.update(Point3(-0.2f, 0f, 0f), null, 1)
        engine.update(
            leftIndexTip = Point3(-0.18f, 0f, 0f),
            rightIndexTip = null,
            timestampNanos = 2,
        )
        engine.update(
            leftIndexTip = Point3(-0.18f, 0f, 0.05f),
            rightIndexTip = null,
            timestampNanos = 3,
        )

        assertEquals(CalibrationState.PLACE_LEFT_BOUNDARY, engine.calibrationState)
        assertEquals(0f, engine.layout.leftPad!!.radiusMeters, 0f)
    }

    @Test
    fun boundaryGestureRequiresTrackedFingerToReachLiftDistance() {
        val engine = AnyControllerEngine()
        engine.start()
        engine.selectSurface(surface)
        engine.update(Point3(-0.2f, 0f, 0f), null, 1)

        engine.update(Point3(-0.1f, 0f, 0f), null, 2)
        engine.update(Point3(-0.1f, 0f, 0.049f), null, 3)
        assertEquals(CalibrationState.PLACE_LEFT_BOUNDARY, engine.calibrationState)

        engine.update(null, null, 4)
        assertEquals(CalibrationState.PLACE_LEFT_BOUNDARY, engine.calibrationState)

        engine.update(Point3(-0.1f, 0f, 0.05f), null, 5)
        assertEquals(CalibrationState.CALIBRATE_LEFT_UP, engine.calibrationState)
        assertEquals(0.1f, engine.layout.leftPad!!.radiusMeters, 0.0001f)
    }

    @Test
    fun boundaryGestureContinuouslyResizesPreviewAndCommitsLastSurfaceContact() {
        val engine = AnyControllerEngine()
        engine.start()
        engine.selectSurface(surface)
        engine.update(Point3(0f, 0f, 0f), null, 1)

        engine.update(Point3(0.08f, 0f, 0f), null, 2)
        assertEquals(0.08f, engine.layout.leftPad!!.radiusMeters, 0.0001f)
        engine.update(Point3(0.12f, 0f, 0f), null, 3)
        assertEquals(0.12f, engine.layout.leftPad!!.radiusMeters, 0.0001f)
        engine.update(Point3(0.06f, 0f, 0f), null, 4)
        assertEquals(0.06f, engine.layout.leftPad!!.radiusMeters, 0.0001f)

        engine.update(Point3(0.4f, 0f, 0.05f), null, 5)

        assertEquals(CalibrationState.CALIBRATE_LEFT_UP, engine.calibrationState)
        assertEquals(0.06f, engine.layout.leftPad!!.radiusMeters, 0.0001f)
    }

    @Test
    fun readyPadProducesNormalizedSmoothedInput() {
        val engine = calibratedEngine()
        val frame =
            engine.update(
                leftIndexTip = Point3(-0.2f, 0.1f, 0f),
                rightIndexTip = Point3(0.3f, 0f, 0f),
                timestampNanos = 7,
            )

        assertTrue(frame.trackingValid)
        assertTrue(frame.leftPad.active)
        assertTrue(frame.rightPad.active)
        assertTrue(frame.leftPad.y > 0f)
        assertTrue(frame.rightPad.x > 0f)
    }

    @Test
    fun padAxesAreNormalizedByEachCalibratedRadius() {
        val engine =
            calibratedEngine(AnyControllerConfig(deadZoneFraction = 0f, smoothingFactor = 1f))

        val frame =
            engine.update(
                leftIndexTip = Point3(-0.15f, 0.05f, 0f),
                rightIndexTip = Point3(0.15f, 0.05f, 0f),
                timestampNanos = 8,
            )

        assertTrue(frame.leftPad.active)
        assertEquals(0.5f, frame.leftPad.x, 0.0001f)
        assertEquals(0.5f, frame.leftPad.y, 0.0001f)
        assertTrue(frame.rightPad.active)
        assertEquals(-0.5f, frame.rightPad.x, 0.0001f)
        assertEquals(0.5f, frame.rightPad.y, 0.0001f)
    }

    @Test
    fun handsRemainClampedWithinDoubleRadiusAndBecomeNeutralBeyondIt() {
        val engine =
            calibratedEngine(AnyControllerConfig(deadZoneFraction = 0f, smoothingFactor = 1f))
        val insideReleaseBoundary =
            engine.update(
                leftIndexTip = Point3(-0.05f, 0f, 0f),
                rightIndexTip = Point3(0.35f, 0f, 0f),
                timestampNanos = 8,
            )
        assertTrue(insideReleaseBoundary.leftPad.active)
        assertEquals(1f, insideReleaseBoundary.leftPad.x, 0f)
        assertTrue(insideReleaseBoundary.rightPad.active)
        assertEquals(1f, insideReleaseBoundary.rightPad.x, 0f)

        val atReleaseBoundary =
            engine.update(
                leftIndexTip = Point3(0f, 0f, 0f),
                rightIndexTip = Point3(0.4f, 0f, 0f),
                timestampNanos = 9,
            )
        assertTrue(atReleaseBoundary.leftPad.active)
        assertEquals(1f, atReleaseBoundary.leftPad.x, 0f)
        assertTrue(atReleaseBoundary.rightPad.active)
        assertEquals(1f, atReleaseBoundary.rightPad.x, 0f)

        val outside =
            engine.update(
                leftIndexTip = Point3(0.01f, 0f, 0f),
                rightIndexTip = Point3(0.41f, 0f, 0f),
                timestampNanos = 10,
            )

        assertTrue(outside.trackingValid)
        assertFalse(outside.leftPad.active)
        assertEquals(0f, outside.leftPad.x, 0f)
        assertEquals(0f, outside.leftPad.y, 0f)
        assertFalse(outside.rightPad.active)
        assertEquals(0f, outside.rightPad.x, 0f)
        assertEquals(0f, outside.rightPad.y, 0f)
    }

    @Test
    fun liftedFingerReturnsNeutralPad() {
        val engine = calibratedEngine()
        val frame =
            engine.update(
                leftIndexTip = Point3(-0.2f, 0.1f, 0.2f),
                rightIndexTip = null,
                timestampNanos = 8,
            )

        assertFalse(frame.leftPad.active)
        assertEquals(0f, frame.leftPad.x, 0f)
        assertEquals(0f, frame.leftPad.y, 0f)
    }

    @Test
    fun rightPadCenterIsIndependentOfLeftPadPosition() {
        val engine = AnyControllerEngine()
        engine.start()
        engine.selectSurface(surface)

        engine.update(Point3(0.4f, 0.2f, 0f), null, 1)
        engine.update(Point3(0.5f, 0.2f, 0f), null, 2)
        engine.update(Point3(0.5f, 0.2f, 0.05f), null, 3)
        engine.update(Point3(0.4f, 0.3f, 0f), null, 4)

        engine.update(null, Point3(-0.45f, -0.3f, 0f), 5)

        assertEquals(Point2(0.4f, 0.2f), engine.layout.leftPad!!.center)
        assertEquals(Point2(-0.45f, -0.3f), engine.layout.rightPad!!.center)
        assertTrue(engine.layout.rightPad!!.center.x < engine.layout.leftPad!!.center.x)
    }

    @Test
    fun surfaceContainmentUsesActualLocalBoundsCenter() {
        val offsetSurface =
            surface.copy(
                halfWidthMeters = 0.25f,
                halfHeightMeters = 0.15f,
                localBoundsCenter = Point2(0.6f, -0.3f),
            )

        assertTrue(offsetSurface.contains(Point3(0.6f, -0.3f, 0f)))
        assertTrue(offsetSurface.contains(Point3(0.84f, -0.44f, 0f)))
        assertFalse(offsetSurface.contains(Point3(0f, 0f, 0f)))
    }

    @Test
    fun selectedSurfaceSnapshotRemainsStableDuringControllerUpdates() {
        val selectedSurface = surface.copy(anchorId = "surface-1")
        val engine = AnyControllerEngine()
        engine.start()
        engine.selectSurface(selectedSurface)
        engine.update(Point3(-0.3f, 0f, 0f), null, 1)

        engine.update(Point3(-0.2f, 0f, 0f), null, 2)

        assertEquals(selectedSurface, engine.layout.surface)
        assertEquals(Point2(-0.3f, 0f), engine.layout.leftPad!!.center)
    }

    @Test
    fun calibratedUpDirectionDefinesBothPadCoordinateFrames() {
        val engine = AnyControllerEngine()
        engine.start()
        engine.selectSurface(surface)
        engine.update(Point3(-0.2f, 0f, 0f), null, 1)
        engine.update(Point3(-0.1f, 0f, 0f), null, 2)
        engine.update(Point3(-0.1f, 0f, 0.05f), null, 3)

        // Surface-local +X is selected as controller +Y (up).
        engine.update(Point3(-0.1f, 0f, 0f), null, 4)
        engine.update(null, Point3(0.2f, 0f, 0f), 5)
        engine.update(null, Point3(0.3f, 0f, 0f), 6)
        engine.update(null, Point3(0.3f, 0f, 0.05f), 7)

        val leftFrame = engine.layout.leftPad!!.frame!!
        val rightFrame = engine.layout.rightPad!!.frame!!
        assertEquals(Point2(1f, 0f), leftFrame.yAxis)
        assertEquals(Point2(0f, -1f), leftFrame.xAxis)
        assertEquals(leftFrame, rightFrame)

        val frame =
            engine.update(
                leftIndexTip = Point3(-0.1f, 0f, 0f),
                rightIndexTip = Point3(0.2f, -0.1f, 0f),
                timestampNanos = 8,
            )
        assertTrue(frame.leftPad.y > 0f)
        assertTrue(frame.rightPad.x > 0f)
    }

    private fun calibratedEngine(
        config: AnyControllerConfig = AnyControllerConfig()
    ): AnyControllerEngine =
        AnyControllerEngine(config).also { engine ->
            engine.start()
            engine.selectSurface(surface)
            engine.update(Point3(-0.2f, 0f, 0f), null, 1)
            engine.update(Point3(-0.1f, 0f, 0f), null, 2)
            engine.update(Point3(-0.1f, 0f, 0.05f), null, 3)
            engine.update(Point3(-0.2f, 0.1f, 0f), null, 4)
            engine.update(null, Point3(0.2f, 0f, 0f), 5)
            engine.update(null, Point3(0.3f, 0f, 0f), 6)
            engine.update(null, Point3(0.3f, 0f, 0.05f), 7)
        }
}
