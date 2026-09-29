package com.example.anycontroller.library

import com.pico.spatial.core.math.Vector3
import org.junit.Assert.assertEquals
import org.junit.Test

class FingertipTrackingPointTest {
    @Test
    fun usesExactReportedJointPosition() {
        val point = fingertipTrackingPoint(position = Vector3(1f, 2f, 3f))

        assertEquals(1f, point.x, EPSILON)
        assertEquals(2f, point.y, EPSILON)
        assertEquals(3f, point.z, EPSILON)
    }

    private companion object {
        const val EPSILON = 0.000_001f
    }
}
