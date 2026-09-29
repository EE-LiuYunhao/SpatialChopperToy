package com.example.anycontroller.library.model

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class PlaneBoundaryTest {
    @Test
    fun sharedTriangleEdgeIsExcludedFromBoundary() {
        val bottomLeft = Point3(0f, 0f, 0f)
        val bottomRight = Point3(1f, 0f, 0f)
        val topRight = Point3(1f, 1f, 0f)
        val topLeft = Point3(0f, 1f, 0f)

        val boundary =
            findBoundaryEdges(
                vertices = listOf(bottomLeft, bottomRight, topRight, topLeft),
                triangleIndices = listOf(0, 1, 2, 0, 2, 3),
            )
        val undirectedEdges = boundary.map { setOf(it.start, it.end) }.toSet()

        assertEquals(
            setOf(
                setOf(bottomLeft, bottomRight),
                setOf(bottomRight, topRight),
                setOf(topRight, topLeft),
                setOf(topLeft, bottomLeft),
            ),
            undirectedEdges,
        )
        assertFalse(setOf(bottomLeft, topRight) in undirectedEdges)
    }

    @Test
    fun invalidAndIncompleteTrianglesDoNotCreateEdges() {
        val vertices = listOf(Point3(0f, 0f, 0f), Point3(1f, 0f, 0f))

        assertEquals(emptyList<SurfaceBoundaryEdge>(), findBoundaryEdges(vertices, listOf(0, 1)))
        assertEquals(emptyList<SurfaceBoundaryEdge>(), findBoundaryEdges(vertices, listOf(0, 1, 9)))
    }

    @Test
    fun boundingBoxEdgesUseTheReportedLocalCenterAndExtents() {
        val surface =
            ControllerSurface(
                origin = Point3(2f, 3f, 4f),
                right = Point3(1f, 0f, 0f),
                up = Point3(0f, 1f, 0f),
                normal = Point3(0f, 0f, 1f),
                rotation = RotationValue(0f, 0f, 0f, 1f),
                halfWidthMeters = 0.4f,
                halfHeightMeters = 0.2f,
                localBoundsCenter = Point2(0.1f, -0.1f),
            )

        val edges = surface.boundingBoxEdges()

        assertEquals(4, edges.size)
        assertEquals(
            setOf(
                Point3(-0.3f, -0.3f, 0f),
                Point3(0.5f, -0.3f, 0f),
                Point3(0.5f, 0.1f, 0f),
                Point3(-0.3f, 0.1f, 0f),
            ),
            edges.flatMap { listOf(it.start, it.end) }.toSet(),
        )
    }

    @Test
    fun surfaceWireframesDisappearAfterSelection() {
        val detected = listOf(testSurface("one"), testSurface("two"))

        assertEquals(detected, visibleSurfacesBeforeSelection(detected, selectedSurface = null))
        assertTrue(
            visibleSurfacesBeforeSelection(detected, selectedSurface = detected.first()).isEmpty()
        )
    }

    private fun testSurface(id: String) =
        ControllerSurface(
            anchorId = id,
            origin = Point3(0f, 0f, 0f),
            right = Point3(1f, 0f, 0f),
            up = Point3(0f, 1f, 0f),
            normal = Point3(0f, 0f, 1f),
            rotation = RotationValue(0f, 0f, 0f, 1f),
            halfWidthMeters = 0.5f,
            halfHeightMeters = 0.5f,
        )
}
