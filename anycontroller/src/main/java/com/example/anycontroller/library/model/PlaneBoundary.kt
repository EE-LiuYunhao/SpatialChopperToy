package com.example.anycontroller.library.model

/** Returns the triangle edges referenced by exactly one face. */
internal fun findBoundaryEdges(
    vertices: List<Point3>,
    triangleIndices: List<Int>,
): List<SurfaceBoundaryEdge> {
    val edgeUseCounts = linkedMapOf<UndirectedEdge, Int>()

    triangleIndices.chunked(3).forEach { triangle ->
        if (
            triangle.size != 3 ||
                triangle.any { it !in vertices.indices } ||
                triangle.toSet().size != 3
        ) {
            return@forEach
        }
        countEdge(triangle[0], triangle[1], vertices.size, edgeUseCounts)
        countEdge(triangle[1], triangle[2], vertices.size, edgeUseCounts)
        countEdge(triangle[2], triangle[0], vertices.size, edgeUseCounts)
    }

    return edgeUseCounts
        .asSequence()
        .filter { (_, faceCount) -> faceCount == 1 }
        .map { (edge, _) -> SurfaceBoundaryEdge(vertices[edge.first], vertices[edge.second]) }
        .toList()
}

/** Returns the four local-space edges of a surface's axis-aligned bounding rectangle. */
internal fun ControllerSurface.boundingBoxEdges(): List<SurfaceBoundaryEdge> {
    val left = localBoundsCenter.x - halfWidthMeters
    val right = localBoundsCenter.x + halfWidthMeters
    val bottom = localBoundsCenter.y - halfHeightMeters
    val top = localBoundsCenter.y + halfHeightMeters
    val bottomLeft = Point3(left, bottom, 0f)
    val bottomRight = Point3(right, bottom, 0f)
    val topRight = Point3(right, top, 0f)
    val topLeft = Point3(left, top, 0f)
    return listOf(
        SurfaceBoundaryEdge(bottomLeft, bottomRight),
        SurfaceBoundaryEdge(bottomRight, topRight),
        SurfaceBoundaryEdge(topRight, topLeft),
        SurfaceBoundaryEdge(topLeft, bottomLeft),
    )
}

/** Returns selectable plane visuals only until a controller surface has been selected. */
internal fun visibleSurfacesBeforeSelection(
    detectedSurfaces: Collection<ControllerSurface>,
    selectedSurface: ControllerSurface?,
): Collection<ControllerSurface> = if (selectedSurface == null) detectedSurfaces else emptyList()

private fun countEdge(
    first: Int,
    second: Int,
    vertexCount: Int,
    counts: MutableMap<UndirectedEdge, Int>,
) {
    if (first == second || first !in 0 until vertexCount || second !in 0 until vertexCount) return
    val edge = if (first < second) UndirectedEdge(first, second) else UndirectedEdge(second, first)
    counts[edge] = (counts[edge] ?: 0) + 1
}

private data class UndirectedEdge(val first: Int, val second: Int)
