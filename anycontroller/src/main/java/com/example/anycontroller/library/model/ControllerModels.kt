package com.example.anycontroller.library.model

import kotlin.math.sqrt

/** Two-dimensional point or vector expressed in a selected plane's local coordinates. */
data class Point2(val x: Float, val y: Float)

/** User-calibrated orthonormal axes for one virtual pad. */
data class PadCoordinateFrame(
    /** Unit vector for positive controller X, expressed in selected-surface local XY. */
    val xAxis: Point2,
    /** Unit vector for positive controller Y, expressed in selected-surface local XY. */
    val yAxis: Point2,
)

/** Three-dimensional point or vector used by the platform-neutral calibration engine. */
data class Point3(val x: Float, val y: Float, val z: Float) {
    /** Adds another point or vector component-wise. */
    operator fun plus(other: Point3) = Point3(x + other.x, y + other.y, z + other.z)

    /** Subtracts another point or vector component-wise. */
    operator fun minus(other: Point3) = Point3(x - other.x, y - other.y, z - other.z)

    /** Scales every component by [scale]. */
    operator fun times(scale: Float) = Point3(x * scale, y * scale, z * scale)

    /** Returns the Euclidean vector length. */
    fun length(): Float = sqrt(x * x + y * y + z * z)

    companion object {
        /** Returns the scalar dot product of [a] and [b]. */
        fun dot(a: Point3, b: Point3): Float = a.x * b.x + a.y * b.y + a.z * b.z
    }
}

/** Quaternion components stored without a dependency on Spatial SDK math types. */
data class RotationValue(val x: Float, val y: Float, val z: Float, val w: Float)

/** One visible boundary segment expressed in selected-surface local coordinates. */
data class SurfaceBoundaryEdge(val start: Point3, val end: Point3)

/** Available visual treatments for planes while the user is selecting a surface. */
enum class SurfaceVisualizationMode {
    BOUNDARY_EDGES,
    MESH_WIREFRAME,
}

/** Lifecycle state of the plane-tracking subsystem. */
enum class SurfaceDetectionStatus {
    STOPPED,
    STARTING,
    RUNNING,
    ERROR,
}

/** Immutable snapshot of a detected plane used throughout one calibration session. */
data class ControllerSurface(
    val anchorId: String? = null,
    val origin: Point3,
    val right: Point3,
    val up: Point3,
    val normal: Point3,
    val rotation: RotationValue,
    val halfWidthMeters: Float,
    val halfHeightMeters: Float,
    val localBoundsCenter: Point2 = Point2(0f, 0f),
    val boundaryEdges: List<SurfaceBoundaryEdge> = emptyList(),
) {
    /** Converts a world-space point to the selected plane's local coordinate system. */
    fun worldToLocal(point: Point3): Point3 {
        val delta = point - origin
        return Point3(
            x = Point3.dot(delta, right),
            y = Point3.dot(delta, up),
            z = Point3.dot(delta, normal),
        )
    }

    /** Converts a plane-local point and normal offset back to world space. */
    fun localToWorld(point: Point2, normalOffsetMeters: Float = 0f): Point3 =
        origin + right * point.x + up * point.y + normal * normalOffsetMeters

    /** Returns whether [local] lies inside this plane's rectangular bounds. */
    fun contains(local: Point3): Boolean =
        kotlin.math.abs(local.x - localBoundsCenter.x) <= halfWidthMeters &&
            kotlin.math.abs(local.y - localBoundsCenter.y) <= halfHeightMeters
}

/** Normalized axes and contact state for one virtual pad. */
data class PadState(val x: Float = 0f, val y: Float = 0f, val active: Boolean = false)

/** Calibrated center, radius, and axes of one virtual pad. */
data class PadGeometry(
    val center: Point2,
    val radiusMeters: Float,
    val frame: PadCoordinateFrame? = null,
)

/** Ordered steps in the two-pad calibration workflow. */
enum class CalibrationState {
    STOPPED,
    WAITING_FOR_SURFACE,
    PLACE_LEFT_CENTER,
    PLACE_LEFT_BOUNDARY,
    CALIBRATE_LEFT_UP,
    PLACE_RIGHT_CENTER,
    PLACE_RIGHT_BOUNDARY,
    READY,
}

/** High-level operational status exposed to host application UI. */
enum class AnyControllerStatus {
    STOPPED,
    STARTING,
    WAITING_FOR_SURFACE,
    CALIBRATING,
    READY,
    UNAVAILABLE,
    ERROR,
}

/** Current selected surface and partially or fully calibrated pad geometry. */
data class ControllerLayout(
    val surface: ControllerSurface? = null,
    val leftPad: PadGeometry? = null,
    val rightPad: PadGeometry? = null,
)

/** Immutable frame-aligned input snapshot published to the host application. */
data class AnyControllerFrame(
    val sequence: Long = 0,
    val timestampNanos: Long = 0,
    val leftPad: PadState = PadState(),
    val rightPad: PadState = PadState(),
    val trackingValid: Boolean = false,
    val calibrationState: CalibrationState = CalibrationState.STOPPED,
) {
    companion object {
        /** Fully inactive default frame. */
        val Neutral = AnyControllerFrame()
    }
}

/**
 * Thresholds and filtering parameters used by
 * [com.example.anycontroller.library.domain.AnyControllerEngine].
 */
data class AnyControllerConfig(
    val contactDistanceMeters: Float = 0.025f,
    val minimumPadRadiusMeters: Float = 0.04f,
    val maximumPadRadiusMeters: Float = 0.25f,
    val minimumAxisCalibrationDistanceMeters: Float = 0.02f,
    val inputReleaseRadiusMultiplier: Float = 2f,
    val deadZoneFraction: Float = 0.12f,
    val smoothingFactor: Float = 0.35f,
    /** Normal distance from the selected surface required to confirm a boundary gesture. */
    val calibrationLiftDistanceMeters: Float = 0.05f,
)
