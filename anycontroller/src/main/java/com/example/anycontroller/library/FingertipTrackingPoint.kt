package com.example.anycontroller.library

import com.example.anycontroller.library.model.Point3
import com.pico.spatial.core.math.Vector3

/**
 * Converts a fingertip joint pose into the point used by AnyController.
 *
 * Contact and marker calculations intentionally use the reported joint position without an inferred
 * forward offset.
 */
internal fun fingertipTrackingPoint(position: Vector3): Point3 =
    Point3(position.x, position.y, position.z)
