package com.example.anycontroller.library

import com.example.anycontroller.library.model.CalibrationState
import com.example.anycontroller.library.model.ControllerLayout
import com.example.anycontroller.library.model.ControllerSurface
import com.example.anycontroller.library.model.PadCoordinateFrame
import com.example.anycontroller.library.model.PadGeometry
import com.example.anycontroller.library.model.Point2
import com.example.anycontroller.library.model.Point3
import com.example.anycontroller.library.model.RotationValue
import com.example.anycontroller.library.model.SurfaceBoundaryEdge
import com.example.anycontroller.library.model.SurfaceVisualizationMode
import com.example.anycontroller.library.model.boundingBoxEdges
import com.example.anycontroller.library.model.visibleSurfacesBeforeSelection
import com.pico.spatial.core.container.SpatialViewContent
import com.pico.spatial.core.ecs.Entity
import com.pico.spatial.core.ecs.ModelComponent
import com.pico.spatial.core.ecs.ModelEntity
import com.pico.spatial.core.ecs.TransformComponent
import com.pico.spatial.core.ecs.resource.BlendingMode
import com.pico.spatial.core.ecs.resource.MaterialCullingMode
import com.pico.spatial.core.ecs.resource.MeshResource
import com.pico.spatial.core.ecs.resource.PolygonFillMode
import com.pico.spatial.core.ecs.resource.UnlitMaterial
import com.pico.spatial.core.math.Color4
import com.pico.spatial.core.math.Quat
import com.pico.spatial.core.math.Vector3
import java.util.UUID
import kotlin.math.PI
import kotlin.math.acos
import kotlin.math.sqrt

internal data class TrackedFingertips(
    val leftMiddle: Point3? = null,
    val leftIndex: Point3? = null,
    val rightMiddle: Point3? = null,
    val rightIndex: Point3? = null,
)

/**
 * Scene-owned debug and calibration visuals.
 *
 * Hand joints and plane anchors are reported in global Stage space. Every global pose is converted
 * through [root] before it is assigned to a child transform; assigning those values directly was
 * the source of the original hand-to-pad visual offset.
 */
@Suppress("LongParameterList")
internal class AnyControllerOverlay
private constructor(
    private val root: Entity,
    private val boxMesh: MeshResource,
    private val padMesh: MeshResource,
    private val leftMaterial: UnlitMaterial,
    private val rightMaterial: UnlitMaterial,
    private val detectedSurfaceMaterial: UnlitMaterial,
    private val detectedSurfaceWireframeMaterial: UnlitMaterial,
    private val selectedSurfaceBoundaryMaterial: UnlitMaterial,
    private val xAxisMaterial: UnlitMaterial,
    private val yAxisMaterial: UnlitMaterial,
    private val zAxisMaterial: UnlitMaterial,
    private val contactMaterial: UnlitMaterial,
    private val leftPadEntity: ModelEntity,
    private val rightPadEntity: ModelEntity,
) {
    private val surfacePlanes = mutableMapOf<String, SurfacePlane>()
    private var renderedSurfaceRevision = Long.MIN_VALUE
    private var renderedSelectedSurfaceId: String? = null
    private var renderedSurfaceVisualizationMode: SurfaceVisualizationMode? = null
    private lateinit var leftMiddleMarker: Marker
    private lateinit var leftIndexMarker: Marker
    private lateinit var rightMiddleMarker: Marker
    private lateinit var rightIndexMarker: Marker
    private lateinit var leftContactMarker: ModelEntity
    private lateinit var rightContactMarker: ModelEntity
    private lateinit var leftPadFrame: AxisFrame
    private lateinit var rightPadFrame: AxisFrame
    private lateinit var upLabel: Marker

    @Suppress("LongParameterList")
    fun render(
        layout: ControllerLayout,
        detectedSurfaces: Collection<ControllerSurface>,
        surfaceRevision: Long,
        fingertips: TrackedFingertips,
        surfaceVisualizationMode: SurfaceVisualizationMode,
        calibrationState: CalibrationState,
        contactDistanceMeters: Float,
    ) {
        val selectedSurface = layout.surface
        val selectedSurfaceId = selectedSurface?.anchorId
        val selectionChanged = selectedSurfaceId != renderedSelectedSurfaceId
        val detectedSurfacesChanged =
            selectedSurface == null && surfaceRevision != renderedSurfaceRevision
        val detectedVisualizationChanged =
            selectedSurface == null && surfaceVisualizationMode != renderedSurfaceVisualizationMode
        if (selectionChanged || detectedSurfacesChanged || detectedVisualizationChanged) {
            renderSurfaces(
                detectedSurfaces = detectedSurfaces,
                selectedSurface = selectedSurface,
                visualizationMode = surfaceVisualizationMode,
                geometryMayHaveChanged =
                    selectionChanged || detectedSurfacesChanged || detectedVisualizationChanged,
            )
        }
        renderedSurfaceRevision = surfaceRevision
        renderedSelectedSurfaceId = selectedSurfaceId
        renderedSurfaceVisualizationMode = surfaceVisualizationMode
        renderPad(leftPadEntity, layout, layout.leftPad)
        renderPad(rightPadEntity, layout, layout.rightPad)

        renderMarker(leftMiddleMarker, fingertips.leftMiddle, layout, detectedSurfaces)
        renderMarker(leftIndexMarker, fingertips.leftIndex, layout, detectedSurfaces)
        renderMarker(rightMiddleMarker, fingertips.rightMiddle, layout, detectedSurfaces)
        renderMarker(rightIndexMarker, fingertips.rightIndex, layout, detectedSurfaces)
        renderContactMarker(
            leftContactMarker,
            fingertips.leftIndex,
            layout,
            detectedSurfaces,
            contactDistanceMeters,
        )
        renderContactMarker(
            rightContactMarker,
            fingertips.rightIndex,
            layout,
            detectedSurfaces,
            contactDistanceMeters,
        )
        renderUpLabel(upLabel, calibrationState, fingertips.leftIndex, layout)
        renderPadFrame(leftPadFrame, layout, layout.leftPad)
        renderPadFrame(rightPadFrame, layout, layout.rightPad)
    }

    fun close() {
        surfacePlanes.values.forEach(SurfacePlane::destroy)
        surfacePlanes.clear()
        leftMiddleMarker.destroy()
        leftIndexMarker.destroy()
        rightMiddleMarker.destroy()
        rightIndexMarker.destroy()
        leftContactMarker.destroy()
        rightContactMarker.destroy()
        leftPadFrame.destroy()
        rightPadFrame.destroy()
        upLabel.destroy()
        leftPadEntity.destroy()
        rightPadEntity.destroy()
        root.destroy()

        leftMaterial.close()
        rightMaterial.close()
        detectedSurfaceMaterial.close()
        detectedSurfaceWireframeMaterial.close()
        selectedSurfaceBoundaryMaterial.close()
        xAxisMaterial.close()
        yAxisMaterial.close()
        zAxisMaterial.close()
        contactMaterial.close()
        padMesh.close()
        boxMesh.close()
    }

    private fun renderSurfaces(
        detectedSurfaces: Collection<ControllerSurface>,
        selectedSurface: ControllerSurface?,
        visualizationMode: SurfaceVisualizationMode,
        geometryMayHaveChanged: Boolean,
    ) {
        // Plane discovery is complete after selection. The calibrated pads and their axes remain
        // visible, but all detection wireframes disappear so they do not clutter flight control.
        val visibleSurfaces =
            visibleSurfacesBeforeSelection(
                detectedSurfaces = detectedSurfaces,
                selectedSurface = selectedSurface,
            )
        val currentIds = visibleSurfaces.mapNotNullTo(mutableSetOf()) { it.anchorId }
        surfacePlanes.keys.filterNot(currentIds::contains).forEach { removedId ->
            surfacePlanes.remove(removedId)?.destroy()
        }

        visibleSurfaces.forEach { surface ->
            val id = surface.anchorId ?: return@forEach
            val selected = selectedSurface != null && id == selectedSurface.anchorId
            val existingPlane = surfacePlanes[id]
            val plane = existingPlane ?: createSurfacePlane(id).also { surfacePlanes[id] = it }
            plane.update(
                surface = surface,
                isSelected = selected,
                visualizationMode = visualizationMode,
                geometryMayHaveChanged = geometryMayHaveChanged || existingPlane == null,
            )
        }
    }

    private fun createSurfacePlane(id: String): SurfacePlane {
        val safeId = id.replace('-', '_')
        val entity = Entity().apply { setName("AnyController_Surface_$safeId") }
        root.addChild(entity)
        val anchorFrame =
            createAxisFrame(
                    parent = entity,
                    name = "AnyController_Surface_${safeId}_AnchorFrame",
                    includeZ = true,
                )
                .apply {
                    root.components[TransformComponent::class.java]?.setPosition(
                        Vector3(0f, 0f, AXIS_SURFACE_OFFSET_METERS)
                    )
                    update(
                        xDirection = Vector3.RIGHT,
                        yDirection = Vector3.UP,
                        zDirection = Vector3.FORWARD,
                        lengthMeters = SURFACE_AXIS_LENGTH_METERS,
                        thicknessMeters = AXIS_LINE_THICKNESS_METERS,
                    )
                    root.enabled = false
                }
        return SurfacePlane(id, safeId, entity, anchorFrame = anchorFrame)
    }

    @Suppress("CyclomaticComplexMethod")
    private fun SurfacePlane.update(
        surface: ControllerSurface,
        isSelected: Boolean,
        visualizationMode: SurfaceVisualizationMode,
        geometryMayHaveChanged: Boolean,
    ) {
        val desiredBoundaryEdges =
            if (isSelected) surface.boundingBoxEdges() else surface.boundaryEdges
        val boundaryChanged =
            selected != isSelected ||
                (geometryMayHaveChanged && boundaryEdges != desiredBoundaryEdges)
        if (boundaryChanged) {
            boundaryEntities.forEach(ModelEntity::destroy)
            boundaryEntities =
                createBoundaryEntities(
                    parent = rootEntity,
                    safeId = safeId,
                    boundaryEdges = desiredBoundaryEdges,
                    material =
                        if (isSelected) selectedSurfaceBoundaryMaterial else detectedSurfaceMaterial,
                )
            boundaryEdges = desiredBoundaryEdges
        }

        val surfaceRotation = surface.rotation.toQuat()
        rootEntity.components[TransformComponent::class.java]?.apply {
            setPosition(root.convertPositionFrom(surface.origin.toVector3(), null))
            setQuaternion(root.convertRotationFrom(surfaceRotation, null))
            setScaleVector(Vector3(1f))
        }

        val planeMeshNeeded =
            !isSelected && visualizationMode == SurfaceVisualizationMode.MESH_WIREFRAME
        if (planeMeshNeeded && (planeMesh == null || geometryMayHaveChanged)) {
            val replacementMesh = MeshResource.loadFromPlaneAnchor(UUID.fromString(anchorId))
            if (wireframeEntity == null) {
                wireframeEntity =
                    ModelEntity(replacementMesh, detectedSurfaceWireframeMaterial).apply {
                        setName("AnyController_Surface_${safeId}_Wireframe")
                        components[TransformComponent::class.java]?.setPosition(
                            Vector3(0f, 0f, SURFACE_OFFSET_METERS)
                        )
                        rootEntity.addChild(this)
                    }
            } else {
                wireframeEntity?.components?.get(ModelComponent::class.java)?.mesh = replacementMesh
            }

            planeMesh?.close()
            planeMesh = replacementMesh
        }

        boundaryEntities.forEach {
            it.enabled = isSelected || visualizationMode == SurfaceVisualizationMode.BOUNDARY_EDGES
        }
        wireframeEntity?.enabled =
            !isSelected && visualizationMode == SurfaceVisualizationMode.MESH_WIREFRAME
        anchorFrame.root.enabled = isSelected
        selected = isSelected
        rootEntity.enabled = true
    }

    private fun createBoundaryEntities(
        parent: Entity,
        safeId: String,
        boundaryEdges: List<SurfaceBoundaryEdge>,
        material: UnlitMaterial,
    ): MutableList<ModelEntity> =
        boundaryEdges.mapIndexedNotNullTo(mutableListOf()) { index, edge ->
            val delta = edge.end - edge.start
            val direction = delta.toVector3()
            val length = direction.length()
            if (length <= MINIMUM_EDGE_LENGTH_METERS) return@mapIndexedNotNullTo null
            val midpoint = (edge.start + edge.end) * 0.5f
            ModelEntity(boxMesh, material).apply {
                setName("AnyController_Surface_${safeId}_Boundary_$index")
                components[TransformComponent::class.java]?.apply {
                    setPosition(Vector3(midpoint.x, midpoint.y, midpoint.z + SURFACE_OFFSET_METERS))
                    setQuaternion(rotationFromPositiveX(direction))
                    setScaleVector(
                        Vector3(length, SURFACE_LINE_WIDTH_METERS, SURFACE_LINE_DEPTH_METERS)
                    )
                }
                parent.addChild(this)
            }
        }

    private fun rotationFromPositiveX(direction: Vector3): Quat {
        val normalized = direction.normalize()
        val dot = Vector3.dot(Vector3.RIGHT, normalized).coerceIn(-1f, 1f)
        return when {
            dot >= 1f - DIRECTION_EPSILON -> Quat()
            dot <= -1f + DIRECTION_EPSILON -> Quat(Vector3.UP, PI.toFloat())
            else -> Quat(Vector3.cross(Vector3.RIGHT, normalized).normalize(), acos(dot))
        }
    }

    private fun renderPad(entity: ModelEntity, layout: ControllerLayout, pad: PadGeometry?) {
        val surface = layout.surface
        if (surface == null || pad == null || pad.radiusMeters <= 0f) {
            entity.enabled = false
            return
        }
        val globalPosition = surface.localToWorld(pad.center, SURFACE_OFFSET_METERS)
        val globalSurfaceRotation = surface.rotation.toQuat()
        val localSurfaceRotation = root.convertRotationFrom(globalSurfaceRotation, null)
        val rotation = localSurfaceRotation * Quat(Vector3.RIGHT, (PI / 2.0).toFloat())
        entity.components[TransformComponent::class.java]?.apply {
            setPosition(root.convertPositionFrom(globalPosition.toVector3(), null))
            setQuaternion(rotation)
            setScaleVector(Vector3(pad.radiusMeters, 1f, pad.radiusMeters))
        }
        entity.enabled = true
    }

    private fun renderPadFrame(
        frameVisual: AxisFrame,
        layout: ControllerLayout,
        pad: PadGeometry?,
    ) {
        val surface = layout.surface
        val frame = pad?.frame
        if (surface == null || pad == null || frame == null || pad.radiusMeters <= 0f) {
            frameVisual.root.enabled = false
            return
        }
        val globalPosition = surface.localToWorld(pad.center, PAD_FRAME_OFFSET_METERS)
        frameVisual.root.components[TransformComponent::class.java]?.apply {
            setPosition(root.convertPositionFrom(globalPosition.toVector3(), null))
            setQuaternion(root.convertRotationFrom(surface.rotation.toQuat(), null))
            setScaleVector(Vector3(1f))
        }
        frameVisual.update(
            frame = frame,
            lengthMeters = pad.radiusMeters * PAD_FRAME_LENGTH_FRACTION,
            thicknessMeters = PAD_AXIS_LINE_THICKNESS_METERS,
        )
        frameVisual.root.enabled = true
    }

    private fun renderContactMarker(
        entity: ModelEntity,
        fingertip: Point3?,
        layout: ControllerLayout,
        detectedSurfaces: Collection<ControllerSurface>,
        contactDistanceMeters: Float,
    ) {
        if (fingertip == null) {
            entity.enabled = false
            return
        }
        val surface = layout.surface ?: detectedSurfaces.nearestContainingSurface(fingertip)
        val local = surface?.worldToLocal(fingertip)
        if (
            surface == null ||
                local == null ||
                kotlin.math.abs(local.z) > contactDistanceMeters ||
                !surface.contains(local)
        ) {
            entity.enabled = false
            return
        }
        val projectedPoint =
            surface.localToWorld(Point2(local.x, local.y), CONTACT_MARKER_OFFSET_METERS)
        val localSurfaceRotation = root.convertRotationFrom(surface.rotation.toQuat(), null)
        entity.components[TransformComponent::class.java]?.apply {
            setPosition(root.convertPositionFrom(projectedPoint.toVector3(), null))
            setQuaternion(localSurfaceRotation * Quat(Vector3.RIGHT, (PI / 2.0).toFloat()))
            setScaleVector(Vector3(CONTACT_MARKER_RADIUS_METERS, 1f, CONTACT_MARKER_RADIUS_METERS))
        }
        entity.enabled = true
    }

    private fun renderUpLabel(
        label: Marker,
        calibrationState: CalibrationState,
        leftIndexTip: Point3?,
        layout: ControllerLayout,
    ) {
        val surface = layout.surface
        if (
            calibrationState != CalibrationState.CALIBRATE_LEFT_UP ||
                leftIndexTip == null ||
                surface == null
        ) {
            label.root.enabled = false
            return
        }
        val labelPosition =
            leftIndexTip +
                surface.up * UP_LABEL_OFFSET_METERS +
                surface.normal * UP_LABEL_NORMAL_OFFSET_METERS
        label.root.components[TransformComponent::class.java]?.apply {
            setPosition(root.convertPositionFrom(labelPosition.toVector3(), null))
            setQuaternion(root.convertRotationFrom(surface.rotation.toQuat(), null))
            setScaleVector(Vector3(1f))
        }
        label.root.enabled = true
    }

    private fun renderMarker(
        marker: Marker,
        fingertip: Point3?,
        layout: ControllerLayout,
        detectedSurfaces: Collection<ControllerSurface>,
    ) {
        if (fingertip == null) {
            marker.root.enabled = false
            return
        }
        val referenceSurface =
            layout.surface ?: detectedSurfaces.nearestContainingSurface(fingertip)
        val globalRotation = referenceSurface?.rotation?.toQuat() ?: Quat()
        marker.root.components[TransformComponent::class.java]?.apply {
            setPosition(root.convertPositionFrom(fingertip.toVector3(), null))
            setQuaternion(root.convertRotationFrom(globalRotation, null))
            setScaleVector(Vector3(1f))
        }
        marker.root.enabled = true
    }

    private fun Collection<ControllerSurface>.nearestContainingSurface(
        point: Point3
    ): ControllerSurface? =
        asSequence()
            .map { surface -> surface to surface.worldToLocal(point) }
            .filter { (surface, local) -> surface.contains(local) }
            .minByOrNull { (_, local) -> kotlin.math.abs(local.z) }
            ?.first

    private fun createXMarker(
        overlayRoot: Entity,
        boxMesh: MeshResource,
        name: String,
        material: UnlitMaterial,
    ): Marker {
        val markerRoot = Entity().apply { setName(name) }
        val strokes =
            listOf(-PI.toFloat() / 4f, PI.toFloat() / 4f).mapIndexed { index, angle ->
                createStroke(
                    boxMesh = boxMesh,
                    parent = markerRoot,
                    name = "${name}_Stroke_$index",
                    material = material,
                    center = Vector3(0f, 0f, 0f),
                    length = FINGER_MARKER_SIZE_METERS,
                    angleRadians = angle,
                )
            }
        markerRoot.enabled = false
        overlayRoot.addChild(markerRoot)
        return Marker(markerRoot, strokes)
    }

    private fun createTriangleMarker(
        overlayRoot: Entity,
        boxMesh: MeshResource,
        name: String,
        material: UnlitMaterial,
    ): Marker {
        val markerRoot = Entity().apply { setName(name) }
        val side = FINGER_MARKER_SIZE_METERS
        val height = side * sqrt(3f) / 2f
        val strokes =
            listOf(
                    StrokeSpec(Vector3(0f, -height / 3f, 0f), 0f),
                    StrokeSpec(Vector3(-side / 4f, height / 6f, 0f), PI.toFloat() / 3f),
                    StrokeSpec(Vector3(side / 4f, height / 6f, 0f), -PI.toFloat() / 3f),
                )
                .mapIndexed { index, spec ->
                    createStroke(
                        boxMesh = boxMesh,
                        parent = markerRoot,
                        name = "${name}_Stroke_$index",
                        material = material,
                        center = spec.center,
                        length = side,
                        angleRadians = spec.angleRadians,
                    )
                }
        markerRoot.enabled = false
        overlayRoot.addChild(markerRoot)
        return Marker(markerRoot, strokes)
    }

    /** Creates a small, geometry-backed "UP" label so the module needs no host attachment. */
    private fun createUpLabel(
        overlayRoot: Entity,
        boxMesh: MeshResource,
        material: UnlitMaterial,
    ): Marker {
        val name = "AnyController_LeftUpLabel"
        val markerRoot = Entity().apply { setName(name) }
        val halfWidth = UP_LABEL_LETTER_WIDTH_METERS / 2f
        val halfHeight = UP_LABEL_LETTER_HEIGHT_METERS / 2f
        val pCenterX = UP_LABEL_LETTER_WIDTH_METERS + UP_LABEL_LETTER_GAP_METERS
        val specs =
            listOf(
                // U
                StrokeSpec(
                    Vector3(-halfWidth, 0f, 0f),
                    PI.toFloat() / 2f,
                    UP_LABEL_LETTER_HEIGHT_METERS,
                ),
                StrokeSpec(
                    Vector3(halfWidth, 0f, 0f),
                    PI.toFloat() / 2f,
                    UP_LABEL_LETTER_HEIGHT_METERS,
                ),
                StrokeSpec(Vector3(0f, -halfHeight, 0f), 0f, UP_LABEL_LETTER_WIDTH_METERS),
                // P
                StrokeSpec(
                    Vector3(pCenterX - halfWidth, 0f, 0f),
                    PI.toFloat() / 2f,
                    UP_LABEL_LETTER_HEIGHT_METERS,
                ),
                StrokeSpec(Vector3(pCenterX, halfHeight, 0f), 0f, UP_LABEL_LETTER_WIDTH_METERS),
                StrokeSpec(Vector3(pCenterX, 0f, 0f), 0f, UP_LABEL_LETTER_WIDTH_METERS),
                StrokeSpec(
                    Vector3(pCenterX + halfWidth, halfHeight / 2f, 0f),
                    PI.toFloat() / 2f,
                    halfHeight,
                ),
            )
        val strokes =
            specs.mapIndexed { index, spec ->
                createStroke(
                    boxMesh = boxMesh,
                    parent = markerRoot,
                    name = "${name}_Stroke_$index",
                    material = material,
                    center = spec.center,
                    length = spec.lengthMeters,
                    angleRadians = spec.angleRadians,
                    thicknessMeters = UP_LABEL_STROKE_METERS,
                    depthMeters = UP_LABEL_DEPTH_METERS,
                )
            }
        markerRoot.enabled = false
        overlayRoot.addChild(markerRoot)
        return Marker(markerRoot, strokes)
    }

    @Suppress("LongParameterList")
    private fun createStroke(
        boxMesh: MeshResource,
        parent: Entity,
        name: String,
        material: UnlitMaterial,
        center: Vector3,
        length: Float,
        angleRadians: Float,
        thicknessMeters: Float = FINGER_MARKER_STROKE_METERS,
        depthMeters: Float = FINGER_MARKER_DEPTH_METERS,
    ): ModelEntity =
        ModelEntity(boxMesh, material).apply {
            setName(name)
            components[TransformComponent::class.java]?.apply {
                setPosition(center)
                setQuaternion(Quat(Vector3.FORWARD, angleRadians))
                setScaleVector(Vector3(length, thicknessMeters, depthMeters))
            }
            parent.addChild(this)
        }

    private fun Point3.toVector3() = Vector3(x, y, z)

    private fun RotationValue.toQuat() = Quat(x, y, z, w)

    private fun createAxisFrame(parent: Entity, name: String, includeZ: Boolean): AxisFrame {
        val frameRoot = Entity().apply { setName(name) }
        val xAxis = createAxisEntity(frameRoot, "${name}_X", xAxisMaterial)
        val yAxis = createAxisEntity(frameRoot, "${name}_Y", yAxisMaterial)
        val zAxis = if (includeZ) createAxisEntity(frameRoot, "${name}_Z", zAxisMaterial) else null
        parent.addChild(frameRoot)
        return AxisFrame(frameRoot, xAxis, yAxis, zAxis)
    }

    private fun createAxisEntity(
        parent: Entity,
        name: String,
        material: UnlitMaterial,
    ): ModelEntity =
        ModelEntity(boxMesh, material).apply {
            setName(name)
            parent.addChild(this)
        }

    private fun ModelEntity.updateAxis(
        direction: Vector3,
        lengthMeters: Float,
        thicknessMeters: Float,
    ) {
        val axisDelta = direction.normalize() * lengthMeters
        components[TransformComponent::class.java]?.apply {
            setPosition(axisDelta * 0.5f)
            setQuaternion(rotationFromPositiveX(axisDelta))
            setScaleVector(Vector3(lengthMeters, thicknessMeters, thicknessMeters))
        }
    }

    private data class StrokeSpec(
        val center: Vector3,
        val angleRadians: Float,
        val lengthMeters: Float = FINGER_MARKER_SIZE_METERS,
    )

    private data class Marker(val root: Entity, val strokes: List<ModelEntity>) {
        fun destroy() {
            strokes.forEach(ModelEntity::destroy)
            root.destroy()
        }
    }

    private inner class AxisFrame(
        val root: Entity,
        private val xAxis: ModelEntity,
        private val yAxis: ModelEntity,
        private val zAxis: ModelEntity?,
    ) {
        fun update(frame: PadCoordinateFrame, lengthMeters: Float, thicknessMeters: Float) {
            update(
                xDirection = Vector3(frame.xAxis.x, frame.xAxis.y, 0f),
                yDirection = Vector3(frame.yAxis.x, frame.yAxis.y, 0f),
                zDirection = null,
                lengthMeters = lengthMeters,
                thicknessMeters = thicknessMeters,
            )
        }

        fun update(
            xDirection: Vector3,
            yDirection: Vector3,
            zDirection: Vector3?,
            lengthMeters: Float,
            thicknessMeters: Float,
        ) {
            xAxis.updateAxis(xDirection, lengthMeters, thicknessMeters)
            yAxis.updateAxis(yDirection, lengthMeters, thicknessMeters)
            zAxis?.apply {
                if (zDirection == null) {
                    enabled = false
                } else {
                    updateAxis(zDirection, lengthMeters, thicknessMeters)
                    enabled = true
                }
            }
        }

        fun destroy() {
            xAxis.destroy()
            yAxis.destroy()
            zAxis?.destroy()
            root.destroy()
        }
    }

    private inner class SurfacePlane(
        val anchorId: String,
        val safeId: String,
        val rootEntity: Entity,
        val anchorFrame: AxisFrame,
        var boundaryEntities: MutableList<ModelEntity> = mutableListOf(),
        var boundaryEdges: List<SurfaceBoundaryEdge> = emptyList(),
        var wireframeEntity: ModelEntity? = null,
        var planeMesh: MeshResource? = null,
        var selected: Boolean = false,
    ) {
        fun destroy() {
            boundaryEntities.forEach(ModelEntity::destroy)
            wireframeEntity?.destroy()
            anchorFrame.destroy()
            rootEntity.destroy()
            planeMesh?.close()
        }
    }

    companion object {
        private const val SURFACE_OFFSET_METERS = 0.004f
        private const val SURFACE_LINE_WIDTH_METERS = 0.004f
        private const val SURFACE_LINE_DEPTH_METERS = 0.004f
        private const val MINIMUM_EDGE_LENGTH_METERS = 0.001f
        private const val DIRECTION_EPSILON = 0.0001f
        private const val FINGER_MARKER_SIZE_METERS = 0.035f
        private const val FINGER_MARKER_STROKE_METERS = 0.006f
        private const val FINGER_MARKER_DEPTH_METERS = 0.008f
        private const val CONTACT_MARKER_RADIUS_METERS = 0.001f
        private const val CONTACT_MARKER_OFFSET_METERS = 0.007f
        private const val AXIS_SURFACE_OFFSET_METERS = 0.008f
        private const val SURFACE_AXIS_LENGTH_METERS = 0.12f
        private const val AXIS_LINE_THICKNESS_METERS = 0.004f
        private const val PAD_FRAME_OFFSET_METERS = 0.009f
        private const val PAD_FRAME_LENGTH_FRACTION = 0.8f
        private const val PAD_AXIS_LINE_THICKNESS_METERS = 0.003f
        private const val UP_LABEL_OFFSET_METERS = 0.05f
        private const val UP_LABEL_NORMAL_OFFSET_METERS = 0.01f
        private const val UP_LABEL_LETTER_WIDTH_METERS = 0.02f
        private const val UP_LABEL_LETTER_HEIGHT_METERS = 0.03f
        private const val UP_LABEL_LETTER_GAP_METERS = 0.012f
        private const val UP_LABEL_STROKE_METERS = 0.003f
        private const val UP_LABEL_DEPTH_METERS = 0.003f

        @Suppress("LongMethod", "CyclomaticComplexMethod")
        suspend fun create(content: SpatialViewContent): AnyControllerOverlay {
            val resources =
                OverlayResources(
                    // These resources are intentionally shared by multiple ModelComponents and
                    // reused after detected-surface entities are destroyed. Local resources are
                    // released when their last component owner is destroyed, so persist them for
                    // the lifetime of the overlay and release them explicitly in close().
                    boxMesh = MeshResource.createBox(Vector3(1f)).apply { toGlobal() },
                    padMesh =
                        MeshResource.createCylinder(height = 0.004f, radius = 1f).apply {
                            toGlobal()
                        },
                    leftMaterial =
                        UnlitMaterial.create(BlendingMode.TRANSPARENT).apply {
                            toGlobal()
                            setBaseColor(Color4(1f, 0.45f, 0.08f, 0.72f))
                        },
                    rightMaterial =
                        UnlitMaterial.create(BlendingMode.TRANSPARENT).apply {
                            toGlobal()
                            setBaseColor(Color4(0.18f, 0.64f, 1f, 0.72f))
                        },
                    detectedSurfaceMaterial =
                        UnlitMaterial.create().apply {
                            toGlobal()
                            setBaseColor(Color4.WHITE)
                        },
                    detectedSurfaceWireframeMaterial =
                        UnlitMaterial.create().apply {
                            toGlobal()
                            setBaseColor(Color4.WHITE)
                            setPolygonFillMode(PolygonFillMode.LINE)
                            setCullingMode(MaterialCullingMode.NONE)
                        },
                    selectedSurfaceBoundaryMaterial =
                        UnlitMaterial.create().apply {
                            toGlobal()
                            setBaseColor(Color4(0f, 1f, 0f, 1f))
                            setCullingMode(MaterialCullingMode.NONE)
                        },
                    xAxisMaterial =
                        UnlitMaterial.create().apply {
                            toGlobal()
                            setBaseColor(Color4(0f, 1f, 0f, 1f))
                        },
                    yAxisMaterial =
                        UnlitMaterial.create().apply {
                            toGlobal()
                            setBaseColor(Color4(1f, 0f, 0f, 1f))
                        },
                    zAxisMaterial =
                        UnlitMaterial.create().apply {
                            toGlobal()
                            setBaseColor(Color4(0f, 0.35f, 1f, 1f))
                        },
                    contactMaterial =
                        UnlitMaterial.create().apply {
                            toGlobal()
                            setBaseColor(Color4(1f, 0.08f, 0.58f, 1f))
                        },
                )
            val root = Entity().apply { setName("AnyController_OverlayRoot") }
            val leftPad =
                ModelEntity(resources.padMesh, resources.leftMaterial).apply {
                    setName("AnyController_LeftPad")
                    enabled = false
                    root.addChild(this)
                }
            val rightPad =
                ModelEntity(resources.padMesh, resources.rightMaterial).apply {
                    setName("AnyController_RightPad")
                    enabled = false
                    root.addChild(this)
                }
            content.addEntity(root)

            return AnyControllerOverlay(
                    root = root,
                    boxMesh = resources.boxMesh,
                    padMesh = resources.padMesh,
                    leftMaterial = resources.leftMaterial,
                    rightMaterial = resources.rightMaterial,
                    detectedSurfaceMaterial = resources.detectedSurfaceMaterial,
                    detectedSurfaceWireframeMaterial = resources.detectedSurfaceWireframeMaterial,
                    selectedSurfaceBoundaryMaterial = resources.selectedSurfaceBoundaryMaterial,
                    xAxisMaterial = resources.xAxisMaterial,
                    yAxisMaterial = resources.yAxisMaterial,
                    zAxisMaterial = resources.zAxisMaterial,
                    contactMaterial = resources.contactMaterial,
                    leftPadEntity = leftPad,
                    rightPadEntity = rightPad,
                )
                .apply {
                    leftMiddleMarker =
                        createXMarker(root, boxMesh, "AnyController_LeftMiddleTip", leftMaterial)
                    leftIndexMarker =
                        createTriangleMarker(
                            root,
                            boxMesh,
                            "AnyController_LeftIndexTip",
                            leftMaterial,
                        )
                    rightMiddleMarker =
                        createXMarker(root, boxMesh, "AnyController_RightMiddleTip", rightMaterial)
                    rightIndexMarker =
                        createTriangleMarker(
                            root,
                            boxMesh,
                            "AnyController_RightIndexTip",
                            rightMaterial,
                        )
                    leftContactMarker =
                        ModelEntity(padMesh, contactMaterial).apply {
                            setName("AnyController_LeftIndexContact")
                            enabled = false
                            root.addChild(this)
                        }
                    rightContactMarker =
                        ModelEntity(padMesh, contactMaterial).apply {
                            setName("AnyController_RightIndexContact")
                            enabled = false
                            root.addChild(this)
                        }
                    leftPadFrame =
                        createAxisFrame(root, "AnyController_LeftPadFrame", includeZ = false)
                            .apply { this.root.enabled = false }
                    rightPadFrame =
                        createAxisFrame(root, "AnyController_RightPadFrame", includeZ = false)
                            .apply { this.root.enabled = false }
                    upLabel = createUpLabel(root, boxMesh, contactMaterial)
                }
        }
    }

    private data class OverlayResources(
        val boxMesh: MeshResource,
        val padMesh: MeshResource,
        val leftMaterial: UnlitMaterial,
        val rightMaterial: UnlitMaterial,
        val detectedSurfaceMaterial: UnlitMaterial,
        val detectedSurfaceWireframeMaterial: UnlitMaterial,
        val selectedSurfaceBoundaryMaterial: UnlitMaterial,
        val xAxisMaterial: UnlitMaterial,
        val yAxisMaterial: UnlitMaterial,
        val zAxisMaterial: UnlitMaterial,
        val contactMaterial: UnlitMaterial,
    )
}
