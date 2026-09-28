package com.example.anycontroller.library

import com.example.anycontroller.library.domain.AnyControllerEngine
import com.example.anycontroller.library.model.AnyControllerConfig
import com.example.anycontroller.library.model.AnyControllerFrame
import com.example.anycontroller.library.model.AnyControllerStatus
import com.example.anycontroller.library.model.CalibrationState
import com.example.anycontroller.library.model.ControllerLayout
import com.example.anycontroller.library.model.ControllerSurface
import com.example.anycontroller.library.model.Point2
import com.example.anycontroller.library.model.Point3
import com.example.anycontroller.library.model.RotationValue
import com.example.anycontroller.library.model.SurfaceDetectionStatus
import com.example.anycontroller.library.model.SurfaceVisualizationMode
import com.example.anycontroller.library.model.findBoundaryEdges
import com.pico.spatial.core.container.SpatialViewContent
import com.pico.spatial.core.ecs.SceneUpdateContext
import com.pico.spatial.core.ecs.System as EcsSystem
import com.pico.spatial.core.math.Vector3
import com.pico.spatial.sense.base.AnchorUpdate
import com.pico.spatial.sense.plane.PlaneAnchor
import com.pico.spatial.sense.plane.PlaneTrackingManager
import com.pico.spatial.tracking.DataProvider
import com.pico.spatial.tracking.hand.HandJoint
import com.pico.spatial.tracking.hand.HandTrackingProvider
import java.util.UUID
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.atomic.AtomicLong
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * Process-local facade for the surface-mounted dual-pad controller.
 *
 * The host must run in Full Space. Call [attachOverlay] after its SpatialView is initialized,
 * [start] from a main-thread coroutine, and [stop] before the host scene is destroyed.
 */
object AnyController {
    private val config = AnyControllerConfig()
    private val engine = AnyControllerEngine(config)
    private val handTracking = HandTrackingProvider()
    private val planeSurfaces = ConcurrentHashMap<UUID, ControllerSurface>()
    private val planeRevision = AtomicLong()

    private val _frames = MutableStateFlow(AnyControllerFrame.Neutral)
    /** Frame-aligned, normalized dual-pad input snapshots. */
    val frames: StateFlow<AnyControllerFrame> = _frames.asStateFlow()

    private val _status = MutableStateFlow(AnyControllerStatus.STOPPED)
    /** High-level controller lifecycle and calibration status. */
    val status: StateFlow<AnyControllerStatus> = _status.asStateFlow()

    private val _layout = MutableStateFlow(ControllerLayout())
    /** Selected surface and current calibration geometry. */
    val layout: StateFlow<ControllerLayout> = _layout.asStateFlow()

    private val _calibrationState = MutableStateFlow(CalibrationState.STOPPED)
    /** Current step of the guided two-pad calibration flow. */
    val calibrationState: StateFlow<CalibrationState> = _calibrationState.asStateFlow()

    private val _surfaceVisualizationMode =
        MutableStateFlow(SurfaceVisualizationMode.BOUNDARY_EDGES)
    /** Current visualization strategy for planes that are available for selection. */
    val surfaceVisualizationMode: StateFlow<SurfaceVisualizationMode> =
        _surfaceVisualizationMode.asStateFlow()

    private val _surfaceDetectionStatus = MutableStateFlow(SurfaceDetectionStatus.STOPPED)
    /** Lifecycle state of standalone or controller-owned plane detection. */
    val surfaceDetectionStatus: StateFlow<SurfaceDetectionStatus> =
        _surfaceDetectionStatus.asStateFlow()

    private val _detectedSurfaceCount = MutableStateFlow(0)
    /** Number of plane anchors currently available to the controller. */
    val detectedSurfaceCount: StateFlow<Int> = _detectedSurfaceCount.asStateFlow()

    /** Most recently published frame for render-loop consumers. */
    val latestFrame: AnyControllerFrame
        get() = _frames.value

    private var controllerRunning = false
    private var standaloneSurfaceDetectionRequested = false
    private var surfaceDetectionActive = false
    private var systemRegistered = false
    private var planeSubscription: com.pico.spatial.core.lifecycle.Cancellable? = null
    private var overlay: AnyControllerOverlay? = null
    private var latestFingertips = TrackedFingertips()

    /** Creates calibration and controller visuals inside [content]. */
    suspend fun attachOverlay(content: SpatialViewContent) {
        overlay?.close()
        overlay = AnyControllerOverlay.create(content)
        renderOverlay()
    }

    /** Removes all controller visuals and releases their scene resources. */
    fun detachOverlay() {
        overlay?.close()
        overlay = null
    }

    /** Starts plane tracking, hand tracking, calibration, and frame publication. */
    suspend fun start() {
        if (controllerRunning) return
        controllerRunning = true
        _status.value = AnyControllerStatus.STARTING
        try {
            engine.start()
            publishEngineState()
            ensureSurfaceDetectionStarted()
            val handStart = handTracking.start()
            ensureSystemRegistered()
            _status.value =
                if (handStart == DataProvider.StartResult.PENDING) AnyControllerStatus.UNAVAILABLE
                else statusFor(engine.calibrationState)
        } catch (failure: Throwable) {
            controllerRunning = false
            handTracking.stop()
            if (!standaloneSurfaceDetectionRequested && surfaceDetectionActive) {
                stopSurfaceDetectionResources()
            }
            unregisterSystemIfIdle()
            _frames.value = engine.stop()
            _layout.value = engine.layout
            _calibrationState.value = engine.calibrationState
            _status.value = AnyControllerStatus.ERROR
            throw failure
        }
    }

    /** Stops controller input and publishes a neutral frame. */
    fun stop() {
        if (!controllerRunning) return
        controllerRunning = false
        handTracking.stop()
        _frames.value = engine.stop()
        _layout.value = engine.layout
        _calibrationState.value = engine.calibrationState
        latestFingertips = TrackedFingertips()
        if (!standaloneSurfaceDetectionRequested) {
            stopSurfaceDetectionResources()
        }
        unregisterSystemIfIdle()
        renderOverlay()
        _status.value = AnyControllerStatus.STOPPED
    }

    /** Starts only plane-anchor tracking and visualization; hand tracking stays stopped. */
    suspend fun startSurfaceDetection() {
        if (standaloneSurfaceDetectionRequested && surfaceDetectionActive) return
        standaloneSurfaceDetectionRequested = true
        try {
            ensureSurfaceDetectionStarted()
        } catch (failure: Throwable) {
            standaloneSurfaceDetectionRequested = false
            throw failure
        }
    }

    /** Releases the standalone plane-tracking request without interrupting a running controller. */
    fun stopSurfaceDetection() {
        standaloneSurfaceDetectionRequested = false
        if (!controllerRunning) {
            stopSurfaceDetectionResources()
        }
    }

    private suspend fun ensureSurfaceDetectionStarted() {
        if (surfaceDetectionActive) {
            _surfaceDetectionStatus.value = SurfaceDetectionStatus.RUNNING
            ensureSystemRegistered()
            return
        }

        _surfaceDetectionStatus.value = SurfaceDetectionStatus.STARTING
        planeSurfaces.clear()
        publishDetectedSurfaceCount()
        planeRevision.incrementAndGet()
        try {
            planeSubscription =
                PlaneTrackingManager.subscribeAnchorUpdate { update ->
                    when (update.event) {
                        AnchorUpdate.Event.REMOVED -> planeSurfaces.remove(update.anchor.anchorUUID)
                        else ->
                            planeSurfaces[update.anchor.anchorUUID] =
                                update.anchor.toControllerSurface()
                    }
                    publishDetectedSurfaceCount()
                    planeRevision.incrementAndGet()
                }
            PlaneTrackingManager.start()
            surfaceDetectionActive = true
            PlaneTrackingManager.loadAllAnchors().forEach {
                planeSurfaces[it.anchorUUID] = it.toControllerSurface()
            }
            publishDetectedSurfaceCount()
            planeRevision.incrementAndGet()
            ensureSystemRegistered()
            _surfaceDetectionStatus.value = SurfaceDetectionStatus.RUNNING
        } catch (failure: Throwable) {
            stopSurfaceDetectionResources()
            _surfaceDetectionStatus.value = SurfaceDetectionStatus.ERROR
            throw failure
        }
    }

    private fun stopSurfaceDetectionResources() {
        planeSubscription?.cancel()
        planeSubscription = null
        if (surfaceDetectionActive) {
            PlaneTrackingManager.stop()
        }
        surfaceDetectionActive = false
        planeSurfaces.clear()
        publishDetectedSurfaceCount()
        planeRevision.incrementAndGet()
        renderOverlay()
        _surfaceDetectionStatus.value = SurfaceDetectionStatus.STOPPED
        unregisterSystemIfIdle()
    }

    private fun ensureSystemRegistered() {
        if (systemRegistered) return
        EcsSystem.register(AnyControllerSystem::class.java)
        systemRegistered = true
    }

    private fun unregisterSystemIfIdle() {
        if (!systemRegistered || controllerRunning || surfaceDetectionActive) return
        EcsSystem.unregister(AnyControllerSystem::class.java)
        systemRegistered = false
    }

    private fun publishDetectedSurfaceCount() {
        _detectedSurfaceCount.value = planeSurfaces.size
    }

    /** Clears pad geometry while preserving the selected surface when one exists. */
    fun recalibrate() {
        engine.recalibrate()
        publishEngineState()
        renderOverlay()
    }

    /** Changes how candidate planes are drawn before the user selects one. */
    fun setSurfaceVisualizationMode(mode: SurfaceVisualizationMode) {
        if (_surfaceVisualizationMode.value == mode) return
        _surfaceVisualizationMode.value = mode
        renderOverlay()
    }

    /**
     * Allows a host-provided surface-selection UI to choose the exact detected plane.
     *
     * Selection stores a value snapshot. Later updates for the same anchor UUID remain in the
     * detection cache but do not move or reshape the controller's calibrated surface.
     */
    fun selectSurface(anchor: PlaneAnchor) {
        val surface = anchor.toControllerSurface()
        planeSurfaces[anchor.anchorUUID] = surface
        publishDetectedSurfaceCount()
        planeRevision.incrementAndGet()
        engine.selectSurface(surface)
        publishEngineState()
        renderOverlay()
    }

    internal fun update(timestampNanos: Long) {
        if (controllerRunning) {
            val data = handTracking.latestData
            latestFingertips =
                TrackedFingertips(
                    leftMiddle = data.left?.get(HandJoint.Index.MIDDLE_TIP)?.toTrackingPoint(),
                    leftIndex = data.left?.get(HandJoint.Index.INDEX_TIP)?.toTrackingPoint(),
                    rightMiddle = data.right?.get(HandJoint.Index.MIDDLE_TIP)?.toTrackingPoint(),
                    rightIndex = data.right?.get(HandJoint.Index.INDEX_TIP)?.toTrackingPoint(),
                )
            selectTouchedSurfaceIfNeeded(latestFingertips.leftIndex)
            _frames.value =
                engine.update(
                    leftIndexTip = latestFingertips.leftIndex,
                    rightIndexTip = latestFingertips.rightIndex,
                    timestampNanos = timestampNanos,
                )
            publishEngineState()
        }
        if (surfaceDetectionActive) {
            renderOverlay()
        }
    }

    /**
     * The default selection gesture is the first valid left-index-fingertip contact. Hosts with
     * their own plane picker can call [selectSurface] before that gesture occurs.
     */
    private fun selectTouchedSurfaceIfNeeded(leftIndexTip: Point3?) {
        if (engine.layout.surface != null || leftIndexTip == null) return
        val touchedSurface =
            planeSurfaces.values
                .asSequence()
                .map { surface -> surface to surface.worldToLocal(leftIndexTip) }
                .filter { (surface, local) ->
                    kotlin.math.abs(local.z) <= config.contactDistanceMeters &&
                        surface.contains(local)
                }
                .minByOrNull { (_, local) -> kotlin.math.abs(local.z) }
                ?.first
        // ControllerSurface is immutable, so this becomes the frozen calibration-space snapshot.
        touchedSurface?.let(engine::selectSurface)
    }

    private fun publishEngineState() {
        _layout.value = engine.layout
        _calibrationState.value = engine.calibrationState
        if (controllerRunning) {
            _status.value =
                if (handTracking.state == DataProvider.State.PENDING) {
                    AnyControllerStatus.UNAVAILABLE
                } else {
                    statusFor(engine.calibrationState)
                }
        }
    }

    private fun renderOverlay() {
        overlay?.render(
            layout = engine.layout,
            detectedSurfaces = planeSurfaces.values,
            surfaceRevision = planeRevision.get(),
            fingertips = latestFingertips,
            surfaceVisualizationMode = _surfaceVisualizationMode.value,
            calibrationState = engine.calibrationState,
            contactDistanceMeters = config.contactDistanceMeters,
        )
    }

    private fun statusFor(state: CalibrationState): AnyControllerStatus =
        when (state) {
            CalibrationState.STOPPED -> AnyControllerStatus.STOPPED
            CalibrationState.WAITING_FOR_SURFACE -> AnyControllerStatus.WAITING_FOR_SURFACE
            CalibrationState.READY -> AnyControllerStatus.READY
            else -> AnyControllerStatus.CALIBRATING
        }

    private fun PlaneAnchor.toControllerSurface(): ControllerSurface {
        val rotation = transform.quaternion
        val localVertices = vertices.map { it.toPoint3() }
        val minX = localVertices.minOfOrNull(Point3::x)
        val maxX = localVertices.maxOfOrNull(Point3::x)
        val minY = localVertices.minOfOrNull(Point3::y)
        val maxY = localVertices.maxOfOrNull(Point3::y)
        val localBoundsCenter =
            if (minX != null && maxX != null && minY != null && maxY != null) {
                Point2((minX + maxX) / 2f, (minY + maxY) / 2f)
            } else {
                Point2(0f, 0f)
            }
        return ControllerSurface(
            anchorId = anchorUUID.toString(),
            origin = transform.position.toPoint3(),
            right = rotation.rotateVector(Vector3.RIGHT).toPoint3(),
            up = rotation.rotateVector(Vector3.UP).toPoint3(),
            normal = rotation.rotateVector(Vector3.FORWARD).toPoint3(),
            rotation = RotationValue(rotation.x, rotation.y, rotation.z, rotation.w),
            halfWidthMeters = boundingBoxSize.x / 2f,
            halfHeightMeters = boundingBoxSize.y / 2f,
            localBoundsCenter = localBoundsCenter,
            boundaryEdges = findBoundaryEdges(localVertices, indices),
        )
    }

    private fun Vector3.toPoint3() = Point3(x, y, z)

    private fun HandJoint.toTrackingPoint(): Point3 = fingertipTrackingPoint(position)
}

/** Frame-aligned Spatial SDK adapter. The public controller contract remains data-only. */
class AnyControllerSystem : EcsSystem() {
    override fun update(context: SceneUpdateContext) {
        AnyController.update(java.lang.System.nanoTime())
    }
}
