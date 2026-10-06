package com.pico.spatial.choppertoy.content

import android.util.Log
import androidx.compose.foundation.layout.size
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.MutableState
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.ui.Modifier
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import com.pico.spatial.core.annotation.RequiredFullSpace
import com.pico.spatial.core.ecs.Entity
import com.pico.spatial.core.ecs.LookAtComponent
import com.pico.spatial.core.ecs.LookAtForwardDirection
import com.pico.spatial.core.ecs.PhysicsWorldComponent
import com.pico.spatial.core.ecs.TransformComponent
import com.pico.spatial.core.math.Quat
import com.pico.spatial.core.math.Vector3
import com.pico.spatial.sense.base.AnchorUpdate
import com.pico.spatial.sense.plane.PlaneTrackingManager
import com.pico.spatial.tracking.hand.HandJoint
import com.pico.spatial.tracking.hand.HandPose
import com.pico.spatial.tracking.hand.HandTrackingData
import com.pico.spatial.tracking.hand.HandTrackingProvider
import com.pico.spatial.tracking.hmd.HMDPose
import com.pico.spatial.tracking.hmd.HMDTrackingData
import com.pico.spatial.tracking.hmd.HMDTrackingProvider
import com.pico.spatial.ui.foundation.content.SpatialView
import com.pico.spatial.ui.foundation.content.attachmentPanelComponent
import com.pico.spatial.ui.foundation.content.panelSize
import com.pico.spatial.ui.foundation.gesture.TargetEntity
import com.pico.spatial.ui.foundation.gesture.detectSpatialDragGesture
import com.pico.spatial.ui.geometry.Offset3D
import com.pico.spatial.ui.platform.LengthUnit
import com.pico.spatial.ui.platform.LocalPhysicalLengthConverter
import com.pico.spatial.ui.platform.PhysicalLengthConverter
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.launch

private const val TAG = "PalmInstruments"
private const val PALM_OFFSET_METERS = 0.10f
private const val FINGER_FORWARD_OFFSET_METERS = 0.20f
private const val LAUNCH_PANEL_WIDTH_METERS = 0.32f
private const val LAUNCH_PANEL_HEIGHT_METERS = 0.15f
private const val TUTORIAL_PANEL_WIDTH_METERS = 0.52f
private const val TUTORIAL_PANEL_HEIGHT_METERS = 0.31f
private const val HEADING_PANEL_WIDTH_METERS = 0.105f
private const val HEADING_PANEL_HEIGHT_METERS = 0.105f
private const val GAUGE_SIDE_OFFSET_METERS = 0.105f
private const val LAUNCH_PANEL_SIDE_OFFSET_METERS = 0.36f
private const val LAUNCH_PANEL_VERTICAL_OFFSET_METERS = 0.14f
private const val ATTITUDE_FILTER_TIME_CONSTANT_SECONDS = 0.12f
private const val ATTITUDE_MAX_ANGULAR_SPEED_DEGREES_PER_SECOND = 180f
private val PALM_LOCAL_OFFSET = Vector3.UP * PALM_OFFSET_METERS

/** Hosts the first-install tutorial, palm instruments, and palm-controlled helicopter scene. */
@RequiredFullSpace
@Composable
fun HomeStage() {
    val handTrackingProvider = remember { HandTrackingProvider() }
    val hmdTrackingProvider = remember { HMDTrackingProvider() }
    val handTrackingData by
        handTrackingProvider.dataFlow.collectAsState(initial = HandTrackingData(null, null, 0L))
    val hmdTrackingData by
        hmdTrackingProvider.dataFlow.collectAsState(
            initial = HMDTrackingData(HMDPose(Vector3.ZERO, Quat.identity()), 0L)
        )
    val coroutineScope = rememberCoroutineScope()
    val context = LocalContext.current
    val physicalLengthConverter = LocalPhysicalLengthConverter.current
    val tutorialPreferences = remember(context) { FlightTutorialPreferences(context) }
    val stageUi = rememberHomeStageUi()
    val instrumentScene = rememberPalmInstrumentScene(stageUi)
    val stageScene = rememberHomeStageScene(stageUi, instrumentScene, tutorialPreferences)

    TrackSpatialInputs(
        handTrackingProvider = handTrackingProvider,
        hmdTrackingProvider = hmdTrackingProvider,
        stageScene = stageScene,
        coroutineScope = coroutineScope,
    )

    SpatialView(
        modifier =
            Modifier.size(1.dp).pointerInput(
                stageScene.helicopterController,
                physicalLengthConverter,
            ) {
                detectSpatialDragGesture(
                    context = context,
                    targetedToEntity =
                        TargetEntity.hit(stageScene.helicopterController.dragTargetEntity),
                    onDragEnd = { stageScene.helicopterController.finishDrag() },
                    onDragCancel = { stageScene.helicopterController.finishDrag() },
                ) { dragValue ->
                    stageScene.helicopterController.dragBy(
                        dragValue.dragAmount.toSceneDeltaMeters(physicalLengthConverter)
                    )
                }
            },
        update = { _, _ ->
            if (hmdTrackingData.hmdPose.position != Vector3.ZERO) {
                stageScene.helicopterController.updateHmdPose(hmdTrackingData.hmdPose)
            }
            if (stageScene.tutorialCoordinator.isActive) {
                stageScene.tutorialCoordinator.update(handTrackingData)
            } else {
                stageScene.instrumentUpdater.update(handTrackingData)
            }
        },
    ) { content, _ ->
        content.addEntity(instrumentScene.sceneRoot)
        stageScene.helicopterController.initialize(content)
    }
}

@Composable
@Suppress("LongMethod")
private fun rememberHomeStageUi(): HomeStageUi {
    val instrumentReadout = remember { mutableStateOf(InstrumentReadout.ZERO) }
    val flightUiState = remember { mutableStateOf(FlightUiState()) }
    val tutorialUiState = remember { mutableStateOf(FlightTutorialUiState()) }
    val helicopterControllerHolder = remember { HelicopterControllerHolder() }
    val tutorialCoordinatorHolder = remember { TutorialCoordinatorHolder() }

    val launchPanelComponent =
        attachmentPanelComponent(
            size = panelSize(LAUNCH_PANEL_WIDTH_METERS, LAUNCH_PANEL_HEIGHT_METERS)
        ) {
            HelicopterLaunchPanel(
                endReason = flightUiState.value.endReason,
                controlReady = flightUiState.value.controlReady,
                onStart = { helicopterControllerHolder.value?.startFlight() },
            )
        }
    val tutorialPanelComponent =
        attachmentPanelComponent(
            size = panelSize(TUTORIAL_PANEL_WIDTH_METERS, TUTORIAL_PANEL_HEIGHT_METERS)
        ) {
            FlightTutorialPanel(
                state = tutorialUiState.value,
                onNext = { tutorialCoordinatorHolder.value?.advance() },
            )
        }
    val headingPanelComponent =
        attachmentPanelComponent(
            size = panelSize(HEADING_PANEL_WIDTH_METERS, HEADING_PANEL_HEIGHT_METERS)
        ) {
            HeadingPanel(headingDegrees = instrumentReadout.value.headingDegrees)
        }
    val launchPanelEntity =
        remember(launchPanelComponent) {
            Entity().apply {
                setName("HelicopterLaunchAttachmentPanel")
                components.set(launchPanelComponent)
                val lookAtComponent =
                    LookAtComponent().apply {
                        alignLocalUpToWorldUp = true
                        lookAtForwardDirection = LookAtForwardDirection.POSITIVE_Z
                    }
                components.set(lookAtComponent)
                lookAtComponent.setViewerAsTarget()
                components[TransformComponent::class.java]?.setPosition(
                    Vector3(
                        LAUNCH_PANEL_SIDE_OFFSET_METERS,
                        LAUNCH_PANEL_VERTICAL_OFFSET_METERS,
                        0f,
                    )
                )
                enabled = false
            }
        }
    val tutorialPanelEntity =
        remember(tutorialPanelComponent) {
            Entity().apply {
                setName("HelicopterTutorialAttachmentPanel")
                components.set(tutorialPanelComponent)
                val lookAtComponent =
                    LookAtComponent().apply {
                        alignLocalUpToWorldUp = true
                        lookAtForwardDirection = LookAtForwardDirection.POSITIVE_Z
                    }
                components.set(lookAtComponent)
                lookAtComponent.setViewerAsTarget()
                components[TransformComponent::class.java]?.setPosition(
                    Vector3(
                        LAUNCH_PANEL_SIDE_OFFSET_METERS,
                        LAUNCH_PANEL_VERTICAL_OFFSET_METERS,
                        0f,
                    )
                )
                enabled = false
            }
        }
    val headingPanelEntity =
        remember(headingPanelComponent) {
            Entity().apply {
                setName("HeadingAttachmentPanel")
                components.set(headingPanelComponent)
                components[TransformComponent::class.java]?.setPosition(
                    Vector3(GAUGE_SIDE_OFFSET_METERS, 0f, 0f)
                )
            }
        }

    return remember(
        instrumentReadout,
        flightUiState,
        tutorialUiState,
        helicopterControllerHolder,
        tutorialCoordinatorHolder,
        launchPanelEntity,
        tutorialPanelEntity,
        headingPanelEntity,
    ) {
        HomeStageUi(
            instrumentReadout = instrumentReadout,
            flightUiState = flightUiState,
            tutorialUiState = tutorialUiState,
            helicopterControllerHolder = helicopterControllerHolder,
            tutorialCoordinatorHolder = tutorialCoordinatorHolder,
            launchPanelEntity = launchPanelEntity,
            tutorialPanelEntity = tutorialPanelEntity,
            headingPanelEntity = headingPanelEntity,
        )
    }
}

@Composable
private fun rememberPalmInstrumentScene(stageUi: HomeStageUi): PalmInstrumentScene {
    val attitudeBallScene = remember { createAttitudeBallScene() }
    val gaugeEntity =
        remember(stageUi.headingPanelEntity, attitudeBallScene) {
            Entity().apply {
                setName("ViewerFacingInstrumentCluster")
                val lookAtComponent =
                    LookAtComponent().apply {
                        alignLocalUpToWorldUp = true
                        lookAtForwardDirection = LookAtForwardDirection.POSITIVE_Z
                    }
                components.set(lookAtComponent)
                lookAtComponent.setViewerAsTarget()
                addChild(attitudeBallScene.root)
                addChild(stageUi.headingPanelEntity)
                enabled = false
            }
        }
    val sceneRoot =
        remember(gaugeEntity) {
            Entity().apply {
                setName("PalmInstrumentRoot")
                components.set(
                    PhysicsWorldComponent().apply {
                        gravity = Vector3(0f, -GRAVITY_METERS_PER_SECOND_SQUARED, 0f)
                    }
                )
                addChild(gaugeEntity)
            }
        }

    return remember(sceneRoot, gaugeEntity, attitudeBallScene) {
        PalmInstrumentScene(
            sceneRoot = sceneRoot,
            gaugeEntity = gaugeEntity,
            attitudeBallEntity = attitudeBallScene.rotatingBall,
        )
    }
}

@Composable
private fun rememberHomeStageScene(
    stageUi: HomeStageUi,
    instrumentScene: PalmInstrumentScene,
    tutorialPreferences: FlightTutorialPreferences,
): HomeStageScene {
    val palmFlightCalibration = remember { PalmFlightCalibration() }
    val helicopterController =
        remember(
            instrumentScene.sceneRoot,
            stageUi.launchPanelEntity,
            stageUi.tutorialPanelEntity,
        ) {
            HelicopterSceneController(
                    sceneRoot = instrumentScene.sceneRoot,
                    launchPanelEntity = stageUi.launchPanelEntity,
                    tutorialPanelEntity = stageUi.tutorialPanelEntity,
                    palmFlightCalibration = palmFlightCalibration,
                    onUiStateChanged = { state -> stageUi.flightUiState.value = state },
                )
                .also { stageUi.helicopterControllerHolder.value = it }
        }
    val dynamics = remember {
        PalmInstrumentDynamics(
            handSelector = RightPreferredHandSelector(),
            spring = SpringFollower3D(stiffness = 58f, dampingRatio = 0.72f),
            attitudeFilter =
                PalmAttitudeLowPassFilter(
                    timeConstantSeconds = ATTITUDE_FILTER_TIME_CONSTANT_SECONDS,
                    maxAngularSpeedDegreesPerSecond = ATTITUDE_MAX_ANGULAR_SPEED_DEGREES_PER_SECOND,
                ),
            frameClock = FrameClock(),
        )
    }
    val instrumentUpdater =
        remember(instrumentScene, dynamics, palmFlightCalibration, helicopterController) {
            PalmInstrumentUpdater(
                scene = instrumentScene,
                dynamics = dynamics,
                palmFlightCalibration = palmFlightCalibration,
                onReadoutChanged = { stageUi.instrumentReadout.value = it },
                onPalmControlChanged = helicopterController::updatePalmControl,
            )
        }
    val tutorialCoordinator =
        remember(tutorialPreferences, helicopterController, instrumentUpdater) {
            FirstInstallFlightTutorialCoordinator(
                    preferences = tutorialPreferences,
                    helicopterController = helicopterController,
                    instrumentUpdater = instrumentUpdater,
                    onUiStateChanged = { state -> stageUi.tutorialUiState.value = state },
                )
                .also { stageUi.tutorialCoordinatorHolder.value = it }
        }

    return remember(helicopterController, instrumentUpdater, tutorialCoordinator) {
        HomeStageScene(
            sceneRoot = instrumentScene.sceneRoot,
            helicopterController = helicopterController,
            instrumentUpdater = instrumentUpdater,
            tutorialCoordinator = tutorialCoordinator,
        )
    }
}

@Composable
private fun TrackSpatialInputs(
    handTrackingProvider: HandTrackingProvider,
    hmdTrackingProvider: HMDTrackingProvider,
    stageScene: HomeStageScene,
    coroutineScope: CoroutineScope,
) {
    val helicopterController = stageScene.helicopterController
    DisposableEffect(handTrackingProvider, hmdTrackingProvider, stageScene, coroutineScope) {
        val planeSubscription =
            PlaneTrackingManager.subscribeAnchorUpdate { update ->
                when (update.event) {
                    AnchorUpdate.Event.ADDED,
                    AnchorUpdate.Event.UPDATED,
                    AnchorUpdate.Event.LOADED ->
                        helicopterController.addOrUpdatePlane(update.anchor)
                    AnchorUpdate.Event.REMOVED ->
                        helicopterController.removePlane(update.anchor.anchorUUID)
                    AnchorUpdate.Event.UNKNOWN -> Unit
                }
            }
        var hmdTrackingStarted = false
        var handTrackingStarted = false
        var planeTrackingStarted = false

        runCatching {
                hmdTrackingProvider.start()
                hmdTrackingStarted = true
                Log.i(TAG, "HMD tracking started")
            }
            .onFailure { Log.e(TAG, "Unable to start HMD tracking", it) }
        runCatching {
                handTrackingProvider.start()
                handTrackingStarted = true
                Log.i(TAG, "Palm-control hand tracking started")
            }
            .onFailure { Log.e(TAG, "Unable to start hand tracking", it) }
        runCatching {
                PlaneTrackingManager.start()
                planeTrackingStarted = true
                Log.i(TAG, "Physical-plane tracking started")
            }
            .onFailure { Log.e(TAG, "Unable to start plane tracking", it) }
        val planeLoadJob: Job =
            coroutineScope.launch {
                runCatching { PlaneTrackingManager.loadAllAnchors() }
                    .onSuccess { anchors ->
                        anchors.forEach(helicopterController::addOrUpdatePlane)
                        Log.i(TAG, "Loaded ${anchors.size} existing plane anchors")
                    }
                    .onFailure { Log.e(TAG, "Unable to load existing plane anchors", it) }
            }

        onDispose {
            planeLoadJob.cancel()
            planeSubscription.cancel()
            if (planeTrackingStarted) PlaneTrackingManager.stop()
            if (handTrackingStarted) handTrackingProvider.stop()
            if (hmdTrackingStarted) hmdTrackingProvider.stop()
            helicopterController.destroy()
            stageScene.sceneRoot.destroy()
            Log.i(TAG, "Spatial tracking stopped and helicopter scene destroyed")
        }
    }
}

private class FirstInstallFlightTutorialCoordinator(
    private val preferences: FlightTutorialPreferences,
    private val helicopterController: HelicopterSceneController,
    private val instrumentUpdater: PalmInstrumentUpdater,
    private val onUiStateChanged: (FlightTutorialUiState) -> Unit,
) {
    private val session = FlightTutorialSession()
    private var tutorialPrepared = false
    private var started = false
    private var latestRightPalmControl: PalmFlightControl? = null

    var isActive: Boolean = !preferences.isCompleted()
        private set

    init {
        Log.i(
            TAG,
            if (isActive) {
                "First-install flight tutorial required"
            } else {
                "First-install flight tutorial already completed"
            },
        )
    }

    fun update(handTrackingData: HandTrackingData) {
        if (!isActive) return
        if (!helicopterController.isReady) {
            publishState(waitingForRightHand = true, preparingHelicopter = true)
            return
        }
        if (!tutorialPrepared) {
            if (!helicopterController.beginTutorial()) return
            tutorialPrepared = true
        }
        val palmControl = instrumentUpdater.update(handTrackingData, rightHandOnly = true)
        latestRightPalmControl = palmControl
        if (palmControl == null) {
            publishState(waitingForRightHand = true, preparingHelicopter = false)
            return
        }
        if (!started) {
            session.begin(palmControl)
            started = true
        }
        val frame = session.update(palmControl) ?: return
        helicopterController.updateTutorial(frame)
        instrumentUpdater.showTutorialReadout(frame.instrumentReadout)
        publishState(waitingForRightHand = false, preparingHelicopter = false)
    }

    fun advance() {
        val palmControl = latestRightPalmControl ?: return
        if (!isActive || !started) return
        when (session.advance(palmControl)) {
            FlightTutorialAdvanceResult.BLOCKED -> Unit
            FlightTutorialAdvanceResult.ADVANCED -> {
                helicopterController.resetTutorialStep()
                instrumentUpdater.showTutorialReadout(InstrumentReadout.ZERO)
                publishState(waitingForRightHand = false, preparingHelicopter = false)
            }
            FlightTutorialAdvanceResult.COMPLETED -> {
                preferences.markCompleted()
                helicopterController.finishTutorial()
                isActive = false
                Log.i(TAG, "All seven first-install flight lessons completed")
            }
        }
    }

    private fun publishState(waitingForRightHand: Boolean, preparingHelicopter: Boolean) {
        onUiStateChanged(
            FlightTutorialUiState(
                step = session.step,
                stepNumber = session.step.ordinal + 1,
                canAdvance = session.canAdvance,
                waitingForRightHand = waitingForRightHand,
                preparingHelicopter = preparingHelicopter,
            )
        )
    }
}

private class PalmInstrumentUpdater(
    private val scene: PalmInstrumentScene,
    private val dynamics: PalmInstrumentDynamics,
    private val palmFlightCalibration: PalmFlightCalibration,
    private val onReadoutChanged: (InstrumentReadout) -> Unit,
    private val onPalmControlChanged: (PalmFlightControl?) -> Unit,
) {
    fun update(
        handTrackingData: HandTrackingData,
        rightHandOnly: Boolean = false,
    ): PalmFlightControl? {
        val leftPalm = handTrackingData.left.validPalm()
        val rightPalm = handTrackingData.right.validPalm()
        val previousHand = dynamics.handSelector.selected
        val selectedHand =
            dynamics.handSelector.select(
                leftTracked = !rightHandOnly && leftPalm != null,
                rightTracked = rightPalm != null,
            )
        val handPose =
            when (selectedHand) {
                TrackedHand.LEFT -> handTrackingData.left
                TrackedHand.RIGHT -> handTrackingData.right
                null -> null
            }
        val palm =
            when (selectedHand) {
                TrackedHand.LEFT -> leftPalm
                TrackedHand.RIGHT -> rightPalm
                null -> null
            }
        val palmFrame =
            if (handPose != null && selectedHand != null) handPose.palmFrame(selectedHand) else null
        if (palm == null || palmFrame == null) {
            handleLostPalm(previousHand)
            return null
        }

        val targetGlobalPosition =
            palm.position +
                palm.rotation.rotateVector(PALM_LOCAL_OFFSET) +
                palmFrame.horizontalForward * FINGER_FORWARD_OFFSET_METERS
        val targetLocalPosition = scene.sceneRoot.convertPositionFrom(targetGlobalPosition, null)
        if (selectedHand != previousHand) {
            dynamics.spring.snapTo(targetLocalPosition)
            dynamics.attitudeFilter.reset()
            dynamics.frameClock.reset()
            Log.i(TAG, "Following $selectedHand palm")
        }

        val deltaSeconds = dynamics.frameClock.nextDeltaSeconds()
        val instrumentPosition = dynamics.spring.step(targetLocalPosition, deltaSeconds)
        val filteredReadout = dynamics.attitudeFilter.step(palmFrame.readout, deltaSeconds)
        val rawPalmControl =
            PalmFlightControl(
                palmHeightMeters = palm.position.y,
                pitchDegrees = filteredReadout.pitchDegrees,
                rollDegrees = filteredReadout.rollDegrees,
                headingDegrees = filteredReadout.headingDegrees,
            )
        val controlledReadout =
            palmFlightCalibration.resolve(rawPalmControl)?.instrumentReadout
                ?: filteredReadout.toControlledAttitude()
        scene.gaugeEntity.components[TransformComponent::class.java]?.setPosition(
            instrumentPosition
        )
        showReadout(controlledReadout)
        onPalmControlChanged(rawPalmControl)
        scene.gaugeEntity.enabled = true
        return rawPalmControl
    }

    fun showTutorialReadout(readout: InstrumentReadout) {
        showReadout(readout)
    }

    private fun showReadout(readout: InstrumentReadout) {
        scene.attitudeBallEntity.components[TransformComponent::class.java]?.setEulerAngles(
            attitudeBallRotation(readout)
        )
        onReadoutChanged(readout)
    }

    private fun handleLostPalm(previousHand: TrackedHand?) {
        if (previousHand != null) Log.i(TAG, "Lost $previousHand palm")
        scene.gaugeEntity.enabled = false
        dynamics.spring.clear()
        dynamics.attitudeFilter.reset()
        dynamics.frameClock.reset()
        onPalmControlChanged(null)
    }
}

private data class HomeStageUi(
    val instrumentReadout: MutableState<InstrumentReadout>,
    val flightUiState: MutableState<FlightUiState>,
    val tutorialUiState: MutableState<FlightTutorialUiState>,
    val helicopterControllerHolder: HelicopterControllerHolder,
    val tutorialCoordinatorHolder: TutorialCoordinatorHolder,
    val launchPanelEntity: Entity,
    val tutorialPanelEntity: Entity,
    val headingPanelEntity: Entity,
)

private data class PalmInstrumentScene(
    val sceneRoot: Entity,
    val gaugeEntity: Entity,
    val attitudeBallEntity: Entity,
)

private data class PalmInstrumentDynamics(
    val handSelector: RightPreferredHandSelector,
    val spring: SpringFollower3D,
    val attitudeFilter: PalmAttitudeLowPassFilter,
    val frameClock: FrameClock,
)

private data class HomeStageScene(
    val sceneRoot: Entity,
    val helicopterController: HelicopterSceneController,
    val instrumentUpdater: PalmInstrumentUpdater,
    val tutorialCoordinator: FirstInstallFlightTutorialCoordinator,
)

private class HelicopterControllerHolder {
    var value: HelicopterSceneController? = null
}

private class TutorialCoordinatorHolder {
    var value: FirstInstallFlightTutorialCoordinator? = null
}

private fun HandPose?.validPalm(): HandJoint? {
    val palm = this?.handJoints?.firstOrNull { it.index == HandJoint.Index.PALM } ?: return null
    val position = palm.position
    val rotation = palm.rotation
    val poseIsFinite =
        position.x.isFinite() &&
            position.y.isFinite() &&
            position.z.isFinite() &&
            rotation.x.isFinite() &&
            rotation.y.isFinite() &&
            rotation.z.isFinite() &&
            rotation.w.isFinite()
    return palm.takeIf { poseIsFinite && position != Vector3.ZERO }
}

private fun Offset3D.toSceneDeltaMeters(converter: PhysicalLengthConverter): Vector3 {
    fun physicalLength(value: Float): Float =
        converter.dpToLength(with(converter) { value.toDp() }, LengthUnit.Meters)

    return Vector3(physicalLength(x), -physicalLength(y), physicalLength(z))
}

private class FrameClock {
    private var lastFrameNanos = 0L

    fun nextDeltaSeconds(nowNanos: Long = System.nanoTime()): Float {
        val previous = lastFrameNanos
        lastFrameNanos = nowNanos
        return if (previous == 0L) 0f else (nowNanos - previous) / 1_000_000_000f
    }

    fun reset() {
        lastFrameNanos = 0L
    }
}
