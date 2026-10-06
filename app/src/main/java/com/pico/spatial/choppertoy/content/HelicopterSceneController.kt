package com.pico.spatial.choppertoy.content

import android.util.Log
import com.pico.spatial.core.container.SpatialViewContent
import com.pico.spatial.core.ecs.CollisionComponent
import com.pico.spatial.core.ecs.Entity
import com.pico.spatial.core.ecs.HoverEffectComponent
import com.pico.spatial.core.ecs.InteractableComponent
import com.pico.spatial.core.ecs.LoadType
import com.pico.spatial.core.ecs.ObjectAudioComponent
import com.pico.spatial.core.ecs.PhysicsVelocityComponent
import com.pico.spatial.core.ecs.RigidBodyComponent
import com.pico.spatial.core.ecs.TransformComponent
import com.pico.spatial.core.ecs.audio.AudioPlayerController
import com.pico.spatial.core.ecs.audio.AudioResourceConfig
import com.pico.spatial.core.ecs.audio.Directivity
import com.pico.spatial.core.ecs.audio.DistanceAttenuationMode
import com.pico.spatial.core.ecs.event.CollisionEvents
import com.pico.spatial.core.ecs.resource.AudioResource
import com.pico.spatial.core.ecs.resource.PhysicsMaterialResource
import com.pico.spatial.core.ecs.resource.ShapeResource
import com.pico.spatial.core.ecs.simulation.CollisionContact
import com.pico.spatial.core.ecs.simulation.CollisionDetectionMode
import com.pico.spatial.core.ecs.simulation.CollisionInfoDetailLevel
import com.pico.spatial.core.ecs.simulation.CollisionResponseMode
import com.pico.spatial.core.ecs.simulation.MassProperties
import com.pico.spatial.core.ecs.simulation.RigidBodyMode
import com.pico.spatial.core.lifecycle.Cancellable
import com.pico.spatial.core.math.Bool3
import com.pico.spatial.core.math.EulerAngles
import com.pico.spatial.core.math.Quat
import com.pico.spatial.core.math.Vector3
import com.pico.spatial.sense.plane.PlaneAnchor
import com.pico.spatial.tracking.hmd.HMDPose
import java.util.UUID
import kotlin.math.atan2
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

private const val TAG = "HelicopterScene"
private const val HELICOPTER_ASSET_URI = "asset://helicopter.glb"
private const val ROTOR_AUDIO_ASSET_URI = "asset://audio/helicopter_rotor_source.ogg"
private const val CRASH_AUDIO_ASSET_URI = "asset://audio/helicopter_crash_source.ogg"
private const val ROTOR_AUDIO_RESOURCE_NAME = "SpatialChopperToyRotorLoop"
private const val CRASH_AUDIO_RESOURCE_NAME = "SpatialChopperToyCrash"
private const val ROTOR_AUDIO_VOLUME = 0.72f
private const val CRASH_AUDIO_VOLUME = 0.90f
private const val AUDIO_REVERB_VOLUME = 0.12f
private const val AUDIO_SOUND_RADIUS_LEVEL = 0.18f
private const val HELICOPTER_MODEL_SCALE = 0.10794f
private const val PLANE_COLLIDER_THICKNESS_METERS = 0.02f
private const val INITIAL_DISTANCE_METERS = 1.0f
private const val INITIAL_VERTICAL_OFFSET_METERS = -0.25f
private const val TUTORIAL_LOOP_DISTANCE_METERS = 0.20f
private const val TUTORIAL_VELOCITY_SCALE = 0.45f

private val HELICOPTER_SOURCE_CENTER = Vector3(0.00580175f, 0.76734895f, -0.82753625f)

/**
 * Owns the draggable helicopter entity, attached launch/tutorial panels, palm flight controls,
 * tutorial motion, tracked-plane colliders, safe-landing classification, and positional audio.
 */
@Suppress("LargeClass")
internal class HelicopterSceneController(
    private val sceneRoot: Entity,
    private val launchPanelEntity: Entity,
    private val tutorialPanelEntity: Entity,
    private val palmFlightCalibration: PalmFlightCalibration,
    private val onUiStateChanged: (FlightUiState) -> Unit,
) {
    private data class PlaneRecord(
        val anchor: PlaneAnchor,
        val anchorEntity: Entity,
        val colliderEntity: Entity,
        val surfaceCenterInAnchor: Vector3,
    )

    private data class PlaneBounds(
        val minimumX: Float,
        val maximumX: Float,
        val minimumY: Float,
        val maximumY: Float,
    ) {
        val width: Float
            get() = maximumX - minimumX

        val height: Float
            get() = maximumY - minimumY

        val centerX: Float
            get() = (minimumX + maximumX) / 2f

        val centerY: Float
            get() = (minimumY + maximumY) / 2f
    }

    private val flightModel = HelicopterFlightModel()
    private val attitudeController = HelicopterAttitudeController()
    private val planeRecords = linkedMapOf<UUID, PlaneRecord>()
    private val planeByColliderId = mutableMapOf<Long, PlaneRecord>()
    // Mass-property generation consumes its input shape, so use a separate shape for the collider.
    private val helicopterMassProperties =
        MassProperties.generateByShapesAndMass(
            listOf(ShapeResource.createBox(HELICOPTER_COLLISION_SIZE_METERS)),
            HELICOPTER_MASS_KILOGRAMS,
        )
    private val helicopterShape = ShapeResource.createBox(HELICOPTER_COLLISION_SIZE_METERS)
    private val physicsMaterial =
        PhysicsMaterialResource(
            staticFriction = 0.65f,
            dynamicFriction = 0.52f,
            restitution = 0.02f,
        )
    private val helicopterCollision =
        CollisionComponent(
            collisionShape = listOf(helicopterShape),
            physicsMaterial = physicsMaterial,
            collisionResponseMode = CollisionResponseMode.COLLIDER_FULL,
            collisionInfoDetailLevel = CollisionInfoDetailLevel.DETAILED,
        )
    private val helicopterRigidBody =
        RigidBodyComponent(
                massProperties = helicopterMassProperties,
                rigidBodyMode = RigidBodyMode.KINEMATIC,
            )
            .apply {
                isAffectedByGravity = false
                isRotationLocked = Bool3(true)
                // Translation is driven by the testable drag-limited velocity solver.
                linearDamping = 0f
                angularDamping = 1f
                collisionDetectionMode = CollisionDetectionMode.CONTINUOUS_DYNAMIC
            }
    private val velocityComponent = PhysicsVelocityComponent(Vector3.ZERO, Vector3.ZERO)
    private val rotorAudioEmitter =
        createSpatialAudioEmitter("MainRotorAudioEmitter", ROTOR_AUDIO_VOLUME)
    private val crashAudioEmitter =
        createSpatialAudioEmitter("CrashAudioEmitter", CRASH_AUDIO_VOLUME)
    private val helicopterEntity =
        Entity().apply {
            setName("PalmControlledHelicopter")
            components.set(helicopterCollision)
            components.set(helicopterRigidBody)
            components.set(velocityComponent)
            components.set(InteractableComponent())
            components.set(HoverEffectComponent())
            addChild(launchPanelEntity)
            addChild(tutorialPanelEntity)
            addChild(rotorAudioEmitter)
            addChild(crashAudioEmitter)
            enabled = false
        }

    private var collisionEnterSubscription: Cancellable? = null
    private var collisionUpdateSubscription: Cancellable? = null
    private var latestHmdPose: HMDPose? = null
    private var latestPalmControl: PalmFlightControl? = null
    private var initialPlacementComplete = false
    private var modelLoaded = false
    private var rotorAudioResource: AudioResource? = null
    private var crashAudioResource: AudioResource? = null
    private var rotorAudioController: AudioPlayerController? = null
    private var crashAudioController: AudioPlayerController? = null
    private var lastControlUpdateNanos = 0L
    private var flightBaselineRotation = Quat()
    private var flightStartHeadingDegrees = 0f
    private var latestCommandedWorldVelocity = Vector3.ZERO
    private var tutorialActive = false
    private var tutorialOriginPosition = Vector3.ZERO
    private var tutorialBaselineRotation = Quat()
    private var tutorialHeadingDegrees = 0f

    init {
        sceneRoot.addChild(helicopterEntity)
    }

    /** Entity targeted by the Spatial drag recognizer while the helicopter is not running. */
    val dragTargetEntity: Entity
        get() = helicopterEntity

    /** Loads the visual/audio assets and registers scene-scoped collision subscriptions. */
    suspend fun initialize(content: SpatialViewContent) {
        if (!modelLoaded) {
            runCatching { Entity.loadSuspend(HELICOPTER_ASSET_URI) }
                .onSuccess { model ->
                    model.setName("HelicopterVisualModel")
                    model.components[TransformComponent::class.java]?.apply {
                        setScaleVector(Vector3(HELICOPTER_MODEL_SCALE))
                        setPosition(HELICOPTER_SOURCE_CENTER * -HELICOPTER_MODEL_SCALE)
                    }
                    helicopterEntity.addChild(model)
                    modelLoaded = true
                    Log.i(TAG, "Helicopter GLB loaded")
                    tryPlaceInFrontOfViewer()
                }
                .onFailure { Log.e(TAG, "Unable to load helicopter GLB", it) }
        }
        initializeSpatialAudio()
        if (collisionEnterSubscription == null) {
            collisionEnterSubscription =
                content.subscribe(CollisionEvents.Enter::class.java, helicopterEntity, null) { event
                    ->
                    handleCollision(event.entityA, event.entityB, event.position, event.contacts)
                }
            collisionUpdateSubscription =
                content.subscribe(CollisionEvents.Update::class.java, helicopterEntity, null) {
                    event ->
                    handleCollision(event.entityA, event.entityB, event.position, event.contacts)
                }
        }
    }

    /** Supplies the latest HMD pose and performs the one-time floating placement. */
    fun updateHmdPose(pose: HMDPose) {
        latestHmdPose = pose
        tryPlaceInFrontOfViewer()
    }

    /** Creates or refreshes the collider corresponding to a detected physical plane. */
    fun addOrUpdatePlane(anchor: PlaneAnchor) {
        removePlane(anchor.anchorUUID)
        val bounds = planeBounds(anchor)

        val anchorEntity =
            Entity().apply {
                setName("DetectedPlaneAnchor-${anchor.anchorUUID}")
                components[TransformComponent::class.java]?.apply {
                    setPosition(sceneRoot.convertPositionFrom(anchor.transform.position, null))
                    setQuaternion(sceneRoot.convertRotationFrom(anchor.transform.quaternion, null))
                }
            }
        val colliderEntity =
            Entity().apply {
                setName("DetectedPlaneCollider-${anchor.anchorUUID}")
                components[TransformComponent::class.java]?.setPosition(
                    Vector3(bounds.centerX, bounds.centerY, -PLANE_COLLIDER_THICKNESS_METERS / 2f)
                )
                components.set(
                    CollisionComponent(
                        collisionShape =
                            listOf(
                                ShapeResource.createBox(
                                    Vector3(
                                        bounds.width.coerceAtLeast(0.02f),
                                        bounds.height.coerceAtLeast(0.02f),
                                        PLANE_COLLIDER_THICKNESS_METERS,
                                    )
                                )
                            ),
                        physicsMaterial = physicsMaterial,
                        collisionResponseMode = CollisionResponseMode.COLLIDER_FULL,
                        collisionInfoDetailLevel = CollisionInfoDetailLevel.DETAILED,
                    )
                )
            }
        anchorEntity.addChild(colliderEntity)
        sceneRoot.addChild(anchorEntity)
        val record =
            PlaneRecord(
                anchor = anchor,
                anchorEntity = anchorEntity,
                colliderEntity = colliderEntity,
                surfaceCenterInAnchor = Vector3(bounds.centerX, bounds.centerY, 0f),
            )
        planeRecords[anchor.anchorUUID] = record
        planeByColliderId[colliderEntity.id] = record
        Log.d(
            TAG,
            "Plane collider ready: ${anchor.anchorUUID}, orientation=${anchor.planeOrientation}",
        )
    }

    /** Removes a physical-plane collider that is no longer tracked. */
    fun removePlane(anchorUuid: UUID) {
        val removed = planeRecords.remove(anchorUuid) ?: return
        planeByColliderId.remove(removed.colliderEntity.id)
        removed.anchorEntity.destroy()
    }

    /** Applies palm cyclic, collective, and heading input only while RUNNING. */
    fun updatePalmControl(control: PalmFlightControl?) {
        val readinessChanged = (latestPalmControl != null) != (control != null)
        latestPalmControl = control
        if (readinessChanged) {
            refreshLaunchPanelVisibility()
            publishUiState()
        }
        if (flightModel.mode != FlightMode.RUNNING) return

        val transform = helicopterEntity.components[TransformComponent::class.java] ?: return
        val calibratedControl = palmFlightCalibration.resolve(control)?.helicopterControl
        val attitude =
            attitudeController.step(
                control = calibratedControl,
                cyclicEnabled = true,
                deltaSeconds = nextControlDeltaSeconds(),
            )
        transform.setQuaternion(composeLocalAttitude(flightBaselineRotation, attitude))
        val liftNewtons = flightModel.liftForPalmHeight(control?.palmHeightMeters)
        latestCommandedWorldVelocity =
            solveHelicopterMotion(
                    attitude = attitude,
                    liftNewtons = liftNewtons,
                    flightStartHeadingDegrees = flightStartHeadingDegrees,
                )
                .worldVelocityMetersPerSecond
        velocityComponent.linearVelocity = latestCommandedWorldVelocity
    }

    /** Starts or restarts flight after zeroing palm control at the click instant. */
    fun startFlight() {
        if (flightModel.mode == FlightMode.RUNNING || tutorialActive) return
        if (!initialPlacementComplete || !modelLoaded) {
            Log.w(TAG, "Ignoring START until the helicopter is loaded and placed")
            return
        }
        if (!controlReady()) {
            Log.w(TAG, "Ignoring START until right-palm control is ready")
            return
        }
        val palmAtStart = requireNotNull(latestPalmControl)
        val transform = helicopterEntity.components[TransformComponent::class.java] ?: return
        flightStartHeadingDegrees = transform.eulerAngles.yaw
        flightBaselineRotation = EulerAngles(yaw = flightStartHeadingDegrees).toQuat()
        transform.setQuaternion(flightBaselineRotation)
        palmFlightCalibration.calibrate(palmAtStart)
        attitudeController.reset(HelicopterAttitudeCommand(0f, 0f, 0f))
        lastControlUpdateNanos = 0L
        velocityComponent.linearVelocity = Vector3.ZERO
        velocityComponent.angularVelocity = Vector3.ZERO
        setDragInteractionEnabled(false)
        launchPanelEntity.enabled = false
        tutorialPanelEntity.enabled = false
        // Gravity is already included in solveHelicopterMotion's local net-force calculation.
        helicopterRigidBody.isAffectedByGravity = false
        helicopterRigidBody.rigidBodyMode = RigidBodyMode.DYNAMIC
        val previousMode = flightModel.mode
        val neutralLift = flightModel.start(palmAtStart.palmHeightMeters)
        applyAudioTransition(previousMode = previousMode, causedByCrash = false)
        latestCommandedWorldVelocity =
            solveHelicopterMotion(
                    attitude = HelicopterAttitudeCommand(0f, 0f, 0f),
                    liftNewtons = neutralLift,
                    flightStartHeadingDegrees = flightStartHeadingDegrees,
                )
                .worldVelocityMetersPerSecond
        velocityComponent.linearVelocity = latestCommandedWorldVelocity
        publishUiState()
        Log.i(TAG, "Flight mode changed to ${flightModel.mode}; palm controls zeroed")
    }

    /** True after the helicopter model has been loaded and placed for either tutorial or flight. */
    val isReady: Boolean
        get() = initialPlacementComplete && modelLoaded

    /**
     * Freezes the helicopter at its current placement and prepares the first single-DoF lesson.
     *
     * @return true when tutorial mode started, or false while the model is still being prepared.
     */
    fun beginTutorial(): Boolean {
        if (!isReady || flightModel.mode == FlightMode.RUNNING) return false
        val transform = helicopterEntity.components[TransformComponent::class.java] ?: return false
        tutorialActive = true
        helicopterRigidBody.isAffectedByGravity = false
        helicopterRigidBody.rigidBodyMode = RigidBodyMode.KINEMATIC
        velocityComponent.linearVelocity = Vector3.ZERO
        velocityComponent.angularVelocity = Vector3.ZERO
        latestCommandedWorldVelocity = Vector3.ZERO
        setDragInteractionEnabled(false)
        launchPanelEntity.enabled = false
        tutorialPanelEntity.enabled = true
        tutorialOriginPosition = transform.position
        tutorialHeadingDegrees = transform.eulerAngles.yaw
        tutorialBaselineRotation = EulerAngles(yaw = tutorialHeadingDegrees).toQuat()
        resetTutorialStep()
        Log.i(TAG, "First-install flight tutorial started")
        return true
    }

    /** Levels the helicopter and resets its looping origin for a newly advanced tutorial lesson. */
    fun resetTutorialStep() {
        if (!tutorialActive) return
        val transform = helicopterEntity.components[TransformComponent::class.java] ?: return
        attitudeController.reset(HelicopterAttitudeCommand(0f, 0f, 0f))
        transform.setPosition(tutorialOriginPosition)
        transform.setQuaternion(tutorialBaselineRotation)
        velocityComponent.linearVelocity = Vector3.ZERO
        velocityComponent.angularVelocity = Vector3.ZERO
        latestCommandedWorldVelocity = Vector3.ZERO
        lastControlUpdateNanos = 0L
    }

    /**
     * Animates the helicopter with production attitude and lift math projected onto one lesson DoF.
     * Translation is integrated kinematically at reduced speed and snaps to the lesson origin after
     * 20 cm, so tutorial motion cannot become a real flight or collide with the environment.
     */
    fun updateTutorial(frame: FlightTutorialFrame) {
        if (!tutorialActive) return
        val transform = helicopterEntity.components[TransformComponent::class.java] ?: return
        val deltaSeconds = nextControlDeltaSeconds()
        val attitude =
            attitudeController.step(
                control =
                    PalmFlightControl(
                        palmHeightMeters = 0f,
                        pitchDegrees = frame.attitude.pitchDegrees,
                        rollDegrees = frame.attitude.rollDegrees,
                        headingDegrees = frame.attitude.headingDegrees,
                    ),
                cyclicEnabled = true,
                deltaSeconds = deltaSeconds,
            )
        transform.setQuaternion(composeLocalAttitude(tutorialBaselineRotation, attitude))

        val solvedVelocity =
            solveHelicopterMotion(
                    attitude = attitude,
                    liftNewtons = frame.liftNewtons,
                    flightStartHeadingDegrees = tutorialHeadingDegrees,
                )
                .worldVelocityMetersPerSecond
        val projectedVelocity =
            when (frame.step) {
                FlightTutorialStep.PITCH_FORWARD,
                FlightTutorialStep.PITCH_BACKWARD,
                FlightTutorialStep.ROLL_LEFT,
                FlightTutorialStep.ROLL_RIGHT -> Vector3(solvedVelocity.x, 0f, solvedVelocity.z)
                FlightTutorialStep.ASCEND,
                FlightTutorialStep.DESCEND -> Vector3(0f, solvedVelocity.y, 0f)
                FlightTutorialStep.YAW -> Vector3.ZERO
            } * TUTORIAL_VELOCITY_SCALE
        val candidatePosition = transform.position + projectedVelocity * deltaSeconds
        transform.setPosition(
            if (
                (candidatePosition - tutorialOriginPosition).length() >=
                    TUTORIAL_LOOP_DISTANCE_METERS
            ) {
                tutorialOriginPosition
            } else {
                candidatePosition
            }
        )
    }

    /** Restores the normal draggable, gravity-free preflight state after tutorial completion. */
    fun finishTutorial() {
        if (!tutorialActive) return
        val transform = helicopterEntity.components[TransformComponent::class.java]
        transform?.setPosition(tutorialOriginPosition)
        transform?.setQuaternion(tutorialBaselineRotation)
        attitudeController.reset(HelicopterAttitudeCommand(0f, 0f, 0f))
        tutorialActive = false
        tutorialPanelEntity.enabled = false
        lastControlUpdateNanos = 0L
        setDragInteractionEnabled(true)
        refreshLaunchPanelVisibility()
        publishUiState()
        Log.i(TAG, "First-install flight tutorial completed")
    }

    /** Applies an incremental, scene-space drag delta while the helicopter is not running. */
    fun dragBy(deltaMeters: Vector3) {
        if (
            flightModel.mode != FlightMode.NOT_RUNNING ||
                !initialPlacementComplete ||
                tutorialActive
        ) {
            return
        }
        helicopterEntity.components[TransformComponent::class.java]?.apply {
            setPosition(position + deltaMeters)
        }
    }

    /** Levels the helicopter with its tail facing the viewer when placement ends or is canceled. */
    fun finishDrag() {
        if (
            flightModel.mode != FlightMode.NOT_RUNNING ||
                !initialPlacementComplete ||
                tutorialActive
        ) {
            return
        }
        val transform = helicopterEntity.components[TransformComponent::class.java] ?: return
        val viewerPosition =
            latestHmdPose?.let { sceneRoot.convertPositionFrom(it.position, null) } ?: return
        val headingDegrees =
            tailTowardViewerHeadingDegrees(
                helicopterPosition = transform.position,
                viewerPosition = viewerPosition,
                fallbackHeadingDegrees = transform.eulerAngles.yaw,
            )
        attitudeController.reset(HelicopterAttitudeCommand(0f, 0f, 0f))
        transform.setEulerAngles(EulerAngles(pitch = 0f, yaw = headingDegrees, roll = 0f))
        Log.i(TAG, "Placement drag finished; helicopter leveled with its tail toward the viewer")
    }

    /** Cancels scene subscriptions and releases audio before the Stage hierarchy is destroyed. */
    fun destroy() {
        collisionEnterSubscription?.cancel()
        collisionEnterSubscription = null
        collisionUpdateSubscription?.cancel()
        collisionUpdateSubscription = null
        stopAndCloseAudio()
        planeRecords.clear()
        planeByColliderId.clear()
    }

    private suspend fun initializeSpatialAudio() {
        if (rotorAudioController == null) {
            var loadedResource: AudioResource? = null
            var preparedController: AudioPlayerController? = null
            runCatching {
                    loadedResource =
                        withContext(Dispatchers.IO) {
                            AudioResource.load(
                                ROTOR_AUDIO_RESOURCE_NAME,
                                ROTOR_AUDIO_ASSET_URI,
                                LoadType.FROM_ASSETS,
                                AudioResourceConfig(
                                    mixerGroupID = "",
                                    randomStart = false,
                                    loopEnable = true,
                                ),
                            )
                        }
                    rotorAudioEmitter.prepareAudio(requireNotNull(loadedResource)).also { controller
                        ->
                        preparedController = controller
                        check(controller.setLoop(true)) { "Rotor controller rejected loop mode" }
                    }
                }
                .onSuccess { controller ->
                    rotorAudioResource = loadedResource
                    rotorAudioController = controller
                    Log.i(TAG, "Rotor spatial audio prepared")
                }
                .onFailure { error ->
                    preparedController?.close()
                    loadedResource?.close()
                    Log.e(TAG, "Unable to prepare rotor spatial audio", error)
                }
        }

        if (crashAudioController == null) {
            var loadedResource: AudioResource? = null
            var preparedController: AudioPlayerController? = null
            runCatching {
                    loadedResource =
                        withContext(Dispatchers.IO) {
                            AudioResource.load(
                                CRASH_AUDIO_RESOURCE_NAME,
                                CRASH_AUDIO_ASSET_URI,
                                LoadType.FROM_ASSETS,
                            )
                        }
                    crashAudioEmitter.prepareAudio(requireNotNull(loadedResource)).also { controller
                        ->
                        preparedController = controller
                        check(controller.setLoop(false)) {
                            "Crash controller rejected one-shot mode"
                        }
                    }
                }
                .onSuccess { controller ->
                    crashAudioResource = loadedResource
                    crashAudioController = controller
                    Log.i(TAG, "Crash spatial audio prepared")
                }
                .onFailure { error ->
                    preparedController?.close()
                    loadedResource?.close()
                    Log.e(TAG, "Unable to prepare crash spatial audio", error)
                }
        }
    }

    private fun applyAudioTransition(previousMode: FlightMode, causedByCrash: Boolean) {
        flightAudioCuesForTransition(previousMode, flightModel.mode, causedByCrash).forEach { cue ->
            when (cue) {
                FlightAudioCue.START_ROTOR -> {
                    val started = rotorAudioController?.play() ?: false
                    if (!started) Log.w(TAG, "Rotor spatial audio was not ready to start")
                }
                FlightAudioCue.STOP_ROTOR -> rotorAudioController?.stop()
                FlightAudioCue.PLAY_CRASH -> {
                    val controller = crashAudioController
                    controller?.stop()
                    val started = controller?.play() ?: false
                    if (!started) Log.w(TAG, "Crash spatial audio was not ready to play")
                }
            }
        }
    }

    private fun stopAndCloseAudio() {
        runCatching { rotorAudioController?.stop() }
        runCatching { crashAudioController?.stop() }
        runCatching { rotorAudioController?.close() }
            .onFailure { Log.w(TAG, "Unable to close rotor audio controller", it) }
        runCatching { crashAudioController?.close() }
            .onFailure { Log.w(TAG, "Unable to close crash audio controller", it) }
        rotorAudioController = null
        crashAudioController = null
        runCatching { rotorAudioResource?.close() }
            .onFailure { Log.w(TAG, "Unable to close rotor audio resource", it) }
        runCatching { crashAudioResource?.close() }
            .onFailure { Log.w(TAG, "Unable to close crash audio resource", it) }
        rotorAudioResource = null
        crashAudioResource = null
    }

    private fun tryPlaceInFrontOfViewer() {
        val hmdPose = latestHmdPose ?: return
        if (!modelLoaded || initialPlacementComplete) return

        val horizontalForward =
            hmdPose.rotation
                .rotateVector(Vector3(0f, 0f, -1f))
                .let { Vector3(it.x, 0f, it.z) }
                .takeIf { it.length() > 0.001f }
                ?.normalize() ?: return
        val globalPosition =
            hmdPose.position +
                horizontalForward * INITIAL_DISTANCE_METERS +
                Vector3.UP * INITIAL_VERTICAL_OFFSET_METERS
        val position = sceneRoot.convertPositionFrom(globalPosition, null)
        val headingDegrees =
            normalizeHeadingDegrees(
                Math.toDegrees(
                        atan2(horizontalForward.x.toDouble(), horizontalForward.z.toDouble())
                    )
                    .toFloat()
            )
        attitudeController.reset(
            HelicopterAttitudeCommand(
                pitchDegrees = 0f,
                rollDegrees = 0f,
                headingDegrees = headingDegrees,
            )
        )
        helicopterEntity.components[TransformComponent::class.java]?.apply {
            setPosition(position)
            setEulerAngles(EulerAngles(yaw = headingDegrees))
        }
        initialPlacementComplete = true
        helicopterEntity.enabled = true
        refreshLaunchPanelVisibility()
        setDragInteractionEnabled(true)
        publishUiState()
        Log.i(TAG, "Helicopter placed at $position in front of the viewer")
    }

    private fun handleCollision(
        entityA: Entity?,
        entityB: Entity?,
        position: Vector3,
        contacts: List<CollisionContact>,
    ) {
        if (flightModel.mode != FlightMode.RUNNING) return
        val otherEntity =
            when (helicopterEntity) {
                entityA -> entityB
                entityB -> entityA
                else -> null
            } ?: return
        val plane = planeByColliderId[otherEntity.id] ?: return
        val helicopterCenterInScene = sceneRoot.convertPositionFrom(Vector3.ZERO, helicopterEntity)
        val surfacePositionInScene =
            sceneRoot.convertPositionFrom(plane.surfaceCenterInAnchor, plane.anchorEntity)
        val rawPlaneNormalInScene =
            sceneRoot
                .convertRotationFrom(Quat(), plane.anchorEntity)
                .rotateVector(Vector3(0f, 0f, 1f))
                .normalize()
        val planeToHelicopter = helicopterCenterInScene - surfacePositionInScene
        val supportNormalInScene =
            if (rawPlaneNormalInScene.dotProduct(planeToHelicopter) >= 0f) {
                rawPlaneNormalInScene
            } else {
                rawPlaneNormalInScene * -1f
            }
        val helicopterUpInScene =
            sceneRoot
                .convertRotationFrom(Quat(), helicopterEntity)
                .rotateVector(Vector3.UP)
                .normalize()
        val landingSurface =
            LandingSurfaceSample(
                positionInScene = surfacePositionInScene,
                supportNormalInScene = supportNormalInScene,
                normalAlignmentWithHelicopterUp =
                    supportNormalInScene.dotProduct(helicopterUpInScene),
            )
        val assessment =
            assessTrackedPlaneCollision(
                mode = flightModel.mode,
                collidedWithTrackedPlane = true,
                surface = landingSurface,
                helicopterCenterInScene = helicopterCenterInScene,
                relativeVelocityMetersPerSecond = latestCommandedWorldVelocity,
            )
        val result = assessment.result
        if (result == TrackedPlaneCollisionResult.IGNORE) return
        Log.i(
            TAG,
            "Tracked-plane contact result=$result totalSpeed=" +
                "${assessment.totalSpeedMetersPerSecond} normalSpeed=" +
                "${assessment.normalSpeedMetersPerSecond} tangentialSpeed=" +
                "${assessment.tangentialSpeedMetersPerSecond} horizontal=" +
                "${assessment.hasHorizontalSurface} horizontalAlignment=" +
                "${assessment.surfaceHorizontalAlignment} below=" +
                "${assessment.isSurfaceBelowHelicopter} verticalSeparation=" +
                "${assessment.surfaceVerticalSeparationMeters} alignedSupport=" +
                "${assessment.hasAlignedSupportSurface} contactCount=${contacts.size} " +
                "surface=$landingSurface helicopterCenter=$helicopterCenterInScene",
        )

        val previousMode = flightModel.mode
        val landed = result == TrackedPlaneCollisionResult.LAND
        if (landed) flightModel.land() else flightModel.crash()
        applyAudioTransition(previousMode = previousMode, causedByCrash = !landed)
        helicopterRigidBody.isAffectedByGravity = false
        helicopterRigidBody.rigidBodyMode = RigidBodyMode.KINEMATIC
        velocityComponent.linearVelocity = Vector3.ZERO
        velocityComponent.angularVelocity = Vector3.ZERO
        latestCommandedWorldVelocity = Vector3.ZERO
        lastControlUpdateNanos = 0L
        setDragInteractionEnabled(true)
        refreshLaunchPanelVisibility()
        publishUiState()
        if (landed) {
            Log.i(
                TAG,
                "Helicopter landed safely at $position; " +
                    "plane=${plane.anchor.planeOrientation}; placement restored",
            )
        } else {
            Log.w(
                TAG,
                "Helicopter crashed at $position; plane=${plane.anchor.planeOrientation}; " +
                    "RESTART and placement restored",
            )
        }
    }

    private fun nextControlDeltaSeconds(nowNanos: Long = System.nanoTime()): Float {
        val previous = lastControlUpdateNanos
        lastControlUpdateNanos = nowNanos
        return if (previous == 0L) 0f else (nowNanos - previous) / 1_000_000_000f
    }

    private fun setDragInteractionEnabled(enabled: Boolean) {
        if (enabled) {
            if (helicopterEntity.components[InteractableComponent::class.java] == null) {
                helicopterEntity.components.set(InteractableComponent())
            }
            if (helicopterEntity.components[HoverEffectComponent::class.java] == null) {
                helicopterEntity.components.set(HoverEffectComponent())
            }
        } else {
            helicopterEntity.components.remove(InteractableComponent::class.java)
            helicopterEntity.components.remove(HoverEffectComponent::class.java)
        }
    }

    private fun controlReady(): Boolean = latestPalmControl != null

    private fun refreshLaunchPanelVisibility() {
        launchPanelEntity.enabled =
            initialPlacementComplete &&
                modelLoaded &&
                flightModel.mode == FlightMode.NOT_RUNNING &&
                !tutorialActive
    }

    private fun publishUiState() {
        onUiStateChanged(
            FlightUiState(
                helicopterReady = initialPlacementComplete && modelLoaded,
                mode = flightModel.mode,
                endReason = flightModel.endReason,
                controlReady = controlReady(),
            )
        )
    }

    private fun planeBounds(anchor: PlaneAnchor): PlaneBounds {
        val finiteVertices = anchor.vertices.filter { it.x.isFinite() && it.y.isFinite() }
        if (finiteVertices.isNotEmpty()) {
            return PlaneBounds(
                minimumX = finiteVertices.minOf { it.x },
                maximumX = finiteVertices.maxOf { it.x },
                minimumY = finiteVertices.minOf { it.y },
                maximumY = finiteVertices.maxOf { it.y },
            )
        }
        return PlaneBounds(
            minimumX = -anchor.boundingBoxSize.x / 2f,
            maximumX = anchor.boundingBoxSize.x / 2f,
            minimumY = -anchor.boundingBoxSize.y / 2f,
            maximumY = anchor.boundingBoxSize.y / 2f,
        )
    }
}

private fun createSpatialAudioEmitter(name: String, volume: Float): Entity =
    Entity().apply {
        setName(name)
        components.set(
            ObjectAudioComponent(
                volume = volume,
                directivity = Directivity(pattern = 0f, sharpness = 0f),
                distanceAttenuationMode = DistanceAttenuationMode.INVERSE_SQUARED,
                reverbVolume = AUDIO_REVERB_VOLUME,
                soundRadiusLevel = AUDIO_SOUND_RADIUS_LEVEL,
            )
        )
    }

private fun Vector3.dotProduct(other: Vector3): Float = x * other.x + y * other.y + z * other.z
