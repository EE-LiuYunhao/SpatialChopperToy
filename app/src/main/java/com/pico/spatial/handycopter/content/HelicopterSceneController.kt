package com.pico.spatial.handycopter.content

import android.util.Log
import com.pico.spatial.core.container.SpatialViewContent
import com.pico.spatial.core.ecs.CollisionComponent
import com.pico.spatial.core.ecs.Entity
import com.pico.spatial.core.ecs.HoverEffectComponent
import com.pico.spatial.core.ecs.InteractableComponent
import com.pico.spatial.core.ecs.LoadType
import com.pico.spatial.core.ecs.ObjectAudioComponent
import com.pico.spatial.core.ecs.PhysicsForceComponent
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
import com.pico.spatial.core.math.Vector3
import com.pico.spatial.sense.plane.PlaneAnchor
import com.pico.spatial.sense.plane.PlaneOrientation
import com.pico.spatial.tracking.hmd.HMDPose
import java.util.UUID
import kotlin.math.atan2
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

private const val TAG = "HelicopterScene"
private const val HELICOPTER_ASSET_URI = "asset://helicopter.glb"
private const val ROTOR_AUDIO_ASSET_URI = "asset://audio/helicopter_rotor_source.ogg"
private const val CRASH_AUDIO_ASSET_URI = "asset://audio/helicopter_crash_source.ogg"
private const val ROTOR_AUDIO_RESOURCE_NAME = "HandyCopterRotorLoop"
private const val CRASH_AUDIO_RESOURCE_NAME = "HandyCopterCrash"
private const val ROTOR_AUDIO_VOLUME = 0.72f
private const val CRASH_AUDIO_VOLUME = 0.90f
private const val AUDIO_REVERB_VOLUME = 0.12f
private const val AUDIO_SOUND_RADIUS_LEVEL = 0.18f
private const val HELICOPTER_MODEL_SCALE = 0.10794f
private const val PLANE_COLLIDER_THICKNESS_METERS = 0.02f
private const val INITIAL_DISTANCE_METERS = 1.0f
private const val INITIAL_VERTICAL_OFFSET_METERS = -0.25f

private val HELICOPTER_COLLISION_SIZE = Vector3(0.380f, 0.150f, 0.500f)
private val HELICOPTER_SOURCE_CENTER = Vector3(0.00580175f, 0.76734895f, -0.82753625f)

/**
 * Owns the draggable helicopter entity, its physics state machine, tracked-plane colliders,
 * palm-driven flight controls, and helicopter-bound positional audio.
 */
internal class HelicopterSceneController(
    private val sceneRoot: Entity,
    private val launchPanelEntity: Entity,
    private val palmFlightCalibration: PalmFlightCalibration,
    private val onUiStateChanged: (FlightUiState) -> Unit,
) {
    private data class PlaneRecord(
        val anchor: PlaneAnchor,
        val anchorEntity: Entity,
        val colliderEntity: Entity,
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
            listOf(ShapeResource.createBox(HELICOPTER_COLLISION_SIZE)),
            HELICOPTER_MASS_KILOGRAMS,
        )
    private val helicopterShape = ShapeResource.createBox(HELICOPTER_COLLISION_SIZE)
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
                // Translational damping is modeled explicitly as a testable aerodynamic force.
                linearDamping = 0f
                angularDamping = 1f
                collisionDetectionMode = CollisionDetectionMode.CONTINUOUS_DYNAMIC
            }
    private val forceComponent = PhysicsForceComponent(Vector3.ZERO, Vector3.ZERO)
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
            components.set(forceComponent)
            components.set(velocityComponent)
            components.set(InteractableComponent())
            components.set(HoverEffectComponent())
            addChild(launchPanelEntity)
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
    private var lastObservedDownwardSpeedMetersPerSecond = 0f
    private var lastControlUpdateNanos = 0L

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
                    handleCollision(
                        event.entityA,
                        event.entityB,
                        event.contacts,
                        event.position,
                        logSafeLanding = true,
                    )
                }
            collisionUpdateSubscription =
                content.subscribe(CollisionEvents.Update::class.java, helicopterEntity, null) {
                    event ->
                    handleCollision(
                        event.entityA,
                        event.entityB,
                        event.contacts,
                        event.position,
                        logSafeLanding = false,
                    )
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
        val record = PlaneRecord(anchor, anchorEntity, colliderEntity)
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
        latestPalmControl = control
        lastObservedDownwardSpeedMetersPerSecond =
            (-velocityComponent.linearVelocity.y).coerceAtLeast(0f)
        if (flightModel.mode != FlightMode.RUNNING) return

        val transform = helicopterEntity.components[TransformComponent::class.java] ?: return
        val calibratedControl = palmFlightCalibration.resolve(control)?.helicopterControl
        val attitude =
            attitudeController.step(
                control = calibratedControl,
                cyclicEnabled = true,
                deltaSeconds = nextControlDeltaSeconds(),
            )
        transform.setEulerAngles(
            EulerAngles(
                pitch = attitude.pitchDegrees,
                yaw = attitude.headingDegrees,
                roll = attitude.rollDegrees,
            )
        )
        val liftNewtons = flightModel.liftForPalmHeight(control?.palmHeightMeters)
        val bodyUp = transform.quaternion.rotateVector(Vector3.UP)
        forceComponent.force =
            calculateHelicopterAerodynamicForce(
                bodyUp = bodyUp,
                liftNewtons = liftNewtons,
                velocityMetersPerSecond = velocityComponent.linearVelocity,
            )
    }

    /**
     * Starts or restarts flight after calibrating palm attitude and height at the click instant.
     */
    fun startFlight() {
        if (flightModel.mode == FlightMode.RUNNING) return
        if (!initialPlacementComplete || !modelLoaded) {
            Log.w(TAG, "Ignoring START until the helicopter is loaded and placed")
            return
        }
        val palmAtStart = latestPalmControl
        if (palmAtStart == null) {
            Log.w(TAG, "Ignoring START until a valid palm pose is tracked for calibration")
            return
        }
        val transform = helicopterEntity.components[TransformComponent::class.java] ?: return
        val currentEuler = transform.eulerAngles
        val helicopterAtStart =
            HelicopterAttitudeCommand(
                pitchDegrees = currentEuler.pitch,
                rollDegrees = currentEuler.roll,
                headingDegrees = currentEuler.yaw,
            )
        palmFlightCalibration.calibrate(palmAtStart, helicopterAtStart)
        attitudeController.reset(helicopterAtStart)
        lastControlUpdateNanos = 0L
        velocityComponent.linearVelocity = Vector3.ZERO
        velocityComponent.angularVelocity = Vector3.ZERO
        setDragInteractionEnabled(false)
        launchPanelEntity.enabled = false
        helicopterRigidBody.isAffectedByGravity = true
        helicopterRigidBody.rigidBodyMode = RigidBodyMode.DYNAMIC
        val previousMode = flightModel.mode
        val neutralLift = flightModel.start(palmAtStart.palmHeightMeters)
        applyAudioTransition(previousMode = previousMode, causedByCrash = false)
        forceComponent.force =
            calculateHelicopterAerodynamicForce(
                bodyUp = transform.quaternion.rotateVector(Vector3.UP),
                liftNewtons = neutralLift,
                velocityMetersPerSecond = Vector3.ZERO,
            )
        publishUiState()
        Log.i(TAG, "Flight mode changed to ${flightModel.mode}; palm pose and height calibrated")
    }

    /** Applies an incremental, scene-space drag delta while the helicopter is not running. */
    fun dragBy(deltaMeters: Vector3) {
        if (flightModel.mode != FlightMode.NOT_RUNNING || !initialPlacementComplete) return
        helicopterEntity.components[TransformComponent::class.java]?.apply {
            setPosition(position + deltaMeters)
        }
    }

    /** Levels the helicopter in stage space when a placement drag ends or is canceled. */
    fun finishDrag() {
        if (flightModel.mode != FlightMode.NOT_RUNNING || !initialPlacementComplete) return
        attitudeController.reset(HelicopterAttitudeCommand(0f, 0f, 0f))
        helicopterEntity.components[TransformComponent::class.java]?.setEulerAngles(
            EulerAngles(pitch = 0f, yaw = 0f, roll = 0f)
        )
        Log.i(TAG, "Placement drag finished; helicopter orientation reset to zero")
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
        launchPanelEntity.enabled = true
        setDragInteractionEnabled(true)
        publishUiState()
        Log.i(TAG, "Helicopter placed at $position in front of the viewer")
    }

    private fun handleCollision(
        entityA: Entity?,
        entityB: Entity?,
        contacts: List<CollisionContact>,
        position: Vector3,
        logSafeLanding: Boolean,
    ) {
        if (flightModel.mode != FlightMode.RUNNING) return
        val otherEntity =
            when (helicopterEntity) {
                entityA -> entityB
                entityB -> entityA
                else -> null
            } ?: return
        val plane = planeByColliderId[otherEntity.id] ?: return
        val contactPositions =
            contacts
                .map { it.position }
                .ifEmpty {
                    // Some runtimes provide the aggregate collision position but omit detailed
                    // points.
                    listOf(position)
                }
        val bodyTransform = helicopterEntity.components[TransformComponent::class.java]
        val localContactHeights =
            if (bodyTransform == null) {
                emptyList()
            } else {
                contactPositions.map { contactPosition ->
                    bodyTransform.quaternion
                        .conjugate()
                        .rotateVector(contactPosition - bodyTransform.position)
                        .y
                }
            }
        val downwardSpeed =
            maxOf(
                (-velocityComponent.linearVelocity.y).coerceAtLeast(0f),
                lastObservedDownwardSpeedMetersPerSecond,
            )
        val safeLanding =
            isSafeLandingContact(
                planeIsHorizontalUpward =
                    plane.anchor.planeOrientation == PlaneOrientation.HORIZONTAL_UPWARD,
                localContactHeightsMeters = localContactHeights,
                downwardSpeedMetersPerSecond = downwardSpeed,
            )
        if (safeLanding) {
            if (logSafeLanding) {
                Log.i(
                    TAG,
                    "Safe landing: contacts=${contactPositions.size}, speed=$downwardSpeed m/s",
                )
            }
            return
        }

        val previousMode = flightModel.mode
        flightModel.crash()
        applyAudioTransition(previousMode = previousMode, causedByCrash = true)
        forceComponent.force = Vector3.ZERO
        helicopterRigidBody.isAffectedByGravity = false
        helicopterRigidBody.rigidBodyMode = RigidBodyMode.KINEMATIC
        velocityComponent.linearVelocity = Vector3.ZERO
        velocityComponent.angularVelocity = Vector3.ZERO
        lastControlUpdateNanos = 0L
        setDragInteractionEnabled(true)
        launchPanelEntity.enabled = true
        publishUiState()
        Log.w(
            TAG,
            "Helicopter crashed at $position; plane=${plane.anchor.planeOrientation}, " +
                "localY=$localContactHeights, speed=$downwardSpeed m/s",
        )
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

    private fun publishUiState() {
        onUiStateChanged(
            FlightUiState(
                helicopterReady = initialPlacementComplete && modelLoaded,
                mode = flightModel.mode,
                crashed = flightModel.crashed,
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

private fun normalizeHeadingDegrees(degrees: Float): Float = ((degrees % 360f) + 360f) % 360f
