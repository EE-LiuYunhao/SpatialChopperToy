package com.pico.spatial.handycopter.content

import com.pico.spatial.core.ecs.Entity
import com.pico.spatial.core.ecs.ModelEntity
import com.pico.spatial.core.ecs.TransformComponent
import com.pico.spatial.core.ecs.resource.BlendingMode
import com.pico.spatial.core.ecs.resource.MeshModel
import com.pico.spatial.core.ecs.resource.MeshResource
import com.pico.spatial.core.ecs.resource.PhysicallyBasedMaterial
import com.pico.spatial.core.math.Color4
import com.pico.spatial.core.math.EulerAngles
import com.pico.spatial.core.math.Vector3
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

private const val BALL_RADIUS_METERS = 0.040f
private const val BALL_LINE_LIFT_METERS = 0.0012f
private const val BALL_LINE_HALF_WIDTH_METERS = 0.00065f
private const val HORIZON_LINE_HALF_WIDTH_METERS = 0.0012f
private const val BEZEL_OUTER_RADIUS_METERS = 0.046f
private const val BEZEL_INNER_RADIUS_METERS = 0.042f
private const val AIRCRAFT_CUE_Z_METERS = 0.047f
private const val HEMISPHERE_STACKS = 12
private const val HEMISPHERE_SLICES = 36

private val SkyColor = Color4(0.20f, 0.56f, 0.80f, 1f)
private val EarthColor = Color4(0.54f, 0.33f, 0.21f, 1f)
private val MarkingColor = Color4(0.96f, 0.97f, 0.98f, 1f)
private val BezelColor = Color4(0.07f, 0.09f, 0.11f, 1f)
private val CueColor = Color4(1.0f, 0.79f, 0.16f, 1f)

internal data class AttitudeBallScene(val root: Entity, val rotatingBall: Entity)

/** Builds an actual 3D artificial-horizon ball with fixed bezel and aircraft cue. */
internal fun createAttitudeBallScene(): AttitudeBallScene {
    val skyMaterial = cockpitMaterial(SkyColor, roughness = 0.28f)
    val earthMaterial = cockpitMaterial(EarthColor, roughness = 0.38f)
    val markingMaterial = cockpitMaterial(MarkingColor, roughness = 0.24f)
    val bezelMaterial = cockpitMaterial(BezelColor, roughness = 0.30f, metallic = 0.18f)
    val cueMaterial = cockpitMaterial(CueColor, roughness = 0.22f, metallic = 0.08f)

    val skyEntity =
        ModelEntity(hemisphereMesh(upper = true, name = "AttitudeSkyHemisphere"), skyMaterial)
            .apply { setName("AttitudeSkyHemisphere") }
    val earthEntity =
        ModelEntity(hemisphereMesh(upper = false, name = "AttitudeEarthHemisphere"), earthMaterial)
            .apply { setName("AttitudeEarthHemisphere") }

    val rotatingBall =
        Entity().apply {
            setName("AttitudeRotatingBall")
            addChild(skyEntity)
            addChild(earthEntity)
            addChild(latitudeRing(0f, HORIZON_LINE_HALF_WIDTH_METERS, markingMaterial))
            listOf(-40f, -20f, 20f, 40f).forEach { latitude ->
                addChild(latitudeRing(latitude, BALL_LINE_HALF_WIDTH_METERS, markingMaterial))
            }
        }

    val bezel =
        ModelEntity(
                createRingMesh(
                    outerRingRadius = BEZEL_OUTER_RADIUS_METERS,
                    innerRingRadius = BEZEL_INNER_RADIUS_METERS,
                ),
                bezelMaterial,
            )
            .apply { setName("AttitudeBezel") }

    val cueBarMesh = MeshResource.createBox(Vector3(0.024f, 0.0035f, 0.0035f), 0.0015f)
    val leftCue =
        ModelEntity(cueBarMesh, cueMaterial).apply {
            setName("AttitudeAircraftCueLeft")
            components[TransformComponent::class.java]?.setPosition(
                Vector3(-0.025f, 0f, AIRCRAFT_CUE_Z_METERS)
            )
        }
    val rightCue =
        ModelEntity(cueBarMesh, cueMaterial).apply {
            setName("AttitudeAircraftCueRight")
            components[TransformComponent::class.java]?.setPosition(
                Vector3(0.025f, 0f, AIRCRAFT_CUE_Z_METERS)
            )
        }
    val centerCue =
        ModelEntity(MeshResource.createSphere(0.0038f), cueMaterial).apply {
            setName("AttitudeAircraftCueCenter")
            components[TransformComponent::class.java]?.setPosition(
                Vector3(0f, 0f, AIRCRAFT_CUE_Z_METERS)
            )
        }

    val root =
        Entity().apply {
            setName("AttitudeIndicator3D")
            addChild(rotatingBall)
            addChild(bezel)
            addChild(leftCue)
            addChild(rightCue)
            addChild(centerCue)
        }
    return AttitudeBallScene(root = root, rotatingBall = rotatingBall)
}

/**
 * The ball moves opposite the fixed aircraft cue on screen: positive pitch lowers the horizon,
 * while a positive right bank raises the horizon on the right.
 */
internal fun attitudeBallRotation(readout: InstrumentReadout): EulerAngles =
    EulerAngles(pitch = readout.pitchDegrees, yaw = 0f, roll = readout.rollDegrees)

private fun hemisphereMesh(upper: Boolean, name: String): MeshResource {
    val phiStart = if (upper) 0f else (PI / 2.0).toFloat()
    val phiEnd = if (upper) (PI / 2.0).toFloat() else PI.toFloat()
    val positions = ArrayList<Vector3>((HEMISPHERE_STACKS + 1) * (HEMISPHERE_SLICES + 1))
    val normals = ArrayList<Vector3>(positions.size)
    val indices = ArrayList<Int>(HEMISPHERE_STACKS * HEMISPHERE_SLICES * 6)

    for (stack in 0..HEMISPHERE_STACKS) {
        val fraction = stack.toFloat() / HEMISPHERE_STACKS
        val phi = phiStart + (phiEnd - phiStart) * fraction
        val sinPhi = sin(phi)
        val cosPhi = cos(phi)
        for (slice in 0..HEMISPHERE_SLICES) {
            val theta = (2.0 * PI * slice / HEMISPHERE_SLICES).toFloat()
            val normal = Vector3(sinPhi * cos(theta), cosPhi, sinPhi * sin(theta))
            normals += normal
            positions += normal * BALL_RADIUS_METERS
        }
    }

    val rowSize = HEMISPHERE_SLICES + 1
    for (stack in 0 until HEMISPHERE_STACKS) {
        for (slice in 0 until HEMISPHERE_SLICES) {
            val current = stack * rowSize + slice
            val nextRow = current + rowSize
            indices += current
            indices += current + 1
            indices += nextRow
            indices += current + 1
            indices += nextRow + 1
            indices += nextRow
        }
    }

    val model = MeshModel(positions = positions, triangleIndices = indices, normals = normals)
    MeshModel.validateOrThrow(model)
    return MeshResource.createWithMeshModel(model = model, name = name)
}

private fun latitudeRing(
    latitudeDegrees: Float,
    halfWidthMeters: Float,
    material: PhysicallyBasedMaterial,
): Entity {
    val latitudeRadians = Math.toRadians(latitudeDegrees.toDouble()).toFloat()
    val y = BALL_RADIUS_METERS * sin(latitudeRadians)
    val liftedRadius = BALL_RADIUS_METERS + BALL_LINE_LIFT_METERS
    val ringRadius = sqrt(liftedRadius * liftedRadius - y * y)
    return ModelEntity(
            createRingMesh(
                outerRingRadius = ringRadius + halfWidthMeters,
                innerRingRadius = ringRadius - halfWidthMeters,
            ),
            material,
        )
        .apply {
            setName("AttitudePitchLine${latitudeDegrees.toInt()}")
            components[TransformComponent::class.java]?.apply {
                setPosition(Vector3(0f, y, 0f))
                setEulerAngles(EulerAngles(pitch = 90f))
            }
        }
}

private fun cockpitMaterial(
    color: Color4,
    roughness: Float,
    metallic: Float = 0.03f,
): PhysicallyBasedMaterial =
    PhysicallyBasedMaterial.create(BlendingMode.OPAQUE).apply {
        setBaseColor(color)
        setRoughness(roughness)
        setMetallic(metallic)
    }

private fun createRingMesh(outerRingRadius: Float, innerRingRadius: Float): MeshResource =
    MeshResource.createTorus(outerRingRadius, innerRingRadius)
