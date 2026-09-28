package com.pico.spatial.handycopter.content

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.nativeCanvas
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.anycontroller.library.model.AnyControllerStatus
import com.example.anycontroller.library.model.CalibrationState
import com.pico.spatial.ui.design.Button
import com.pico.spatial.ui.design.ButtonDefaults
import com.pico.spatial.ui.design.Text
import java.util.Locale
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

// Cockpit-instrument colors intentionally stay literal rather than adapting as UI chrome.
private val InstrumentBlack = Color(0xFF11151A) // design-style: fixed-figma-color instrument black
private val InstrumentBezel = Color(0xFF3B424A) // design-style: fixed-figma-color metal bezel
private val InstrumentWhite = Color(0xFFF4F7FA) // design-style: fixed-figma-color instrument marks
private val InstrumentYellow = Color(0xFFFFC928) // design-style: fixed-figma-color compass cue
private val PanelScrim = Color(0xCC11151A) // design-style: fixed-figma-color panel scrim
private val StartGreen = Color(0xFF168A45) // design-style: fixed-figma-color start control

/** Renders launch/restart state for the selected control mode on the helicopter entity. */
@Composable
internal fun HelicopterLaunchPanel(
    crashed: Boolean,
    controlMode: FlightControlMode?,
    controlReady: Boolean,
    onStart: () -> Unit,
) {
    Column(
        modifier =
            Modifier.fillMaxSize().background(PanelScrim, RoundedCornerShape(16.dp)).padding(8.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(10.dp, Alignment.CenterVertically),
    ) {
        Text(
            text =
                when {
                    crashed -> "Oops, crashed. Drag the helicopter to its next start position"
                    !controlReady && controlMode == FlightControlMode.REALISTIC ->
                        "Show a palm to enable START"
                    else -> "Drag the helicopter to its start position"
                },
            color = InstrumentWhite,
            fontSize = 12.sp,
            fontWeight = FontWeight.SemiBold,
            textAlign = TextAlign.Center,
            modifier = Modifier.fillMaxWidth(),
        )
        Button(
            onClick = onStart,
            enabled = controlReady,
            size = ButtonDefaults.Small,
            colors =
                ButtonDefaults.buttonColors(
                    containerColor = StartGreen,
                    contentColor = InstrumentWhite,
                ),
        ) {
            // design-style: inherited-content-color Button
            Text(
                text = if (crashed) "RESTART" else "START",
                fontSize = 11.sp,
                fontWeight = FontWeight.Bold,
            )
        }
    }
}

/** Renders the launch-time mode choice and the guided AnyController calibration sequence. */
@Composable
internal fun ControllerSetupPanel(
    controlMode: FlightControlMode?,
    controllerStatus: AnyControllerStatus,
    calibrationState: CalibrationState,
    detectedSurfaceCount: Int,
    onModeSelected: (FlightControlMode) -> Unit,
) {
    Column(
        modifier =
            Modifier.fillMaxSize().background(PanelScrim, RoundedCornerShape(20.dp)).padding(18.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(12.dp, Alignment.CenterVertically),
    ) {
        Text(
            text =
                if (controlMode == null) "Choose a flight control mode" else "RC controller setup",
            color = InstrumentWhite,
            fontSize = 20.sp,
            fontWeight = FontWeight.Bold,
            textAlign = TextAlign.Center,
        )
        if (controlMode == null) {
            Text(
                text =
                    "Realistic uses your palm as cyclic, pedals, and collective. " +
                        "RC Drone uses two finger-operated touchpads.",
                color = InstrumentWhite,
                fontSize = 13.sp,
                textAlign = TextAlign.Center,
            )
            Button(
                onClick = { onModeSelected(FlightControlMode.REALISTIC) },
                size = ButtonDefaults.Small,
            ) {
                // design-style: inherited-content-color Button
                Text("REALISTIC HELICOPTER")
            }
            Button(
                onClick = { onModeSelected(FlightControlMode.RC_DRONE) },
                size = ButtonDefaults.Small,
            ) {
                // design-style: inherited-content-color Button
                Text("RC DRONE")
            }
        } else {
            Text(
                text =
                    rcCalibrationInstruction(
                        status = controllerStatus,
                        calibrationState = calibrationState,
                        detectedSurfaceCount = detectedSurfaceCount,
                    ),
                color = InstrumentWhite,
                fontSize = 14.sp,
                textAlign = TextAlign.Center,
            )
            Text(
                text = "Left pad: lift + yaw   •   Right pad: pitch + roll",
                color = InstrumentWhite,
                fontSize = 12.sp,
                fontWeight = FontWeight.SemiBold,
                textAlign = TextAlign.Center,
            )
        }
    }
}

private fun rcCalibrationInstruction(
    status: AnyControllerStatus,
    calibrationState: CalibrationState,
    detectedSurfaceCount: Int,
): String =
    when {
        status == AnyControllerStatus.ERROR ->
            "Controller startup failed. Relaunch HandyCopter to retry."
        status == AnyControllerStatus.UNAVAILABLE ->
            "Hand tracking is unavailable. Make both hands visible and try again."
        calibrationState == CalibrationState.WAITING_FOR_SURFACE ->
            "Scanning $detectedSurfaceCount planes. Touch a white plane outline with your left index finger."
        calibrationState == CalibrationState.PLACE_LEFT_CENTER ->
            "Touch the center of the LEFT virtual pad with your left index finger."
        calibrationState == CalibrationState.PLACE_LEFT_BOUNDARY ->
            "Slide left index outward to draw the LEFT circle, then lift it at least 5 cm."
        calibrationState == CalibrationState.CALIBRATE_LEFT_UP ->
            "Touch above the LEFT center to define the forward (+Y) direction."
        calibrationState == CalibrationState.PLACE_RIGHT_CENTER ->
            "Touch the center of the RIGHT virtual pad with your right index finger."
        calibrationState == CalibrationState.PLACE_RIGHT_BOUNDARY ->
            "Slide right index outward to draw the RIGHT circle, then lift it at least 5 cm."
        calibrationState == CalibrationState.READY ->
            "Controller ready. Use START on the helicopter."
        else -> "Starting plane and hand tracking…"
    }

/** Renders the absolute stage-space heading indicator beside the attitude ball. */
@Composable
internal fun HeadingPanel(headingDegrees: Float) {
    Column(
        modifier =
            Modifier.fillMaxSize().background(PanelScrim, RoundedCornerShape(16.dp)).padding(6.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.Center,
    ) {
        Text(
            text = "HEADING",
            color = InstrumentWhite,
            fontSize = 10.sp,
            fontWeight = FontWeight.SemiBold,
        )
        Box(contentAlignment = Alignment.Center) {
            Canvas(modifier = Modifier.size(82.dp)) {
                val center = Offset(size.width / 2f, size.height / 2f)
                val radius = min(size.width, size.height) / 2f - 3.dp.toPx()
                drawCircle(InstrumentBezel, radius + 3.dp.toPx(), center)
                drawCircle(InstrumentBlack, radius, center)
                drawCircle(InstrumentWhite, radius, center, style = Stroke(1.dp.toPx()))

                for (bearing in 0 until 360 step 10) {
                    val radians = (bearing - headingDegrees) * PI.toFloat() / 180f
                    val isMajor = bearing % 30 == 0
                    val outer = radius * 0.91f
                    val inner = radius * if (isMajor) 0.72f else 0.82f
                    drawLine(
                        InstrumentWhite,
                        Offset(center.x + sin(radians) * inner, center.y - cos(radians) * inner),
                        Offset(center.x + sin(radians) * outer, center.y - cos(radians) * outer),
                        strokeWidth = if (isMajor) 1.5.dp.toPx() else 0.75.dp.toPx(),
                    )
                }

                val labels = mapOf(0 to "N", 90 to "E", 180 to "S", 270 to "W")
                drawContext.canvas.nativeCanvas.apply {
                    val paint =
                        android.graphics.Paint().apply {
                            color = android.graphics.Color.WHITE
                            textSize = 9.sp.toPx()
                            textAlign = android.graphics.Paint.Align.CENTER
                            isAntiAlias = true
                            typeface = android.graphics.Typeface.DEFAULT_BOLD
                        }
                    labels.forEach { (bearing, label) ->
                        val radians = (bearing - headingDegrees) * PI.toFloat() / 180f
                        drawText(
                            label,
                            center.x + sin(radians) * radius * 0.55f,
                            center.y - cos(radians) * radius * 0.55f + paint.textSize * 0.35f,
                            paint,
                        )
                    }
                }

                val pointer =
                    Path().apply {
                        moveTo(center.x, center.y - radius * 0.96f)
                        lineTo(center.x - 4.dp.toPx(), center.y - radius * 0.78f)
                        lineTo(center.x + 4.dp.toPx(), center.y - radius * 0.78f)
                        close()
                    }
                drawPath(pointer, InstrumentYellow)
            }
            Text(
                text = String.format(Locale.US, "%05.1f°", headingDegrees),
                color = InstrumentWhite,
                fontSize = 13.sp,
                fontWeight = FontWeight.Bold,
            )
        }
    }
}
