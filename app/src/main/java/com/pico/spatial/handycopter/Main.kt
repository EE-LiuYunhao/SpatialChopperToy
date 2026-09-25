package com.pico.spatial.handycopter

import androidx.compose.ui.platform.LocalContext
import com.pico.spatial.handycopter.content.HomeStage
import com.pico.spatial.ui.design.PicoTheme
import com.pico.spatial.ui.design.systemColorScheme
import com.pico.spatial.ui.foundation.dsl.DefaultStage
import com.pico.spatial.ui.foundation.dsl.SpatialAppScope

fun mainApp(scope: SpatialAppScope) =
    with(scope) {
        DefaultStage {
            val system = systemColorScheme(LocalContext.current)
            val appColorScheme =
                system.copy(
                    fillPrimary = system.fillPrimary,
                    fillSecondary = system.fillSecondary,
                    fillTertiary = system.fillTertiary,
                    fillLight = system.fillLight,
                    labelPrimaryLight = system.labelPrimaryLight,
                    labelPrimary = system.labelPrimary,
                    labelSecondary = system.labelSecondary,
                    labelTertiary = system.labelTertiary,
                    labelQuaternary = system.labelQuaternary,
                    lightenHover = system.lightenHover,
                    lightenPressed = system.lightenPressed,
                    error = system.error,
                    alert = system.alert,
                    passable = system.passable,
                    interaction = system.interaction,
                    dividerLine = system.dividerLine,
                )
            PicoTheme(colorScheme = appColorScheme) { HomeStage() }
        }
    }
