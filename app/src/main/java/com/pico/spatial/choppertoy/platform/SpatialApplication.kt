package com.pico.spatial.choppertoy.platform

import android.app.Application
import com.pico.spatial.choppertoy.mainApp
import com.pico.spatial.ui.foundation.dsl.launch

class SpatialApplication : Application() {
    override fun onCreate() {
        super.onCreate()
        launch(::mainApp)
    }
}
