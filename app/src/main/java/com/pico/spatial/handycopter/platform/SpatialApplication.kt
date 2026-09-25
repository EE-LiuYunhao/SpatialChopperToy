package com.pico.spatial.handycopter.platform

import android.app.Application
import com.pico.spatial.handycopter.mainApp
import com.pico.spatial.ui.foundation.dsl.launch

class SpatialApplication : Application() {
    override fun onCreate() {
        super.onCreate()
        launch(::mainApp)
    }
}
