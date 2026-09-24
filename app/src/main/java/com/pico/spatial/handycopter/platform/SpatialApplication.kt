package com.pico.spatial.handycopter.platform

import android.app.Application
import com.pico.spatial.ui.foundation.dsl.launch
import com.pico.spatial.handycopter.mainApp

class SpatialApplication : Application() {
    override fun onCreate() {
        super.onCreate()
        launch(::mainApp)
    }
}
