package com.pico.spatial.choppertoy.content

import android.content.Context

/** Stores whether the first-install flight tutorial has been completed. */
internal class FlightTutorialPreferences(context: Context) {
    private val preferences =
        context.applicationContext.getSharedPreferences(PREFERENCES_NAME, Context.MODE_PRIVATE)

    /** Returns true after the user has completed all seven lessons. */
    fun isCompleted(): Boolean = preferences.getBoolean(KEY_COMPLETED, false)

    /** Persists completion so subsequent launches skip the tutorial. */
    fun markCompleted() {
        preferences.edit().putBoolean(KEY_COMPLETED, true).apply()
    }

    private companion object {
        const val PREFERENCES_NAME = "choppertoy_first_install_tutorial"
        const val KEY_COMPLETED = "completed"
    }
}
