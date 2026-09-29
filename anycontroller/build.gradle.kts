import io.gitlab.arturbosch.detekt.extensions.DetektExtension

plugins {
    alias(libs.plugins.android.library)
    alias(libs.plugins.kotlin.android)
    alias(libs.plugins.detekt)
}

android {
    namespace = "com.example.anycontroller.library"
    compileSdk = 35

    defaultConfig {
        minSdk = 35
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }
    kotlinOptions { jvmTarget = "11" }
}

configure<DetektExtension> {
    buildUponDefaultConfig = true
    config.setFrom(rootProject.file("config/detekt/detekt.yml"))
    source.setFrom(
        fileTree(projectDir) {
            include("src/main/**/*.kt", "src/test/**/*.kt")
            exclude("**/build/**")
        }
    )
}

dependencies {
    api(platform(libs.spatial.bom))
    api(libs.spatial.core)
    api(libs.spatial.ui.sense)
    implementation(libs.spatial.foundation)
    implementation(libs.spatial.ui.tracking)
    implementation(libs.kotlinx.coroutines.core)
    testImplementation(libs.junit)
}
