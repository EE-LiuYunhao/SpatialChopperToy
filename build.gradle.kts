// Top-level build file where you can add configuration options common to all sub-projects/modules.
plugins {
    alias(libs.plugins.android.application) apply false
    alias(libs.plugins.android.library) apply false
    alias(libs.plugins.kotlin.android) apply false
    alias(libs.plugins.spotless)
}

val commonSpotlessExcludes =
    arrayOf(
        "**/build/**",
        "**/.gradle/**",
        "**/.idea/**",
        "**/out/**",
        "**/tmp/**",
        "**/generated/**",
        "**/test-resources/**",
        "**/skills/**",
        "skills/**",
    )

spotless {
    java {
        target("**/*.java")
        targetExclude(*commonSpotlessExcludes)
        googleJavaFormat("1.17.0").aosp().reflowLongStrings()
    }
    kotlin {
        target("**/*.kt")
        targetExclude(*commonSpotlessExcludes)
        ktfmt().kotlinlangStyle()
        toggleOffOn()
    }
    kotlinGradle {
        target("**/*.gradle.kts")
        targetExclude(*commonSpotlessExcludes)
        ktfmt().kotlinlangStyle()
    }
    format("misc") {
        target("**/*.md", "**/*.xml", "**/*.yaml", "**/*.yml", ".gitignore")
        targetExclude(*commonSpotlessExcludes)
        trimTrailingWhitespace()
        endWithNewline()
    }
}
