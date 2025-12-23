// global build settings shared by all jme3-jolt subprojects

rootProject.name = "jme3-jolt"

dependencyResolutionManagement {
    repositories {
        //mavenLocal() // to find libraries installed locally
        mavenCentral() // to find libraries released to the Maven Central repository
    }
}

// subprojects:
include("library")
