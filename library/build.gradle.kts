// Gradle script to build and publish the "library" subproject of jme3-jolt

import org.gradle.nativeplatform.platform.internal.DefaultNativePlatform

plugins {
    checkstyle    // to analyze Java sourcecode for style violations
    `java-library`  // to build JVM libraries
}

val artifact = "jme3-jolt"
val libraryVersion = property("libraryVersion") as String
val baseName = "${artifact}-${libraryVersion}" // for artifacts
val btf = property("btf") as String
val javaVendor = System.getProperty("java.vendor")
val javaVersion = JavaVersion.current()
val os = DefaultNativePlatform.getCurrentOperatingSystem()

java {
    sourceCompatibility = JavaVersion.VERSION_11
    targetCompatibility = JavaVersion.VERSION_11
}

dependencies {
    api(libs.jme3.core)
    api(libs.jme3.terrain)
    api(libs.jolt.jni.linux64fma)
    implementation(libs.heart)
    implementation(libs.jme3.desktop)
    implementation(libs.oshi.core)

    if (os.isLinux()) {
        testRuntimeOnly(variantOf(libs.jolt.jni.linux64){ classifier(btf) })
        testRuntimeOnly(variantOf(libs.jolt.jni.linux64fma){ classifier(btf) })
        testRuntimeOnly(variantOf(libs.jolt.jni.linuxarm32hf){ classifier(btf) })
        testRuntimeOnly(variantOf(libs.jolt.jni.linuxarm64){ classifier(btf) })
    }
    if (os.isMacOsX()) {
        testRuntimeOnly(variantOf(libs.jolt.jni.macosx64){ classifier(btf) })
        testRuntimeOnly(variantOf(libs.jolt.jni.macosxarm64){ classifier(btf) })
    }
    if (os.isWindows()) {
        testRuntimeOnly(variantOf(libs.jolt.jni.windows64){ classifier(btf) })
        testRuntimeOnly(variantOf(libs.jolt.jni.windows64avx2){ classifier(btf) })
    }
}

configurations.all {
    resolutionStrategy.cacheChangingModulesFor(0, "seconds") // to disable caching of snapshots
}

checkstyle {
    toolVersion = libs.versions.checkstyle.get()
}

tasks.withType<JavaCompile>().all { // Java compile-time options:
    options.compilerArgs.add("-Xdiags:verbose")
    options.compilerArgs.add("-Xlint:unchecked")
    options.encoding = "UTF-8"
    options.release = 11
    //options.setDeprecation(true) // to provide detailed deprecation warnings
}

tasks.jar {
    archiveBaseName.set(baseName)
    doLast {
        println("built using Java $javaVersion ($javaVendor)")
    }
    manifest {
        attributes["Created-By"] = "$javaVersion ($javaVendor)"
    }
}
