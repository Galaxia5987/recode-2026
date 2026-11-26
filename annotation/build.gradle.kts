plugins {
    kotlin("jvm")
}

group = "org.team5987"

dependencies {
    implementation("com.squareup:kotlinpoet:1.14.2")
    implementation("com.squareup:kotlinpoet-ksp:1.14.2")
    implementation("com.google.devtools.ksp:symbol-processing-api:2.3.10")
    implementation("org.jetbrains.kotlin:kotlin-compiler-embeddable:2.4.10")
    implementation("com.google.code.gson:gson:2.10.1")
}

repositories {
    mavenCentral()
}

sourceSets.main {
    java.srcDirs("src/main/kotlin")
}

