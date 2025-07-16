plugins {
    application
    antlr
}

group = "org.connection-guide"
version = "1.0-SNAPSHOT"

repositories {
    mavenCentral()
}

dependencies {
    // Dependencias de Webots
    implementation(files("/Applications/Webots.app/Contents/lib/controller/java/Controller.jar"))
    implementation(files("/Applications/Webots.app/Contents/lib/controller/java/vehicle.jar"))

    // Dependencias de JUnit (solo si las estás utilizando para pruebas)
    testImplementation(platform("org.junit:junit-bom:5.10.0"))
    testImplementation("org.junit.jupiter:junit-jupiter")

    // Dependencia de Lombok
    compileOnly("org.projectlombok:lombok:1.18.28")
    annotationProcessor("org.projectlombok:lombok:1.18.28")

    // Dependencias de ANTLR
    implementation("org.antlr:antlr4-runtime:4.13.2")
    antlr("org.antlr:antlr4:4.13.2")
}

application {
    mainClass.set("org.connection.DjiController")
    applicationDefaultJvmArgs = listOf(
        "-Djava.library.path=/Applications/Webots.app/Contents/lib/controller",
        "-Djava.library.path=/Applications/Webots.app/Contents/lib/controller/java"
    )
}

tasks.test {
    useJUnitPlatform()
}

tasks.withType<AntlrTask> {
    arguments.addAll(listOf("-visitor", "-listener"))
}

sourceSets {
    main {
        java {
            srcDir("src/main/antlr")
        }
    }
}

tasks.withType<JavaCompile> {
    options.encoding = "UTF-8"
    dependsOn(tasks.named("generateGrammarSource"))
}

tasks.named("compileJava") {
    dependsOn(tasks.named("generateGrammarSource"))
}