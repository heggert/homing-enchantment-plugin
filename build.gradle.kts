plugins {
    `java`
}

java {
    toolchain.languageVersion.set(JavaLanguageVersion.of(21))
}

repositories { 
    mavenCentral() 
    maven {
        name = "papermc"
        url = uri("https://repo.papermc.io/repository/maven-public/")
    }    
}

dependencies {
    compileOnly("io.papermc.paper:paper-api:1.21.4-R0.1-SNAPSHOT")
}

// Set group and version dynamically
group = "art.chibi"
version = "dev"

// Ensure the JAR uses the project version
tasks.jar {
    archiveBaseName.set("HomingEnchantmentPlugin")
    archiveVersion.set(project.version.toString())
}

