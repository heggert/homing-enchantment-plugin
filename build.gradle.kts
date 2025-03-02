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

// Optionally set a group and version
group = "art.chibi"

version = "1.0.1"

// If you want to build a jar with a specific name, you can customize:
tasks.jar {
    archiveBaseName.set("HomingEnchantmentPlugin")
    archiveVersion.set("1.0.1")
}
