package org.team5987.annotation.autoMechanismInitializer

import com.google.devtools.ksp.getAllSuperTypes
import com.google.devtools.ksp.processing.*
import com.google.devtools.ksp.symbol.*
import com.google.devtools.ksp.validate
import com.squareup.kotlinpoet.ClassName
import com.squareup.kotlinpoet.CodeBlock
import com.squareup.kotlinpoet.FileSpec
import com.squareup.kotlinpoet.ParameterizedTypeName.Companion.parameterizedBy
import com.squareup.kotlinpoet.PropertySpec
import com.squareup.kotlinpoet.TypeSpec
import com.squareup.kotlinpoet.WildcardTypeName
import com.squareup.kotlinpoet.asClassName
import com.squareup.kotlinpoet.ksp.toClassName
import kotlin.reflect.KClass

class MechanismProcessor(
    private val codeGenerator: CodeGenerator,
    private val logger: KSPLogger
) : SymbolProcessor {

    private val targetBaseClassName = "org.wpilib.command3.Mechanism"
    // Store ClassName instead of KSClassDeclaration to avoid lifetime exceptions
    private val collectedClassNames = mutableSetOf<ClassName>()

    override fun process(resolver: Resolver): List<KSAnnotated> {
        val newFiles = resolver.getAllFiles()

        newFiles.forEach { file ->
            file.declarations
                .filterIsInstance<KSClassDeclaration>()
                .filter { it.validate() }
                .filter { isTargetSubclass(it) }
                .forEach { classDeclaration ->
                    collectedClassNames += classDeclaration.toClassName()
                }
        }

        return emptyList()
    }

    private fun isTargetSubclass(classDeclaration: KSClassDeclaration): Boolean {
        return classDeclaration.getAllSuperTypes().any { type ->
            type.declaration.qualifiedName?.asString() == targetBaseClassName
        }
    }

    override fun finish() {
        if (collectedClassNames.isEmpty()) return

        generateRegistryFile()
    }

    private fun generateRegistryFile() {
        val packageName = "frc.robot.lib"
        val fileName = "MechanismRegistry"
        val baseClassClassName = ClassName("org.wpilib.command3", "Mechanism")

        val listType = List::class.asClassName().parameterizedBy(
            KClass::class.asClassName().parameterizedBy(
                WildcardTypeName.producerOf(baseClassClassName)
            )
        )

        val listBuilder = CodeBlock.builder()
        listBuilder.add("listOf(\n")
        listBuilder.indent()

        collectedClassNames.forEach { className ->
            listBuilder.add("%T::class,\n", className)
        }

        listBuilder.unindent()
        listBuilder.add(")")

        val registryObject = TypeSpec.objectBuilder(fileName)
            .addProperty(
                PropertySpec.builder("allMechanisms", listType)
                    .initializer(listBuilder.build())
                    .build()
            )
            .build()

        val fileSpec = FileSpec.builder(packageName, fileName)
            .addType(registryObject)
            .build()

        // Aggregating dependencies without explicit sources
        val dependencies = Dependencies(aggregating = true)

        codeGenerator.createNewFile(
            dependencies = dependencies,
            packageName = packageName,
            fileName = fileName
        ).writer().use { writer ->
            fileSpec.writeTo(writer)
        }
    }
}