package org.team5987.annotation.autoMechanismInitializer

import com.google.devtools.ksp.getAllSuperTypes
import com.google.devtools.ksp.processing.*
import com.google.devtools.ksp.symbol.*
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
    private val logger: KSPLogger,
) : SymbolProcessor {

    private val targetBaseClassName = "org.wpilib.command3.Mechanism"

    // Store ClassName instead of KSClassDeclaration to avoid lifetime
    // exceptions
    private val collectedClassNames = linkedSetOf<ClassName>()

    // Files that declare a mechanism. The registry depends on them so that,
    // in incremental builds, KSP hands them back to us even when only some
    // other file changed (getAllFiles() only returns dirty files otherwise).
    private val sourceFiles = linkedSetOf<KSFile>()

    // Classes whose supertypes could not be resolved yet (e.g. they extend a
    // type generated in this round). Retried in later rounds.
    private val pending = mutableSetOf<String>()

    override fun process(resolver: Resolver): List<KSAnnotated> {
        resolver.getAllFiles().forEach { file ->
            file.declarations.classDeclarations().forEach { check(it) }
        }

        val retry = pending.toList()
        pending.clear()
        retry.forEach { name ->
            resolver
                .getClassDeclarationByName(resolver.getKSNameFromString(name))
                ?.let { check(it) }
        }

        return emptyList()
    }

    private fun Sequence<KSDeclaration>.classDeclarations():
        Sequence<KSClassDeclaration> =
        filterIsInstance<KSClassDeclaration>().flatMap {
            sequenceOf(it) + it.declarations.classDeclarations()
        }

    private fun check(classDeclaration: KSClassDeclaration) {
        // The registry can't reference private or local classes.
        if (
            Modifier.PRIVATE in classDeclaration.modifiers ||
                classDeclaration.qualifiedName == null
        )
            return
        val superTypes = classDeclaration.getAllSuperTypes().toList()
        if (
            superTypes.any {
                it.declaration.qualifiedName?.asString() == targetBaseClassName
            }
        ) {
            collectedClassNames += classDeclaration.toClassName()
            classDeclaration.containingFile?.let { sourceFiles += it }
        } else if (superTypes.any { it.isError }) {
            classDeclaration.qualifiedName?.asString()?.let { pending += it }
        }
    }

    override fun finish() {
        // Always generate the registry, even when it is empty: code
        // references it unconditionally.
        generateRegistryFile()
    }

    private fun generateRegistryFile() {
        val packageName = "frc.robot.lib"
        val fileName = "MechanismRegistry"
        val baseClassClassName = ClassName("org.wpilib.command3", "Mechanism")

        val listType =
            List::class.asClassName()
                .parameterizedBy(
                    KClass::class.asClassName()
                        .parameterizedBy(
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

        val registryObject =
            TypeSpec.objectBuilder(fileName)
                .addProperty(
                    PropertySpec.builder("allMechanisms", listType)
                        .initializer(listBuilder.build())
                        .build()
                )
                .build()

        val fileSpec =
            FileSpec.builder(packageName, fileName)
                .addType(registryObject)
                .build()

        val dependencies =
            Dependencies(aggregating = true, *sourceFiles.toTypedArray())

        codeGenerator
            .createNewFile(
                dependencies = dependencies,
                packageName = packageName,
                fileName = fileName,
            )
            .writer()
            .use { writer -> fileSpec.writeTo(writer) }
    }
}
