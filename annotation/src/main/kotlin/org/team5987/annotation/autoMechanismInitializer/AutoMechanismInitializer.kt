package org.team5987.annotation.autoMechanismInitializer

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
import kotlin.reflect.KClass
import org.team5987.annotation.SubtypeCollector

class MechanismProcessor(
    private val codeGenerator: CodeGenerator,
    private val logger: KSPLogger,
) : SymbolProcessor {

    private val mechanisms = SubtypeCollector("org.wpilib.command3.Mechanism")

    override fun process(resolver: Resolver): List<KSAnnotated> {
        mechanisms.collect(resolver)
        return emptyList()
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

        mechanisms.classNames.forEach { className ->
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
            Dependencies(
                aggregating = true,
                *mechanisms.sourceFiles.toTypedArray(),
            )

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
