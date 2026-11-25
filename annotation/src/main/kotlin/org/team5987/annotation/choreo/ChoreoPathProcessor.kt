package org.team5987.annotation.choreo

import com.google.devtools.ksp.processing.CodeGenerator
import com.google.devtools.ksp.processing.Dependencies
import com.google.devtools.ksp.processing.Resolver
import com.google.devtools.ksp.processing.SymbolProcessor
import com.google.devtools.ksp.processing.SymbolProcessorEnvironment
import com.google.devtools.ksp.processing.SymbolProcessorProvider
import com.google.devtools.ksp.symbol.ClassKind
import com.google.devtools.ksp.symbol.KSAnnotated
import com.google.gson.JsonParser
import com.squareup.kotlinpoet.ClassName
import com.squareup.kotlinpoet.CodeBlock
import com.squareup.kotlinpoet.FileSpec
import com.squareup.kotlinpoet.FunSpec
import com.squareup.kotlinpoet.KModifier
import com.squareup.kotlinpoet.ParameterizedTypeName.Companion.parameterizedBy
import com.squareup.kotlinpoet.PropertySpec
import com.squareup.kotlinpoet.STRING
import com.squareup.kotlinpoet.TypeSpec
import com.squareup.kotlinpoet.asTypeName
import com.squareup.kotlinpoet.joinToCode
import java.io.File
import org.team5987.annotation.SubtypeCollector

const val CHOREO_DIR_OPTION = "choreoDir"

private const val GENERATED_PACKAGE = "frc.robot.autonomous.paths"
private val choreoPathClass = ClassName("frc.robot.autonomous", "ChoreoPath")
private val coroutineClass = ClassName("org.wpilib.command3", "Coroutine")

private data class TrajectoryFile(val name: String, val events: Set<String>)

/**
 * Generates an abstract [choreoPathClass] subclass for every Choreo `.traj`
 * file, with one abstract function per event marker, and a `ChoreoPaths`
 * registry of every object that implements one of them.
 */
class ChoreoPathProcessor(
    private val codeGenerator: CodeGenerator,
    choreoDir: File,
) : SymbolProcessor {

    private val trajectories =
        choreoDir
            .listFiles { file -> file.extension == "traj" }
            .orEmpty()
            .sortedBy { it.name }
            .map(::parseTrajectory)

    private val implementations =
        SubtypeCollector(choreoPathClass.canonicalName) {
            it.classKind == ClassKind.OBJECT
        }

    private var generatedPathClasses = false

    override fun process(resolver: Resolver): List<KSAnnotated> {
        if (!generatedPathClasses) {
            trajectories.forEach(::generatePathClass)
            generatedPathClasses = true
        }
        implementations.collect(resolver)
        return emptyList()
    }

    override fun finish() {
        generateRegistry()
    }

    private fun parseTrajectory(file: File): TrajectoryFile {
        val json = JsonParser.parseString(file.readText()).asJsonObject
        val events =
            json
                .getAsJsonArray("events")
                .map { it.asJsonObject.get("name").asString }
                .filter { it.isNotBlank() }
                .toSortedSet()
        return TrajectoryFile(file.nameWithoutExtension, events)
    }

    private fun generatePathClass(trajectory: TrajectoryFile) {
        val className = trajectory.name.toPascalCase() + "Path"
        val eventFunctions =
            trajectory.events.associateWith {
                FunSpec.builder(it.toCamelCase())
                    .addModifiers(KModifier.PROTECTED, KModifier.ABSTRACT)
                    .receiver(coroutineClass)
                    .build()
            }

        val runEvent =
            FunSpec.builder("runEvent")
                .addModifiers(KModifier.FINAL, KModifier.OVERRIDE)
                .receiver(coroutineClass)
                .addParameter("event", STRING)
                .beginControlFlow("when (event)")
                .apply {
                    eventFunctions.forEach { (event, function) ->
                        addStatement("%S -> %N()", event, function)
                    }
                }
                .addStatement(
                    "else -> throw IllegalArgumentException(%P)",
                    "Unknown event '\$event' in ${trajectory.name}",
                )
                .endControlFlow()
                .build()

        val pathClass =
            TypeSpec.classBuilder(className)
                .addModifiers(KModifier.ABSTRACT)
                .superclass(choreoPathClass)
                .addSuperclassConstructorParameter("%S", trajectory.name)
                .addFunctions(eventFunctions.values)
                .addFunction(runEvent)
                .build()

        FileSpec.builder(GENERATED_PACKAGE, className)
            .addType(pathClass)
            .build()
            .writeTo(Dependencies(aggregating = false))
    }

    private fun generateRegistry() {
        val listType = List::class.asTypeName().parameterizedBy(choreoPathClass)
        val all =
            PropertySpec.builder("all", listType)
                .initializer(
                    implementations.classNames
                        .map { CodeBlock.of("%T", it) }
                        .joinToCode(prefix = "listOf(", suffix = ")")
                )
                .build()

        FileSpec.builder(GENERATED_PACKAGE, "ChoreoPaths")
            .addType(
                TypeSpec.objectBuilder("ChoreoPaths").addProperty(all).build()
            )
            .build()
            .writeTo(
                Dependencies(
                    aggregating = true,
                    *implementations.sourceFiles.toTypedArray(),
                )
            )
    }

    private fun FileSpec.writeTo(dependencies: Dependencies) {
        codeGenerator
            .createNewFile(dependencies, packageName, name)
            .writer()
            .use(::writeTo)
    }
}

private fun String.words(): List<String> =
    split(Regex("[^A-Za-z0-9]+")).filter { it.isNotEmpty() }

private fun String.toPascalCase(): String =
    words().joinToString("") { it.replaceFirstChar(Char::uppercaseChar) }

private fun String.toCamelCase(): String =
    toPascalCase().replaceFirstChar(Char::lowercaseChar)

class ChoreoPathProcessorProvider : SymbolProcessorProvider {
    override fun create(
        environment: SymbolProcessorEnvironment
    ): SymbolProcessor {
        val choreoDir =
            environment.options[CHOREO_DIR_OPTION]
                ?: throw IllegalArgumentException(
                    "Missing $CHOREO_DIR_OPTION KSP argument"
                )
        return ChoreoPathProcessor(environment.codeGenerator, File(choreoDir))
    }
}
