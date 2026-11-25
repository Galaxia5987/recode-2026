package org.team5987.annotation.choreo

import com.google.devtools.ksp.processing.CodeGenerator
import com.google.devtools.ksp.processing.Dependencies
import com.google.devtools.ksp.processing.Resolver
import com.google.devtools.ksp.processing.SymbolProcessor
import com.google.devtools.ksp.processing.SymbolProcessorEnvironment
import com.google.devtools.ksp.processing.SymbolProcessorProvider
import com.google.devtools.ksp.symbol.KSAnnotated
import com.google.gson.JsonParser
import com.squareup.kotlinpoet.ClassName
import com.squareup.kotlinpoet.FileSpec
import com.squareup.kotlinpoet.FunSpec
import com.squareup.kotlinpoet.KModifier
import com.squareup.kotlinpoet.STRING
import com.squareup.kotlinpoet.TypeSpec
import java.io.File

const val CHOREO_DIR_OPTION = "choreoDir"

private const val GENERATED_PACKAGE = "frc.robot.autonomous.paths"
private const val MARKERS_CLASS = "ChoreoMarkers"
private val choreoPathClass = ClassName("frc.robot.autonomous", "ChoreoPath")
private val markerBindingsClass =
    ClassName("frc.robot.autonomous", "MarkerBindings")
private val markersObject = ClassName("frc.robot.autonomous", "Markers")
private val coroutineClass = ClassName("org.wpilib.command3", "Coroutine")

private data class TrajectoryFile(val name: String, val events: Set<String>)

/**
 * Generates an object for every Choreo `.traj` file, and an abstract
 * `ChoreoMarkers` class with one abstract function per event marker name across
 * all of them, which the `Markers` object implements.
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

    private var generated = false

    override fun process(resolver: Resolver): List<KSAnnotated> {
        if (!generated) {
            generateMarkers()
            generatePaths()
            generated = true
        }
        return emptyList()
    }

    private fun parseTrajectory(file: File): TrajectoryFile {
        val events =
            try {
                JsonParser.parseString(file.readText())
                    .asJsonObject
                    .getAsJsonArray("events")
                    .map { it.asJsonObject.get("name").asString }
                    .filter { it.isNotBlank() }
                    .toSortedSet()
            } catch (e: RuntimeException) {
                throw IllegalArgumentException(
                    "Invalid Choreo trajectory ${file.name}",
                    e,
                )
            }
        return TrajectoryFile(file.nameWithoutExtension, events)
    }

    private fun generateMarkers() {
        val markerFunctions =
            identifiers(
                    trajectories.flatMap { it.events }.toSortedSet(),
                    "marker",
                    String::toCamelCase,
                )
                .mapValues { (_, functionName) ->
                    FunSpec.builder(functionName)
                        .addModifiers(KModifier.PROTECTED, KModifier.ABSTRACT)
                        .receiver(coroutineClass)
                        .build()
                }

        val runMarker =
            FunSpec.builder("runMarker")
                .addModifiers(KModifier.FINAL, KModifier.OVERRIDE)
                .receiver(coroutineClass)
                .addParameter("marker", STRING)
                .beginControlFlow("when (marker)")
                .apply {
                    markerFunctions.forEach { (marker, function) ->
                        addStatement("%S -> %N()", marker, function)
                    }
                }
                .addStatement(
                    "else -> throw IllegalArgumentException(%P)",
                    "Unknown Choreo marker '\$marker'",
                )
                .endControlFlow()
                .build()

        val markersClass =
            TypeSpec.classBuilder(MARKERS_CLASS)
                .addKdoc(
                    "The bindings of every Choreo event marker, implemented by %T.",
                    markersObject,
                )
                .addModifiers(KModifier.ABSTRACT)
                .addSuperinterface(markerBindingsClass)
                .addFunctions(markerFunctions.values)
                .addFunction(runMarker)
                .build()

        FileSpec.builder(GENERATED_PACKAGE, MARKERS_CLASS)
            .addType(markersClass)
            .build()
            .writeTo(Dependencies(aggregating = false))
    }

    private fun generatePaths() {
        val pathObjects =
            identifiers(
                    trajectories.map { it.name },
                    "trajectory",
                    String::toPascalCase,
                )
                .map { (trajectory, objectName) ->
                    TypeSpec.objectBuilder(objectName)
                        .superclass(choreoPathClass)
                        .addSuperclassConstructorParameter("%S", trajectory)
                        .addSuperclassConstructorParameter("%T", markersObject)
                        .build()
                }

        FileSpec.builder(GENERATED_PACKAGE, "ChoreoPaths")
            .apply { pathObjects.forEach(::addType) }
            .build()
            .writeTo(Dependencies(aggregating = false))
    }

    /**
     * Maps each of [names] to the identifier [convert] makes of it, failing
     * when one isn't valid or several map to the same identifier.
     */
    private fun identifiers(
        names: Collection<String>,
        kind: String,
        convert: (String) -> String,
    ): Map<String, String> {
        val identifiers = names.associateWith(convert)
        identifiers.forEach { (name, identifier) ->
            require(identifier.isIdentifier()) {
                "Choreo $kind '$name' doesn't form a valid identifier"
            }
        }
        identifiers.entries
            .groupBy({ it.value }, { it.key })
            .filterValues { it.size > 1 }
            .forEach { (identifier, clashing) ->
                throw IllegalArgumentException(
                    "Choreo ${kind}s ${clashing.joinToString { "'$it'" }} " +
                        "all map to $identifier"
                )
            }
        return identifiers
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

private fun String.isIdentifier(): Boolean = firstOrNull()?.isLetter() == true

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
