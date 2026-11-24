package org.team5987.annotation

import com.google.devtools.ksp.getAllSuperTypes
import com.google.devtools.ksp.getDeclaredProperties
import com.google.devtools.ksp.processing.*
import com.google.devtools.ksp.symbol.*
import com.squareup.kotlinpoet.*
import com.squareup.kotlinpoet.ksp.toClassName
import com.squareup.kotlinpoet.ksp.writeTo

// Call `LoggedRegistry.registerAll()` on robot init!!!!!!

private const val GENERATED_PACKAGE = "frc.robot.lib.logged_output.generated"
private const val GENERATED_FILE = "LoggedRegistry"
private const val ANNOTATION_NAME = "org.team5987.annotation.LoggedOutput"
private val MECHANISM_BASE_TYPES =
    setOf(
        "org.wpilib.command3.Mechanism",
        "edu.wpi.first.wpilibj2.command.SubsystemBase",
    )

class LoggedOutputProcessor(
    private val codeGenerator: CodeGenerator,
    private val logger: KSPLogger,
) : SymbolProcessor {
    private var generated = false

    override fun process(resolver: Resolver): List<KSAnnotated> {
        val symbols =
            resolver.getSymbolsWithAnnotation(ANNOTATION_NAME).toList()

        // The registry is a single file, so it can only be written once. It is
        // written in the first round; symbols in later rounds can only come
        // from generated code.
        if (generated) {
            if (symbols.isNotEmpty())
                logger.warn(
                    "@LoggedOutput on generated code is not supported; ignoring ${symbols.size} symbol(s)."
                )
            return emptyList()
        }
        generated = true

        logger.info("Found ${symbols.size} @LoggedOutput symbols.")

        val funSpecBuilder =
            FunSpec.builder("registerAllLoggedOutputs")
                .addKdoc(
                    "Auto-generated. Safe to call even if no @LoggedOutput symbols were found.\n"
                )
                .addStatement(
                    "// [LoggedOutputManager] registers all LoggedOutput fields and methods"
                )

        for (symbol in symbols) registerSymbol(symbol, funSpecBuilder)

        val fileSpec =
            FileSpec.builder(GENERATED_PACKAGE, GENERATED_FILE)
                .addImport(
                    "frc.robot.lib.logged_output",
                    "LoggedOutputManager",
                )
                .addFunction(funSpecBuilder.build())
                .build()

        // Depend only on the files that contain @LoggedOutput. Using
        // Dependencies.ALL_FILES would make every source file dirty on every
        // change and disable incremental processing for all processors.
        val sources =
            symbols
                .mapNotNull { (it as? KSDeclaration)?.containingFile }
                .distinct()
                .toTypedArray()
        fileSpec.writeTo(codeGenerator, Dependencies(true, *sources))

        logger.info(
            "LoggedOutputProcessor finished generating $GENERATED_FILE.kt"
        )
        return emptyList()
    }

    private fun registerSymbol(symbol: KSAnnotated, fn: FunSpec.Builder) {
        val args =
            symbol.annotations
                .first { it.shortName.asString() == "LoggedOutput" }
                .arguments
        fun arg(name: String) =
            args.firstOrNull { it.name?.asString() == name }?.value?.toString()

        val key = arg("key").orEmpty()
        var path = arg("path").orEmpty()
        val logLevel = arg("level")?.substringAfterLast(".") ?: "COMP"

        when (symbol) {
            is KSPropertyDeclaration -> {
                val name = symbol.simpleName.asString()
                val owner = symbol.parentDeclaration as? KSClassDeclaration
                if (owner == null) {
                    val pkg = symbol.packageName.asString()
                    if (path.isEmpty() && key.isBlank()) path = name
                    fn.addStatement(
                        "LoggedOutputManager.registerField(%S, %T.%L, ::%M, %S)",
                        key,
                        LogLevel::class,
                        logLevel,
                        MemberName(pkg, name),
                        path,
                    )
                    return
                }
                if (!requireObject(owner, symbol)) return
                val ownerName = owner.simpleName.asString()
                if (path.isEmpty() && isMechanism(owner))
                    path = "Subsystems/$ownerName"
                if (path.isEmpty() && key.isBlank()) path = ownerName
                fn.addStatement(
                    "LoggedOutputManager.registerField(%S, %T.%L, %T::%L, %S)",
                    key,
                    LogLevel::class,
                    logLevel,
                    owner.toClassName(),
                    name,
                    path,
                )
            }

            is KSFunctionDeclaration -> {
                val name = symbol.simpleName.asString()
                val owner = symbol.parentDeclaration as? KSClassDeclaration
                if (owner == null) {
                    val pkg = symbol.packageName.asString()
                    if (path.isEmpty() && key.isBlank()) path = name
                    fn.addStatement(
                        "LoggedOutputManager.registerMethod(%S, %T.%L, ::%M, %S)",
                        key,
                        LogLevel::class,
                        logLevel,
                        MemberName(pkg, name),
                        path,
                    )
                    return
                }
                if (!requireObject(owner, symbol)) return
                val ownerName = owner.simpleName.asString()
                if (path.isEmpty() && isMechanism(owner))
                    path = "Subsystems/$ownerName"
                if (path.isEmpty() && key.isBlank()) path = ownerName
                fn.addStatement(
                    "LoggedOutputManager.registerMethod(%S, %T.%L, %T::%L, %S)",
                    key,
                    LogLevel::class,
                    logLevel,
                    owner.toClassName(),
                    name,
                    path,
                )
            }

            is KSClassDeclaration -> {
                if (!requireObject(symbol, symbol)) return
                // Every property is logged under "<path>/<propertyName>".
                val basePath =
                    listOf(path, key)
                        .filter { it.isNotBlank() }
                        .joinToString("/")
                        .ifBlank { symbol.simpleName.asString() }
                val classType = symbol.toClassName()
                symbol.getDeclaredProperties().forEach { property ->
                    fn.addStatement(
                        "LoggedOutputManager.registerField(%S, %T.%L, %T::%L, %S)",
                        "",
                        LogLevel::class,
                        logLevel,
                        classType,
                        property.simpleName.asString(),
                        basePath,
                    )
                }
            }

            else ->
                logger.error(
                    "@LoggedOutput is not supported on this element",
                    symbol,
                )
        }
    }

    /**
     * Generated code references members as `Owner::member`, which is only a
     * zero-argument reference when `Owner` is an object.
     */
    private fun requireObject(
        owner: KSClassDeclaration,
        symbol: KSNode,
    ): Boolean {
        if (owner.classKind == ClassKind.OBJECT) return true
        logger.error(
            "@LoggedOutput members must be top-level or inside an object (${owner.simpleName.asString()} is a ${owner.classKind})",
            symbol,
        )
        return false
    }

    private fun isMechanism(owner: KSClassDeclaration) =
        owner.getAllSuperTypes().any {
            it.declaration.qualifiedName?.asString() in MECHANISM_BASE_TYPES
        }
}

class LoggedOutputProcessorProvider : SymbolProcessorProvider {
    override fun create(
        environment: SymbolProcessorEnvironment
    ): SymbolProcessor {
        return LoggedOutputProcessor(
            environment.codeGenerator,
            environment.logger,
        )
    }
}
