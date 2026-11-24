package org.team5987.annotation

import com.google.devtools.ksp.getAllSuperTypes
import com.google.devtools.ksp.getDeclaredProperties
import com.google.devtools.ksp.processing.*
import com.google.devtools.ksp.symbol.*
import com.squareup.kotlinpoet.*
import com.squareup.kotlinpoet.ParameterizedTypeName.Companion.parameterizedBy
import com.squareup.kotlinpoet.ksp.toClassName
import com.squareup.kotlinpoet.ksp.toTypeName
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
    private val registered = mutableSetOf<String>()
    private lateinit var resolver: Resolver

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
        this.resolver = resolver

        logger.info("Found ${symbols.size} @LoggedOutput symbols.")

        val funSpecBuilder =
            FunSpec.builder("registerAllLoggedOutputs")
                .addKdoc(
                    "Auto-generated. Safe to call even if no @LoggedOutput symbols were found.\n"
                )
                .addStatement(
                    "// [LoggedOutputManager] registers all LoggedOutput fields and methods"
                )

        // Member annotations win over an enclosing annotated object, so
        // register classes last.
        symbols
            .sortedBy { it.target() is KSClassDeclaration }
            .forEach { registerSymbol(it, funSpecBuilder) }

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
                .mapNotNull { (it.target() as? KSDeclaration)?.containingFile }
                .distinct()
                .toTypedArray()
        fileSpec.writeTo(codeGenerator, Dependencies(true, *sources))

        logger.info(
            "LoggedOutputProcessor finished generating $GENERATED_FILE.kt"
        )
        return emptyList()
    }

    private class Args(val key: String, val path: String, val level: String)

    private fun KSAnnotated.loggedOutputArgs(): Args {
        val args =
            annotations
                .first { it.shortName.asString() == "LoggedOutput" }
                .arguments
        fun arg(name: String) =
            args.firstOrNull { it.name?.asString() == name }?.value?.toString()
        return Args(
            key = arg("key").orEmpty(),
            path = arg("path").orEmpty(),
            level = arg("level")?.substringAfterLast(".") ?: "COMP",
        )
    }

    /**
     * Supports every target @LoggedOutput allows:
     * - properties and fields (`val`, `var`, `const val`, `@field:`),
     * - property getters (`@get:` or an annotation on `get()`), registered as
     *   their property,
     * - zero-argument functions,
     * - objects (or Java classes with static fields), registering each of their
     *   properties.
     *
     * Members must be top-level, inside an object or companion object, or Java
     * statics, so the generated `::member` / `Owner::member` reference needs no
     * instance.
     */
    private fun registerSymbol(symbol: KSAnnotated, fn: FunSpec.Builder) {
        val args = symbol.loggedOutputArgs()
        when (val target = symbol.target()) {
            is KSPropertyDeclaration,
            is KSFunctionDeclaration -> registerMember(target, args, symbol, fn)
            is KSClassDeclaration -> registerClass(target, args, fn)
            else ->
                logger.error(
                    "@LoggedOutput is not supported on this element",
                    symbol,
                )
        }
    }

    private fun registerMember(
        member: KSDeclaration,
        args: Args,
        errorNode: KSNode,
        fn: FunSpec.Builder,
    ) {
        if (!checkMember(member, errorNode)) return
        val qualifiedName = member.qualifiedName?.asString() ?: return
        // Annotating both a property and its getter (or a member of an
        // annotated object) must not log it twice.
        if (!registered.add(qualifiedName)) return

        val isFunction = member is KSFunctionDeclaration
        val register = if (isFunction) "registerMethod" else "registerField"
        val name = member.simpleName.asString()
        val owner = member.parentDeclaration as? KSClassDeclaration

        var path = args.path
        if (owner != null && path.isEmpty() && isMechanism(owner))
            path = "Subsystems/${owner.displayName()}"
        if (path.isEmpty() && args.key.isBlank())
            path = owner?.displayName() ?: name

        val reference =
            if (owner == null)
                CodeBlock.of(
                    "::%M",
                    MemberName(member.packageName.asString(), name),
                )
            else CodeBlock.of("%T::%L", owner.toClassName(), name)

        // `::f` is ambiguous when `f` is overloaded; an expected type picks
        // the zero-argument overload.
        val callable =
            if (
                member is KSFunctionDeclaration && isOverloaded(member, owner)
            ) {
                val returnType = member.returnType?.toTypeName() ?: UNIT
                val type =
                    ClassName("kotlin.reflect", "KFunction0")
                        .parameterizedBy(returnType)
                CodeBlock.of("run<%T> { %L }", type, reference)
            } else reference

        fn.addStatement(
            "LoggedOutputManager.$register(%S, %T.%L, %L, %S)",
            args.key,
            LogLevel::class,
            args.level,
            callable,
            path,
        )
    }

    private fun registerClass(
        cls: KSClassDeclaration,
        args: Args,
        fn: FunSpec.Builder,
    ) {
        val isJava = cls.origin == Origin.JAVA || cls.origin == Origin.JAVA_LIB
        if (cls.classKind != ClassKind.OBJECT && !isJava) {
            logger.error(
                "@LoggedOutput on a class requires an object (${cls.simpleName.asString()} is a ${cls.classKind})",
                cls,
            )
            return
        }
        // Every property is logged under "<path>/<propertyName>".
        val basePath =
            listOf(args.path, args.key)
                .filter { it.isNotBlank() }
                .joinToString("/")
                .ifBlank { cls.displayName() }
        cls.getDeclaredProperties()
            .filter {
                Modifier.PRIVATE !in it.modifiers &&
                    it.extensionReceiver == null &&
                    (cls.classKind == ClassKind.OBJECT ||
                        Modifier.JAVA_STATIC in it.modifiers)
            }
            .forEach { property ->
                registerMember(
                    property,
                    Args("", basePath, args.level),
                    property,
                    fn,
                )
            }
    }

    /** Reports why a member can't be referenced from generated code. */
    private fun checkMember(member: KSDeclaration, errorNode: KSNode): Boolean {
        fun fail(reason: String): Boolean {
            logger.error(
                "@LoggedOutput on ${member.simpleName.asString()}: $reason",
                errorNode,
            )
            return false
        }
        val modifiers = member.modifiers
        if (Modifier.PRIVATE in modifiers || Modifier.PROTECTED in modifiers)
            return fail(
                "private or protected members can't be referenced from generated code; make it internal or public"
            )
        val owner = member.parentDeclaration as? KSClassDeclaration
        if (
            owner != null &&
                owner.classKind != ClassKind.OBJECT &&
                Modifier.JAVA_STATIC !in modifiers
        )
            return fail(
                "members must be top-level, inside an object or companion object, or Java static (${owner.simpleName.asString()} is a ${owner.classKind})"
            )
        when (member) {
            is KSPropertyDeclaration ->
                if (member.extensionReceiver != null)
                    return fail("extension properties are not supported")
            is KSFunctionDeclaration -> {
                if (member.extensionReceiver != null)
                    return fail("extension functions are not supported")
                if (member.parameters.isNotEmpty())
                    return fail("only zero-argument functions are supported")
                if (member.typeParameters.isNotEmpty())
                    return fail("generic functions are not supported")
                if (Modifier.SUSPEND in modifiers)
                    return fail("suspend functions are not supported")
                val returnType =
                    member.returnType?.resolve()?.declaration?.qualifiedName
                if (returnType?.asString() == "kotlin.Unit")
                    return fail(
                        "the function returns Unit, so there is nothing to log"
                    )
            }
        }
        return true
    }

    private fun isOverloaded(
        function: KSFunctionDeclaration,
        owner: KSClassDeclaration?,
    ): Boolean {
        val name = function.simpleName.asString()
        val count =
            if (owner != null)
                owner.declarations
                    .filterIsInstance<KSFunctionDeclaration>()
                    .count { it.simpleName.asString() == name }
            else
                function.qualifiedName
                    ?.let {
                        resolver.getFunctionDeclarationsByName(
                            it,
                            includeTopLevel = true,
                        )
                    }
                    ?.count() ?: 1
        return count > 1
    }

    /** A companion object is named after the class that contains it. */
    private fun KSClassDeclaration.displayName(): String =
        (parentDeclaration as? KSClassDeclaration)
            ?.takeIf { isCompanionObject }
            ?.displayName() ?: simpleName.asString()

    /** The declaration a symbol is registered as. */
    private fun KSAnnotated.target(): KSAnnotated =
        (this as? KSPropertyGetter)?.receiver ?: this

    private fun isMechanism(owner: KSClassDeclaration): Boolean {
        // A companion's logs belong to the class that contains it.
        val cls =
            (owner.parentDeclaration as? KSClassDeclaration)?.takeIf {
                owner.isCompanionObject
            } ?: owner
        return cls.getAllSuperTypes().any {
            it.declaration.qualifiedName?.asString() in MECHANISM_BASE_TYPES
        }
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
