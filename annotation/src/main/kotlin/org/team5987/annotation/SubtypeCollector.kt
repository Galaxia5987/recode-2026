package org.team5987.annotation

import com.google.devtools.ksp.getAllSuperTypes
import com.google.devtools.ksp.processing.Resolver
import com.google.devtools.ksp.symbol.KSClassDeclaration
import com.google.devtools.ksp.symbol.KSDeclaration
import com.google.devtools.ksp.symbol.KSFile
import com.google.devtools.ksp.symbol.Modifier
import com.squareup.kotlinpoet.ClassName
import com.squareup.kotlinpoet.ksp.toClassName

/**
 * Collects, across processing rounds, every public class that extends
 * [superTypeName] and matches [filter].
 */
class SubtypeCollector(
    private val superTypeName: String,
    private val filter: (KSClassDeclaration) -> Boolean = { true },
) {
    // Store ClassName instead of KSClassDeclaration to avoid lifetime
    // exceptions
    val classNames = linkedSetOf<ClassName>()

    // Files that declare a collected class. Outputs built from this collector
    // should depend on them so that, in incremental builds, KSP hands them
    // back even when only some other file changed (getAllFiles() only returns
    // dirty files otherwise).
    val sourceFiles = linkedSetOf<KSFile>()

    // Classes whose supertypes could not be resolved yet (e.g. they extend a
    // type generated in this round). Retried in later rounds.
    private val pending = mutableSetOf<String>()

    fun collect(resolver: Resolver) {
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
    }

    private fun Sequence<KSDeclaration>.classDeclarations():
        Sequence<KSClassDeclaration> =
        filterIsInstance<KSClassDeclaration>().flatMap {
            sequenceOf(it) + it.declarations.classDeclarations()
        }

    private fun check(classDeclaration: KSClassDeclaration) {
        // Generated code can't reference private or local classes.
        if (
            Modifier.PRIVATE in classDeclaration.modifiers ||
                classDeclaration.qualifiedName == null ||
                !filter(classDeclaration)
        )
            return
        val superTypes = classDeclaration.getAllSuperTypes().toList()
        if (
            superTypes.any {
                it.declaration.qualifiedName?.asString() == superTypeName
            }
        ) {
            classNames += classDeclaration.toClassName()
            classDeclaration.containingFile?.let { sourceFiles += it }
        } else if (superTypes.any { it.isError }) {
            classDeclaration.qualifiedName?.asString()?.let { pending += it }
        }
    }
}
