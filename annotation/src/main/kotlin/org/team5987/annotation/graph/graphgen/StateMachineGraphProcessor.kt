@file:OptIn(CompilerConfiguration.Internals::class, K1Deprecation::class)
package org.team5987.annotation.graph.graphgen

import com.google.devtools.ksp.processing.*
import com.google.devtools.ksp.symbol.KSAnnotated
import com.google.devtools.ksp.symbol.KSPropertyDeclaration
import org.jetbrains.kotlin.K1Deprecation
import org.jetbrains.kotlin.cli.common.messages.MessageCollector
import org.jetbrains.kotlin.cli.jvm.compiler.EnvironmentConfigFiles
import org.jetbrains.kotlin.cli.jvm.compiler.KotlinCoreEnvironment
import org.jetbrains.kotlin.com.intellij.openapi.util.Disposer
import org.jetbrains.kotlin.config.CommonConfigurationKeys
import org.jetbrains.kotlin.config.CompilerConfiguration
import org.jetbrains.kotlin.psi.*
import java.io.OutputStream

class StateMachineGraphProcessor(
    private val codeGenerator: CodeGenerator,
    private val logger: KSPLogger
) : SymbolProcessor {

    // Initialize headless environment for PSI parsing
    private val project by lazy {
        val configuration = CompilerConfiguration().apply {
            put(CommonConfigurationKeys.MESSAGE_COLLECTOR_KEY, MessageCollector.NONE)
        }
        KotlinCoreEnvironment.createForProduction(
            Disposer.newDisposable(),
            configuration,
            EnvironmentConfigFiles.JVM_CONFIG_FILES
        ).project
    }

    override fun process(resolver: Resolver): List<KSAnnotated> {
        val symbols = resolver.getSymbolsWithAnnotation("GenerateStateMachineGraph")

        symbols.filterIsInstance<KSPropertyDeclaration>().forEach { property ->
            generateGraphForProperty(property)
        }

        return emptyList()
    }

    private fun generateGraphForProperty(property: KSPropertyDeclaration) {
        val annotation = property.annotations.first {
            it.shortName.asString() == "GenerateStateMachineGraph"
        }
        val fileName = annotation.arguments.first().value as String

        val containingFile = property.containingFile ?: return
        val sourceText = java.io.File(containingFile.filePath).readText()

        // Create the PSI tree from the source file text
        val psiFactory = KtPsiFactory(project)
        val ktFile = psiFactory.createFile(sourceText)

        // Find the specific property in the PSI tree
        val propertyName = property.simpleName.asString()
        val psiProperty = ktFile.findDescendantOfType<KtProperty> { it.name == propertyName }

        if (psiProperty != null) {
            val transitions = mutableListOf<Transition>()
            var initialState: String? = null

            psiProperty.accept(object : KtTreeVisitorVoid() {
                // Parse transitions like: A on B switchTo C
                override fun visitBinaryExpression(expression: KtBinaryExpression) {
                    super.visitBinaryExpression(expression)

                    if (expression.operationReference.text == "switchTo") {
                        val targetStateText = expression.right?.text ?: return
                        parseSwitchToSource(expression.left, targetStateText, transitions)
                    }
                }

                // Parse standard function calls like: (A on B).switchTo(C)
                override fun visitDotQualifiedExpression(expression: KtDotQualifiedExpression) {
                    super.visitDotQualifiedExpression(expression)

                    val selector = expression.selectorExpression

                    if (selector is KtCallExpression && selector.calleeExpression?.text == "switchTo") {
                        val targetStateText = selector.valueArguments.firstOrNull()?.text ?: return
                        parseSwitchToSource(expression.receiverExpression, targetStateText, transitions)
                    }
                    // Parse initial state: IDLE { ... }.initial()
                    else if (selector?.text == "initial()") {
                        val receiver = expression.receiverExpression
                        initialState = extractStateNameFromReceiver(receiver)
                    }
                }
            })

            writeMermaidGraph(fileName, initialState, transitions)
        }
    }

    private fun parseSwitchToSource(
        sourceExpression: KtExpression?,
        targetState: String,
        transitions: MutableList<Transition>
    ) {
        when (sourceExpression) {
            is KtBinaryExpression -> {
                val op = sourceExpression.operationReference.text
                if (op == "on" || op == "completeAnd") {
                    val condition = sourceExpression.right?.text?.cleanConditionText() ?: ""
                    val finalCondition = if (op == "completeAnd") "Complete & $condition" else condition

                    extractSources(sourceExpression.left).forEach { sourceState ->
                        transitions.add(Transition(sourceState, targetState, finalCondition))
                    }
                }
            }
            is KtDotQualifiedExpression -> {
                if (sourceExpression.selectorExpression?.text == "onComplete") {
                    extractSources(sourceExpression.receiverExpression).forEach { sourceState ->
                        transitions.add(Transition(sourceState, targetState, "onComplete"))
                    }
                }
            }
        }
    }

    private fun extractSources(expression: KtExpression?): List<String> {
        return when (expression) {
            is KtCallExpression -> {
                when (expression.calleeExpression?.text) {
                    "anyOf" -> expression.valueArguments.map { it.text }
                    "allOf" -> listOf("[*]")
                    else -> emptyList()
                }
            }
            is KtNameReferenceExpression -> listOf(expression.text)
            else -> emptyList()
        }
    }

    private fun extractStateNameFromReceiver(receiver: KtExpression): String? {
        return when (receiver) {
            // Handles IDLE { ... }
            is KtCallExpression -> receiver.calleeExpression?.text
            // Handles simple IDLE
            is KtNameReferenceExpression -> receiver.text
            else -> null
        }
    }

    private fun String.cleanConditionText(): String {
        return this.replace(Regex("\\s+"), " ")
            .replace("\n", "")
            .trim()
    }

    private fun writeMermaidGraph(fileName: String, initialState: String?, transitions: List<Transition>) {
        val file: OutputStream = codeGenerator.createNewFile(
            dependencies = Dependencies.ALL_FILES,
            packageName = "",
            fileName = fileName,
            extensionName = "md"
        )

        val sb = StringBuilder()
        sb.appendLine("```mermaid")
        sb.appendLine("stateDiagram-v2")

        if (initialState != null) {
            sb.appendLine("    [*] --> $initialState")
        }

        transitions.forEach { transition ->
            sb.appendLine("    ${transition.from} --> ${transition.to} :${transition.condition}")
        }

        sb.appendLine("```")

        file.write(sb.toString().toByteArray())
        file.close()
    }

    data class Transition(val from: String, val to: String, val condition: String)
}

// Utility extension for traversing AST to find specific node types
private inline fun <reified T : KtElement> KtElement.findDescendantOfType(crossinline predicate: (T) -> Boolean): T? {
    var result: T? = null
    this.accept(object : KtTreeVisitorVoid() {
        override fun visitKtElement(element: KtElement) {
            if (result != null) return
            if (element is T && predicate(element)) {
                result = element
            }
            super.visitKtElement(element)
        }
    })
    return result
}