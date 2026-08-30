@file:OptIn(CompilerConfiguration.Internals::class, K1Deprecation::class)

package org.team5987.annotation.graph.graphgen

import com.google.devtools.ksp.processing.*
import com.google.devtools.ksp.symbol.KSAnnotated
import com.google.devtools.ksp.symbol.KSFile
import com.google.devtools.ksp.symbol.KSPropertyDeclaration
import java.io.File
import org.jetbrains.kotlin.K1Deprecation
import org.jetbrains.kotlin.cli.common.messages.MessageCollector
import org.jetbrains.kotlin.cli.jvm.compiler.EnvironmentConfigFiles
import org.jetbrains.kotlin.cli.jvm.compiler.KotlinCoreEnvironment
import org.jetbrains.kotlin.com.intellij.openapi.util.Disposer
import org.jetbrains.kotlin.config.CommonConfigurationKeys
import org.jetbrains.kotlin.config.CompilerConfiguration
import org.jetbrains.kotlin.psi.*

class StateMachineGraphProcessor(
    private val logger: KSPLogger,
    private val projectDir: String,
) : SymbolProcessor {

    private val disposable = Disposer.newDisposable()

    private val project by lazy {
        val configuration =
            CompilerConfiguration().apply {
                put(
                    CommonConfigurationKeys.MESSAGE_COLLECTOR_KEY,
                    MessageCollector.NONE,
                )
            }
        KotlinCoreEnvironment.createForProduction(
                disposable,
                configuration,
                EnvironmentConfigFiles.JVM_CONFIG_FILES,
            )
            .project
    }

    override fun process(resolver: Resolver): List<KSAnnotated> {
        val symbols =
            resolver.getSymbolsWithAnnotation(
                "org.team5987.annotation.graph.graphgen.GenerateStateMachineGraph"
            )

        symbols.filterIsInstance<KSPropertyDeclaration>().forEach { property ->
            try {
                generateGraphForProperty(property)
            } catch (e: Exception) {
                logger.error(
                    "Failed to generate state machine graph: ${e.message}",
                    property,
                )
            }
        }

        return emptyList()
    }

    private fun generateGraphForProperty(property: KSPropertyDeclaration) {
        val annotation =
            property.annotations.firstOrNull {
                it.shortName.asString() == "GenerateStateMachineGraph"
            } ?: return

        val fileNameArg =
            annotation.arguments.firstOrNull {
                it.name?.asString() == "outputFileName"
            }
        val fileName = (fileNameArg?.value as? String) ?: error("State Machine Graph annotation doesn't provide an `outputFileName`")

        val containingFile =
            property.containingFile
                ?: run {
                    logger.error(
                        "Could not locate containing file for property",
                        property,
                    )
                    return
                }

        val sourceText = java.io.File(containingFile.filePath).readText()

        val psiFactory = KtPsiFactory(project)
        val ktFile = psiFactory.createFile(sourceText)

        val propertyName = property.simpleName.asString()
        val psiProperty =
            ktFile.findDescendantOfType<KtProperty> { it.name == propertyName }

        if (psiProperty != null) {
            val transitions = mutableListOf<Transition>()
            var initialState: String? = null

            psiProperty.accept(
                object : KtTreeVisitorVoid() {
                    override fun visitBinaryExpression(
                        expression: KtBinaryExpression
                    ) {
                        super.visitBinaryExpression(expression)

                        if (expression.operationReference.text == "switchTo") {
                            val targetStateText =
                                expression.right?.text ?: return
                            parseSwitchToSource(
                                expression.left,
                                targetStateText,
                                transitions,
                            )
                        }
                    }

                    override fun visitDotQualifiedExpression(
                        expression: KtDotQualifiedExpression
                    ) {
                        super.visitDotQualifiedExpression(expression)

                        val selector = expression.selectorExpression
                        if (selector is KtCallExpression) {
                            val callee = selector.calleeExpression?.text

                            if (callee == "switchTo") {
                                val targetStateText =
                                    selector.valueArguments.firstOrNull()?.text
                                        ?: return
                                parseSwitchToSource(
                                    expression.receiverExpression,
                                    targetStateText,
                                    transitions,
                                )
                            } else if (callee == "initial") {
                                initialState =
                                    extractStateNameFromReceiver(
                                        expression.receiverExpression
                                    )
                            }
                        }
                    }
                }
            )

            writeMermaidGraph(
                fileName,
                initialState,
                transitions,
                containingFile,
            )
        } else {
            logger.warn("Could not find PSI property for $propertyName")
        }
    }

    override fun finish() {
        disposable.dispose()
    }

    override fun onError() {
        disposable.dispose()
    }

    private fun parseSwitchToSource(
        sourceExpression: KtExpression?,
        targetState: String,
        transitions: MutableList<Transition>,
    ) {
        when (sourceExpression) {
            is KtBinaryExpression -> {
                val op = sourceExpression.operationReference.text
                if (op == "on" || op == "completeAnd") {
                    val condition =
                        sourceExpression.right?.text?.cleanConditionText() ?: ""
                    val finalCondition =
                        if (op == "completeAnd") "Complete & $condition"
                        else condition

                    extractSources(sourceExpression.left).forEach { sourceState
                        ->
                        transitions.add(
                            Transition(sourceState, targetState, finalCondition)
                        )
                    }
                }
            }
            is KtDotQualifiedExpression -> {
                if (sourceExpression.selectorExpression?.text == "onComplete") {
                    extractSources(sourceExpression.receiverExpression)
                        .forEach { sourceState ->
                            transitions.add(
                                Transition(
                                    sourceState,
                                    targetState,
                                    "onComplete",
                                )
                            )
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
            is KtCallExpression -> receiver.calleeExpression?.text
            is KtNameReferenceExpression -> receiver.text
            else -> null
        }
    }

    private fun String.cleanConditionText(): String {
        return this.replace(Regex("\\s+"), " ").replace(" .", ".").trim()
    }

    private fun writeMermaidGraph(
        fileName: String,
        initialState: String?,
        transitions: List<Transition>,
        containingFile: KSFile,
    ) {
        val outputFile = File(projectDir, "$fileName.md")

        val sb = StringBuilder()
        sb.appendLine("```mermaid")
        sb.appendLine("stateDiagram-v2")

        if (initialState != null) {
            sb.appendLine("    [*] --> $initialState")
        }

        transitions.forEach { transition ->
            sb.appendLine(
                "    ${transition.from} --> ${transition.to} :${transition.condition}"
            )
        }

        sb.appendLine("```")

        outputFile.writeText(sb.toString())
        logger.info(
            "Generated State Machine Graph at: ${outputFile.absolutePath}"
        )
    }

    data class Transition(
        val from: String,
        val to: String,
        val condition: String,
    )
}

private inline fun <reified T : KtElement> KtElement.findDescendantOfType(
    crossinline predicate: (T) -> Boolean
): T? {
    var result: T? = null
    this.accept(
        object : KtTreeVisitorVoid() {
            override fun visitKtElement(element: KtElement) {
                if (result != null) return
                if (element is T && predicate(element)) {
                    result = element
                }
                super.visitKtElement(element)
            }
        }
    )
    return result
}
