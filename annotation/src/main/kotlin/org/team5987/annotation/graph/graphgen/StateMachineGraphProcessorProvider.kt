package org.team5987.annotation.graph.graphgen

import com.google.devtools.ksp.processing.SymbolProcessor
import com.google.devtools.ksp.processing.SymbolProcessorEnvironment
import com.google.devtools.ksp.processing.SymbolProcessorProvider

class StateMachineGraphProcessorProvider : SymbolProcessorProvider {
    override fun create(
        environment: SymbolProcessorEnvironment
    ): SymbolProcessor {
        return StateMachineGraphProcessor(
            environment.codeGenerator,
            environment.logger
        )
    }
}