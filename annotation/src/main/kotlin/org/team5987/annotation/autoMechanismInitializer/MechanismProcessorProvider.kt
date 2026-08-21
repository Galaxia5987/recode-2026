package org.team5987.annotation.autoMechanismInitializer

import com.google.devtools.ksp.processing.SymbolProcessor
import com.google.devtools.ksp.processing.SymbolProcessorEnvironment
import com.google.devtools.ksp.processing.SymbolProcessorProvider

class MechanismProcessorProvider : SymbolProcessorProvider {
    override fun create(
        environment: SymbolProcessorEnvironment
    ): SymbolProcessor {
        return MechanismProcessor(
            codeGenerator = environment.codeGenerator,
            logger = environment.logger
        )
    }
}