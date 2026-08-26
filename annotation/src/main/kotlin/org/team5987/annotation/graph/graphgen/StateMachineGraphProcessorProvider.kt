package org.team5987.annotation.graph.graphgen

import com.google.devtools.ksp.processing.SymbolProcessor
import com.google.devtools.ksp.processing.SymbolProcessorEnvironment
import com.google.devtools.ksp.processing.SymbolProcessorProvider
import java.io.File

class StateMachineGraphProcessorProvider : SymbolProcessorProvider {
    override fun create(
        environment: SymbolProcessorEnvironment
    ): SymbolProcessor {
        val projectDir = environment.options["graphRootDir"]
            ?: throw IllegalArgumentException("Missing graphRootDir KSP argument")

        File(projectDir).mkdirs()

        return StateMachineGraphProcessor(
            environment.logger,
            projectDir
        )
    }
}