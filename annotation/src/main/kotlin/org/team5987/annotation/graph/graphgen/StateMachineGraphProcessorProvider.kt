package org.team5987.annotation.graph.graphgen

import com.google.devtools.ksp.processing.SymbolProcessor
import com.google.devtools.ksp.processing.SymbolProcessorEnvironment
import com.google.devtools.ksp.processing.SymbolProcessorProvider
import java.io.File

class StateMachineGraphProcessorProvider : SymbolProcessorProvider {
    override fun create(
        environment: SymbolProcessorEnvironment
    ): SymbolProcessor {
        val projectDir =
            environment.options["graphRootDir"]
                ?: throw IllegalArgumentException(
                    "Missing graphRootDir KSP argument"
                )

        val targetDir = File(projectDir)
        if (!targetDir.exists() && !targetDir.mkdirs()) {
            error("Failed to create state machine graph output directory: $projectDir")
        }

        return StateMachineGraphProcessor(
            environment.logger,
            environment.codeGenerator,
        )
    }
}
