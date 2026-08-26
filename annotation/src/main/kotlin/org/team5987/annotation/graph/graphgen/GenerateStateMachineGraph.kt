package org.team5987.annotation.graph.graphgen

@Target(AnnotationTarget.PROPERTY, AnnotationTarget.CLASS)
@Retention(AnnotationRetention.SOURCE)
annotation class GenerateStateMachineGraph(
    val outputFileName: String
)