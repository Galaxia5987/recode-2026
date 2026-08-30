package frc.robot

import frc.robot.lib.state_machine.buildStateMachine
import org.team5987.annotation.graph.graphgen.GenerateStateMachineGraph

enum class IntakeStates {
    INTAKE,
    OUTTAKE,
    STOP;

    companion object {
        @GenerateStateMachineGraph("IntakeStates")
        val stateMachine = buildStateMachine<IntakeStates>("IntakeStates") {
            INTAKE {

            }

            OUTTAKE {

            }

            STOP {

            }

            INTAKE on {true} switchTo OUTTAKE
            OUTTAKE on {false} switchTo STOP

        }
    }
}