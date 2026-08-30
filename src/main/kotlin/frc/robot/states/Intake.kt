package frc.robot.states

import frc.robot.RobotContainer.Buttons.intakeButton
import frc.robot.RobotContainer.Buttons.outtakeButton
import frc.robot.field.inExtendedAllianceZone
import frc.robot.field.isInDoubleFeedingZone
import frc.robot.lib.commands.unaryPlus
import frc.robot.lib.extensions.not
import frc.robot.lib.state_machine.buildStateMachine
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.funnel.Funnel
import frc.robot.subsystems.intake.roller.Roller
import frc.robot.subsystems.spindexer.Spindexer
import org.wpilib.command3.Trigger

val isShooting = Trigger { true }

enum class IntakeState {
    IDLE,
    PUMPING,
    INTAKING,
    OUTTAKING;

    companion object {
        val stateMachine =
            buildStateMachine<IntakeState>("Intake State Machine") {
                IDLE {
                        +[
                            Roller.stop(),
                            Funnel.stop(),
                            Extender.close(),
                            Spindexer.stop(),
                        ]
                    }
                    .initial()

                PUMPING {
                    +[
                        Roller.intake(),
                        Funnel.intake(),
                        Extender.pump(),
                        Spindexer.convey(),
                    ]
                }

                INTAKING {
                    +[
                        Roller.intake(),
                        Funnel.intake(),
                        Extender.open(),
                        Spindexer.convey(),
                    ]
                }

                OUTTAKING {
                    +[
                        Roller.outtake(),
                        Funnel.outtake(),
                        Extender.open(),
                        Spindexer.stop(),
                    ]
                }

                IDLE on isShooting switchTo PUMPING
                IDLE on intakeButton switchTo INTAKING
                IDLE on
                    outtakeButton
                        .and(!inExtendedAllianceZone)
                        .and(isInDoubleFeedingZone) switchTo
                    OUTTAKING
                PUMPING on !isShooting switchTo IDLE
                PUMPING on intakeButton switchTo INTAKING
                PUMPING on
                    outtakeButton
                        .and(isInDoubleFeedingZone)
                        .and(!inExtendedAllianceZone) switchTo
                    OUTTAKING
                INTAKING on !intakeButton switchTo IDLE
                INTAKING on
                    outtakeButton
                        .and(isInDoubleFeedingZone)
                        .and(!inExtendedAllianceZone) switchTo
                    OUTTAKING
                OUTTAKING on
                    !outtakeButton
                        .or(isInDoubleFeedingZone.or(inExtendedAllianceZone))
                        .negate() switchTo
                    IDLE

                // TODO("isShooting is currently a mock variable)
            }
    }
}
