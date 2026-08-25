package frc.robot.stateMachine

import frc.robot.lib.commands.command
import frc.robot.setpoint_manager.SetpointCalculator
import frc.robot.setpoint_manager.SetpointManager
import frc.robot.subsystems.preShooter.PreShooter
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.spindexer.Spindexer
import org.wpilib.command3.Command
import org.wpilib.command3.Trigger

var shouldShoot = true

val isReadyToShoot = Flywheel.atSetpoint.and(PreShooter.atSetpoint)

fun shoot() : Command =
    command {
        while(true) {
            while(shouldShoot) {
                while (isReadyToShoot.negate().asBoolean) {
                    Flywheel.setVelocity(SetpointManager.flywheelSetpoint)
                    PreShooter.convey()
                    yield()
                }
                while(isReadyToShoot.asBoolean) {
                    Flywheel.setVelocity(SetpointManager.flywheelSetpoint)
                    PreShooter.convey()
                    Spindexer.convey()
                    yield()
                }
                yield()
            }
            yield()
        }
    }.named("StateMachine/Shooting/shoot")