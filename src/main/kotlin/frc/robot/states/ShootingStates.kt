package frc.robot.states

import frc.robot.lib.commands.command
import frc.robot.lib.commands.unaryPlus
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.unified_controller.PS5Gamepad
import frc.robot.setpoint_manager.SetpointManager
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.preShooter.PreShooter
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.spindexer.Spindexer
import frc.robot.subsystems.turret.Turret
import org.littletonrobotics.junction.AutoLogOutput
import org.team5987.annotation.LogLevel
import org.team5987.annotation.Logged
import org.team5987.annotation.LoggedOutput
import org.wpilib.command3.Command

var shouldShoot = true

@LoggedOutput(LogLevel.COMP)
val isReadyToShoot =
    Flywheel.atSetpoint.and(Turret.atSetpoint).and(Hood.atSetpoint)

fun shoot(): Command =
    command {
            while (true) {
                while (shouldShoot) {
                    while (isReadyToShoot.negate().asBoolean) {
                        +Flywheel.setVelocity(SetpointManager.flywheelSetpoint)
                        +PreShooter.stop()
                        +Spindexer.stop()
                        yield()
                    }
                    while (isReadyToShoot.asBoolean) {
                        +Flywheel.setVelocity(SetpointManager.flywheelSetpoint)
                        +PreShooter.convey()
                        +Spindexer.convey()
                        yield()
                    }
                    yield()
                }
                yield()
            }
        }
        .named("StateMachine/Shooting/shoot")
