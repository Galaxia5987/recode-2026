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
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import org.wpilib.command3.Command

@LoggedOutput(LogLevel.COMP)
var shouldShoot = true

@LoggedOutput(LogLevel.COMP)
val isReadyToShoot =
    Flywheel.atSetpoint.and(Turret.atSetpoint).and(Hood.atSetpoint)

fun shoot(): Command =
    command {
            while (true) {
                while (shouldShoot) {
                    +PreShooter.stop()
                    +Spindexer.stop()
                    +Flywheel.setVelocity(SetpointManager.flywheelSetpoint)
                    yield()
                    //warm up while not in setpoint
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
        .named("states/Shooting/shoot")


fun setShouldShoot(newValue: Boolean) : Command =
    command {
        shouldShoot = newValue
    }.named("states/Shooting/setShouldShoot")