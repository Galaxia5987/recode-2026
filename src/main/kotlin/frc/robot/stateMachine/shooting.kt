package frc.robot.stateMachine

import frc.robot.subsystems.preShooter.PreShooter
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.spindexer.Spindexer
import org.wpilib.command3.Command
import org.wpilib.command3.Trigger

var shouldShoot = true

val isReadyToShoot = Flywheel.atSetpoint.and(PreShooter.atSetpoint).and(Spindexer.atSetpoint)

