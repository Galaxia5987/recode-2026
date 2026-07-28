package frc.robot.lib

import com.ctre.phoenix6.controls.CoastOut
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.wpilib.command3.Command
import org.wpilib.command3.Trigger
import org.wpilib.driverstation.RobotState

private val coastOut = CoastOut()

fun createDisableTriggerForCoast(motor: UniversalTalonFX) {
    val coast = { motor.setControl(coastOut) }
    Trigger { RobotState.isDisabled() }
        .onTrue(Command.noRequirements{coast.invoke()}.named("CoastMotor"))
    coast.invoke()
}
