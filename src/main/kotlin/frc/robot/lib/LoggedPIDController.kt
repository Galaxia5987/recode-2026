package frc.robot.lib

import org.wpilib.math.controller.PIDController
import kotlin.properties.ReadOnlyProperty

class LoggedPIDController(private val kp: Double, private val ki: Double, private val kd: Double) {
    fun delegate(): ReadOnlyProperty<Any?, LoggedPIDControllerInstance> =
        ReadOnlyProperty { _, property ->
            LoggedPIDControllerInstance(property.name, kp, ki, kd)
        }
}

class LoggedPIDControllerInstance(
    val name: String,
    kp: Double,
    ki: Double,
    kd: Double
) : PIDController(kp, ki, kd) {

    private val loggedGains = LoggedNetworkGains(name, kp, ki, kd)

    fun update() {
        if (loggedGains.updatePIDGains()) {
            setPID(loggedGains.kP.value, loggedGains.kI.value, loggedGains.kD.value)
        }
    }
}

fun loggedPIDController(kp: Double, ki: Double, kd: Double) =
    LoggedPIDController(kp, ki, kd).delegate()