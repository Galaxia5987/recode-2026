package frc.robot.subsystems.leds

import com.ctre.phoenix6.CANBus.systemcore
import com.ctre.phoenix6.controls.LarsonAnimation
import com.ctre.phoenix6.controls.RainbowAnimation
import com.ctre.phoenix6.hardware.CANdle
import com.ctre.phoenix6.signals.RGBWColor
import frc.robot.lib.commands.addPeriodic
import org.wpilib.command3.Mechanism
import org.wpilib.driverstation.RobotState

private const val startIndex = 8
private const val endIndex = 73
private val BLUE = RGBWColor.fromHex("#0000FF").get()

object Leds : Mechanism() {

    init {
        addPeriodic(::periodic)
    }

    private val candle = CANdle(39, systemcore(0))
    private val rainbowRequest = RainbowAnimation(startIndex, endIndex)
    private val chaseRequest = LarsonAnimation(startIndex, endIndex).withSize(5)

    private fun setRainbow() {
        candle.setControl(rainbowRequest)
    }

    private fun setChase(color: RGBWColor) {
        candle.setControl(chaseRequest.withColor(color))
    }

    fun periodic() {
        if (RobotState.isEnabled()) {
            setRainbow()
        } else {
            setChase(BLUE)
        }
    }
}
