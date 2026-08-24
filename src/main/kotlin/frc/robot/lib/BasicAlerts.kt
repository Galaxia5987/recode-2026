package frc.robot.lib

import frc.robot.lib.commands.command
import frc.robot.lib.commands.onChange
import frc.robot.lib.extensions.sec
import org.littletonrobotics.conduit.ConduitApi
import org.wpilib.command3.Trigger
import org.wpilib.driverstation.Alert

object BasicAlerts {

    private const val BATTERY_LOW_VOLTAGE = 12.3

    private val batteryLowAlert =
        Alert("Battery is running low!", Alert.Level.MEDIUM).also {
            val isBatteryLow = {
                ConduitApi.getInstance().pdpVoltage < BATTERY_LOW_VOLTAGE
            }
            Trigger(isBatteryLow)
                .debounce(2.sec)
                .onChange(
                    command { it.set(isBatteryLow()) }
                        .named("changeBatteryLowAlert")
                )
        }
}
