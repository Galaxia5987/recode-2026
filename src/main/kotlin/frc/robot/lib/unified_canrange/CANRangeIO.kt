package frc.robot.lib.unified_canrange

import frc.robot.lib.extensions.cm
import org.team5987.annotation.Logged
import org.wpilib.units.measure.Distance

interface CANRangeIO {
    val inputs: LoggedSensorInputs

    fun updateInputs() {}

    @Logged
    open class SensorInputs {
        var distance: Distance = 0.cm
        var isDetecting: Boolean = false
        var signalStrength: Double = 0.0
    }
}
