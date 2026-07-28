package frc.robot.lib

import java.util.function.BooleanSupplier
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import org.wpilib.command3.Trigger

class LoggedNetworkTrigger(key: String, booleanSupplier: BooleanSupplier) :
    Trigger({ false }) {

    private val loggedNetworkBoolean =
        LoggedNetworkBoolean(key, booleanSupplier.asBoolean)

    init {
        this.or { loggedNetworkBoolean.get() }
    }
}
