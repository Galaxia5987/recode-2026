package frc.robot.lib

import frc.robot.lib.extensions.sec
import org.wpilib.system.DataLogManager
import org.littletonrobotics.junction.AutoLogOutput
import org.wpilib.command3.Command

const val ABNORMAL_EVENT_NAME = "### [Error] DRIVER PROBLEM REPORTED ###"

@AutoLogOutput private var marked = false

private fun markEvent(eventName: String): Command =
    Command.noRequirements {
        DataLogManager.log(eventName)
        marked = true
        it.wait(5.0.sec)
        marked = false
    }.named("MarkEvent")


fun markAbnormalEvent(): Command = markEvent(ABNORMAL_EVENT_NAME)
