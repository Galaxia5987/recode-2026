package frc.robot.lib.unified_controller

import org.wpilib.command3.Trigger
import org.wpilib.command3.button.CommandGamepad

class PS5Gamepad(port: Int) : CommandGamepad(port) {
    fun cross(): Trigger = super.southFace()

    fun circle(): Trigger = super.eastFace()

    fun triangle(): Trigger = super.northFace()

    fun square(): Trigger = super.westFace()

    fun options(): Trigger = super.guide() // TODO: Might be wrong!

    fun create(): Trigger = super.back() // TODO: Might be wrong!
}
