package frc.robot.lib.unified_controller

import org.wpilib.command3.Trigger
import org.wpilib.command3.button.CommandGamepad

class PS5Gamepad(port: Int) : CommandGamepad(port) {
    fun cross(): Trigger = super.faceDown()

    fun circle(): Trigger = super.faceRight()

    fun triangle(): Trigger = super.faceUp()

    fun square(): Trigger = super.faceLeft()

    fun options(): Trigger = super.guide() // TODO: Might be wrong!

    fun create(): Trigger = super.back() // TODO: Might be wrong!
}
