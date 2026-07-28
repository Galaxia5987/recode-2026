package frc.robot.lib.commands

import org.wpilib.command3.Command

fun parallel(vararg commands: Command): Command {
    require(commands.size >= 2)
    return commands[0].alongWith(*commands.drop(1).toTypedArray()).withAutomaticName()
}

fun sequence(vararg commands: Command): Command {
    require(commands.size >= 2)
    var head = commands[0].andThen(commands[1])
    for(command in commands.drop(2)){
        head = head.andThen(command)
    }
    return head.withAutomaticName()
}