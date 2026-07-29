package frc.robot.lib

import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import org.wpilib.driverstation.Alliance
import org.wpilib.driverstation.MatchState

@LoggedOutput(LogLevel.COMP)
val IS_RED: Boolean
    get() =
        MatchState.getAlliance().isPresent &&
            MatchState.getAlliance().get() == Alliance.RED
