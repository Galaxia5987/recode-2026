package frc.robot

import frc.robot.lib.Mode
import frc.robot.lib.commands.command
import frc.robot.lib.commands.emptyCommand
import frc.robot.lib.commands.onChange
import frc.robot.lib.commands.unaryPlus
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.extensions.rps
import frc.robot.lib.unified_controller.PS5Gamepad
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.roller.Roller
import frc.robot.subsystems.leds.Leds
import frc.robot.subsystems.preShooter.PreShooter
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.spindexer.Spindexer
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.conduit.ConduitApi
import org.littletonrobotics.junction.LoggedPowerDistribution
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.wpilib.command3.Command
import org.wpilib.command3.Trigger
import org.wpilib.driverstation.Alert
import org.wpilib.smartdashboard.SendableChooser

object RobotContainer {
    private val driverController = PS5Gamepad(0)
    private val autoChooser: LoggedDashboardChooser<Command>

    private val batteryLowAlert = Alert("Battery is running low!", Alert.Level.MEDIUM).also {
        val BATTERY_LOW_VOLTAGE = 12.3
        val isBatteryLow = { ConduitApi.getInstance().pdpVoltage < BATTERY_LOW_VOLTAGE }
        Trigger(isBatteryLow).onChange(command { it.set(isBatteryLow()) }.named("changeBatteryLowAlert"))
    }


    init {
        drive // Ensure Drive is initialized
        autoChooser =
            LoggedDashboardChooser(
                "Auto Choices",
                SendableChooser(),
            )
        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()

        if (CURRENT_MODE == Mode.SIM) {
            SimulatedArena.getInstance()
                .addDriveTrainSimulation(driveSimulation)
            SimulatedArena.getInstance().resetFieldForAuto()
        }

        enableAutoLogOutputFor(this)
        Flywheel
        Extender
        Roller
        Spindexer
        PreShooter
        Leds
    }

    private fun configureDefaultCommands() {
        drive.defaultCommand =
            DriveCommands.joystickDrive(
                { -driverController.leftY },
                { -driverController.leftX },
                { -driverController.rightX },
            )
    }

    private fun configureButtonBindings() {
        driverController.create().onTrue(DriveCommands.resetGyro())
        driverController.rightBumper().onTrue(command {
            +[Extender.open(), Roller.intake()]
        }.named("openIntake")).onFalse(
            command {
                +[Extender.close(), Roller.stop()]
            }.named("closeIntake")
        )

        val dontShoot = driverController.leftTrigger()
        val shouldShoot = dontShoot.negate()
        val shootVelocity = 35.rps

        shouldShoot.onTrue(command {
            +Flywheel.setVelocity(shootVelocity)
            +[Spindexer.convey(), PreShooter.convey()]
        }.named("Shoot")).onFalse(
            command {
                +Flywheel.setVelocity(0.rps)
                +Spindexer.stop()
                +PreShooter.stop()
            }.named("StopShooting")
        )
    }

    fun getAutonomousCommand(): Command = autoChooser.get()

    private fun registerAutoCommands() {
        autoChooser.addDefaultOption("Empty", emptyCommand())

        // SysIds
        autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(),
        )
        autoChooser.addOption(
            "Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization(),
        )

        // TODO: Uncomment when I figure out what happened to SysId
        //        autoChooser.addOption(
        //            "Drive SysId (Quasistatic Forward)",
        //            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        //        )
        //        autoChooser.addOption(
        //            "Drive SysId (Quasistatic Reverse)",
        //            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        //        )
        //        autoChooser.addOption(
        //            "Drive SysId (Dynamic Forward)",
        //            drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        //        )
        //        autoChooser.addOption(
        //            "Drive SysId (Dynamic Reverse)",
        //            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        //        )

        autoChooser.addOption(
            "swerveFFCharacterization",
            DriveCommands.feedforwardCharacterization(),
        )
    }
}
