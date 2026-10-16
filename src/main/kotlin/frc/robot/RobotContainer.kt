package frc.robot

import frc.robot.RobotContainer.Buttons.intakeButton
import frc.robot.RobotContainer.Buttons.outtakeButton
import frc.robot.autonomous.elinorPath
import frc.robot.autonomous.joespath
import frc.robot.lib.BasicAlerts
import frc.robot.lib.Mode
import frc.robot.lib.commands.ChoreoAutoFactory
import frc.robot.lib.commands.emptyCommand
import frc.robot.lib.commands.initializeAllMechanisms
import frc.robot.lib.commands.onChange
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.extensions.not
import frc.robot.lib.extensions.onFalse
import frc.robot.lib.extensions.onTrue
import frc.robot.setpoint_manager.SetpointManager
import frc.robot.states.shouldShoot
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.funnel.Funnel
import frc.robot.subsystems.intake.roller.Roller
import frc.robot.subsystems.spindexer.Spindexer
import frc.robot.subsystems.turret.Turret
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.wpilib.command3.Command
import org.wpilib.command3.Trigger
import org.wpilib.command3.button.CommandNiDsPS5Controller
import org.wpilib.smartdashboard.SendableChooser

object RobotContainer {
    private val driverController = CommandNiDsPS5Controller(0)
    private val autoChooser: LoggedDashboardChooser<Command>
    val autoFactory =
        ChoreoAutoFactory(
            drive::getPose,
            drive::resetOdometry,
            drive::followTrajectory,
            true,
            drive,
        )

    object Buttons {
        var intakeButton = Trigger { false }
        var outtakeButton = Trigger { false }
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
        initializeAllMechanisms()
        BasicAlerts
        SetpointManager
    }

    private fun configureDefaultCommands() {
        drive.defaultCommand =
            DriveCommands.joystickDrive(
                { -driverController.leftY },
                { -driverController.leftX },
                { -driverController.rightX },
            )

        Turret.defaultCommand = Turret.setAngle {
            SetpointManager.turretSetpoint
        }
        Hood.defaultCommand = Hood.setPosition { SetpointManager.hoodSetpoint }

        //        IntakeState.stateMachine.register()
    }

    private fun configureButtonBindings() {
        driverController.create().onTrue(DriveCommands.resetGyro())
        intakeButton =
            driverController
                .R1()
                .onTrue(
                    Roller.intake(),
                    Funnel.intake(),
                    Extender.open(),
                    Spindexer.convey(),
                )
        outtakeButton =
            driverController
                .L1()
                .onTrue(
                    Roller.outtake(),
                    Funnel.outtake(),
                    Extender.open(),
                    Spindexer.stop(),
                )
        (intakeButton.or(outtakeButton)).onFalse(
            Roller.stop(),
            Funnel.stop(),
            Extender.close(),
            Spindexer.stop(),
        )

        driverController.circle().onChange {
            shouldShoot = !it
        }
    }

    fun getAutonomousCommand(): Command = autoChooser.get()

    private fun registerAutoCommands() {
        autoChooser.addDefaultOption("Empty", emptyCommand())
        autoChooser.addOption("elinorPath", elinorPath())
        autoChooser.addOption("joespath", joespath())
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
