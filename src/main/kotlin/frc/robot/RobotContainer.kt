package frc.robot

import frc.robot.RobotContainer.Buttons.intakeButton
import frc.robot.RobotContainer.Buttons.outtakeButton
import frc.robot.autonomous.digonal
import frc.robot.autonomous.line
import frc.robot.autonomous.linewith180
import frc.robot.lib.AutoChooser
import frc.robot.lib.BasicAlerts
import frc.robot.lib.Mode
import frc.robot.lib.commands.emptyCommand
import frc.robot.lib.commands.initializeAllMechanisms
import frc.robot.lib.commands.onChange
import frc.robot.lib.extensions.enableAutoLogOutputFor
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
import org.wpilib.command3.Command
import org.wpilib.command3.Trigger
import org.wpilib.command3.button.CommandNiDsPS5Controller
import org.wpilib.tunable.Tunables

object RobotContainer {

    private val driverController = CommandNiDsPS5Controller(0)
    private val autoChooser: AutoChooser

    object Buttons {
        var intakeButton = Trigger { false }
        var outtakeButton = Trigger { false }
    }

    init {
        drive // Ensure Drive is initialized
        autoChooser = AutoChooser()

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

    fun getAutonomousCommand(): Command = autoChooser.selectedCommand()

    private fun registerAutoCommands() {
        autoChooser.addCmd("Empty", ::emptyCommand)
        autoChooser.addCmd("Line.trag", ::line)
        autoChooser.addCmd("Linewith180.trag", ::linewith180)
        autoChooser.addCmd("Digonal.trag", ::digonal)
        // SysIds
        autoChooser.addCmd(
            "Drive Wheel Radius Characterization",
            DriveCommands::wheelRadiusCharacterization,
        )
        autoChooser.addCmd(
            "Drive Simple FF Characterization",
            DriveCommands::feedforwardCharacterization,
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

        autoChooser.addCmd(
            "swerveFFCharacterization",
            DriveCommands::feedforwardCharacterization,
        )
        Tunables.publish("Chooser", autoChooser)
    }
}
