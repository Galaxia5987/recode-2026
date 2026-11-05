package frc.robot

import frc.robot.lib.AutoChooser
import frc.robot.lib.BasicAlerts
import frc.robot.lib.MechanismRegistry
import frc.robot.lib.Mode
import frc.robot.lib.commands.emptyCommand
import frc.robot.lib.commands.initializeAllMechanisms
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.sysid.SysIdable
import frc.robot.lib.sysid.sysId
import frc.robot.subsystems.drive.DriveCommands
import org.ironmaple.simulation.SimulatedArena
import org.wpilib.command3.Command
import org.wpilib.command3.button.CommandNiDsPS5Controller
import org.wpilib.tunable.Tunables

object RobotContainer {

    private val driverController = CommandNiDsPS5Controller(0)
    private val autoChooser: AutoChooser

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
    }

    fun getAutonomousCommand(): Command = autoChooser.selectedCommand()

    private fun registerAutoCommands() {
        autoChooser.addCmd("Empty", ::emptyCommand)
        // SysIds
        autoChooser.addCmd(
            "Drive Wheel Radius Characterization",
            DriveCommands::wheelRadiusCharacterization,
        )
        autoChooser.addCmd(
            "Drive Simple FF Characterization",
            DriveCommands::feedforwardCharacterization,
        )

        autoChooser.addCmd(
            "swerveFFCharacterization",
            DriveCommands::feedforwardCharacterization,
        )

        // Register all mechanisms that implement SysIdable
        MechanismRegistry.allMechanisms.forEach { mechanismClass ->
            val mechanism = mechanismClass.objectInstance
            if (mechanism is SysIdable) {
                autoChooser.addCmd(
                    "${mechanism.name} SysId",
                    mechanism::sysId,
                )
            }
        }

        Tunables.publish("Chooser", autoChooser)
    }
}
