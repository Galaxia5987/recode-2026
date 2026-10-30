// Copyright (c) Choreo contributors

package frc.robot.lib;

import static org.wpilib.util.Alert.Level.HIGH;

import choreo.util.ChoreoAlert;
import java.util.HashMap;
import java.util.Optional;
import java.util.function.Supplier;

import frc.robot.lib.commands.CommandExtensionsKt;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableConfig;
import org.wpilib.tunable.TunableOption;
import org.wpilib.tunable.TunableTable;
import org.wpilib.util.Alert;

/**
 *
 * <p>You can set the Robot's autonomous command to the chooser's chosen auto routine via <code>
 * RobotModeTriggers.autonomous.whileTrue(chooser.autoSchedulingCmd());</code>
 */
public class AutoChooser implements ComplexTunable {
    private final String DO_NOTHING_NAME;
    private static final Alert selectedNonexistentAuto =
            ChoreoAlert.alert("Selected an auto that isn't an option", HIGH);

    private final HashMap<String, Supplier<Command>> autoRoutines = new HashMap<>();

    private String selected;
    private String[] options = new String[] {};

    private Optional<Alliance> allianceAtGeneration = Optional.empty();
    private String nameAtGeneration;
    private Command generatedCommand = CommandExtensionsKt.emptyCommand();

    /** Constructs a new {@link AutoChooser}. */
    public AutoChooser() {
        this("Nothing");
    }

    /**
     * Constructs a new {@link AutoChooser} with the given name for the do-nothing default option.
     *
     * @param doNothingName The option name for the default choice.
     */
    public AutoChooser(String doNothingName) {
        DO_NOTHING_NAME = doNothingName;
        nameAtGeneration = DO_NOTHING_NAME;
        generatedCommand = CommandExtensionsKt.emptyCommand();
        addCmd(DO_NOTHING_NAME, CommandExtensionsKt::emptyCommand);
        select(DO_NOTHING_NAME);
    }

    /**
     * Returns the name of the default do-nothing option.
     *
     * @return the name of the default do-nothing option.
     */
    public String getDefaultName() {
        return DO_NOTHING_NAME;
    }

    /**
     * Select a new option in the chooser.
     *
     * <p>This method is called automatically when published as a tunable.
     *
     * @param selectStr The name of the option to select.
     * @return The name of the selected option.
     */
    public String select(String selectStr) {
        return select(selectStr, false);
    }

    private String select(String selectStr, boolean force) {
        selected = selectStr;
        if (selected.equals(nameAtGeneration)
                && allianceAtGeneration.equals(MatchState.getAlliance())) {
            // early return if the selected auto matches the active auto
            return nameAtGeneration;
        }
        boolean dsValid = RobotState.isDisabled() && MatchState.getAlliance().isPresent();
        if (dsValid || force) {
            if (!autoRoutines.containsKey(selected) && !selected.equals(DO_NOTHING_NAME)) {
                selected = DO_NOTHING_NAME;
                selectedNonexistentAuto.set(true);
            } else {
                selectedNonexistentAuto.set(false);
            }
            allianceAtGeneration = MatchState.getAlliance();
            nameAtGeneration = selected;
            generatedCommand = autoRoutines.get(nameAtGeneration).get();
        } else {
            allianceAtGeneration = Optional.empty();
            nameAtGeneration = DO_NOTHING_NAME;
            generatedCommand = CommandExtensionsKt.emptyCommand();
        }
        return nameAtGeneration;
    }

    /**
     * Adds a Command to the auto chooser.
     *
     * <p>This is done to load autonomous commands when and only when they are selected, in order to
     * save memory and file loading time for unused autonomous commands.
     *
     * <p>The generators are only run when the DriverStation is disabled and the alliance is known.
     *
     * <h3>Example:</h3>
     *
     * <pre><code>
     * AutoChooser chooser;
     * Autos autos = new Autos(swerve, shooter, intake, feeder);
     * public Robot() {
     *   chooser = new AutoChooser("/Choosers");
     *   Tunables.publish("Choosers/Auto", chooser);
     *   // fourPieceLeft is a method that accepts an AutoFactory and returns a command.
     *   chooser.addCmd("4 Piece left", autos::fourPieceLeft);
     *   chooser.addCmd("Just Shoot", shooter::shoot);
     * }
     * </code></pre>
     *
     * @param name The name of the autonomous command.
     * @param generator The function that generates an autonomous command.
     * @return This {@link AutoChooser} instance, to allow for method chaining.
     */
    public AutoChooser addCmd(String name, Supplier<Command> generator) {
        autoRoutines.put(name, generator);
        options = autoRoutines.keySet().toArray(new String[0]);
        return this;
    }
    /**
     * Returns the currently selected command
     *
     * @return The currently selected command.
     */
    public Command selectedCommand() {
        if (RobotBase.isSimulation() && nameAtGeneration == DO_NOTHING_NAME) {
            select(selected, true);
        }
        return generatedCommand;
    }

    @Override
    public void publishTunable(TunableTable table) {
        table.publishValue(
                "default",
                () -> DO_NOTHING_NAME,
                null,
                String.class,
                TunableConfig.of(TunableOption.IMMUTABLE));
        table.publishValue(
                "options", () -> options, null, String[].class, TunableConfig.of(TunableOption.IMMUTABLE));
        table.publishValue(
                "selected",
                () -> selected,
                this::select,
                String.class,
                TunableConfig.of(TunableOption.ROBUST));
        table.publishValue(
                "active",
                () -> select(selected),
                null,
                String.class,
                TunableConfig.of(TunableOption.IMMUTABLE));
    }

    @Override
    public String getTunableType() {
        return "Selectable";
    }
}
