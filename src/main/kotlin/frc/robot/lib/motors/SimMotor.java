package frc.robot.lib.motors;

import java.util.function.DoubleSupplier;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;

public class SimMotor {

    protected final DCMotorSim motorSim;
    protected PIDController controller = null;
    protected ProfiledPIDController profiledController = null;

    protected double lastTimestampSeconds = 0;
    protected MotorSetpoint voltageRequest = MotorSetpoint.simpleVoltage(0);

    protected final double conversionFactor;

    public SimMotor(
            LinearSystem<N2, N1, N2> model,
            DCMotor motor,
            double gearing,
            double conversionFactor) {
        this.motorSim = new DCMotorSim(model, motor.withReduction(gearing));
        this.conversionFactor = conversionFactor / gearing;
    }

    public SimMotor(
            DCMotor motor, double jKgMetersSquared, double gearing, double conversionFactor) {
        this(
                Models.singleJointedArmFromPhysicalConstants(motor, jKgMetersSquared, gearing),
                motor,
                gearing,
                conversionFactor);
    }

    public void setController(PIDController controller) {
        this.controller = controller;
    }

    public void setProfiledController(ProfiledPIDController profiledController) {
        this.profiledController = profiledController;
    }

    public void update(double timestampSeconds) {
        motorSim.setInputVoltage(voltageRequest.getAsDouble());
        motorSim.update(timestampSeconds - lastTimestampSeconds);
        lastTimestampSeconds = timestampSeconds;
    }

    protected interface MotorSetpoint extends DoubleSupplier {

        static MotorSetpoint simpleVoltage(double voltage) {
            return () -> voltage;
        }
    }
}
