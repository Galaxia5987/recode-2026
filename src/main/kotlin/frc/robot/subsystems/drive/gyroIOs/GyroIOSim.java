package frc.robot.subsystems.drive.gyroIOs;

import frc.robot.lib.PhoenixUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.wpilib.math.util.Units;

import static org.wpilib.units.Units.RadiansPerSecond;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyroSimulation.getGyroReading();
        inputs.yawVelocityRadPerSec =
                Units.degreesToRadians(
                        gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

        inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    }
}
