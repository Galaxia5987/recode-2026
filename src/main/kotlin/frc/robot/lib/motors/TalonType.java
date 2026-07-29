package frc.robot.lib.motors;

import org.wpilib.math.system.DCMotor;

public enum TalonType {
    FALCON,
    FALCON_FOC,
    KRAKEN,
    KRAKEN_FOC;

    static DCMotor getDCMotor(TalonType motorType, int numMotors) {
        return switch (motorType) {
            case FALCON -> DCMotor.getFalcon500(numMotors);
            case FALCON_FOC -> DCMotor.getFalcon500Foc(numMotors);
            case KRAKEN -> DCMotor.getKrakenX60(numMotors);
            case KRAKEN_FOC -> DCMotor.getKrakenX60Foc(numMotors);
        };
    }
}
