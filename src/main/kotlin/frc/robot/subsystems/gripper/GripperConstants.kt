package frc.robot.subsystems.gripper

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.MotorLogConfig
import org.wpilib.units.measure.Voltage

val INTAKE= 5.volts
val OUTAKE= -5.volts
val STOP = 0.volts
val REAL_GAINS= Gains(kP = 0.0, kV= 0.0)
val SIM_GAINS= Gains(kP = 0.0, kV= 0.0)
const val PORT = 0
 val CONFIG  : TalonFXConfiguration=
    TalonFXConfiguration().apply {
        MotorOutput=
            MotorOutputConfigs().apply {
                NeutralMode= NeutralModeValue.Brake
                Inverted= InvertedValue.Clockwise_Positive
            }
        CurrentLimits = createCurrentLimits()
        Slot0 = REAL_GAINS.toSlotConfig()

    }

val LOG_CONFIG =
    MotorLogConfig(
    position = false,
    statorCurrent= false,
    current = true,
    velocity = false,
    absoluteEncoder = false,
    voltage = true,
    controlRequest = true
)

