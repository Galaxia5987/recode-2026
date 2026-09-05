package frc.robot.lib.extensions

import com.ctre.phoenix6.controls.*
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Current
import org.wpilib.units.measure.Voltage

// Open-Loop Output Requests
infix fun DutyCycleOut.with(output: Double): DutyCycleOut = withOutput(output)

infix fun VoltageOut.with(output: Double): VoltageOut = withOutput(output)

infix fun VoltageOut.with(output: Voltage): VoltageOut = withOutput(output)

infix fun TorqueCurrentFOC.with(output: Double): TorqueCurrentFOC =
    withOutput(output)

infix fun TorqueCurrentFOC.with(output: Current): TorqueCurrentFOC =
    withOutput(output)

// Position Requests
infix fun PositionDutyCycle.with(position: Double): PositionDutyCycle =
    withPosition(position)

infix fun PositionDutyCycle.with(position: Angle): PositionDutyCycle =
    withPosition(position)

infix fun PositionVoltage.with(position: Double): PositionVoltage =
    withPosition(position)

infix fun PositionVoltage.with(position: Angle): PositionVoltage =
    withPosition(position)

infix fun PositionTorqueCurrentFOC.with(
    position: Double
): PositionTorqueCurrentFOC = withPosition(position)

infix fun PositionTorqueCurrentFOC.with(
    position: Angle
): PositionTorqueCurrentFOC = withPosition(position)

// Velocity Requests
infix fun VelocityDutyCycle.with(velocity: Double): VelocityDutyCycle =
    withVelocity(velocity)

infix fun VelocityDutyCycle.with(velocity: AngularVelocity): VelocityDutyCycle =
    withVelocity(velocity)

infix fun VelocityVoltage.with(velocity: Double): VelocityVoltage =
    withVelocity(velocity)

infix fun VelocityVoltage.with(velocity: AngularVelocity): VelocityVoltage =
    withVelocity(velocity)

infix fun VelocityTorqueCurrentFOC.with(
    velocity: Double
): VelocityTorqueCurrentFOC = withVelocity(velocity)

infix fun VelocityTorqueCurrentFOC.with(
    velocity: AngularVelocity
): VelocityTorqueCurrentFOC = withVelocity(velocity)

// Motion Magic Position Requests
infix fun MotionMagicDutyCycle.with(position: Double): MotionMagicDutyCycle =
    withPosition(position)

infix fun MotionMagicDutyCycle.with(position: Angle): MotionMagicDutyCycle =
    withPosition(position)

infix fun MotionMagicVoltage.with(position: Double): MotionMagicVoltage =
    withPosition(position)

infix fun MotionMagicVoltage.with(position: Angle): MotionMagicVoltage =
    withPosition(position)

infix fun MotionMagicTorqueCurrentFOC.with(
    position: Double
): MotionMagicTorqueCurrentFOC = withPosition(position)

infix fun MotionMagicTorqueCurrentFOC.with(
    position: Angle
): MotionMagicTorqueCurrentFOC = withPosition(position)

// Motion Magic Expo Position Requests
infix fun MotionMagicExpoDutyCycle.with(
    position: Double
): MotionMagicExpoDutyCycle = withPosition(position)

infix fun MotionMagicExpoDutyCycle.with(
    position: Angle
): MotionMagicExpoDutyCycle = withPosition(position)

infix fun MotionMagicExpoVoltage.with(
    position: Double
): MotionMagicExpoVoltage = withPosition(position)

infix fun MotionMagicExpoVoltage.with(position: Angle): MotionMagicExpoVoltage =
    withPosition(position)

infix fun MotionMagicExpoTorqueCurrentFOC.with(
    position: Double
): MotionMagicExpoTorqueCurrentFOC = withPosition(position)

infix fun MotionMagicExpoTorqueCurrentFOC.with(
    position: Angle
): MotionMagicExpoTorqueCurrentFOC = withPosition(position)

// Dynamic Motion Magic Position Requests
infix fun DynamicMotionMagicDutyCycle.with(
    position: Double
): DynamicMotionMagicDutyCycle = withPosition(position)

infix fun DynamicMotionMagicDutyCycle.with(
    position: Angle
): DynamicMotionMagicDutyCycle = withPosition(position)

infix fun DynamicMotionMagicVoltage.with(
    position: Double
): DynamicMotionMagicVoltage = withPosition(position)

infix fun DynamicMotionMagicVoltage.with(
    position: Angle
): DynamicMotionMagicVoltage = withPosition(position)

infix fun DynamicMotionMagicTorqueCurrentFOC.with(
    position: Double
): DynamicMotionMagicTorqueCurrentFOC = withPosition(position)

infix fun DynamicMotionMagicTorqueCurrentFOC.with(
    position: Angle
): DynamicMotionMagicTorqueCurrentFOC = withPosition(position)

// Motion Magic Velocity Requests
infix fun MotionMagicVelocityDutyCycle.with(
    velocity: Double
): MotionMagicVelocityDutyCycle = withVelocity(velocity)

infix fun MotionMagicVelocityDutyCycle.with(
    velocity: AngularVelocity
): MotionMagicVelocityDutyCycle = withVelocity(velocity)

infix fun MotionMagicVelocityVoltage.with(
    velocity: Double
): MotionMagicVelocityVoltage = withVelocity(velocity)

infix fun MotionMagicVelocityVoltage.with(
    velocity: AngularVelocity
): MotionMagicVelocityVoltage = withVelocity(velocity)

infix fun MotionMagicVelocityTorqueCurrentFOC.with(
    velocity: Double
): MotionMagicVelocityTorqueCurrentFOC = withVelocity(velocity)

infix fun MotionMagicVelocityTorqueCurrentFOC.with(
    velocity: AngularVelocity
): MotionMagicVelocityTorqueCurrentFOC = withVelocity(velocity)
