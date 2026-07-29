package frc.robot.lib

import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import org.wpilib.math.geometry.*
import org.wpilib.units.Units.Meters
import org.wpilib.units.Units.Rotations
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.Distance

fun getTranslation2d(x: Double = 0.0, y: Double = 0.0) = Translation2d(x, y)

fun getTranslation2d(x: Distance = Meters.zero(), y: Distance = Meters.zero()) =
    Translation2d(x, y)

fun getRotation2d(x: Double = 0.0, y: Double = 0.0) = Rotation2d(x, y)

fun getRotation2d(x: Distance = Meters.zero(), y: Distance = Meters.zero()) =
    Rotation2d(x[m], y[m])

fun getPose2d(
    translation: Translation2d = Translation2d(),
    rotation: Rotation2d = Rotation2d(),
) = Pose2d(translation, rotation)

fun getPose2d(
    x: Double = 0.0,
    y: Double = 0.0,
    rotation: Rotation2d = Rotation2d(),
) = Pose2d(x, y, rotation)

fun getPose2d(
    x: Distance = Meters.zero(),
    y: Distance = Meters.zero(),
    rotation: Rotation2d = Rotation2d(),
) = Pose2d(x, y, rotation)

fun getTranslation3d(x: Double = 0.0, y: Double = 0.0, z: Double = 0.0) =
    Translation3d(x, y, z)

fun getTranslation3d(
    x: Distance = Meters.zero(),
    y: Distance = Meters.zero(),
    z: Distance = Meters.zero(),
) = Translation3d(x, y, z)

fun getRotation3d(roll: Double = 0.0, pitch: Double = 0.0, yaw: Double = 0.0) =
    Rotation3d(roll, pitch, yaw)

fun getRotation3d(
    roll: Angle = Rotations.zero(),
    pitch: Angle = Rotations.zero(),
    yaw: Angle = Rotations.zero(),
) = Rotation3d(roll, pitch, yaw)

fun getPose3d(
    translation: Translation3d = Translation3d(),
    rotation: Rotation3d = Rotation3d(),
) = Pose3d(translation, rotation)

fun getPose3d(
    x: Double = 0.0,
    y: Double = 0.0,
    z: Double = 0.0,
    rotation: Rotation3d = Rotation3d(),
) = Pose3d(x, y, z, rotation)

fun getPose3d(
    x: Distance = Meters.zero(),
    y: Distance = Meters.zero(),
    z: Distance = Meters.zero(),
    rotation: Rotation3d = Rotation3d(),
) = Pose3d(x, y, z, rotation)
