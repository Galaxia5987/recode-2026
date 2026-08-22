package frc.robot.field

import frc.robot.lib.extensions.flipIfNeeded
import frc.robot.lib.extensions.mm
import org.wpilib.math.geometry.Translation2d

val HUB_TRANSLATION_BLUE = Translation2d(4620.41.mm, 4034.63.mm)

val HUB_TRANSLATION: Translation2d
    get() = HUB_TRANSLATION_BLUE.flipIfNeeded()
