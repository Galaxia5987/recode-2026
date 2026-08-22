package frc.robot.field

import frc.robot.lib.extensions.flipIfNeeded
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.periodic
import org.wpilib.math.geometry.Translation2d

private val HUB_TRANSLATION_BLUE = Translation2d(4620.41.mm, 4034.63.mm)

val HUB_TRANSLATION: Translation2d by periodic {
    HUB_TRANSLATION_BLUE.flipIfNeeded()
}
