package frc.robot.lib

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import org.wpilib.units.measure.Current
import org.wpilib.units.measure.Time
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.sec

fun createCurrentLimits(
    supplyCurrentLimit: Current = 30.amps,
    supplyCurrentPeakDifference: Current = 5.0.amps,
    supplyCurrentLowerTime: Time = 0.4.sec,
    statorCurrentLimit: Current? = null
): CurrentLimitsConfigs =
    CurrentLimitsConfigs().apply {
        SupplyCurrentLimitEnable = true
        StatorCurrentLimitEnable = true

        SupplyCurrentLimit = supplyCurrentLimit[amps]
        StatorCurrentLimit =
            statorCurrentLimit?.get(amps) ?: (supplyCurrentLimit[amps] * 2)

        SupplyCurrentLowerLimit =
            (supplyCurrentLimit - supplyCurrentPeakDifference)[amps]
        SupplyCurrentLowerTime = supplyCurrentLowerTime[sec]
    }
