package frc.robot.lib

import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import frc.robot.lib.extensions.*
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import org.wpilib.units.measure.AngularAcceleration
import org.wpilib.units.measure.AngularVelocity

data class Gains(
    var kP: Double = 0.0,
    var kI: Double = 0.0,
    var kD: Double = 0.0,
    var kS: Double = 0.0,
    var kV: Double = 0.0,
    var kA: Double = 0.0,
    var kG: Double = 0.0,
    val motionMagicGains: MotionMagicGains = MotionMagicGains(),
) {
    /**
     * A function to convert a [Gains] type to the [Slot0Configs] that the motor
     * uses.
     */
    fun toSlotConfig() =
        Slot0Configs().apply {
            kP = this@Gains.kP
            kI = this@Gains.kI
            kD = this@Gains.kD
            kA = this@Gains.kA
            kS = this@Gains.kS
            kV = this@Gains.kV
            kG = this@Gains.kG
        }
}

data class MotionMagicGains(
    var cruiseVelocity: AngularVelocity = 0.rps,
    var acceleration: AngularAcceleration = 0.rps_squared,
    var jerk: Double = 0.0, // m/s^3
) {
    fun toMotionMagicConfig() =
        MotionMagicConfigs().apply {
            MotionMagicCruiseVelocity =
                this@MotionMagicGains.cruiseVelocity[rps]
            MotionMagicAcceleration =
                this@MotionMagicGains.acceleration[rps_squared]
            MotionMagicJerk = this@MotionMagicGains.jerk
        }
}

class LoggedNetworkGains(
    name: String,
    var kP: Double = 0.0,
    var kI: Double = 0.0,
    var kD: Double = 0.0,
    var kS: Double = 0.0,
    var kV: Double = 0.0,
    var kA: Double = 0.0,
    var kG: Double = 0.0,
    var cruiseVelocity: AngularVelocity = 0.rps,
    var acceleration: AngularAcceleration = 0.rps_squared,
    var jerk: Double = 0.0, // m/s
    key: String =
        (Throwable().stackTrace[1]?.fileName?.substringBeforeLast('.') + ""),
) {
    private val path = "/Tuning/$key/$name"
    val lKP: LoggedNetworkNumber = LoggedNetworkNumber("$path/PID/kP", kP)
    val lKI: LoggedNetworkNumber = LoggedNetworkNumber("$path/PID/kI", kI)
    val lKD: LoggedNetworkNumber = LoggedNetworkNumber("$path/PID/kD", kD)

    var lKS: LoggedNetworkNumber? = null
    var lKV: LoggedNetworkNumber? = null
    var lKA: LoggedNetworkNumber? = null
    var lKG: LoggedNetworkNumber? = null

    var lJerk: LoggedNetworkNumber? = null
    var lCruiseVelocity: LoggedNetworkNumber? = null
    var lAcceleration: LoggedNetworkNumber? = null

    init {
        if (
            jerk != 0.0 ||
                cruiseVelocity[rad_ps] != 0.0 ||
                acceleration[rad_ps_ps] != 0.0
        ) {
            this@LoggedNetworkGains.lJerk =
                LoggedNetworkNumber("$path/MotionMagic/jerk", jerk)
            this@LoggedNetworkGains.lCruiseVelocity =
                LoggedNetworkNumber(
                    "$path/MotionMagic/cruiseVelocity",
                    cruiseVelocity[rad_ps],
                )
            this@LoggedNetworkGains.lAcceleration =
                LoggedNetworkNumber(
                    "$path/MotionMagic/acceleration",
                    acceleration[rad_ps_ps],
                )
        }
        if (kS != 0.0 || kV != 0.0 || kA != 0.0 || kG != 0.0) {
            this@LoggedNetworkGains.lKS =
                LoggedNetworkNumber("$path/Feedforward/kS", kS)
            this@LoggedNetworkGains.lKV =
                LoggedNetworkNumber("$path/Feedforward/kV", kV)
            this@LoggedNetworkGains.lKA =
                LoggedNetworkNumber("$path/Feedforward/kA", kA)
            this@LoggedNetworkGains.lKG =
                LoggedNetworkNumber("$path/Feedforward/kG", kG)
        }
    }

    fun toSlotConfig() =
        Slot0Configs().apply {
            kP = this@LoggedNetworkGains.lKP.get()
            kI = this@LoggedNetworkGains.lKI.get()
            kD = this@LoggedNetworkGains.lKD.get()
            kA = this@LoggedNetworkGains.lKA?.get() ?: 0.0
            kS = this@LoggedNetworkGains.lKS?.get() ?: 0.0
            kV = this@LoggedNetworkGains.lKV?.get() ?: 0.0
            kG = this@LoggedNetworkGains.lKG?.get() ?: 0.0
        }

    fun toMotionMagicConfig() =
        MotionMagicConfigs().apply {
            MotionMagicCruiseVelocity =
                this@LoggedNetworkGains.lCruiseVelocity?.get() ?: 0.0
            MotionMagicAcceleration =
                this@LoggedNetworkGains.lAcceleration?.get() ?: 0.0
            MotionMagicJerk = this@LoggedNetworkGains.lJerk?.get() ?: 0.0
        }

    private inline fun updateIfChanged(
        current: Double,
        updated: Double,
        update: (Double) -> Unit,
    ): Boolean = (current != updated).also { if (it) update(updated) }

    fun hasPIDChanged() =
        updateIfChanged(kP, lKP.get()) { kP = it } or
            updateIfChanged(kI, lKI.get()) { kI = it } or
            updateIfChanged(kD, lKD.get()) { kD = it } or
            updateIfChanged(kS, lKS?.get() ?: 0.0) { kS = it } or
            updateIfChanged(kV, lKV?.get() ?: 0.0) { kV = it } or
            updateIfChanged(kA, lKA?.get() ?: 0.0) { kA = it } or
            updateIfChanged(kG, lKG?.get() ?: 0.0) { kG = it }

    fun hasMotionMagicChanged() =
        updateIfChanged(cruiseVelocity[rad_ps], lCruiseVelocity?.get() ?: 0.0) {
            cruiseVelocity = it.rad_ps
        } or
            updateIfChanged(
                acceleration[rad_ps_ps],
                lAcceleration?.get() ?: 0.0,
            ) {
                acceleration = it.rad_ps_ps
            } or
            updateIfChanged(jerk, lJerk?.get() ?: 0.0) { jerk = it }
}

fun Slot0Configs.toLoggedNetworkGains(
    name: String,
    subsystem: String,
    motionMagicConfigs: MotionMagicConfigs = MotionMagicConfigs(),
) =
    LoggedNetworkGains(
        name,
        kP = kP,
        kI = kI,
        kD = kD,
        kV = kV,
        kA = kA,
        kG = kG,
        kS = kS,
        cruiseVelocity = motionMagicConfigs.MotionMagicCruiseVelocity.rad_ps,
        acceleration = motionMagicConfigs.MotionMagicAcceleration.rad_ps_ps,
        jerk = motionMagicConfigs.MotionMagicJerk,
        key = subsystem,
    )

fun Gains.toNetworkLogged(
    name: String,
    subsystem: String,
    motionMagicConfigs: MotionMagicConfigs = MotionMagicConfigs(),
) =
    LoggedNetworkGains(
        name,
        kP = kP,
        kI = kI,
        kD = kD,
        kV = kV,
        kA = kA,
        kG = kG,
        kS = kS,
        key = subsystem,
        cruiseVelocity = motionMagicConfigs.MotionMagicCruiseVelocity.rad_ps,
        acceleration = motionMagicConfigs.MotionMagicAcceleration.rad_ps_ps,
        jerk = motionMagicConfigs.MotionMagicJerk,
    )
