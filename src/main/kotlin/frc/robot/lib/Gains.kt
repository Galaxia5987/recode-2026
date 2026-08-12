package frc.robot.lib

import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rps
import frc.robot.lib.extensions.rps_squared
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import org.wpilib.units.measure.AngularAcceleration
import org.wpilib.units.measure.AngularVelocity

enum class GainType {
    PID,
    FEEDFORWARD,
    MOTION_MAGIC
}

enum class GainsEnum(val displayedName: String, val type: GainType) {
    KP("kP", GainType.PID),
    KI("kI", GainType.PID),
    KD("kD", GainType.PID),
    KS("kS", GainType.FEEDFORWARD),
    KV("kV", GainType.FEEDFORWARD),
    KA("kA", GainType.FEEDFORWARD),
    KG("kG", GainType.FEEDFORWARD),
    CRUISE_VELOCITY("cruiseVelocity", GainType.MOTION_MAGIC),
    ACCELERATION("acceleration", GainType.MOTION_MAGIC),
    JERK("jerk", GainType.MOTION_MAGIC)
}

data class Tunable(
    val pidTune: Boolean = true,
    val feedForwardTune: Boolean = false,
    val motionMagicTune: Boolean = false,
)

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
    kP: Double = 0.0,
    kI: Double = 0.0,
    kD: Double = 0.0,
    kS: Double = 0.0,
    kV: Double = 0.0,
    kA: Double = 0.0,
    kG: Double = 0.0,
    cruiseVelocity: AngularVelocity = 0.rps,
    acceleration: AngularAcceleration = 0.rps_squared,
    jerk: Double = 0.0, // m/s
    key: String =
        (Throwable().stackTrace[1]?.fileName?.substringBeforeLast('.') + ""),
    enableTune: Tunable = Tunable(),
) {
    private val path = "/Tuning/$key/$name"

    operator fun get(gain: GainsEnum): Double =
        tunableGains.getValue(gain).value

    private inner class TunableEnable(
        type: GainType,
        initialValue: Boolean,
    ) {
        private var logger =
            LoggedNetworkBoolean(
                "$path/${type.name.lowercase()}",
                initialValue,
            )

        var value = initialValue
            private set

        fun isEnabled(): Boolean {
            if (value) {
                logger.set(true)
                return true
            }
            if (logger.get()) {
                value = true
                return true
            }
            return false
        }
    }

    private val tuningEnabled =
        mapOf(
            GainType.PID to TunableEnable(GainType.PID, enableTune.pidTune),
            GainType.FEEDFORWARD to
                    TunableEnable(
                        GainType.FEEDFORWARD,
                        enableTune.feedForwardTune,
                    ),
            GainType.MOTION_MAGIC to
                    TunableEnable(
                        GainType.MOTION_MAGIC,
                        enableTune.motionMagicTune,
                    ),
        )

    private inner class TunableGain(
        val gain: GainsEnum,
        initialValue: Double,
    ) {
        private var logger =
            if (tuningEnabled[gain.type]!!.value)
                LoggedNetworkNumber(
                    "$path/${gain.type.name.lowercase()}/${gain.displayedName}",
                    initialValue,
                )
            else null

        var value: Double = initialValue
            private set

        fun updated(): Boolean {
            val networkVal = logger?.get()
            if (networkVal != null && networkVal != value) {
                value = networkVal
                return true
            }
            return false
        }

        fun enable() {
            if (logger != null) return
            logger =
                LoggedNetworkNumber(
                    "$path/${gain.type.name.lowercase()}/${gain.displayedName}",
                    value,
                )
        }
    }

    private val tunableGains =
        mapOf(
            GainsEnum.KP to TunableGain(GainsEnum.KP, kP),
            GainsEnum.KI to TunableGain(GainsEnum.KI, kI),
            GainsEnum.KD to TunableGain(GainsEnum.KD, kD),
            GainsEnum.KS to TunableGain(GainsEnum.KS, kS),
            GainsEnum.KV to TunableGain(GainsEnum.KV, kV),
            GainsEnum.KA to TunableGain(GainsEnum.KA, kA),
            GainsEnum.KG to TunableGain(GainsEnum.KG, kG),
            GainsEnum.JERK to TunableGain(GainsEnum.JERK, jerk),
            GainsEnum.CRUISE_VELOCITY to
                    TunableGain(GainsEnum.CRUISE_VELOCITY, cruiseVelocity[rps]),
            GainsEnum.ACCELERATION to
                    TunableGain(GainsEnum.ACCELERATION, acceleration[rps_squared]),
        )

    fun toSlotConfig() =
        Slot0Configs().apply {
            kP = this@LoggedNetworkGains[GainsEnum.KP]
            kI = this@LoggedNetworkGains[GainsEnum.KI]
            kD = this@LoggedNetworkGains[GainsEnum.KD]
            kA = this@LoggedNetworkGains[GainsEnum.KA]
            kS = this@LoggedNetworkGains[GainsEnum.KS]
            kV = this@LoggedNetworkGains[GainsEnum.KV]
            kG = this@LoggedNetworkGains[GainsEnum.KG]
        }

    fun toMotionMagicConfig() =
        MotionMagicConfigs().apply {
            MotionMagicCruiseVelocity =
                tunableGains[GainsEnum.CRUISE_VELOCITY]!!.value
            MotionMagicAcceleration =
                tunableGains[GainsEnum.ACCELERATION]!!.value
            MotionMagicJerk = tunableGains[GainsEnum.JERK]!!.value
        }

    fun hasPIDChanged(): Boolean {
        val pidTunable = tunableGains
            .filter { it.key.type != GainType.MOTION_MAGIC }
        tuningEnabled.filter { it.key != GainType.MOTION_MAGIC }.forEach { (type, switch) ->
            if (switch.isEnabled()) {
                pidTunable.filter { it.key.type == type }.forEach { (_, gain) ->
                    gain.enable()
                }
            }
        }
        return pidTunable.map { it.value.updated() }.any()
    }

    fun hasMotionMagicChanged(): Boolean {
        val motionMagicTunable = tunableGains
            .filter { it.key.type == GainType.MOTION_MAGIC }
        tuningEnabled.filter { it.key == GainType.MOTION_MAGIC }.forEach { (type, switch) ->
            if (switch.isEnabled()) {
                motionMagicTunable.filter { it.key.type == type }.forEach { (_, gain) ->
                    gain.enable()
                }
            }
        }
        return motionMagicTunable
            .map { it.value.updated() }.any()
    }

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
        cruiseVelocity = motionMagicConfigs.MotionMagicCruiseVelocity.rps,
        acceleration = motionMagicConfigs.MotionMagicAcceleration.rps_squared,
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
        cruiseVelocity = motionMagicConfigs.MotionMagicCruiseVelocity.rps,
        acceleration = motionMagicConfigs.MotionMagicAcceleration.rps_squared,
        jerk = motionMagicConfigs.MotionMagicJerk,
    )
