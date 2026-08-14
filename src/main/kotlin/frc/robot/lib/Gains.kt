package frc.robot.lib

import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rps
import frc.robot.lib.extensions.rps_squared
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

fun MotionMagicConfigs.isEmpty() =
    this.MotionMagicCruiseVelocity == 0.0 &&
        this.MotionMagicAcceleration == 0.0 &&
        this.MotionMagicJerk == 0.0

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
) {
    private val path = "/Tuning/$key/$name"

    private val isMotionMagicEnabled =
        cruiseVelocity != rps.zero() ||
            acceleration != rps_squared.zero() ||
            jerk != 0.0

    private val pidGains = mutableListOf<Tunable>()
    private val motionMagicGains = mutableListOf<Tunable>()

    val kP = Tunable("PID/kP", kP, GainType.PID)
    val kI = Tunable("PID/kI", kI, GainType.PID)
    val kD = Tunable("PID/kD", kD, GainType.PID)
    val kS = Tunable("FeedForward/kS", kS, GainType.PID)
    val kV = Tunable("FeedForward/kV", kV, GainType.PID)
    val kA = Tunable("FeedForward/kA", kA, GainType.PID)
    val kG = Tunable("FeedForward/kG", kG, GainType.PID)

    val cruiseVelocity =
        Tunable(
            "MotionMagic/cruiseVelocity",
            cruiseVelocity[rps],
            GainType.MOTION_MAGIC,
            isMotionMagicEnabled,
        )
    val acceleration =
        Tunable(
            "MotionMagic/acceleration",
            acceleration[rps_squared],
            GainType.MOTION_MAGIC,
            isMotionMagicEnabled,
        )
    val jerk =
        Tunable(
            "MotionMagic/jerk",
            jerk,
            GainType.MOTION_MAGIC,
            isMotionMagicEnabled,
        )

    enum class GainType {
        PID,
        MOTION_MAGIC,
    }

    inner class Tunable(
        subPath: String,
        initialValue: Double,
        gainType: GainType,
        isActive: Boolean = true,
    ) {

        init {
            when (gainType) {
                GainType.PID -> pidGains += this
                GainType.MOTION_MAGIC -> motionMagicGains += this
            }
        }

        private val logger =
            if (isActive) LoggedNetworkNumber("$path/$subPath", initialValue)
            else null
        var value: Double = initialValue
            private set

        fun update(): Boolean {
            val networkVal = logger?.get()
            if (networkVal != null && networkVal != value) {
                value = networkVal
                return true
            }
            return false
        }
    }

    fun toSlotConfig() =
        Slot0Configs().apply {
            kP = this@LoggedNetworkGains.kP.value
            kI = this@LoggedNetworkGains.kI.value
            kD = this@LoggedNetworkGains.kD.value
            kA = this@LoggedNetworkGains.kA.value
            kS = this@LoggedNetworkGains.kS.value
            kV = this@LoggedNetworkGains.kV.value
            kG = this@LoggedNetworkGains.kG.value
        }

    fun toMotionMagicConfig() =
        MotionMagicConfigs().apply {
            MotionMagicCruiseVelocity =
                this@LoggedNetworkGains.cruiseVelocity.value
            MotionMagicAcceleration = this@LoggedNetworkGains.acceleration.value
            MotionMagicJerk = this@LoggedNetworkGains.jerk.value
        }

    /** @return Whether the PID gains changed value */
    fun updatePIDGains(): Boolean {
        var changed = false
        pidGains.forEach { if (it.update()) changed = true }
        return changed
    }

    /** @return Whether the Motion Magic gains changed value */
    fun updateMotionMagicGains(): Boolean {
        var changed = false
        motionMagicGains.forEach { if (it.update()) changed = true }
        return changed
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
