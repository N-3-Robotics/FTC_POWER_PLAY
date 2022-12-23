package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config

class QOL {
    companion object {
        fun inchesToMeters(inches: Double): Double {
            return inches / 39.3701
        }
        fun metersToInches(meters: Double): Double {
            return meters * 39.3701
        }
        fun inchesToTicks(inches: Double): Double {
            return inchesToMeters(inches) * MotorConstants.GoBilda312.TICKS_PER_METER
        }
        fun metersToTicks(meters: Double): Double {
            return meters * MotorConstants.GoBilda312.TICKS_PER_METER
        }
    }
}

enum class Side {
    LEFT, RIGHT, BOTH
}

enum class Direction {
    FORWARD, BACKWARD, LEFT, RIGHT
}

enum class MotorConstants(val TICKS_PER_REV: Double, val WHEEL_DIAMETER: Double, val TICKS_PER_METER: Double) {
    GoBilda312(((((1+(46/17))) * (1+(46/11))) * 28).toDouble(), 96.0 / 1000.0, ((((1+(46/17))) * (1+(46/11))) * 28).toDouble() / (96.0 / 1000.0 * Math.PI))
}
@Config
object DriveConstants{
    @JvmField
    var driveTime = 1600 // milliseconds

    @JvmField
    var strafeTime = 2100 // milliseconds

    @JvmField
    var turnTime = 1050 // milliseconds

    @JvmField
    var tileLength = 23.5 //inches

    @JvmField
    var kP = 0.01
}

// create an enum class where each value is a double that represents the strength of the rumble
enum class RumbleStrength(val strength: Double) {
    LOW(0.25),
    MEDIUM(0.5),
    HIGH(0.75),
    MAX(1.0)
}