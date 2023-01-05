@file:Suppress("unused")
package org.firstinspires.ftc.teamcode.utilities

import com.acmerobotics.dashboard.config.Config
import kotlin.math.pow
import kotlin.math.roundToInt

class QOL {
    companion object {
        fun inchesToMeters(inches: Int): Double {
            return inches / 39.3701
        }
        fun metersToInches(meters: Double): Double {
            return meters * 39.3701
        }
        fun inchesToTicks(inches: Int): Int {
            return (inchesToMeters(inches) * MotorConstants.GoBilda312.TICKS_PER_METER).roundToInt()
        }
        fun ticksToInches(ticks: Int): Double {
            return metersToInches(ticks / MotorConstants.GoBilda312.TICKS_PER_METER)
        }
        fun metersToTicks(meters: Double): Int {
            return (meters * MotorConstants.GoBilda312.TICKS_PER_METER).roundToInt()
        }
        fun ticksToMeters(ticks: Int): Double {
            return ticks / MotorConstants.GoBilda312.TICKS_PER_METER
        }
        fun calcPower(target: Int, current: Int): Double {
            val p = DriveConstants.drive_kP * (target - current)
            return 2 * (1 / (1 + Math.E.pow(-p))) - 1
        }
        fun calcTurnPower(target: Float, current: Float): Double {
            val p = DriveConstants.turn_kP * (target - current)
            return 2 * (1 / (1 + Math.E.pow(-p))) - 1
        }
        fun radToDeg(radians: Float): Double {
            return radians * 180 / Math.PI
        }
        fun degToRad(degrees: Double): Double {
            return degrees * Math.PI / 180
        }
        fun rED(current: Boolean, previous: Boolean): Boolean { // Rising Edge Detector
            return current && !previous
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
@Config()
object DriveConstants{
    @JvmField
    var tileLength = 24 //inches

    @JvmField
    var drive_kP = 0.01

    @JvmField
    var turn_kP = 0.01

    @JvmField
    var AutoDriveTolerance = 50 // tick

    @JvmField
    var AutoTurnTolerance = 1 // degree

    @JvmField
    var ClawOpen = 0.0

    @JvmField
    var ClawClose = 0.6

    @JvmField
    var SlidesSpeed = 0.75
}

// create an enum class where each value is a double that represents the strength of the rumble
enum class RumbleStrength(val strength: Double) {
    LOW(0.25),
    MEDIUM(0.5),
    HIGH(0.75),
    MAX(1.0)
}