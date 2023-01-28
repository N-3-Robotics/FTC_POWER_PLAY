@file:Suppress("unused")
package org.firstinspires.ftc.teamcode.utilities

import com.acmerobotics.dashboard.config.Config
import kotlin.math.pow
import kotlin.math.roundToInt

class QOL {
    companion object {
        fun centimetersToTicks(meters: Double): Int {
            return (meters * MotorConstants.GoBilda312.TICKS_PER_CENTIMETER).roundToInt()
        }
        fun ticksToCentimeters(ticks: Int): Double {
            return ticks / MotorConstants.GoBilda312.TICKS_PER_CENTIMETER
        }
        fun calcPower(target: Int, current: Int): Double {
            val p = AJDriveConstants.drive_kP * (target - current)
            val power = 2 * (1 / (1 + Math.E.pow(-p))) - 1
            return if (power > 0.1) {
                0.2
            } else {
                power
            }
        }
        fun sansSigmoid(target: Int, current: Int): Double {
            return 0.3 * (target - current)
        }
        fun calcTurnPower(target: Double, current: Double): Double {
            val p = AJDriveConstants.turn_kP * (target - current)
            return 2 * (1 / (1 + Math.E.pow(-p))) - 1
        }
        fun radToDeg(radians: Float): Double {
            return radians * 180 / Math.PI
        }
        fun degToRad(degrees: Int): Double {
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

enum class MotorConstants(val TICKS_PER_REV: Double, val WHEEL_DIAMETER: Double, val TICKS_PER_CENTIMETER: Double) {
    GoBilda312(537.7, 96.0 / 1000.0, 537.7 / ((96.0 / 10) * Math.PI))
}
@Config()
object AJDriveConstants{
    @JvmField
    var tileLength = 24 //inches

    @JvmField
    var drive_kP = 0.04

    @JvmField
    var turn_kP = 0.02

    @JvmField
    var strafeMultiplier = 1.1 // multiplier

    @JvmField
    var AutoDriveTolerance = 50 // tick

    @JvmField
    var AutoTurnTolerance = 0.5 // degree

    @JvmField
    var ClawOpen = 0.52

    @JvmField
    var ClawClose = 0.6

    @JvmField
    var SlidesSpeed = 1.0

    @JvmField
    var tallPole = 5000

    @JvmField
    var midPole = 3500

    @JvmField
    var lowPole = 2000

    @JvmField
    var aboveGround = 500
}

// create an enum class where each value is a double that represents the strength of the rumble
enum class RumbleStrength(val strength: Double) {
    LOW(0.25),
    MEDIUM(0.5),
    HIGH(0.75),
    MAX(1.0)
}