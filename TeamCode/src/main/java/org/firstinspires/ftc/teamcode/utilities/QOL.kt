@file:Suppress("unused")
package org.firstinspires.ftc.teamcode.utilities

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.gearRatio
import org.firstinspires.ftc.teamcode.utilities.QOL.Companion.rpmToVelocity
import kotlin.math.pow
import kotlin.math.roundToInt

class QOL {
    companion object {
        fun inchesToTicks(inches: Number): Int {
            return (inches.toDouble() * MotorConstants.GoBilda312.TICKS_PER_INCH).roundToInt()
        }
        fun ticksToInches(ticks: Number): Double {
            return ticks.toDouble() / MotorConstants.GoBilda312.TICKS_PER_INCH
        }
        fun calcPower(target: Int, current: Int): Double {
            val p = DriveConstants.drive_kP * (target - current)
            val power = 2 * (1 / (1 + Math.E.pow(-p))) - 1
            return if (power > 0.2) {
                0.2
            } else {
                power
            }
        }
        fun sansSigmoid(target: Int, current: Int): Double {
            return 0.3 * (target - current)
        }
        fun calcTurnPower(target: Double, current: Double): Double {
            val p = DriveConstants.turn_kP * (target - current)
            return 2 * (1 / (1 + Math.E.pow(-p))) - 1
        }
        fun radToDeg(radians: Float): Double {
            return radians * 180 / Math.PI
        }
        fun degToRad(degrees: Int): Double {
            return degrees * Math.PI / 180
        }
        fun rpmToVelocity(rpm: Double): Double {
            return rpm * gearRatio * MotorConstants.GoBilda312.WHEEL_DIAMETER * Math.PI / 60
        }
        fun rED(current: Boolean, previous: Boolean): Boolean { // Rising Edge Detector
            return current && !previous
        }
        fun lerp(p0: Double, p1: Double, t: Double) : Double {
            return p0 * (1.0 - t) + (p1 * t)
        }
    }
}

enum class Side {
    LEFT, RIGHT, BOTH
}

enum class Direction {
    FORWARD, BACKWARD, LEFT, RIGHT
}

enum class AutoMode {
    TURN, STRAIGHT, UNKNOWN, MANUAL
}

enum class LiftControlType {
    MANUAL, PID
}

enum class MotorConstants(val MAX_RPM: Double, val TICKS_PER_REV: Double, val WHEEL_DIAMETER: Double, val TICKS_PER_INCH: Double) {
    GoBilda312(312.0, 537.7, 96.0 / 25.4, 537.7 / ((96.0 / 25.4) * Math.PI))
}
@Config()
object DriveConstants{
    @JvmField
    var tileLength = 24 //inches

    @JvmField
    var gearRatio = 1.02
    var WHEEL_RADIUS = MotorConstants.GoBilda312.WHEEL_DIAMETER / 2

    @JvmField
    var kV = 1.0 / rpmToVelocity(MotorConstants.GoBilda312.MAX_RPM)
    @JvmField
    var kA = 0.0
    @JvmField
    var kStatic = 0.0

    var MAX_VEL = 50.0
    var MAX_ACCEL = 45.0
    var MAX_ANG_VEL = Math.toRadians(310.0)
    var MAX_ANG_ACCEL = Math.toRadians(60.0)


    @JvmField
    var drive_kP = 0.04

    @JvmField
    var drive_kI = 0.00

    @JvmField
    var drive_kD = 0.00

    @JvmField
    var heading_kP = 0.02

    @JvmField
    var heading_kI = 0.00

    @JvmField
    var heading_kD = 0.00

    @JvmField
    var turn_kP = 0.02

    @JvmField
    var turn_kI = 0.01

    @JvmField
    var turn_kD = 0.0

    @JvmField
    var strafeMultiplier = 1.1 // multiplier

    @JvmField
    var AutoDriveTolerance = 50 // tick

    @JvmField
    var AutoTurnTolerance = 0.25 // degree

    @JvmField
    var SlidesTolerance = 50 // tick

    @JvmField
    var ClawOpen = 0.52

    @JvmField
    var ClawClose = 0.6

    var SlidesSpeed = 1.0

    var SlidesMax = 5359

    var SlidesMin = 0

    var highPole = SlidesMax

    var midPole = SlidesMax / 3 * 2

    var lowPole = SlidesMax / 3

    var slightRaise = 300

    @JvmField
    var HoldingPower = 0.0001
}

// create an enum class where each value is a double that represents the strength of the rumble
enum class RumbleStrength(val strength: Double) {
    LOW(0.25),
    MEDIUM(0.5),
    HIGH(0.75),
    MAX(1.0)
}