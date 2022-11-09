package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs

enum class Side {
    LEFT, RIGHT, BOTH
}

enum class Direction {
    FORWARD, BACKWARD, LEFT, RIGHT
}

// create an enum class where each value is a double that represents the strength of the rumble

enum class RumbleStrength(val strength: Double) {
    LOW(0.25),
    MEDIUM(0.5),
    HIGH(0.75),
    MAX(1.0)
}

class RobotConfig(hwMap: HardwareMap?) {
    var FL: DcMotorEx
    var FR: DcMotorEx
    var BL: DcMotorEx
    var BR: DcMotorEx


    private var hardwareMap: HardwareMap? = null

    fun drive(drive: Double, turn: Double, strafe: Double) {
        val max = abs(drive).coerceAtLeast(abs(strafe).coerceAtLeast(abs(turn)))

        FL.power = drive + turn + strafe
        FR.power = drive - turn - strafe
        BL.power = drive + turn - strafe
        BR.power = drive - turn + strafe

    }

    fun gamepadDrive(controller: Gamepad) {
        drive(
            -controller.right_stick_y.toDouble(),
            controller.left_stick_y.toDouble(),
            controller.right_stick_x.toDouble()
        )
    }

    fun stop() {
        drive(0.0, 0.0, 0.0)
    }

    fun pidDrive(distanceInM: Double, direction: Direction){
        val TICKS_PER_REV = ((((1+(46/17))) * (1+(46/11))) * 28)

        val WHEEL_DIAMETER = 96.0 / 1000.0

        val initialWheelPosition = FR.currentPosition

        val TICKS_PER_METER = TICKS_PER_REV / (WHEEL_DIAMETER * Math.PI)

        val Kp = 0.1
        val Ki = 0.1
        val Kd = 0.1

        val target = distanceInM * TICKS_PER_METER
        var lastReference = target
        var integralSum = 0.0
        var lastError = 0.0

        val maxIntegralSum = 0.5

        val a = 0.8
        var previousFilterEstimate = 0.0
        var currentFilterEstimate = 0.0

        val timer: ElapsedTime = ElapsedTime()

        while (abs(FR.currentPosition - initialWheelPosition) < abs(target)) {
            val error = target - (FR.currentPosition - initialWheelPosition)

            val errorChange = error - lastError

            currentFilterEstimate = a * previousFilterEstimate + (1 - a) * errorChange
            previousFilterEstimate = currentFilterEstimate

            val derivative = currentFilterEstimate / timer.seconds()

            integralSum += (error * timer.seconds())

            if (integralSum > maxIntegralSum) {
                integralSum = maxIntegralSum
            } else if (integralSum < -maxIntegralSum) {
                integralSum = -maxIntegralSum
            }

            if (target != lastReference) {
                integralSum = 0.0
            }

            val output = (Kp * error) + (Ki * integralSum) + (Kd * derivative)

            when (direction) {
                Direction.FORWARD -> drive(output, 0.0, 0.0)
                Direction.BACKWARD -> drive(-output, 0.0, 0.0)
                Direction.LEFT -> drive(0.0, 0.0, output)
                Direction.RIGHT -> drive(0.0, 0.0, -output)
            }

            lastError = error
            lastReference = target
            timer.reset()
        }
    }

    fun rumble(controller: Gamepad, side: Side, power: RumbleStrength, duration: Int = 100) {
        val pwr = power.strength
        when (side) {
            Side.LEFT -> {
                controller.rumble(pwr, 0.0, duration)
            }
            Side.RIGHT -> {
                controller.rumble(0.0, pwr, duration)
            }
            Side.BOTH -> {
                controller.rumble(pwr / 2, pwr / 2, duration)
            }
        }
    }

    init {
        hardwareMap = hwMap

        FL = hardwareMap!!.get(DcMotorEx::class.java, "FL")
        FR = hardwareMap!!.get(DcMotorEx::class.java, "FR")
        BL = hardwareMap!!.get(DcMotorEx::class.java, "BL")
        BR = hardwareMap!!.get(DcMotorEx::class.java, "BR")

        FL.direction = DcMotorSimple.Direction.REVERSE
        BL.direction = DcMotorSimple.Direction.REVERSE
    }
}
