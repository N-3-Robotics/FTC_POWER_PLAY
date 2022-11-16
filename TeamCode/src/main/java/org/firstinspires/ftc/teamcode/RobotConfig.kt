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

    var TURRET: DcMotorEx
    var SLIDES: DcMotorEx
    var ARM: DcMotorEx

    var CLAW: Servo
    var CLAW_ROTATE: Servo


    private var hardwareMap: HardwareMap? = null

    fun drive(drive: Double, turn: Double, strafe: Double) {
        var max: Double;
        var leftFrontPower: Double = drive + turn + strafe
        var rightFrontPower: Double = drive - turn - strafe
        var leftBackPower: Double = drive - turn + strafe
        var rightBackPower: Double = drive + turn - strafe

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower))
        max = Math.max(max, Math.abs(leftBackPower))
        max = Math.max(max, Math.abs(rightBackPower))

        if (max > 1.0) {
            leftFrontPower /= max
            rightFrontPower /= max
            leftBackPower /= max
            rightBackPower /= max
        }
        FL.setPower(leftFrontPower)
        FR.setPower(rightFrontPower)
        BL.setPower(leftBackPower)
        BR.setPower(rightBackPower)

    }

    fun gamepadDrive(controller: Gamepad) {
        drive(
            -controller.right_stick_y.toDouble(),
            controller.left_stick_x.toDouble(),
            controller.right_stick_x.toDouble()
        )
    }

    fun stop() {
        drive(0.0, 0.0, 0.0)
    }

    fun pidDrive(distanceInM: Double, direction: Direction){
        val TICKS_PER_REV = ((((1+(46/17))) * (1+(46/11))) * 28)

        val WHEEL_DIAMETER = 96.0 / 1000.0

        val initialWheelPosition = FL.currentPosition

        val TICKS_PER_METER = TICKS_PER_REV / (WHEEL_DIAMETER * Math.PI)

        val Kp = 25.0
        val Ki = 0.0
        val Kd = 18.0

        val target = distanceInM * TICKS_PER_METER
        var lastReference = target
        var integralSum = 0.0
        var lastError = 0.0

        val maxIntegralSum = 0.5

        val a = 0.8
        var previousFilterEstimate = 0.0
        var currentFilterEstimate = 0.0

        val timer: ElapsedTime = ElapsedTime()

        while (abs(FL.currentPosition - initialWheelPosition) < abs(target)) {
            val error = target - (FL.currentPosition - initialWheelPosition)

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


        FL = hardwareMap!!.get(DcMotorEx::class.java, "fl")
        FR = hardwareMap!!.get(DcMotorEx::class.java, "fr")
        BL = hardwareMap!!.get(DcMotorEx::class.java, "bl")
        BR = hardwareMap!!.get(DcMotorEx::class.java, "br")

        TURRET = hardwareMap!!.get(DcMotorEx::class.java, "turret")
        SLIDES = hardwareMap!!.get(DcMotorEx::class.java, "slides")
        ARM = hardwareMap!!.get(DcMotorEx::class.java, "arm")

        CLAW = hardwareMap!!.get(Servo::class.java, "claw")
        CLAW_ROTATE = hardwareMap!!.get(Servo::class.java, "clawAngle")




        FL.direction = DcMotorSimple.Direction.REVERSE
        BL.direction = DcMotorSimple.Direction.REVERSE
    }
}
