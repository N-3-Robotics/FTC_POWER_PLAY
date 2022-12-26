package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin


class RobotConfig(hwMap: HardwareMap?) {
    var FL: DcMotorEx
    var FR: DcMotorEx
    var BL: DcMotorEx
    var BR: DcMotorEx

    var SLIDES: DcMotorEx

    var CLAW: CRServo

    var CONE_SENSOR: Rev2mDistanceSensor

    var IMU: BNO055IMU

    val currentPosition: Int
        get() {
            return (FL.currentPosition + FR.currentPosition + BL.currentPosition + BR.currentPosition) / 4
        }

    private var hardwareMap: HardwareMap? = null

    fun funnyDrive(drive: Double, turn: Double){
        FL.power = drive + turn
        FR.power = drive - turn
        BL.power = drive + turn
        BR.power = drive - turn
    }

    fun RCDrive(drive: Double, strafe: Double, turn: Double) {
        var max: Double;
        var leftFrontPower: Double = drive + strafe + turn
        var rightFrontPower: Double = drive - strafe - turn
        var leftBackPower: Double = drive - strafe + turn
        var rightBackPower: Double = drive + strafe - turn

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

    fun FCDrive(x: Double, y: Double, turn: Double, telemetry: Telemetry) {
        val botHeading: Double = -IMU.angularOrientation.firstAngle.toDouble()
        val rotX = x * cos(botHeading) - y * sin(botHeading)
        val rotY = x * sin(botHeading) + y * cos(botHeading)

        val denominator = max(abs(y) + abs(x) + abs(turn), 1.0)

        FL.power = (rotY + rotX + turn) / denominator
        BL.power = (rotY - rotX + turn) / denominator
        FR.power = (rotY - rotX - turn) / denominator
        BR.power = (rotY + rotX - turn) / denominator
    }

    fun gamepadDrive(controller: Gamepad, multiplier: Double, telemetry: Telemetry) {
        FCDrive(
            -controller.left_stick_y.toDouble() * multiplier,
            controller.left_stick_x.toDouble() * multiplier,
            controller.right_stick_x.toDouble() * multiplier,
            telemetry
        )
    }

    fun stop() {
        RCDrive(0.0, 0.0, 0.0)
    }

    // PID Turn function
    fun pidConeTrackingTurn(reference: Double, kP: Double, kI: Double, kD: Double) {
        val target = 320.0
        var integralSum = 0.0
        var lastError = 0.0

        val timer = ElapsedTime()

        if (abs(reference - target) > 20){
            val error = target - reference
            val errorChange = error - lastError
            val derivative = errorChange / timer.seconds()

            integralSum += error * timer.seconds()

            val out = (kP * error) + (kI * integralSum) + (kD * derivative)

            funnyDrive(0.0, out)
            lastError = error
            timer.reset()
        }
    }

    fun lerp(p0: Double, p1: Double, t: Double) : Double {
        return p0 * (1.0 - t) + (p1 * t)
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

    fun getHeading(): Double {
        return IMU.angularOrientation.firstAngle.toDouble()
    }

    init {
        hardwareMap = hwMap


        FL = hardwareMap!!.get(DcMotorEx::class.java, "FL")
        FR = hardwareMap!!.get(DcMotorEx::class.java, "FR")
        BL = hardwareMap!!.get(DcMotorEx::class.java, "BL")
        BR = hardwareMap!!.get(DcMotorEx::class.java, "BR")

        CLAW = hardwareMap!!.get(CRServo::class.java, "CLAW")

        SLIDES = hardwareMap!!.get(DcMotorEx::class.java, "SLIDES")

        CONE_SENSOR = hardwareMap!!.get(Rev2mDistanceSensor::class.java, "CONE_SENSOR")


        BL.direction = DcMotorSimple.Direction.REVERSE
        BR.direction = DcMotorSimple.Direction.REVERSE

        //CLAW.direction = Servo.Direction.REVERSE

        FR.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        FL.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        BR.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        BL.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        SLIDES.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        IMU = hardwareMap!!.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        IMU.initialize(parameters)
    }
}
