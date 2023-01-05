@file:Suppress("unused", "NAME_SHADOWING")
package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.*
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

    var CLAW: Servo

    var CONE_SENSOR: Rev2mDistanceSensor

    var IMU: BNO055IMU

    val currentPosition: Int
        get() {
            return (FL.currentPosition + FR.currentPosition + BL.currentPosition + BR.currentPosition) / 4
        }

    val botHeading: Float
        get() {
            return IMU.angularOrientation.firstAngle
        }

    private var hardwareMap: HardwareMap? = null

    fun funnyDrive(drive: Double, turn: Double){
        FL.power = drive + turn
        FR.power = drive - turn
        BL.power = drive + turn
        BR.power = drive - turn
    }

    fun RCDrive(drive: Double, strafe: Double, turn: Double) {
        var max: Double
        var leftFrontPower: Double = drive + strafe + turn
        var rightFrontPower: Double = drive - strafe - turn
        var leftBackPower: Double = drive - strafe + turn
        var rightBackPower: Double = drive + strafe - turn

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = max(abs(leftFrontPower), abs(rightFrontPower))
        max = max(max, abs(leftBackPower))
        max = max(max, abs(rightBackPower))

        if (max > 1.0) {
            leftFrontPower /= max
            rightFrontPower /= max
            leftBackPower /= max
            rightBackPower /= max
        }
        FL.power = leftFrontPower
        FR.power = rightFrontPower
        BL.power = leftBackPower
        BR.power = rightBackPower

    }

    fun FCDrive(x: Double, y: Double, turn: Double) {
        val x = x * 1.1
        val rotX = x * cos(-botHeading) - y * sin(-botHeading)
        val rotY = x * sin(-botHeading) + y * cos(-botHeading)

        val denominator = max(abs(y) + abs(x) + abs(turn), 1.0)

        FL.power = (rotY + rotX + turn) / denominator
        BL.power = (rotY - rotX + turn) / denominator
        FR.power = (rotY - rotX - turn) / denominator
        BR.power = (rotY + rotX - turn) / denominator
    }

    fun gamepadDrive(controller: Gamepad, multiplier: Double) {
        RCDrive(
            controller.left_stick_x.toDouble() * multiplier,
            -controller.left_stick_y.toDouble() * multiplier,
            controller.right_stick_x.toDouble() * multiplier
        )
    }

    fun stop() {
        RCDrive(0.0, 0.0, 0.0)
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

    init {
        hardwareMap = hwMap


        FL = hardwareMap!!.get(DcMotorEx::class.java, "FL")
        FR = hardwareMap!!.get(DcMotorEx::class.java, "FR")
        BL = hardwareMap!!.get(DcMotorEx::class.java, "BL")
        BR = hardwareMap!!.get(DcMotorEx::class.java, "BR")

        CLAW = hardwareMap!!.get(Servo::class.java, "CLAW")

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
