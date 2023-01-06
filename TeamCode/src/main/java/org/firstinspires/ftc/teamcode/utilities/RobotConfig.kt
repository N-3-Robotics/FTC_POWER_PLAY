@file:Suppress("unused", "NAME_SHADOWING")
package org.firstinspires.ftc.teamcode.utilities

import android.R.attr.x
import android.R.attr.y
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.strafeMultiplier
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

    fun RCDrive(y: Double, x: Double, rx: Double) {
        val x = x * strafeMultiplier

        val denominator = max(abs(y) + abs(x) + abs(rx), 1.0).toDouble()
        val frontLeftPower: Double = (y + x + rx) / denominator
        val backLeftPower: Double = (y - x + rx) / denominator
        val frontRightPower: Double = (y - x - rx) / denominator
        val backRightPower: Double = (y + x - rx) / denominator

        FL.power = frontLeftPower
        BL.power = backLeftPower
        FR.power = frontRightPower
        BR.power = backRightPower
    }

    fun FCDrive(x: Double, y: Double, turn: Double) {
        val x = x * strafeMultiplier
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
            -controller.left_stick_y.toDouble() * multiplier,
            controller.left_stick_x.toDouble() * multiplier,
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
