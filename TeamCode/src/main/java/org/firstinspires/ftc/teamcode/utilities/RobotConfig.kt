@file:Suppress("unused", "NAME_SHADOWING")
package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.strafeMultiplier
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin


class RobotConfig(hwMap: HardwareMap?) {
    var fl: DcMotorEx
    var fr: DcMotorEx
    var bl: DcMotorEx
    var br: DcMotorEx

    var slides: DcMotorEx
    var claw: Servo
    var cone: Rev2mDistanceSensor
    var slidesReset: DigitalChannel
    var IMU: BNO055IMU

    val currentPosition: Int
        get() {
            return (fl.currentPosition + fr.currentPosition + bl.currentPosition + br.currentPosition) / 4
        }

    val botHeading: Float
        get() {
            return IMU.angularOrientation.firstAngle
        }

    private var hardwareMap: HardwareMap? = null

    fun funnyDrive(drive: Double, turn: Double){
        fl.power = drive + turn
        fr.power = drive - turn
        bl.power = drive + turn
        br.power = drive - turn
    }

    fun RCDrive(y: Double, x: Double, rx: Double) {
        val x = x * strafeMultiplier

        //val denominator = max(abs(y) + abs(x) + abs(rx), 1.0).toDouble()

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        val leftFrontPower: Double = y + x + rx
        val rightFrontPower: Double = y - x - rx
        val leftBackPower: Double = y - x + rx
        val rightBackPower: Double = y + x - rx

        fl.power = leftFrontPower
        bl.power = leftBackPower
        fr.power = rightFrontPower
        br.power = rightBackPower
    }

    fun FCDrive(y: Double, x: Double, turn: Double) {
        val x = x * strafeMultiplier
        val rotX = x * cos(-botHeading) - y * sin(-botHeading)
        val rotY = y * sin(-botHeading) + x * cos(-botHeading)

        val denominator = max(abs(y) + abs(x) + abs(turn), 1.0)

        fl.power = (rotY + rotX + turn) / denominator
        bl.power = (rotY - rotX + turn) / denominator
        fr.power = (rotY - rotX - turn) / denominator
        br.power = (rotY + rotX - turn) / denominator
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
        val allHubs = hardwareMap!!.getAll(LynxModule::class.java)

        fl = hardwareMap!!.get(DcMotorEx::class.java, "fl")
        fr = hardwareMap!!.get(DcMotorEx::class.java, "fr")
        bl = hardwareMap!!.get(DcMotorEx::class.java, "bl")
        br = hardwareMap!!.get(DcMotorEx::class.java, "br")

        claw = hardwareMap!!.get(Servo::class.java, "claw")
        slides = hardwareMap!!.get(DcMotorEx::class.java, "slides")
        cone = hardwareMap!!.get(Rev2mDistanceSensor::class.java, "cone")
        slidesReset = hardwareMap!!.get(DigitalChannel::class.java, "slidesReset")


        fl.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE

        //claw.direction = Servo.Direction.REVERSE
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
        fr.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        fl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        br.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        bl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        fr.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        fl.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        br.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        bl.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        slides.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        slides.direction = DcMotorSimple.Direction.REVERSE

        IMU = hardwareMap!!.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        IMU.initialize(parameters)
    }
    infix fun slidesGo(position:String)=apply{
        val slideSet = CursedCode()
        slides.targetPosition=slideSet go position
        slides.mode=DcMotor.RunMode.RUN_TO_POSITION
    }
    infix fun withPower(power:Double)=apply{
        slides.power=power
    }
}
