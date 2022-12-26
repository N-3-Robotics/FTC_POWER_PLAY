package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.DriveConstants.AutoTurnTolerance
import org.firstinspires.ftc.teamcode.DriveConstants.tileLength
import org.firstinspires.ftc.teamcode.QOL.Companion.calcPower
import org.firstinspires.ftc.teamcode.QOL.Companion.calcTurnPower
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.QOL.Companion.inchesToTicks

@Autonomous(name = "Auto")
class Auto: LinearOpMode() {
    fun forward(tiles: Int){
        val target = RC!!.FR.currentPosition + inchesToTicks(tiles * tileLength)
        while (abs(RC!!.FR.currentPosition - target) > 50) {

            val power = calcPower(target, RC!!.FR.currentPosition)

            RC!!.RCDrive(power, 0.0, 0.0)
            telemetry.addData("Position", RC!!.FR.currentPosition)
            telemetry.addData("Target", target)
            telemetry.addData("Distance Remaining", abs(RC!!.FR.currentPosition - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
        RC!!.stop()
    }
    fun backward(tiles: Int){
        val target = RC!!.FR.currentPosition + inchesToTicks(-tiles * tileLength)
        while (abs(RC!!.FR.currentPosition - target) > 50) {

            val power = calcPower(target, RC!!.FR.currentPosition)

            RC!!.RCDrive(power, 0.0, 0.0)
            telemetry.addData("Position", RC!!.FR.currentPosition)
            telemetry.addData("Target", target)
            telemetry.addData("Distance Remaining", abs(RC!!.FR.currentPosition - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
        RC!!.stop()
    }
    fun left(tiles: Int){
        val target = RC!!.FR.currentPosition + inchesToTicks(-tiles * tileLength)
        while (abs(RC!!.FR.currentPosition - target) > 50) {

            val power = calcPower(target, RC!!.FR.currentPosition)

            RC!!.RCDrive(0.0, power, 0.0)
            telemetry.addData("Position", RC!!.FR.currentPosition)
            telemetry.addData("Target", target)
            telemetry.addData("Distance Remaining", abs(RC!!.FR.currentPosition - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
        RC!!.stop()
    }

    fun right(tiles: Int){
        val target = RC!!.FR.currentPosition + inchesToTicks(tiles * tileLength)
        while (abs(RC!!.FR.currentPosition - target) > 50) {
            //start at 0.75 power, and decrease power curve-ly as the robot gets closer to the target

            val power = calcPower(target, RC!!.FR.currentPosition)

            RC!!.RCDrive(0.0, power, 0.0)
            telemetry.addData("Position", RC!!.FR.currentPosition)
            telemetry.addData("Target", target)
            telemetry.addData("Distance Remaining", abs(RC!!.FR.currentPosition - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
        RC!!.stop()
    }


    // #TODO: Implement Proportional turn control, using the IMU.
    fun turnRight(angle: Int = -90){
        val target = RC!!.IMU.angularOrientation.firstAngle + angle
        // while the IMU is not withing the AutoTurnTolerance of the target angle, calculate the power and turn right
        while (abs(RC!!.IMU.angularOrientation.firstAngle - target) > AutoTurnTolerance) {
            val power = calcTurnPower(target, RC!!.IMU.angularOrientation.firstAngle)
            RC!!.RCDrive(0.0, 0.0, power)
            telemetry.addData("Angle", RC!!.IMU.angularOrientation.firstAngle)
            telemetry.addData("Target", angle)
            telemetry.addData("Angle Remaining", abs(RC!!.IMU.angularOrientation.firstAngle - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
    }
    fun turnLeft(angle: Int = 90){
        val target = RC!!.IMU.angularOrientation.firstAngle + angle
        // while the IMU is not withing the AutoTurnTolerance of the target angle, calculate the power and turn right
        while (abs(RC!!.IMU.angularOrientation.firstAngle - target) > AutoTurnTolerance) {
            val power = calcTurnPower(target, RC!!.IMU.angularOrientation.firstAngle)
            RC!!.RCDrive(0.0, 0.0, power)
            telemetry.addData("Angle", RC!!.IMU.angularOrientation.firstAngle)
            telemetry.addData("Target", angle)
            telemetry.addData("Angle Remaining", abs(RC!!.IMU.angularOrientation.firstAngle - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
    }


    var RC: RobotConfig? = null
    override fun runOpMode() {
        RC = RobotConfig(hardwareMap)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)



        waitForStart()
        forward(2)
    }
}
