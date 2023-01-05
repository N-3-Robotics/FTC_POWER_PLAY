@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.autos

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.AutoDriveTolerance
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.AutoTurnTolerance
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.tileLength
import org.firstinspires.ftc.teamcode.utilities.QOL.Companion.calcPower
import org.firstinspires.ftc.teamcode.utilities.QOL.Companion.calcTurnPower
import org.firstinspires.ftc.teamcode.utilities.QOL.Companion.inchesToTicks
import org.firstinspires.ftc.teamcode.utilities.RobotConfig
import kotlin.math.abs


@Autonomous(name = "Auto")
class Auto: LinearOpMode() {
    var ROBOT: RobotConfig? = null
    fun forward(tiles: Int){
        val target = ROBOT!!.currentPosition + inchesToTicks(tiles * tileLength)
        while (abs(ROBOT!!.currentPosition - target) > AutoDriveTolerance) {

            val power = calcPower(target, ROBOT!!.currentPosition)

            ROBOT!!.RCDrive(power, 0.0, 0.0)
            telemetry.addData("Position", ROBOT!!.currentPosition)
            telemetry.addData("Target", target)
            telemetry.addData("Distance Remaining", abs(ROBOT!!.currentPosition - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
        ROBOT!!.stop()
    }
    fun backward(tiles: Int){
        val target = ROBOT!!.currentPosition + inchesToTicks(-tiles * tileLength)
        while (abs(ROBOT!!.currentPosition - target) > AutoDriveTolerance) {

            val power = calcPower(target, ROBOT!!.currentPosition)

            ROBOT!!.RCDrive(power, 0.0, 0.0)
            telemetry.addData("Position", ROBOT!!.currentPosition)
            telemetry.addData("Target", target)
            telemetry.addData("Distance Remaining", abs(ROBOT!!.currentPosition - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
        ROBOT!!.stop()
    }
    fun left(tiles: Int){
        val target = ROBOT!!.currentPosition + inchesToTicks(-tiles * tileLength)
        while (abs(ROBOT!!.currentPosition - target) > AutoDriveTolerance) {

            val power = calcPower(target, ROBOT!!.currentPosition)

            ROBOT!!.RCDrive(0.0, power, 0.0)
            telemetry.addData("Position", ROBOT!!.currentPosition)
            telemetry.addData("Target", target)
            telemetry.addData("Distance Remaining", abs(ROBOT!!.currentPosition - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
        ROBOT!!.stop()
    }

    fun right(tiles: Int){
        val target = ROBOT!!.currentPosition + inchesToTicks(tiles * tileLength)
        while (abs(ROBOT!!.currentPosition - target) > AutoDriveTolerance) {
            //start at 0.75 power, and decrease power curve-ly as the robot gets closer to the target

            val power = calcPower(target, ROBOT!!.currentPosition)

            ROBOT!!.RCDrive(0.0, power, 0.0)
            telemetry.addData("Position", ROBOT!!.currentPosition)
            telemetry.addData("Target", target)
            telemetry.addData("Distance Remaining", abs(ROBOT!!.currentPosition - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
        ROBOT!!.stop()
    }

    fun turnRight(angle: Int = -90){
        val target = ROBOT!!.botHeading + angle
        // while the IMU is not withing the AutoTurnTolerance of the target angle, calculate the power and turn right
        while (abs(ROBOT!!.botHeading - target) > AutoTurnTolerance) {
            val power = calcTurnPower(target, ROBOT!!.botHeading)
            ROBOT!!.RCDrive(0.0, 0.0, power)
            telemetry.addData("Angle", ROBOT!!.botHeading)
            telemetry.addData("Target", angle)
            telemetry.addData("Angle Remaining", abs(ROBOT!!.botHeading - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
    }
    fun turnLeft(angle: Int = 90){
        val target = ROBOT!!.botHeading + angle
        // while the IMU is not withing the AutoTurnTolerance of the target angle, calculate the power and turn right
        while (abs(ROBOT!!.botHeading - target) > AutoTurnTolerance) {
            val power = calcTurnPower(target, ROBOT!!.botHeading)
            ROBOT!!.RCDrive(0.0, 0.0, power)
            telemetry.addData("Angle", ROBOT!!.botHeading)
            telemetry.addData("Target", angle)
            telemetry.addData("Angle Remaining", abs(ROBOT!!.botHeading - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
    }


    
        
    override fun runOpMode() {
        ROBOT = RobotConfig(hardwareMap)
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        waitForStart()
        left(1)
    }
}
