package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.DriveConstants.tileLength
import org.firstinspires.ftc.teamcode.DriveConstants.turnTime
import org.firstinspires.ftc.teamcode.QOL.Companion.inchesToMeters
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.QOL.Companion.inchesToTicks
import kotlin.math.exp

@Autonomous(name = "Auto")
class Auto: LinearOpMode() {

    fun forward(tiles: Int){
        val target = RC!!.FR.currentPosition + (tiles * inchesToTicks(tileLength))
        while (RC!!.FR.currentPosition < target) {
            //start at 0.75 power, and decrease power curve-ly as the robot gets closer to the target
            var power = abs(RC!!.FR.currentPosition - target) / target
            //square the power
            power *= power
            RC!!.drive(power, 0.0, 0.0)
            telemetry.addData("Position", RC!!.FR.currentPosition)
            telemetry.addData("Target", target)
            telemetry.addData("Distance Remaining", abs(RC!!.FR.currentPosition - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
        RC!!.stop()
    }
    fun backward(tiles: Int){
        val target = RC!!.FR.currentPosition - (tiles * inchesToTicks(tileLength))
        while (RC!!.FR.currentPosition > target) {

            var power = abs(RC!!.FR.currentPosition - target) / target
            power *= power

            RC!!.drive(-power, 0.0, 0.0)
            telemetry.addData("Position", RC!!.FR.currentPosition)
            telemetry.addData("Target", target)
            telemetry.addData("Distance Remaining", abs(RC!!.FR.currentPosition - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
        RC!!.stop()
    }
    fun left(tiles: Int){
        val target = RC!!.FR.currentPosition + (tiles * inchesToTicks(tileLength))
        while (RC!!.FR.currentPosition < target) {
            var power = abs(RC!!.FR.currentPosition - target) / target
            power *= power

            RC!!.drive(0.0, power, 0.0)
            telemetry.addData("Position", RC!!.FR.currentPosition)
            telemetry.addData("Target", target)
            telemetry.addData("Distance Remaining", abs(RC!!.FR.currentPosition - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
        RC!!.stop()
    }

    fun right(tiles: Int){
        val target = RC!!.FR.currentPosition - (tiles * inchesToTicks(tileLength))
        while (RC!!.FR.currentPosition > target) {
            var power = abs(RC!!.FR.currentPosition - target) / target
            power *= power

            RC!!.drive(0.0, -power, 0.0)
            telemetry.addData("Position", RC!!.FR.currentPosition)
            telemetry.addData("Target", target)
            telemetry.addData("Distance Remaining", abs(RC!!.FR.currentPosition - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
        RC!!.stop()
    }



    fun turnRight(){
        RC!!.drive(0.0, 0.0, 0.3)
        sleep(turnTime.toLong())
        RC!!.stop()
    }
    fun turnLeft(){
        RC!!.drive(0.0, 0.0, -0.3)
        sleep(turnTime.toLong())
        RC!!.stop()
    }


    var RC: RobotConfig? = null
    override fun runOpMode() {
        RC = RobotConfig(hardwareMap)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)



        waitForStart()
        forward(2)
    }
}
