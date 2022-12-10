package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs

@Config()
object Vars {
    @JvmField
    var driveTime: Long = 1600

    @JvmField
    var strafeTime: Long = 2100

    @JvmField
    var turnTime: Long = 1050

    var tileLength = 23.5
}

@Autonomous(name = "Auto")
class Auto: LinearOpMode() {

    //take the ticks per revolution, and turn it into ticks per meter
    var WHEEL_DIAMETER: Double = 0.0

    //convert ticks per rev to ticks per meter
    var TICKS_PER_METER: Double = 0.0


    fun forward(tiles: Int){
        //while the front right wheel hasn't traveled 23.5 inches, drive forward at 0.3 power
        var target = RC!!.FR.currentPosition + (tiles * TICKS_PER_METER)
        while (RC!!.FR.currentPosition < target) {
            RC!!.drive(0.3, 0.0, 0.0)
            telemetry.addData("Position", RC!!.FR.currentPosition)
            telemetry.addData("Target", target)
            telemetry.update()
        }
        RC!!.stop()
    }


    fun turnRight(){
        RC!!.drive(0.0, 0.0, 0.3)
        sleep(Vars.turnTime)
        RC!!.stop()
    }
    fun turnLeft(){
        RC!!.drive(0.0, 0.0, -0.3)
        sleep(Vars.turnTime)
        RC!!.stop()
    }


    var RC: RobotConfig? = null
    override fun runOpMode() {
        RC = RobotConfig(hardwareMap)

        WHEEL_DIAMETER = 96.0 / 1000.0

        TICKS_PER_METER = RC!!.TICKS_PER_REV_312 / (WHEEL_DIAMETER * Math.PI)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        waitForStart()
        //use the pidDrive function to drive in a 1 meter circle


        forward(2)
    }
}
