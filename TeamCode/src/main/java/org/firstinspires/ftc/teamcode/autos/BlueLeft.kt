package org.firstinspires.ftc.teamcode.autos

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.utilities.Auto

@Autonomous(name = "Blue Left", group = "Blue")
class BlueLeft: Auto() {

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)


        telemetry.addLine("> Initialized")
        telemetry.update()
        waitForStart()
        telemetry.addLine("> Started")
        // add the starting position of the robot to telemetry
        telemetry.addData("Starting Position", ROBOT.currentPosition)
        telemetry.update()
        forward(1)
        telemetry.addData("Final Position", ROBOT.currentPosition)


    }

}