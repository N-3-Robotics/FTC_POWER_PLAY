package org.firstinspires.ftc.teamcode.autos

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.utilities.Auto
import org.firstinspires.ftc.teamcode.utilities.QOL.Companion.ticksToInches
import kotlin.math.abs

@Autonomous(name = "Blue Left", group = "Blue")
class BlueLeft: Auto() {

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)


        telemetry.addLine("> Initialized")
        telemetry.update()
        waitForStart()
        telemetry.addLine("> Started")
        // add the starting position of the robot to telemetry
        val startingPos = ROBOT.currentPosition
        telemetry.addData("Starting Position", ROBOT.currentPosition)

        forward(1)
        telemetry.addData("Final Position", ROBOT.currentPosition)
        telemetry.addData("Distance Traveled", ticksToInches(abs(startingPos - ROBOT.currentPosition)))
        telemetry.update()


    }

}