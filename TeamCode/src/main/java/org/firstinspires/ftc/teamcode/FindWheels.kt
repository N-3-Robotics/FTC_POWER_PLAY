package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "FindWheels")
class FindWheels: LinearOpMode() {
    var RC: RobotConfig? = null

    override fun runOpMode() {

        RC = RobotConfig(hardwareMap)

        while(!opModeIsActive()){
            telemetry.addData("Status", "Waiting for start")
            telemetry.update()
        }
        waitForStart()
        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {RC!!.FR.power = 1.0} else {RC!!.FR.power = 0.0}
            if (gamepad1.left_bumper) {RC!!.FL.power = 1.0} else {RC!!.FL.power = 0.0}
            if (gamepad1.right_trigger.toDouble() > 0.1) {RC!!.BR.power = 1.0} else {RC!!.BR.power = 0.0}
            if (gamepad1.left_trigger.toDouble() > 0.1) {RC!!.BL.power = 1.0} else {RC!!.BL.power = 0.0}

            RC!!.SLIDES_LEFT.power = if (gamepad1.dpad_left) {1.0} else {0.0}
            RC!!.SLIDES_RIGHT.power = if (gamepad1.dpad_right) {1.0} else {0.0}

            //set up a system to measuer the loop time of the program
            val startTime = System.currentTimeMillis()
            //set up a system to measuer the loop time of the program
            val endTime = System.currentTimeMillis()
            val loopTime = endTime - startTime
            telemetry.addData("Loop Time", loopTime)
            telemetry.update()
        }
    }
}
