package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name="TeleOP")
class TeleOP: LinearOpMode() {
    var RC: RobotConfig? = null

    override fun runOpMode() {

        RC = RobotConfig(hardwareMap)

        waitForStart()

        while (opModeIsActive()){

            RC!!.gamepadDrive(gamepad1, 1.0)

            if (-gamepad2.right_stick_y.toDouble() > 0.0) {
                RC!!.SLIDES_RIGHT.power = -gamepad2.right_stick_y.toDouble()
                RC!!.SLIDES_LEFT.power = -gamepad2.right_stick_y.toDouble()
            }

        }

    }

}