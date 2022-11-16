package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.RumbleStrength.*
import org.firstinspires.ftc.teamcode.Side.*

@TeleOp(name = "TeleOP")
class TeleOP: LinearOpMode() {
    var RC: RobotConfig? = null
    override fun runOpMode() {

        RC = RobotConfig(hardwareMap)

        while(!opModeIsActive()){
            telemetry.addData("Status", "Waiting for start")
            telemetry.update()

            RC!!.rumble(gamepad1, LEFT, HIGH)
            RC!!.rumble(gamepad2, RIGHT, HIGH)

        }
        waitForStart()

        while (opModeIsActive()) {
            RC!!.gamepadDrive(gamepad1)

            if (RC!!.SLIDES.currentPosition < 0) {
                RC!!.SLIDES.power = 0.0
                RC!!.SLIDES.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            } else {
                gamepad2.left_stick_y.toDouble()
            }
            RC!!.ARM.power = gamepad2.left_stick_y.toDouble()
            RC!!.TURRET.power = gamepad2.left_stick_x.toDouble()

            telemetry.addData("SLIDES position", RC!!.SLIDES.currentPosition)
            telemetry.update()
        }


    }
}
