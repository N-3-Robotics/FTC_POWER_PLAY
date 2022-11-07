package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
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
        }


    }
}
