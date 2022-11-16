package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.RumbleStrength.*
import org.firstinspires.ftc.teamcode.Side.*

@Autonomous(name = "Auto")
class Auto: LinearOpMode() {
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

            //use the pidDrive function to drive in a 1 meter circle
            RC!!.pidDrive(0.5, Direction.FORWARD)
            sleep(15000)
            RC!!.pidDrive(0.5, Direction.BACKWARD)
        }
    }
}