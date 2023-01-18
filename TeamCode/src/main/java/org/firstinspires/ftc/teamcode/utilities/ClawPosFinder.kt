package org.firstinspires.ftc.teamcode.utilities

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robot.Robot

@Config()
object ConePos {
    @JvmField
    var ConePosition = 0.0
}

@TeleOp(name = "CLAW FINDER")
class ClawPosFinder: LinearOpMode() {
    var RC: Robot? = null
    override fun runOpMode() {
        RC = Robot(hardwareMap)
        waitForStart()
        while (opModeIsActive()) {
            RC!!.CLAW.position = ConePos.ConePosition
            telemetry.addData("Cone Position", ConePos.ConePosition)
            telemetry.update()
        }
    }
}