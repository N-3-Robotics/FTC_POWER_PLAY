package org.firstinspires.ftc.teamcode.utilities

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@Config()
object ConePos {
    @JvmField
    var ConePosition = 0.0
}

@TeleOp(name = "CLAW FINDER")
public class ClawPosFinder: LinearOpMode() {
    var RC: RobotConfig? = null
    override fun runOpMode() {
        RC = RobotConfig(hardwareMap)
        waitForStart()
        while (opModeIsActive()) {
            RC!!.CLAW.power = ConePos.ConePosition
            telemetry.addData("Cone Position", ConePos.ConePosition)
            telemetry.update()
        }
    }
}