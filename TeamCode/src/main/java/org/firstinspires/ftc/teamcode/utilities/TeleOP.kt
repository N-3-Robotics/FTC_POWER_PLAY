package org.firstinspires.ftc.teamcode.utilities

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.ClawClose
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.ClawOpen

open class TeleOP: LinearOpMode() {
    val ROBOT = RobotConfig(hardwareMap)

    fun closeClaw(claw: CRServo) {
        claw.power = ClawClose
    }
    fun openClaw(claw: CRServo){
        claw.power = ClawOpen
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)


        // Rumble the left controller medium once, and the right controller medium twice, at the same time, using the ROBOT.rumble function
        ROBOT.rumble(gamepad1, Side.BOTH, RumbleStrength.MEDIUM, 100)
        ROBOT.rumble(gamepad2, Side.BOTH, RumbleStrength.MEDIUM, 100)
        ROBOT.rumble(gamepad2, Side.BOTH, RumbleStrength.MEDIUM, 100)
        
    }

}