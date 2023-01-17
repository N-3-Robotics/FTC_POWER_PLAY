package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robot.RobotConfig
import org.firstinspires.ftc.teamcode.utilities.*
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.ClawClose
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.ClawOpen
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.HoldingPower
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.SlidesMax
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.SlidesMin
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.SlidesSpeed

@TeleOp(name = "TeleOp")
class TeleOP: LinearOpMode() {
    var ROBOT: RobotConfig? = null

    fun closeClaw(claw: Servo = ROBOT!!.CLAW) {
        claw.position = ClawClose
    }
    fun openClaw(claw: Servo = ROBOT!!.CLAW) {
        claw.position = ClawOpen
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        ROBOT = RobotConfig(hardwareMap)

        // Rumble the left controller medium once, and the right controller medium twice, at the same time, using the ROBOT!!.rumble function
        ROBOT!!.rumble(gamepad1, Side.BOTH, RumbleStrength.MEDIUM, 100)
        ROBOT!!.rumble(gamepad2, Side.BOTH, RumbleStrength.MEDIUM, 100)
        ROBOT!!.rumble(gamepad2, Side.BOTH, RumbleStrength.MEDIUM, 100)

        waitForStart()
        openClaw()

        val timer = ElapsedTime()

        while (opModeIsActive()) {
            timer.reset()

            ROBOT!!.gamepadDrive(gamepad1, 1.0)

            // setup the claw motor to open and close
            when {
                gamepad2.left_bumper -> {
                    openClaw()
                }
                gamepad2.right_bumper -> {
                    closeClaw()
                }
            }

            if (gamepad2.right_stick_y.toDouble() == 0.0) {
                ROBOT!!.SLIDES.power = HoldingPower
            }
            else {
                 if (!ROBOT!!.SLIDES.isBusy){
                     ROBOT!!.SLIDES.mode = DcMotor.RunMode.RUN_USING_ENCODER
                     if (ROBOT!!.SLIDES.currentPosition > SlidesMax) {
                         ROBOT!!.SLIDES.power = -0.1
                     }
                     else if (ROBOT!!.SLIDES.currentPosition < SlidesMin){
                         ROBOT!!.SLIDES.power = 0.1
                     }
                     else {

                         ROBOT!!.SLIDES.power = -gamepad2.right_stick_y.toDouble() * SlidesSpeed
                     }
                 }
            }

            if (ROBOT!!.coneDetected && ROBOT!!.SLIDES.currentPosition < 500) {
                telemetry.addData("Cone Sensor", "Cone Detected")
                closeClaw()
                ROBOT!!.SLIDES.targetPosition += 200
                ROBOT!!.SLIDES.mode = DcMotor.RunMode.RUN_TO_POSITION
                ROBOT!!.SLIDES.power = 1.0
            } else {
                telemetry.addData("Cone Sensor", "No Cone Detected")
            }

            //add the loop time of the program to the telemetry
            telemetry.addData("Loop Time", timer.milliseconds())

            telemetry.addData("Slides Position", ROBOT!!.SLIDES.currentPosition)

            telemetry.update()
        }
        
    }

}