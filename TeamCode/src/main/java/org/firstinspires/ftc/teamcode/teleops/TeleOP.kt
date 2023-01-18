package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.outoftheboxrobotics.photoncore.PhotonCore
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.utilities.*
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.SlidesSpeed
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.highPole
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.lowPole
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.midPole
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.slightRaise
import kotlin.math.abs

@TeleOp(name = "TeleOp")
class TeleOP: LinearOpMode() {
    override fun runOpMode() {

        PhotonCore.enable()

        val timer = ElapsedTime()


        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val ROBOT = Robot(hardwareMap)

        var lastLiftState = false

        var m = 1.0

        while (!opModeIsActive()){
            ROBOT.rumble(gamepad1, Side.BOTH, RumbleStrength.HIGH)
            ROBOT.rumble(gamepad2, Side.BOTH, RumbleStrength.HIGH)
        }

        var liftState = LiftControlType.MANUAL

        var slideHeight = 0

        var coneGrabbed = false

        waitForStart()

        while (opModeIsActive()){
            telemetry.addData("Loop Time", timer.milliseconds())
            timer.reset()


            // Slides Control

            val slidesResetState = ROBOT.SLIDES_RESET.state
            val slidesPos = ROBOT.SLIDES.currentPosition

            if (slidesPos < 750){
                coneGrabbed = if (ROBOT.coneDetected){
                    ROBOT.closeClaw()
                    true
                } else {
                    false
                }
            }

            when (liftState) {
                LiftControlType.MANUAL -> {
                    if (gamepad2.circle || gamepad2.square || gamepad2.cross || gamepad2.triangle || coneGrabbed) {
                        ROBOT.rumble(gamepad2, Side.RIGHT, RumbleStrength.LOW)
                        liftState = LiftControlType.PID
                    }
                    else if (!lastLiftState) {
                        ROBOT.SLIDES.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                    }
                    else if (abs(slidesPos) > 5359){
                        ROBOT.SLIDES.stop()
                    }
                    else {
                        ROBOT.SLIDES.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                        ROBOT.SLIDES.power = gamepad2.right_stick_y * SlidesSpeed
                    }
                }
                LiftControlType.PID -> {
                    if (gamepad2.cross){
                        ROBOT.SLIDES.goTo(0)
                    }
                    else if (gamepad2.triangle){
                        ROBOT.SLIDES.goTo(highPole)
                    }
                    else if (gamepad2.square){
                        ROBOT.SLIDES.goTo(midPole)
                    }
                    else if (gamepad2.circle){
                        ROBOT.SLIDES.goTo(lowPole)
                    }
                    else if (coneGrabbed){
                        ROBOT.SLIDES.goTo(slightRaise)
                    }
                    else if (gamepad2.ps || abs(gamepad2.right_stick_y) > 0.15){
                        liftState = LiftControlType.MANUAL
                    }
                }
            }

            lastLiftState = slidesResetState

            // Claw control

            when {
                gamepad2.left_bumper -> {
                    ROBOT.openClaw()
                    coneGrabbed = false
                }
                gamepad2.right_bumper -> {
                    ROBOT.closeClaw()
                    coneGrabbed = false
                }
            }

            when {
                gamepad1.cross -> {
                    m = 0.25
                }
                gamepad1.circle -> {
                    m = 0.5
                }
                gamepad1.square -> {
                    m = 0.75
                }
                gamepad1.triangle -> {
                    m = 1.0
                }
            }


            // Drivetrain Control
            ROBOT.gamepadDrive(gamepad1, m)

            telemetry.addData("Slides Pos", slidesPos)
            telemetry.addData("Slides Reset State", slidesResetState)
            telemetry.addData("Lift State", liftState)

            telemetry.update()
        }

        
    }

}