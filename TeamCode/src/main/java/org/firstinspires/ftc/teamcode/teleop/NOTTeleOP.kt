package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.utilities.*
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.ClawClose
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.ClawOpen

@Disabled
@TeleOp(name = "NOT A TeleOp")
class NOTTeleOP: LinearOpMode() {
    var ROBOT: RobotConfig? = null

    fun closeClaw(claw: Servo = ROBOT!!.claw) {
        claw.position = ClawClose
    }
    fun openClaw(claw: Servo = ROBOT!!.claw) {
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

            val distanceSensorReading = ROBOT!!.cone.getDistance(DistanceUnit.INCH);

            telemetry.addData("Distance Sensor", distanceSensorReading)

            ROBOT!!.gamepadDrive(gamepad1, 1.0)

            ROBOT!!.slides.power = gamepad2.right_stick_y.toDouble() * DriveConstants.SlidesSpeed

            // setup the claw motor to open and close
            when {
                gamepad2.left_bumper -> {
                    openClaw()
                }
                gamepad2.right_bumper -> {
                    closeClaw()
                }
            }


            if (distanceSensorReading <= 2.3 && kotlin.math.abs(ROBOT!!.slides.currentPosition) <500) { //change this value to more or less if it doesn't work
                telemetry.addData("Cone Sensor", "Cone Detected")
                closeClaw()

            } else {
                telemetry.addData("Cone Sensor", "No Cone Detected")
            }

            telemetry.addData("Heading", QOL.radToDeg(ROBOT!!.botHeading).toString() + "°")
            //add the looptime of the program to the telemetry
            telemetry.addData("Loop Time", timer.milliseconds())
            telemetry.update()
        }
        
    }

}