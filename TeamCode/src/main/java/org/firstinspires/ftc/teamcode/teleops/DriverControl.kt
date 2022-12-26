package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.exception.RobotCoreException
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.utilities.QOL.Companion.radToDeg
import org.firstinspires.ftc.teamcode.utilities.TeleOP

@TeleOp(name = "Driver Control", group = "TeleOp")
class DriverControl: TeleOP() {
    
    override fun runOpMode() {
        super.runOpMode()
        waitForStart()
        openClaw(ROBOT.CLAW)

        val controller1 = Gamepad() //CC1 = Current Controller 1 state
        val controller2 = Gamepad() //CC2 = Current Controller 2 state

        val pC1 = Gamepad() //PC1 = Previous Controller 1 state
        val pC2 = Gamepad() //PC2 = Previous Controller 2 state

        val timer = ElapsedTime()

        while (opModeIsActive()) {
            timer.reset()

            try {
                pC1.copy(controller1)
                pC2.copy(controller2)

                controller1.copy(gamepad1)
                controller2.copy(gamepad2)
            } catch (e: RobotCoreException) {
                telemetry.addData("Error", e.message)
                telemetry.update()
            }


            telemetry.addData("Distance", ROBOT.CONE_SENSOR.getDistance(DistanceUnit.INCH))

            ROBOT.gamepadDrive(gamepad1, 1.0, telemetry)

            ROBOT.SLIDES.power = -gamepad1.right_stick_y.toDouble() * 0.75

            // setup the claw motor to open and close
            when {
                gamepad2.left_bumper -> {
                    openClaw(ROBOT.CLAW)
                }
                gamepad2.right_bumper -> {
                    closeClaw(ROBOT.CLAW)
                }
            }


            if (ROBOT.CONE_SENSOR.getDistance(DistanceUnit.INCH) <= 2.3) {
                telemetry.addData("Cone Sensor", "Cone Detected")
                closeClaw(ROBOT.CLAW)

            } else {
                telemetry.addData("Cone Sensor", "No Cone Detected")
            }


            telemetry.addData("Cone Sensor", ROBOT.CONE_SENSOR.getDistance(DistanceUnit.INCH))

            telemetry.addData("Heading", radToDeg(ROBOT.botHeading))
            //add the looptime of the program to the telemetry
            telemetry.addData("Loop Time", timer.milliseconds())
            telemetry.update()
        }
    }
}
