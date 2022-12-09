package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp(name="TeleOP")
class TeleOP: LinearOpMode() {
    var RC: RobotConfig? = null

    override fun runOpMode() {

        RC = RobotConfig(hardwareMap)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        waitForStart()

        while (opModeIsActive()){


            telemetry.addData("Distance", RC!!.CONE_SENSOR.getDistance(DistanceUnit.INCH))

            RC!!.gamepadDrive(gamepad1, 1.0)

            RC!!.SLIDES.power = -gamepad2.right_stick_y.toDouble()

            // setup the claw motor to open and close
            if (gamepad2.right_bumper) {
                RC!!.CLAW.power = 1.0
            } else if (gamepad2.left_bumper) {
                RC!!.CLAW.power = -1.0
            } else {
                RC!!.CLAW.power = 0.0
            }


            if (RC!!.CONE_SENSOR.getDistance(DistanceUnit.INCH) <= 1.5) {
                telemetry.addData("Cone Sensor", "Cone Detected")
            } else {
                telemetry.addData("Cone Sensor", "No Cone Detected")
            }


            telemetry.addData("Cone Sensor", RC!!.CONE_SENSOR.getDistance(DistanceUnit.INCH))

            telemetry.update()

        }

    }

}