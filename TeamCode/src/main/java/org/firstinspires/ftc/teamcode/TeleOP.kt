package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp(name="TeleOP")
class TeleOP: LinearOpMode() {
    var RC: RobotConfig? = null

    fun closeClaw(claw: CRServo) {
        claw.power = 0.6
    }
    fun openClaw(claw: CRServo){
        claw.power = 0.0
    }


    override fun runOpMode() {
        RC = RobotConfig(hardwareMap)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        waitForStart()

        openClaw(RC!!.CLAW)

        while (opModeIsActive()){


            telemetry.addData("Distance", RC!!.CONE_SENSOR.getDistance(DistanceUnit.INCH))

            RC!!.gamepadDrive(gamepad1, 1.0)

            RC!!.SLIDES.power = -gamepad2.right_stick_y.toDouble() * 0.75

            // setup the claw motor to open and close
            when {
                gamepad2.left_bumper -> {openClaw(RC!!.CLAW)}
                gamepad2.right_bumper -> {closeClaw(RC!!.CLAW)}
            }


            if (RC!!.CONE_SENSOR.getDistance(DistanceUnit.INCH) <= 2.3) {
                telemetry.addData("Cone Sensor", "Cone Detected")
                closeClaw(RC!!.CLAW)

            } else {
                telemetry.addData("Cone Sensor", "No Cone Detected")
            }


            telemetry.addData("Cone Sensor", RC!!.CONE_SENSOR.getDistance(DistanceUnit.INCH))

            telemetry.update()

        }

    }

}