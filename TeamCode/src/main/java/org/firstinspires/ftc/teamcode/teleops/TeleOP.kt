package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.exception.RobotCoreException
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.utilities.RobotConfig
import org.firstinspires.ftc.teamcode.utilities.RumbleStrength
import org.firstinspires.ftc.teamcode.utilities.Side

@TeleOp(name="TeleOP")
class TeleOP: LinearOpMode() {
    var RC: RobotConfig? = null

    fun closeClaw(claw: CRServo) {
        claw.power = 0.6
    }
    fun openClaw(claw: CRServo){
        claw.power = 0.0
    }
    fun rED(x: Boolean, y: Boolean): Boolean { // Rising Edge Detector
        return x && !y
    }

    override fun runOpMode() {
        RC = RobotConfig(hardwareMap)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)


        // Rumble the left controller medium once, and the right controller medium twice, at the same time, using the RC!!.rumble function
        RC!!.rumble(gamepad1, Side.BOTH, RumbleStrength.MEDIUM, 100)
        RC!!.rumble(gamepad2, Side.BOTH, RumbleStrength.MEDIUM, 100)
        RC!!.rumble(gamepad2, Side.BOTH, RumbleStrength.MEDIUM, 100)






        waitForStart()

        openClaw(RC!!.CLAW)

        val controller1 = Gamepad() //CC1 = Current Controller 1 state
        val controller2 = Gamepad() //CC2 = Current Controller 2 state

        val pC1 = Gamepad() //PC1 = Previous Controller 1 state
        val pC2 = Gamepad() //PC2 = Previous Controller 2 state

        val timer = ElapsedTime()

        while (opModeIsActive()){
            timer.reset()

            try {
                pC1.copy(controller1)
                pC2.copy(controller2)

                controller1.copy(gamepad1)
                controller2.copy(gamepad2)
            }
            catch (e: RobotCoreException){
                telemetry.addData("Error", e.message)
                telemetry.update()
            }


            telemetry.addData("Distance", RC!!.CONE_SENSOR.getDistance(DistanceUnit.INCH))

            RC!!.gamepadDrive(gamepad1, 1.0, telemetry)

            RC!!.SLIDES.power = -gamepad1.right_stick_y.toDouble() * 0.75

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

            //add the looptime of the program to the telemetry
            telemetry.addData("Loop Time", timer.milliseconds())
            telemetry.update()
        }

    }

}