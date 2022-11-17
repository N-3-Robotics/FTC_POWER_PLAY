package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.RumbleStrength.*
import org.firstinspires.ftc.teamcode.Side.*
import kotlin.math.abs

@Config()
object TV{
    @JvmField
    var topSlidePosition = 1930 // tune
    @JvmField
    var startServoPosition = .65;
}
@TeleOp(name = "TeleOP")
class TeleOP: LinearOpMode() {
    var RC: RobotConfig? = null
    var vars: TV? = null


    override fun runOpMode() {

        RC = RobotConfig(hardwareMap)
        vars = TV
        var dashboard: FtcDashboard = FtcDashboard.getInstance()
        var m = 0.0

        while(!opModeIsActive()){
            telemetry.addData("Status", "Waiting for start")
            telemetry.update()

            RC!!.rumble(gamepad1, LEFT, HIGH)
            RC!!.rumble(gamepad2, RIGHT, HIGH)

        }
        waitForStart()

        RC!!.ARM.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        RC!!.ARM.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        while (opModeIsActive()) {
            //set up a system to measuer the loop time of the program
            val startTime = System.currentTimeMillis()



            RC!!.ARM.power = gamepad2.left_stick_y.toDouble() * 0.5
            var p0 = vars!!.startServoPosition.toDouble()
            var p1 = ((1 + vars!!.startServoPosition).toDouble())
            var t = (abs(RC!!.ARM.currentPosition) / vars!!.topSlidePosition).toDouble()


            //RC!!.CLAW_ROTATE.position = RC!!.lerp(p0, p1, t)
            RC!!.TURRET.power = gamepad2.left_stick_x.toDouble() * 0.3
            RC!!.SLIDES.power = -gamepad2.right_stick_y.toDouble()

            if (gamepad2.square) {
                RC!!.CLAW.position = 0.4
            } else if (gamepad2.circle) {
                RC!!.CLAW.position = 0.85
            }

            // use the gamepad1 face buttons to select a multiplier
            when {
                gamepad1.cross -> m = 0.25
                gamepad1.circle -> m = 0.5
                gamepad1.square -> m = 0.75
                gamepad1.triangle -> m = 1.0
            }




            //As the arm moves, the claw rotation should move to keep the claw level



            if (gamepad2.dpad_up) {
                RC!!.CLAW_ROTATE.power = -0.3
            } else if (gamepad2.dpad_down) {
                RC!!.CLAW_ROTATE.power = 0.3
            }
            else {
                RC!!.CLAW_ROTATE.power = 0.0
            }


            RC!!.gamepadDrive(gamepad1, m)

            var packet: TelemetryPacket = TelemetryPacket()
            packet.addLine("SLIDES Position: " + RC!!.SLIDES.currentPosition )
            packet.addLine("Servo Position: "+ RC!!.CLAW_ROTATE.power)
            packet.addLine("Arm Position: "+ RC!!.ARM.currentPosition)

            packet.addLine("\n P0: "+ p0)
            packet.addLine("P1: " + p1)
            packet.addLine("t: " + t)
            val endTime = System.currentTimeMillis()
            val loopTime = endTime - startTime
            packet.addLine("Loop Time: "+ loopTime)

            packet.put("x", RC!!.FR.power)
            dashboard.sendTelemetryPacket(packet)


        }


    }
}
