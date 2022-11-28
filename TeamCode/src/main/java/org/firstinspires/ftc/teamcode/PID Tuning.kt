package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.RumbleStrength.*
import org.firstinspires.ftc.teamcode.Side.*
import kotlin.math.abs

@Config()
object PID {
    @JvmField
    var Kp = 0.0008
    @JvmField
    var Ki = 0.0
    @JvmField
    var Kd = 0.0005
}

@Autonomous(name = "Auto")
class Auto: LinearOpMode() {
    var RC: RobotConfig? = null
    override fun runOpMode() {
        RC = RobotConfig(hardwareMap)
        while(!opModeIsActive()){
            telemetry.addData("Status", "Waiting for start")
            telemetry.update()
            RC!!.rumble(gamepad1, LEFT, HIGH)
            RC!!.rumble(gamepad2, RIGHT, HIGH)
        }
        waitForStart()
        //use the pidDrive function to drive in a 1 meter circle

        var packet: TelemetryPacket = TelemetryPacket()
        var dashboard: FtcDashboard = FtcDashboard.getInstance()
        while (opModeIsActive()){

            val WHEEL_DIAMETER = 96.0 / 1000.0

            val initialWheelPosition = RC!!.FR.currentPosition

            //convert ticks per rev to ticks per meter
            val TICKS_PER_METER = RC!!.TICKS_PER_REV_312 / (WHEEL_DIAMETER * Math.PI)


            val target = .5 * TICKS_PER_METER
            var lastReference = target
            var integralSum = 0.0
            var lastError = 0.0

            val maxIntegralSum = 0.5


            val timer: ElapsedTime = ElapsedTime()



            while (abs(RC!!.FR.currentPosition - initialWheelPosition) < abs(target)) {
                var packet = TelemetryPacket()
                val error = target - (RC!!.FR.currentPosition - initialWheelPosition)

                val errorChange = error - lastError

                val derivative = errorChange / timer.seconds()

                integralSum += (error * timer.seconds())

                if (integralSum > maxIntegralSum) {
                    integralSum = maxIntegralSum
                } else if (integralSum < -maxIntegralSum) {
                    integralSum = -maxIntegralSum
                }

                if (target != lastReference) {
                    integralSum = 0.0
                }

                val output = (PID.Kp * error) + (PID.Ki * integralSum) + (PID.Kd * derivative)

                RC!!.funnyDrive(output, 0.0)


                lastError = error
                lastReference = target
                timer.reset()
                packet.put("Encoder Position", RC!!.FR.currentPosition)
                packet.put("Reference", target)
                packet.put("Output", output)
                dashboard.sendTelemetryPacket(packet)
            }
            RC!!.stop()


        }
    }
}