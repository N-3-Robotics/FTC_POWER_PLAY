package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
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

        var packet: TelemetryPacket = TelemetryPacket()
        var dashboard: FtcDashboard = FtcDashboard.getInstance()

        val WHEEL_DIAMETER = 96.0 / 1000.0

        val initialWheelPosition = RC!!.BR.currentPosition

        //convert ticks per rev to ticks per meter
        val TICKS_PER_METER = RC!!.TICKS_PER_REV_312 / (WHEEL_DIAMETER * Math.PI)


        var target = 1 * TICKS_PER_METER
        var lastReference = target
        var integralSum = 0.0
        var lastError = 0.0

        val maxIntegralSum = 0.5


        val timer: ElapsedTime = ElapsedTime()

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry);

        telemetry.addData("Encoder Position", RC!!.BR.currentPosition)
        telemetry.addData("Output", 0.0)
        telemetry.addData("Reference", 0.0)
        telemetry.update()
        waitForStart()
        //use the pidDrive function to drive in a 1 meter circle


        while (opModeIsActive()){

            RC!!.FR.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            RC!!.FR.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER



            while (abs(RC!!.BR.currentPosition - initialWheelPosition) < abs(target)) {
                var packet = TelemetryPacket()
                val error = target - (RC!!.BR.currentPosition - initialWheelPosition)

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
                telemetry.addData("Encoder Position", RC!!.BR.currentPosition)
                telemetry.addData("Reference", target)
                telemetry.addData("Output", output)
                telemetry.update()
            }
            RC!!.stop()
            RC!!.FR.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            RC!!.FR.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER


            target = -1 * TICKS_PER_METER
            lastReference = target
            integralSum = 0.0
            lastError = 0.0


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
                telemetry.addData("Encoder Position", RC!!.FR.currentPosition)
                telemetry.addData("Reference", target)
                telemetry.addData("Output", output)
                telemetry.update()
            }
            RC!!.stop()


        }
    }
}