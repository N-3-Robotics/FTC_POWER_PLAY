package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlin.Throws
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
@Disabled
class LocalizationTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)
        drive.parallelEncoderServo.position = TeleopVariables.parallel
        drive.perpendicularEncoderServo.position = TeleopVariables.perpendicular
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
        waitForStart()
        while (!isStopRequested) {
            drive.setWeightedDrivePower(Pose2d((-gamepad1.left_stick_y).toDouble(),
                (-gamepad1.left_stick_x).toDouble(),
                (-gamepad1.right_stick_x).toDouble()))
            drive.update()
            val (x, y, heading) = drive.poseEstimate
            telemetry.addData("x", x)
            telemetry.addData("y", y)
            telemetry.addData("heading", heading)
            telemetry.update()
        }
    }
}