package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import kotlin.Throws
import org.firstinspires.ftc.teamcode.drive.opmode.StrafeTest
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Disabled
@Autonomous(group = "drive")
class StrafeTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)
        drive.parallelEncoderServo.position = TeleopVariables.parallel
        drive.perpendicularEncoderServo.position = TeleopVariables.perpendicular
        val trajectory = drive.trajectoryBuilder(Pose2d()).strafeRight(DISTANCE).build()
        waitForStart()
        if (isStopRequested) return
        drive.followTrajectory(trajectory)
        val (x, y, heading) = drive.poseEstimate
        telemetry.addData("finalX", x)
        telemetry.addData("finalY", y)
        telemetry.addData("finalHeading", heading)
        telemetry.update()
        while (!isStopRequested && opModeIsActive());
    }

    companion object {
        var DISTANCE = 60.0 // in
    }
}