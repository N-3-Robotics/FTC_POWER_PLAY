package org.firstinspires.ftc.teamcode.autos

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.outoftheboxrobotics.photoncore.PhotonCore
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.pipelines.AprilTagPipeline
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.cone4
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.cone5
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.slidePower
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.slidesAboveGround
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.slidesHigh
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.slidesLow
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.slidesMid
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utilities.DriveConstants
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.ClawClose
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.ClawOpen
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam

@Autonomous(group="Real Autos", name = "Stephan's REAL AUTO")
class ihatejava : LinearOpMode() {
    /** camera stuff **/
    var camera: OpenCvCamera? = null
    var aprilTagDetectionPipeline: AprilTagPipeline? = null

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the Logitech HD Webcam C270.
    //https://github.com/FIRST-Tech-Challenge/SkyStone/blob/master/TeamCode/src/main/res/xml/teamwebcamcalibrations.xml
    // You will need to do your own calibration for other configurations!
    var fx = 822.317
    var fy = 822.317
    var cx = 319.495
    var cy = 242.502

    // UNITS ARE METERS
    var tagsize = 0.032
    var numFramesWithoutDetection = 0
    val DECIMATION_HIGH = 3f
    val DECIMATION_LOW = 2f
    val THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f
    val THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4
    var parkingId=0

    override fun runOpMode() {
        PhotonCore.enable()
        //get all variables and stuff set up
        val dashboard= FtcDashboard.getInstance()
        val drive = SampleMecanumDrive(hardwareMap)
        fun closeClaw(claw: Servo = drive.claw) {
            claw.position = DriveConstants.ClawClose
        }
        fun openClaw(claw: Servo = drive.claw) {
            claw.position = DriveConstants.ClawOpen
        }
        var telemetry= MultipleTelemetry(telemetry, dashboard.telemetry)
        telemetry.addLine("Robot has been turned on. Run for your life!")
        telemetry.update()
        closeClaw()
        drive.slides.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        closeClaw()
        drive.updatePoseEstimate()
        telemetry.addData("angle", drive.poseEstimate.heading)
        telemetry.update()
        /*** set up camera ***/
        val cameraMonitorViewId =
            hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId",
                "id",
                hardwareMap.appContext.packageName)
        camera = OpenCvCameraFactory.getInstance()
            .createWebcam(hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)
        aprilTagDetectionPipeline = AprilTagPipeline(tagsize, fx, fy, cx, cy)
        (camera as OpenCvWebcam?)!!.setPipeline(aprilTagDetectionPipeline)
        (camera as OpenCvWebcam?)!!.openCameraDeviceAsync(object :
            OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                (camera as OpenCvWebcam?)!!.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)
            }
            override fun onError(errorCode: Int) {}
        })
        dashboard.startCameraStream(camera, 30.0)
        val zone1 = drive.trajectorySequenceBuilder(Pose2d(-36.0, -66.0, Math.toRadians(90.0)))
            .splineTo(Vector2d(-29.0, -28.0), Math.toRadians(45.0))
            .UNSTABLE_addTemporalMarkerOffset(0.0){
                drive.claw.position=ClawOpen
            }
            .back(7.0)
            .lineToLinearHeading(Pose2d(-46.0, -12.0, Math.toRadians(180.0)))
            .UNSTABLE_addTemporalMarkerOffset(0.0){
                drive.slides.targetPosition=cone5
                drive.slides.mode= DcMotor.RunMode.RUN_TO_POSITION
                drive.slides.power=slidePower
            }
            .lineTo(Vector2d(-65.0, -12.0))
            .UNSTABLE_addTemporalMarkerOffset(0.0){
                drive.claw.position=ClawClose
            }
            .UNSTABLE_addTemporalMarkerOffset(0.5){
                drive.slides.targetPosition= slidesLow
                drive.slides.mode= DcMotor.RunMode.RUN_TO_POSITION
                drive.slides.power=slidePower
            }
            .build()
                val loop1=drive.trajectorySequenceBuilder(zone1.end())
                    .lineTo(Vector2d(-57.0, -12.0))
            .lineToLinearHeading(Pose2d(-54.0, -21.0, Math.toRadians(-45.0)))
                    .UNSTABLE_addDisplacementMarkerOffset(0.0){
                        drive.claw.position = ClawOpen
                    }
                    .UNSTABLE_addTemporalMarkerOffset(0.5){
                        drive.slides.targetPosition= cone4
                        drive.slides.mode= DcMotor.RunMode.RUN_TO_POSITION
                        drive.slides.power=slidePower
                    }
            .lineToLinearHeading(Pose2d(-57.0, -12.0, Math.toRadians(180.0)))
                    .build()
                val loop2 = drive.trajectorySequenceBuilder(loop1.end())
                    .lineTo(Vector2d(-65.0, -19.0))
                    .UNSTABLE_addDisplacementMarkerOffset(0.0){
                        drive.claw.position = ClawClose
                    }
                    .UNSTABLE_addTemporalMarkerOffset(0.2){
                        drive.slides.targetPosition= slidesLow
                        drive.slides.mode= DcMotor.RunMode.RUN_TO_POSITION
                        drive.slides.power=slidePower
                    }
                    .waitSeconds(.25)
                    .lineTo(Vector2d(-57.0, -12.0))
                    .lineToLinearHeading(Pose2d(-56.0, -25.0, Math.toRadians(-45.0)))
                    .UNSTABLE_addDisplacementMarkerOffset(0.0){
                        drive.claw.position = ClawOpen
                    }
                    .build()
/*
            .lineToLinearHeading(Pose2d(-52.0, -20.0, Math.toRadians(-45.0)))
            .lineToLinearHeading(Pose2d(-64.0, -12.0, Math.toRadians(180.0)))
            .lineToLinearHeading(Pose2d(-52.0, -20.0, Math.toRadians(-45.0)))
            .lineToLinearHeading(Pose2d(-64.0, -12.0, Math.toRadians(180.0)))
            .lineToLinearHeading(Pose2d(-52.0, -20.0, Math.toRadians(-45.0)))
            .lineToLinearHeading(Pose2d(-64.0, -12.0, Math.toRadians(180.0)))
            .lineToLinearHeading(Pose2d(-52.0, -20.0, Math.toRadians(-45.0)))
            .lineToLinearHeading(Pose2d(-64.0, -12.0, Math.toRadians(180.0))) */
        val zone2 = drive.trajectorySequenceBuilder(Pose2d(-36.0, -66.0, Math.toRadians(90.0)))
            .forward(55.0)
            .addTemporalMarker(1.0) {
                drive.slides.targetPosition=slidesHigh
                drive.slides.mode= DcMotor.RunMode.RUN_TO_POSITION
                drive.slides.power= slidePower
            }
            .turn(Math.toRadians(-45.0))
            .forward(10.0)
            .waitSeconds(1.0)
            .addTemporalMarker(8.0) {
                openClaw()
            }
            .back(10.0)
            .turn(Math.toRadians(135.0))
            .forward(25.0)
            .strafeRight(10.0)
            .forward(5.0)
            .addTemporalMarker(12.0){
                drive.slides.targetPosition= slidesMid
                drive.slides.mode= DcMotor.RunMode.RUN_TO_POSITION
                drive.slides.power= slidePower
            }
            .addTemporalMarker(18.0){
                closeClaw()
            }
            .back(5.0)
            .strafeLeft(18.0)
            .turn(Math.toRadians(140.0))
            .forward(10.0)
            .back(5.0)
            .turn(Math.toRadians(135.0))
            .strafeRight(20.0)
            .build()
        val zone3 = drive.trajectorySequenceBuilder(Pose2d(0.0, 0.0, 0.0))
            .forward(30.0)
            .strafeRight(20.0)
            .forward(15.0)
            .build()

        /** detect apriltags to get parking spot**/
        while(!opModeIsActive() && !isStopRequested) {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            val detections = aprilTagDetectionPipeline!!.detectionsUpdate

            // If there's been a new frame...
            if (detections != null) {
                telemetry.addData("FPS", (camera as OpenCvWebcam?)!!.fps)
                telemetry.addData("Overhead ms", (camera as OpenCvWebcam?)!!.overheadTimeMs)
                telemetry.addData("Pipeline ms", (camera as OpenCvWebcam?)!!.pipelineTimeMs)

                // If we don't see any tags
                if (detections.size == 0) {
                    numFramesWithoutDetection++

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        aprilTagDetectionPipeline!!.setDecimation(DECIMATION_LOW)
                    }
                } else {
                    numFramesWithoutDetection = 0

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (detections[0].pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        aprilTagDetectionPipeline!!.setDecimation(DECIMATION_HIGH)
                    }
                    for (detection in detections) {
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id))
                        parkingId=detection.id
                    }
                }
                telemetry.update()
            }
            sleep(20)
            telemetry.addData("Tag found: ", parkingId)
            telemetry.update()
        }
        /*** wait for start ***/
        drive.poseEstimate = zone1.start()
        waitForStart()
        drive.slides.targetPosition= slidesMid
        drive.slides.mode= DcMotor.RunMode.RUN_TO_POSITION
        drive.slides.power=slidePower
        sleep(250)
        /*** run paths ***/
        when(parkingId) {
            21 -> drive.followTrajectorySequence(zone1)
            22 -> drive.followTrajectorySequence(zone2)
            23 -> drive.followTrajectorySequence(zone3)
        }
        while(drive.isBusy && !isStopRequested){
        }
            while(drive.slides.currentPosition<drive.slides.targetPosition){
                //do nothing
            }
        drive.followTrajectorySequence(loop1)
        while(drive.isBusy && !isStopRequested){
        }
        while(drive.slides.currentPosition<drive.slides.targetPosition){
            //do nothing
        }
        drive.followTrajectorySequence(loop2)
        while(drive.isBusy && !isStopRequested){
        }
    }
}