package org.firstinspires.ftc.teamcode.autos

import android.annotation.SuppressLint
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.outoftheboxrobotics.photoncore.PhotonCore
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.pipelines.AprilTagPipeline
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.cone4
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.cone5
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.slidePower
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.slidesLow
import org.firstinspires.ftc.teamcode.teleop.TeleopVariables.slidesMid
import org.firstinspires.ftc.teamcode.utilities.AJDriveConstants.ClawClose
import org.firstinspires.ftc.teamcode.utilities.AJDriveConstants.ClawOpen
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam

@Autonomous(group="Real Autos", name = "Stephan's PARK ONLY")
class finallynormalfilename : LinearOpMode() {
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

    @SuppressLint("DiscouragedApi")
    override fun runOpMode() {
        PhotonCore.enable()
        //get all variables and stuff set up
        val dashboard= FtcDashboard.getInstance()
        val drive = SampleMecanumDrive(hardwareMap)
        fun closeClaw(claw: Servo = drive.claw) {
            claw.position = ClawClose
        }
        fun openClaw(claw: Servo = drive.claw) {
            claw.position = ClawOpen
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
        val park1 = drive.trajectorySequenceBuilder(Pose2d(0.0, 0.0, 0.0))
            .forward(32.0)
            .strafeLeft(28.0)
            .build()
        val park2 = drive.trajectorySequenceBuilder(Pose2d(0.0, 0.0, 0.0))
            .forward(32.0)
            .build()
        val park3 = drive.trajectorySequenceBuilder(Pose2d(0.0, 0.0, 0.0))
            .forward(32.0)
            .strafeRight(28.0)
            .build()
        // drive.parallelEncoderServo.position = TeleopVariables.parallel
        //drive.perpendicularEncoderServo.position = TeleopVariables.perpendicular
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
        waitForStart()

        when(parkingId) {
            21 -> drive.followTrajectorySequence(park1)
            22 -> drive.followTrajectorySequence(park2)
            23 -> drive.followTrajectorySequence(park3)
        }
        while(drive.isBusy && !isStopRequested){
            //do nothing
        }
    }
}