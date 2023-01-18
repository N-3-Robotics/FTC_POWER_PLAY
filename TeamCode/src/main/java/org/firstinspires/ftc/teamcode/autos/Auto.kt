@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.autos

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.pipelines.AprilTagPipeline
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.highPole
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.midPole
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.slightRaise
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam


@Autonomous(name = "Cone Parking")
class Auto: LinearOpMode() {
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
        val ROBOT = Robot(hardwareMap)
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
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
        //todo /*** build paths**/
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
        ROBOT.SLIDES.goTo(slightRaise)


        // These paths should work the same for roadrunner, just change the syntax

        when(parkingId) {
            21 -> {
                // Drop off preload
                ROBOT.straight(42.0)
                ROBOT.SLIDES.goTo(midPole)
                ROBOT.turnRight()
                ROBOT.straight(7.5)
                ROBOT.openClaw()

                //Pick up new cone
                ROBOT.straight(-7.5)
                ROBOT.turnLeft()
                ROBOT.straight(12.0 - 0.25)
                ROBOT.turnLeft()
                ROBOT.SLIDES.goTo(slightRaise)
                ROBOT.straight(30.0)
                ROBOT.closeClaw()
                ROBOT.SLIDES.goTo(highPole)

                //Drop of new cone at high pole
                ROBOT.turnLeft(180)
                ROBOT.straight(30.0)
                ROBOT.turnLeft()
                ROBOT.straight(12.25)
                ROBOT.turnRight()
                ROBOT.straight(7.5)
                ROBOT.openClaw()

                //Park
                ROBOT.straight(-7.5)
                ROBOT.turnLeft()
                ROBOT.straight(-36.0)
                ROBOT.turnLeft()
                ROBOT.straight(12.0)
                ROBOT.turnRight()
            }
            22 -> {
                // Drop off preload
                ROBOT.straight(42.0)
                ROBOT.SLIDES.goTo(midPole)
                ROBOT.turnRight()
                ROBOT.straight(7.5)
                ROBOT.openClaw()

                //Pick up new cone
                ROBOT.straight(-7.5)
                ROBOT.turnLeft()
                ROBOT.straight(12.0 - 0.25)
                ROBOT.turnLeft()
                ROBOT.SLIDES.goTo(slightRaise)
                ROBOT.straight(30.0)
                ROBOT.closeClaw()
                ROBOT.SLIDES.goTo(highPole)

                //Drop of new cone at high pole
                ROBOT.turnLeft(180)
                ROBOT.straight(30.0)
                ROBOT.turnLeft()
                ROBOT.straight(12.25)
                ROBOT.turnRight()
                ROBOT.straight(7.5)
                ROBOT.openClaw()

                //Park
                ROBOT.straight(-7.5)
                ROBOT.turnLeft()
                ROBOT.straight(-36.0)
            }
            23 -> {
                // Drop off preload
                ROBOT.straight(42.0)
                ROBOT.SLIDES.goTo(midPole)
                ROBOT.turnRight()
                ROBOT.straight(7.5)
                ROBOT.openClaw()

                //Pick up new cone
                ROBOT.straight(-7.5)
                ROBOT.turnLeft()
                ROBOT.straight(12.0 - 0.25)
                ROBOT.turnLeft()
                ROBOT.SLIDES.goTo(slightRaise)
                ROBOT.straight(30.0)
                ROBOT.closeClaw()
                ROBOT.SLIDES.goTo(highPole)

                //Drop of new cone at high pole
                ROBOT.turnLeft(180)
                ROBOT.straight(30.0)
                ROBOT.turnLeft()
                ROBOT.straight(12.25)
                ROBOT.turnRight()
                ROBOT.straight(7.5)
                ROBOT.openClaw()

                //Park
                ROBOT.straight(-7.5)
                ROBOT.turnLeft()
                ROBOT.straight(-36.0)
                ROBOT.turnRight()
                ROBOT.straight(12.0)
                ROBOT.turnLeft()
            }
        }
        while(ROBOT.isBusy && !isStopRequested){
            //do nothing
        }
    }
}
