package org.firstinspires.ftc.teamcode.autos

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.pipelines.AprilTagPipeline
import org.firstinspires.ftc.teamcode.utilities.DriveConstants
import org.firstinspires.ftc.teamcode.utilities.QOL
import org.firstinspires.ftc.teamcode.robot.Robot
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam
import kotlin.math.abs

@Disabled
@Autonomous(name = "Blue Right")
class BlueRight: LinearOpMode() {
    var ROBOT: Robot? = null

    fun forward(centimeters: Double){
        val target = ROBOT!!.currentPosition + QOL.centimetersToTicks(centimeters)
        while (abs(target - ROBOT!!.currentPosition) > DriveConstants.AutoDriveTolerance) {

            val power = QOL.calcPower(target, ROBOT!!.currentPosition)

            ROBOT!!.RCDrive(power, 0.0, 0.0)
            telemetry.addData("Position", QOL.ticksToCentimeters(ROBOT!!.currentPosition))
            telemetry.addData("Target", QOL.ticksToCentimeters(target))
            telemetry.addData("Distance Remaining", abs(target - ROBOT!!.currentPosition))
            telemetry.addData("Power", power)
            telemetry.update()
        }
        ROBOT!!.stop()
    }
    fun microForward(centimeters: Double){
        val target = ROBOT!!.currentPosition + QOL.centimetersToTicks(centimeters)
        while (abs(target - ROBOT!!.currentPosition) > DriveConstants.AutoDriveTolerance) {

            val power = QOL.sansSigmoid(target, ROBOT!!.currentPosition)

            ROBOT!!.RCDrive(power, 0.0, 0.0)
            telemetry.addData("Position", QOL.ticksToCentimeters(ROBOT!!.currentPosition))
            telemetry.addData("Target", QOL.ticksToCentimeters(target))
            telemetry.addData("Distance Remaining", abs(target - ROBOT!!.currentPosition))
            telemetry.addData("Power", power)
            telemetry.update()
        }
        ROBOT!!.stop()
    }
    fun backward(centimeters: Double){
        val target = ROBOT!!.currentPosition + QOL.centimetersToTicks(-centimeters)
        while (abs(target - ROBOT!!.currentPosition) > DriveConstants.AutoDriveTolerance) {

            val power = QOL.calcPower(target, ROBOT!!.currentPosition)

            ROBOT!!.RCDrive(power, 0.0, 0.0)
            telemetry.addData("Position", ROBOT!!.currentPosition)
            telemetry.addData("Target", target)
            telemetry.addData("Distance Remaining", abs(target - ROBOT!!.currentPosition))
            telemetry.addData("Power", power)
            telemetry.update()
        }
        ROBOT!!.stop()
    }

    fun turnRight(angle: Int = -90){
        val target = QOL.radToDeg(ROBOT!!.botHeading) + angle
        // while the IMU is not withing the AutoTurnTolerance of the target angle, calculate the power and turn right
        while (abs(QOL.radToDeg(ROBOT!!.botHeading) - target) > DriveConstants.AutoTurnTolerance) {
            val power = -QOL.calcTurnPower(target, QOL.radToDeg(ROBOT!!.botHeading))
            ROBOT!!.RCDrive(0.0, 0.0, power)
            telemetry.addData("Angle", QOL.radToDeg(ROBOT!!.botHeading))
            telemetry.addData("Target", angle)
            telemetry.addData("Angle Remaining", abs(QOL.radToDeg(ROBOT!!.botHeading) - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
    }
    fun turnLeft(angle: Int = 90){
        val target = QOL.radToDeg(ROBOT!!.botHeading) + angle
        // while the IMU is not withing the AutoTurnTolerance of the target angle, calculate the power and turn right
        while (abs(QOL.radToDeg(ROBOT!!.botHeading) - target) > DriveConstants.AutoTurnTolerance) {
            val power = -QOL.calcTurnPower(target, QOL.radToDeg(ROBOT!!.botHeading))
            ROBOT!!.RCDrive(0.0, 0.0, power)
            telemetry.addData("Angle", QOL.radToDeg(ROBOT!!.botHeading))
            telemetry.addData("Target", angle)
            telemetry.addData("Angle Remaining", abs(QOL.radToDeg(ROBOT!!.botHeading) - target))
            telemetry.addData("Power", power)
            telemetry.update()
        }
    }

    fun closeClaw(claw: Servo = ROBOT!!.CLAW) {
        claw.position = DriveConstants.ClawClose + 0.1
    }
    fun openClaw(claw: Servo = ROBOT!!.CLAW) {
        claw.position = DriveConstants.ClawOpen
    }

    fun slightRaise(lift: DcMotorEx = ROBOT!!.SLIDES){
        lift.power = 0.3
        sleep(800)
        lift.power = 0.1
    }

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
        var telemetry= MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.addLine("Robot has been turned on. Run for your life!")
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
        FtcDashboard.getInstance().startCameraStream(camera, 30.0)

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

        when (parkingId) {
            21 -> {
                // The robot needs to go forward 30 inches, then go left 26, but there is no strafe left button, so we must turn
                // 90 degrees to the left, then go forward 26 inches, then turn 90 degrees to the right
                forward(30.0 * 2.54)
                turnLeft()
                forward(26.0 * 2.54)
                turnRight()

            }
            22 -> {
                telemetry.addData("Tag found: ", parkingId)
                telemetry.update()
            }
            23 -> {
                telemetry.addData("Tag found: ", parkingId)
                telemetry.update()
            }
        }
    }

}