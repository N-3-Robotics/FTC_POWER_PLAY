@file:Suppress("unused")
package org.firstinspires.ftc.teamcode.autos

import android.annotation.SuppressLint
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.pipelines.*
import org.firstinspires.ftc.teamcode.utilities.RobotConfig
import org.opencv.core.*
import org.opencv.imgproc.Imgproc.*
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam


@Disabled
@Autonomous(name = "Red Cone Tracking")
class RedConeTracking: LinearOpMode(){

    var webcam: OpenCvWebcam? = null
    var RC: RobotConfig? = null

    @SuppressLint("DiscouragedApi")
    override fun runOpMode() {

        //RC = RobotConfig(hardwareMap)

        val dash = FtcDashboard.getInstance()

        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)

        webcam!!.setPipeline(RedPipeline(telemetry))

        webcam!!.setMillisecondsPermissionTimeout(2500) // Timeout for obtaining permission is configurable. Set before opening.

        webcam!!.openCameraDeviceAsync(object : AsyncCameraOpenListener { override fun onOpened() {webcam!!.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)}override fun onError(errorCode: Int) {} })

        dash.startCameraStream(webcam, 30.0)

        waitForStart()

        while (opModeIsActive()){
            telemetry.addData("Frame Count", webcam!!.frameCount)
            telemetry.addData("FPS", String.format("%.2f", webcam!!.fps))
            telemetry.addData("Total frame time ms", webcam!!.totalFrameTimeMs)
            telemetry.addData("Pipeline time ms", webcam!!.pipelineTimeMs)
            telemetry.addData("Overhead time ms", webcam!!.overheadTimeMs)
            telemetry.addData("Theoretical max FPS", webcam!!.currentPipelineMaxFps)
            telemetry.update()
        }

    }
}

@Disabled
@Autonomous(name = "Blue Cone Tracking")
class BlueConeTracking: LinearOpMode(){

    var webcam: OpenCvWebcam? = null
    var RC: RobotConfig? = null
    var pipeline: BluePipeline? = null

    @SuppressLint("DiscouragedApi")
    override fun runOpMode() {

        //RC = RobotConfig(hardwareMap)

        val dash = FtcDashboard.getInstance()

        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)

        pipeline = BluePipeline(telemetry)
        webcam!!.setPipeline(pipeline)

        webcam!!.setMillisecondsPermissionTimeout(2500) // Timeout for obtaining permission is configurable. Set before opening.

        webcam!!.openCameraDeviceAsync(object : AsyncCameraOpenListener { override fun onOpened() {webcam!!.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)}override fun onError(errorCode: Int) {} })

        dash.startCameraStream(webcam, 30.0)

        waitForStart()

        while (opModeIsActive()){
            telemetry.addData("Frame Count", webcam!!.frameCount)
            telemetry.addData("FPS", String.format("%.2f", webcam!!.fps))
            telemetry.addData("Total frame time ms", webcam!!.totalFrameTimeMs)
            telemetry.addData("Pipeline time ms", webcam!!.pipelineTimeMs)
            telemetry.addData("Overhead time ms", webcam!!.overheadTimeMs)
            telemetry.addData("Theoretical max FPS", webcam!!.currentPipelineMaxFps)
            telemetry.update()
        }

    }
}
