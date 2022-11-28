package org.firstinspires.ftc.teamcode

import android.annotation.SuppressLint
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.PID.Kd
import org.firstinspires.ftc.teamcode.PID.Ki
import org.firstinspires.ftc.teamcode.PID.Kp
import org.opencv.core.*
import org.opencv.core.Core.bitwise_and
import org.opencv.core.Core.inRange
import org.opencv.imgproc.Imgproc.*
import org.opencv.imgproc.Moments
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import org.openftc.easyopencv.OpenCvWebcam


@Autonomous(name = "Blue Cone Tracking")
class ConeTracking: LinearOpMode(){

    var webcam: OpenCvWebcam? = null
    var RC: RobotConfig? = null

    @SuppressLint("DiscouragedApi")
    override fun runOpMode() {

        RC = RobotConfig(hardwareMap)

        val dash = FtcDashboard.getInstance()

        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)

        webcam!!.setPipeline(ConePipeline())

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


    inner class ConePipeline: OpenCvPipeline(){
        var viewportPaused: Boolean = false


        override fun processFrame(input: Mat?): Mat? {           //cap.read(frame);
            val hsv = Mat()
            cvtColor(input, hsv, COLOR_BGR2HSV)

            val lower_bound = Scalar(102.0, 128.0, 64.0)
            val upper_bound = Scalar(134.0, 255.0, 148.0)

            val mask = Mat()
            inRange(hsv, lower_bound, upper_bound, mask)

            val kernel = Mat.ones(7, 7, 8)

            val maskImg = Mat()

            bitwise_and(input, input, maskImg, mask)

            val contours: List<MatOfPoint> = ArrayList()
            val hierarchy = Mat()

            findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)


            var closestConeH: Double = 0.0
            if (contours.isNotEmpty()) {
                closestConeH = boundingRect(contours[0]).height.toDouble()
            } else {
                println("No cones found")
            }

            for (contour in contours) {
                val x: Double = boundingRect(contour).x.toDouble()
                val y: Double = boundingRect(contour).y.toDouble()
                val w: Double = boundingRect(contour).width.toDouble()
                val h: Double = boundingRect(contour).height.toDouble()
                if (h > 100) {
                    if (h >= closestConeH) {
                        closestConeH = h
                        rectangle(input, Point(x, y), Point(x + w, y + h), Scalar(0.0, 255.0, 0.0), 2)
                        val M: Moments = moments(contour)
                        val cX = (M._m10 / M._m00)
                        val cY = (M._m01 / M._m00)
                        circle(input, Point(cX, cY), 7, Scalar(0.0, 0.0, 255.0), -1)
                        putText(
                            input,
                            "center",
                            Point(cX - 20, cY - 20),
                            FONT_HERSHEY_SIMPLEX,
                            0.5,
                            Scalar(255.0, 255.0, 255.0),
                            2
                        )
                        // place the height of the contour above the dot in the center
                        putText(
                            input,
                            h.toString() + "px",
                            Point(cX - 20, cY + 25),
                            FONT_HERSHEY_SIMPLEX,
                            0.5,
                            Scalar(255.0, 255.0, 255.0),
                            2
                        )
                        if (cX > 300 && cX < 340) {
                            circle(input, Point(cX, cY), 7, Scalar(.0, 255.0, 0.0), -1)


                        }
                        else {
                            RC!!.pidConeTrackingTurn(320.0, Kp, Ki, Kd)
                        }
                    } else {
                        rectangle(input, Point(x, y), Point(x + w, y + h), Scalar(255.0, 0.0, 0.0), 2)
                    }
                }
            }

            return input
        }

        override fun onViewportTapped() {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */
            viewportPaused = !viewportPaused
            if (viewportPaused) {
                webcam!!.pauseViewport()
            } else {
                webcam!!.resumeViewport()
            }
        }

    }

}
