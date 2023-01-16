package org.firstinspires.ftc.teamcode.pipelines;

import static org.opencv.core.Core.bitwise_and;
import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_SIMPLEX;
import static org.opencv.imgproc.Imgproc.RETR_EXTERNAL;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.circle;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.moments;
import static org.opencv.imgproc.Imgproc.putText;
import static org.opencv.imgproc.Imgproc.rectangle;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.List;

public class JunctionPipeline extends OpenCvPipeline {

    public Scalar LOWER_BOUND_COLOR = new Scalar(14, 45, 182);
    public Scalar HIGHER_BOUND_COLOR = new Scalar(140, 255, 255);

    Telemetry telemetry;

    public JunctionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    double closestDifference = 300.0;
    double error = 0.0;
    int numOfContours = 0;

    @Override
    public Mat processFrame(Mat input) {

        int oldNumOfContours = numOfContours;

        Mat hsv_image = new Mat();
        cvtColor(input, hsv_image, COLOR_RGB2HSV);


        Mat mask = new Mat();
        inRange(hsv_image, LOWER_BOUND_COLOR, HIGHER_BOUND_COLOR, mask);

        Mat maskImg = new Mat();

        bitwise_and(input, input, maskImg, mask);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);



        if (contours.isEmpty()){
            System.out.println("No contours found");
            closestDifference = 50.0;
            return input;
        }

        // find all of the contours greater than 100px, and add them to a list
        List<MatOfPoint> filteredContours = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            if (boundingRect(contour).width > 20) {
                filteredContours.add(contour);
            }
        }
        numOfContours = filteredContours.size();

        if (numOfContours != oldNumOfContours) {
            closestDifference = 300.0;
        }



        telemetry.addData("Number of contours", filteredContours.size());
        for (MatOfPoint contour : contours){
            int x, y, w, h;
            x = boundingRect(contour).x;
            y = boundingRect(contour).y;
            w = boundingRect(contour).width;
            h = boundingRect(contour).height;

            double diffIn = Math.abs(320 - x);

            BigDecimal diff = new BigDecimal(diffIn).setScale(0, RoundingMode.HALF_UP);
            if (w > 20){
                if (diff.doubleValue() <= closestDifference){
                    closestDifference = diff.doubleValue();
                    telemetry.addData("Closet Pole Error", diff.doubleValue());
                    rectangle(input, new Point(x, y), new Point(x + w, y + h), new Scalar(0, 255, 0), 2);
                    Moments M = moments(contour);
                    int cX = (int) (M.m10 / M.m00);
                    int cY = (int) (M.m01 / M.m00);

                    error = diff.doubleValue();

                    circle(input, new Point(cX, cY), 7, new Scalar(0, 0, 255), -1);

                    putText(input, "Error: " + diff, new Point(cX - 50, cY + 50), FONT_HERSHEY_SIMPLEX, 0.75, new Scalar(255, 0, 0), 2);
                    telemetry.addData("Width", w);

                    if (cX > 300 && cX < 340) {
                        circle(input, new Point(cX, cY), 7, new Scalar(0, 255, 0), -1);
                        telemetry.addLine("Centered");
                    }
                    else {
                        telemetry.addData("Not Centered: ", cX);
                    }
                }
                else {
                    rectangle(input, new Point(x, y), new Point(x + w, y + h), new Scalar(255, 0, 0), 3);
                }
            }

        }

        //release all Mats
        hsv_image.release();
        mask.release();
        maskImg.release();
        hierarchy.release();

        telemetry.update();
        return input;
    }


    public double getError() {
        return error;
    }
}
