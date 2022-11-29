package org.firstinspires.ftc.teamcode.pipelines;

import static org.opencv.core.Core.*;
import static org.opencv.imgproc.Imgproc.*;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BluePipeline extends OpenCvPipeline {

    public Scalar LOWER_BOUND_COLOR = new Scalar(100, 100, 100);
    public Scalar HIGHER_BOUND_COLOR = new Scalar(140, 255, 255);

    @Override
    public Mat processFrame(Mat input) {


        Mat hsv_image = new Mat();
        cvtColor(input, hsv_image, COLOR_RGB2HSV);


        Mat mask = new Mat();
        inRange(hsv_image, LOWER_BOUND_COLOR, HIGHER_BOUND_COLOR, mask);

        Mat maskImg = new Mat();

        bitwise_and(input, input, maskImg, mask);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        int closestConeH = 0;

        if (!contours.isEmpty()){
            closestConeH = boundingRect(contours.get(0)).height;
        }
        else{
            System.out.println("No contours found");
        }

        for (MatOfPoint contour : contours){
            int x, y, w, h;
            x = boundingRect(contour).x;
            y = boundingRect(contour).y;
            w = boundingRect(contour).width;
            h = boundingRect(contour).height;

            if (h > 100){
                if (h > closestConeH){
                    closestConeH = h;
                    rectangle(input, new Point(x, y), new Point(x + w, y + h), new Scalar(0, 255, 0), 2);
                    Moments M = moments(contour);
                    int cX = (int) (M.m10 / M.m00);
                    int cY = (int) (M.m01 / M.m00);

                    circle(input, new Point(cX, cY), 7, new Scalar(0, 0, 255), -1);

                    putText(input, h + "px", new Point(cX - 20, cY + 25), FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);

                    if (cX > 300 && cX < 340) {
                        circle(input, new Point(cX, cY), 7, new Scalar(0, 255, 0), -1);
                        System.out.println("Centered");
                    }
                    else {
                        System.out.println("Not Centered: " + cX);
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

        return input;
    }



}
