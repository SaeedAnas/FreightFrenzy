package org.firstinspires.ftc.teamcode.vision;

import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.findContours;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ColorFilterPipeline extends OpenCvPipeline {

    public static int xPos = 0;

    public int getPos() {
        return xPos;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Mat masked = new Mat();
        Scalar lower = new Scalar(0, 55, 100);
        Scalar upper = new Scalar(37, 132, 135);

        Mat thresh = new Mat();
        inRange(mat, lower, upper, masked);

        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        drawContours(input, contours, -1, new Scalar(0, 255, 0), 1);

        xPos = boundingRect(contours.get(0)).x;
        return input;

    }
}