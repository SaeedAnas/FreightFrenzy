package org.firstinspires.ftc.teamcode.vision;

import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.boundingRect;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.rectangle;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Point;
import java.util.Collections;

public class ColorFilterPipeline extends OpenCvPipeline {

    private Telemetry telemetry = null;

    public static int xPos = 0;

    public static int lowerh = 20;
    public static int lowers = 36;
    public static int lowerv = 150;

    public static int upperh = 28;
    public static int uppers = 153;
    public static int upperv = 237;

    public Scalar lower = new Scalar(lowerh, lowers, lowerv);
    public Scalar upper = new Scalar(upperh, uppers, upperv);

    public int getPos() {
        return xPos;
    }

    public ColorFilterPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Mat masked = new Mat();

        inRange(mat, lower, upper, masked);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        findContours(masked, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        drawContours(input, contours, -1, new Scalar(0, 255, 0), 1);

        telemetry.addData("[>]", "Change these values in tuner menu");
        telemetry.addData("x val", xPos);
        telemetry.addData("[Lower Scalar]", lower);
        telemetry.addData("[Upper Scalar]", upper);
        telemetry.update();

        Collections.sort(contours, (c1, c2) -> boundingRect(c2).width - boundingRect(c1).width);

        if (contours.size() > 0) {
            // for (int i = 0; i < contours.size(); i ++) {
            //     int x = boundingRect(contours.get(i)).x;
            //     int y = boundingRect(contours.get(i)).y;
            //     int width = boundingRect(contours.get(i)).width;
            //     int height = boundingRect(contours.get(i)).height;
            //     rectangle(input, new Point(x, y), new Point((x + width), (y + height)), new Scalar(0, 0, 255), 2);

            //     xPos = (int) (x + width/2);
            // }

            int x = boundingRect(contours.get(0)).x;
            int y = boundingRect(contours.get(0)).y;
            int width = boundingRect(contours.get(0)).width;
            int height = boundingRect(contours.get(0)).height;
            rectangle(input, new Point(x, y), new Point((x + width), (y + height)), new Scalar(0, 0, 255), 2);

            xPos = (int) (x + width/2);

        } else {
            telemetry.addData("nothing found :(", 0);
            telemetry.update();
        }

        return input;

    }
}