package org.firstinspires.ftc.teamcode.vision;

import static org.opencv.core.Core.BORDER_DEFAULT;
import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.rectangle;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class ColorFilterPipeline extends OpenCvPipeline {

    private Telemetry telemetry;

    public static int xPos = 0;
    public static double boxWidth = 0;
    public static int BLUR = 5;

    public static int lowerh = 23;
    public static int lowers = 40;
    public static int lowerv = 71;

    public static int upperh = 27;
    public static int uppers = 148;
    public static int upperv = 218;

    public Scalar lower = new Scalar(lowerh, lowers, lowerv);
    public Scalar upper = new Scalar(upperh, uppers, upperv);

    public int getPos() {
        return xPos;
    }

    @Override
    public Mat processFrame(Mat input) {

        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Mat masked = new Mat();

        GaussianBlur(mat, mat, new Size(BLUR, BLUR), BORDER_DEFAULT);
        GaussianBlur(mat, mat, new Size(5, 5), BORDER_DEFAULT);

        inRange(mat, lower, upper, masked);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        findContours(masked, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        drawContours(input, contours, -1, new Scalar(0, 255, 0), 1);

//        telemetry.addData("[>]", "Change these values in tuner menu");
//        telemetry.addData("x val", xPos);
//        telemetry.addData("[Lower Scalar]", lower);
//        telemetry.addData("[Upper Scalar]", upper);
//
        contours.sort((c1, c2) -> boundingRect(c2).width * boundingRect(c2).height - boundingRect(c1).width * boundingRect(c1).height);
//        telemetry.addData("numContours:", contours.size());
//
//        telemetry.update();

        if (contours.size() > 0) {
            for (int i = 0; i < contours.size(); i++) {
                int x = boundingRect(contours.get(i)).x;
                int y = boundingRect(contours.get(i)).y;
                int width = boundingRect(contours.get(i)).width;
                int height = boundingRect(contours.get(i)).height;
                rectangle(input, new Point(x, y), new Point((x + width), (y + height)), new Scalar(0, 0, 255), 2);
                xPos = (int) (x + width / 2);
            }

            int x = boundingRect(contours.get(0)).x;
            int y = boundingRect(contours.get(0)).y;
            int width = boundingRect(contours.get(0)).width;
            int height = boundingRect(contours.get(0)).height;
//            rectangle(input, new Point(x, y), new Point((x + width), (y + height)), new Scalar(0, 0, 255), 2);

            xPos = (int) (x + width / 2);
            boxWidth = width;

        } else {
//            telemetry.addData("nothing found :(", 0);
//            telemetry.update();
        }

        return input;

    }
}