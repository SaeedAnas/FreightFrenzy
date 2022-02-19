package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Webcam {
    OpenCvCamera camera;

    public Webcam(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webCamName = hardwareMap.get(WebcamName.class, "camera");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webCamName, cameraMonitorViewId);
    }

    public Webcam setPipeline(OpenCvPipeline pipeline) {
        camera.setPipeline(pipeline);
        return this;
    }

    public Webcam aprilTagDetection() {
        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.166;

        TestPipeline aprilTagDetectionPipeline = new TestPipeline(tagsize, fx, fy, cx, cy);
        return this;
    }

    public Webcam colorfilter() {
        camera.setPipeline(new ColorFilterPipeline());

        return this;
    }

    public void start() {
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)

                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 0);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

}
