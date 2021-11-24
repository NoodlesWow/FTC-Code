package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    private POS pos;

    public enum POS{
        LEFT,
        CENTER,
        RIGHT
    }

    static final Rect LEFTBOX = new Rect(
            new Point(0, 40),
            new Point(106, 140)
    );
    static final Rect CENTERBOX = new Rect(
            new Point(106, 40),
            new Point(212, 140)
    );
    static final Rect RIGHTBOX = new Rect(
            new Point(212, 40),
            new Point(320, 140)
    );

    private Mat mat = new Mat();
    public Scalar lowHSV = new Scalar(106.3, 50, 70); // duck lower
    public Scalar highHSV = new Scalar(109.1, 255, 255); // duck upper


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat left = mat.submat(LEFTBOX);
        Mat center = mat.submat(CENTERBOX);
        Mat right = mat.submat(RIGHTBOX);
        double leftValue = Core.sumElems(left).val[0] / LEFTBOX.area();
        double centerValue = Core.sumElems(center).val[0] / CENTERBOX.area();
        double rightValue = Core.sumElems(right).val[0] / RIGHTBOX.area();
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Scalar colorTarget = new Scalar(0, 255, 0);
        Scalar colorEmpty = new Scalar(255, 0, 0);
        double maxLeftRight = Math.max(leftValue, rightValue);
        double max = Math.max(maxLeftRight, centerValue);
        if (max == leftValue)
            pos = POS.LEFT;
        else if (max == centerValue)
            pos = POS.CENTER;
        else if (max == rightValue)
            pos = POS.RIGHT;
        // draw green rect on target box/red on empty box
        Imgproc.rectangle(mat, LEFTBOX, pos == POS.LEFT ? colorTarget:colorEmpty);
        Imgproc.rectangle(mat, CENTERBOX, pos == POS.CENTER ? colorTarget:colorEmpty);
        Imgproc.rectangle(mat, RIGHTBOX, pos == POS.RIGHT ? colorTarget:colorEmpty);



        return mat;
    }
    public POS getPosition() {return pos;}
    public Pipeline(Telemetry telemetry){

    }
    public static class Vision {
        public Pipeline detector;
        OpenCvCamera camera;

        public Vision(HardwareMap hardwareMap, Telemetry telemetry) {
            int cameraMonitorViewId = hardwareMap.appContext.
                    getResources().getIdentifier("cameraMonitorViewId",
                    "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().
                    createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
            detector = new Pipeline(telemetry);
        }

        public void setPipeline() {
            camera.setPipeline(detector);
        }

        public void startStreaming() {
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                public void onOpened()
                {
                    camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }
                public void onError(int errorCode) { }
            });
        }
    }
}
