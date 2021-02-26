
        package org.man.c;


        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.opencv.core.Core;
        import org.opencv.core.Mat;
        import org.opencv.core.Point;
        import org.opencv.core.Rect;
        import org.opencv.core.Scalar;
        import org.opencv.imgproc.Imgproc;
        import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        Zero,
        One,
        Two
    }
    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(60, 140),
            new Point(120, 165));
    static final Rect RIGHT_ROI = new Rect(
            new Point(60, 200),
            new Point(120, 215));
    static double thresh = 0.1;

    public SkystoneDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(5, 50, 50);
        Scalar highHSV = new Scalar(15, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();



        boolean stoneLeft = leftValue > thresh;
        boolean stoneRight = rightValue > thresh;

        if (stoneLeft && stoneRight) {
            location = Location.Two;
            telemetry.addData("Skystone Location", "2");
        }
        else if (stoneRight) {
            location = Location.One;
            telemetry.addData("Skystone Location", "One");
        }
        else {
            location = Location.Zero;
            telemetry.addData("Skystone Location", "0");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.One? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.Two? colorSkystone:colorStone);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}


