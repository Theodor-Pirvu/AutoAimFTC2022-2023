package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;
import java.lang.Math;

public class OkHearMeOut extends OpenCvPipeline {
    Scalar YELLOW = new Scalar(221, 163, 27);
    public static Scalar scalarLowerYCrCb = new Scalar(190, 10, 50); // ycbcr value for iluminated pole: (190, 12, 96)
    public static Scalar scalarUpperYCrCb = new Scalar(240, 180, 105);
    public volatile boolean error = false;
    public volatile Exception debug;
    private double borderLeftX;
    private double borderRightX;
    private double borderTopY;
    private double borderBottomY;
    private int CAMERA_WIDTH;
    private int CAMERA_HEIGHT;
    private int loopCounter = 0;
    private int pLoopCounter = 0;
    private final Mat mat = new Mat();
    private final Mat processed = new Mat();
    private Rect maxRect = new Rect(600,1,1,1);
    private double maxArea = 11000;
    private boolean first = false;
    private final Object sync = new Object();
    public double min_midpoint = 310;
    public double max_midpoint = 330;
    public boolean area_ranges = getRectArea() >= min_area && getRectArea() <= max_area;        //same thing with these two ranges
    public boolean midpoint_range = getRectMidpointX() >= 155.0 && getRectMidpointX() <= 165.0;
    public double x_distance_pixel = 0.0;
    private double tangent;
    private double servo_angle;
    private double servo_angle_final;
    private boolean bool1 = false;
    public boolean pole_midpoint_range = getRectMidpointX() >= min_area && getRectMidpointX() <= max_area;

    public double err = 0;
    public double swivelServoPos = 0;
    public double tangent2 = 0;
    public double wristCorrF;
    public boolean hold69 = false;
    public double previousX;
    public double currentX;
    public boolean noCorrect = false;

    public OkHearMeOut(double borderLeftX, double borderRightX, double borderTopY, double borderBottomY) {
        this.borderLeftX = borderLeftX;
        this.borderRightX = borderRightX;
        this.borderTopY = borderTopY;
        this.borderBottomY = borderBottomY;
    }
    public OkHearMeOut() {
        getRectArea();
        getRectMidpointX();
        getRectX();
        getRectWidth();
        wrist();
        tangents();
        validTarget();
        absValue();
    }

    public void configureScalarLower(double y, double cr, double cb) {
        scalarLowerYCrCb = new Scalar(y, cr, cb);
    }
    public void configureScalarUpper(double y, double cr, double cb) {
        scalarUpperYCrCb = new Scalar(y, cr, cb);
    }
    public void configureScalarLower(int y, int cr, int cb) {
        scalarLowerYCrCb = new Scalar(y, cr, cb);
    }
    public void configureScalarUpper(int y, int cr, int cb) {
        scalarUpperYCrCb = new Scalar(y, cr, cb);
    }
    public void configureBorders(double borderLeftX, double borderRightX, double borderTopY, double borderBottomY) {
        this.borderLeftX = borderLeftX;
        this.borderRightX = borderRightX;
        this.borderTopY = borderTopY;
        this.borderBottomY = borderBottomY;
    }
    Telemetry telemetry;
    public OkHearMeOut(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        CAMERA_WIDTH = input.width();
        CAMERA_HEIGHT = input.height();
        try {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
            Core.inRange(mat, scalarLowerYCrCb, scalarUpperYCrCb, processed);
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());
            Imgproc.GaussianBlur(processed, processed, new Size(5.0, 15.0), 0.00);
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));
            synchronized (sync) {
                for (MatOfPoint contour : contours) {
                    Point[] contourArray = contour.toArray();
                    if (contourArray.length >= 15) {
                        MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                        Rect rect = Imgproc.boundingRect(areaPoints);
                        if (rect.area() > maxArea
                                && rect.x + (rect.width / 2.0) > (borderLeftX * CAMERA_WIDTH)
                                && rect.x + (rect.width / 2.0) < CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH) && rect.y + (rect.height / 2.0) > (borderTopY * CAMERA_HEIGHT)
                                && rect.y + (rect.height / 2.0) < CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT) || loopCounter - pLoopCounter > 6
                                && rect.x + (rect.width / 2.0) > (borderLeftX * CAMERA_WIDTH) && rect.x + (rect.width / 2.0) < CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH)
                                && rect.y + (rect.height / 2.0) > (borderTopY * CAMERA_HEIGHT) && rect.y + (rect.height / 2.0) < CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT)
                        ) {
                            maxArea = 500;
                            maxRect = rect;
                            pLoopCounter++;
                            loopCounter = pLoopCounter;
                            first = true;
                        } else if (loopCounter - pLoopCounter > 10) {
                            maxArea = new Rect().area();
                            maxRect = new Rect();
                        }
                        areaPoints.release();
                    }
                    contour.release();
                }
                if (contours.isEmpty()) {
                    maxRect = new Rect(600, 1, 1, 1);
                }
            }
            if (first && maxRect.area() > 4200) { //swivel length == 10.5 in. pole distance == 8in.
                Imgproc.rectangle(input, maxRect, new Scalar(0, 255, 0), 2);
            }
            Imgproc.rectangle(input, new Rect(
                    (int) (borderLeftX * CAMERA_WIDTH),
                    (int) (borderTopY * CAMERA_HEIGHT),
                    (int) (CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH) - (borderLeftX * CAMERA_WIDTH)),
                    (int) (CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT) - (borderTopY * CAMERA_HEIGHT))
            ), YELLOW, 2);
            //wrist correcting
            if (!pole_midpoint_range) {
                x_distance_pixel = ((160 - getRectMidpointX()) / 112);
                wristCorrF = (-0.0005 * getRectArea()) + 12.184;
                tangent = (Math.toDegrees(Math.tan(x_distance_pixel / wristCorrF))) * 0.015;
            }
            //Swivel Correction(in testing)

//            currentX = Math.abs(getRectMidpointX());
//            if (currentX - previousX > 32){
//                tangent = 0;
//                noCorrect = true;
//            }
//            previousX = currentX;
//            err = 8.5 - ((-0.0005 * getRectArea()) + 15.184);
//            tangent2 = Math.toDegrees(Math.atan(err / 10.5));
//            swivelServoPos = 0.63 - (0.00333 * tangent2);
            //Imgproc.putText(input, "multi: " + getRectArea(), new Point(5, CAMERA_HEIGHT - 5), 0, 0.6, new Scalar(255, 255, 255), 2);
            loopCounter++;
        } catch (Exception e) {
            debug = e;
            error = true;
        }
        return input;
    }
    public double absValue(){ return currentX - previousX; }
    public boolean validTarget(){ return first; }
    public boolean wrist() { return bool1; }
    public double tangents() { return tangent; }
    public int getRectHeight() {synchronized (sync) {return maxRect.height;}}
    public int getRectWidth() {synchronized (sync) {return maxRect.width;}}
    public int getRectX() {synchronized (sync) {return maxRect.x;}}
    public int getRectY() {synchronized (sync) {return maxRect.y;}}
    public double getRectMidpointX() {synchronized (sync) {return getRectX() + (getRectWidth() / 2.0);}}
    public double getRectMidpointY() {synchronized (sync) {return getRectY() + (getRectHeight() / 2.0);}}
    public Point getRectMidpointXY() {synchronized (sync) {return new Point(getRectMidpointX(), getRectMidpointY());}}
    public double getRectArea() {synchronized (sync) {return maxRect.area();}}
}