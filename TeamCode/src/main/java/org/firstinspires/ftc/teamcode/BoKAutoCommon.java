package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Bitmap;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import android.graphics.Typeface;
import android.util.Log;
import android.view.View;
import android.widget.RelativeLayout;
import android.widget.TextView;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;
import java.util.concurrent.Semaphore;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Created by Krishna Saxena on 11/15/2016.
 * Implements the common algorithms used both by BoKAutoBlue* and BoKAutoRed*.
 * Its primary responsibilities include:
 * initSoftware() method which
 *   1. initialize OpenCV
 *   2. initialize Vuforia
 * moveForward() method
 */
public abstract class BoKAutoCommon implements BoKAuto
{
    // CONSTANTS
    private static final double DT_RAMP_SPEED_INIT = 0.2;
    private static final double P_TURN_COEFF = 0.5;

    private static final int SPHERE_LOC_Y_MIN = 300;
    private static final int SPHERE_LOC_Y_MAX = 500;
    private static final int CUBE_LOC_Y_RIGHT = 225;
    private static final int CUBE_LOC_Y_CENTER = 500;
    private static final int ROI_WIDTH = 50;
    private static final int ROI_HEIGHT = 50;
    private static final int YELLOW_PERCENT = 50;
    private static final String VUFORIA_CUBE_IMG = "vuImage.png";
    private static final String VUFORIA_ROI_IMG = "vuImageROI.png";

    protected static final float mmPerInch        = 25.4f;
    protected static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    protected static final float mmTargetHeight   = (6) * mmPerInch;// the height of the center of the target image above the floor

    private static final double Kp = 0.1;
    private static final double Ki = 0; //0.165;
    private static final double Kd = 0; //0.093;
    private static final double  SAMPLE_RATE_SEC = 0.05;
    private static final int RS_DIFF_THRESHOLD_CM = 5;
    private static final double DETECT_BUMP_THRESHOLD = 1.5;

    // Constants for runAuto
    private static final double HANGLIFT_POWER = 0.75;
    private static final double STRAFE_POWER = 0.5;
    private static final double STRAFE_ROTATIONS = 3;
    private static final double MOVE_POWER = 0.3;
    private static final int DISTANCE_TO_WALL_LEFT_CUBE_INIT = 98;     // cm
    private static final int DISTANCE_TO_WALL_LEFT_CUBE_FINAL = 71;    // cm
    private static final int DISTANCE_TO_WALL_CENTER_CUBE_INIT = 120;  // cm
    private static final int DISTANCE_TO_WALL_CENTER_CUBE_FINAL = 112; // cm
    private static final int DISTANCE_TO_WALL_RIGHT_CUBE_INIT = 120;  // cm
    private static final int DISTANCE_TO_WALL_RIGHT_CUBE_FINAL = 150; // cm
    private static final int DISTANCE_TO_WALL_BEFORE_TURN = 30; // cm

    private boolean targetVisible = false;
    List<VuforiaTrackable> allTrackables;
    VuforiaTrackable blueRover, redFootprint, frontCraters, backSpace;

    public enum BoKAutoCubeLocation {
        BOK_CUBE_UNKNOWN,
        BOK_CUBE_LEFT,
        BOK_CUBE_CENTER,
        BOK_CUBE_RIGHT
    }

    //private static final double HEADING_THRESHOLD = 1;
    protected static final boolean DEBUG_OPEN_CV = false;

    private AppUtil appUtil = AppUtil.getInstance();
    private VuforiaLocalizer vuforiaFTC;
    VuforiaTrackables targetsRoverRuckus;

    protected ElapsedTime runTime  = new ElapsedTime();

    protected BoKAllianceColor allianceColor;
    protected BoKAutoOpMode opMode;  // save a copy of the current opMode and robot
    protected BoKHardwareBot robot;

    protected boolean far = false;
    protected Orientation angles;

    private BaseLoaderCallback loaderCallback = new BaseLoaderCallback(appUtil.getActivity())
    {
        @Override
        public void onManagerConnected(int status) {
            super.onManagerConnected(status);
        }
    };

    @Override
    public BoKAutoStatus initSoftware(BoKAutoOpMode opMode,
                                      BoKHardwareBot robot)
    {
        Log.v("BOK", "Initializing OpenCV");
        // Initialize OpenCV
        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION,
                    appUtil.getActivity(), loaderCallback);
        }
        else {
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        Log.v("BOK", "Initializing Vuforia");
        // Initialize Vuforia
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the
         * RC phone); If no camera monitor is desired, use the parameterless constructor instead.
         */
        int cameraMonitorViewId =
                opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                        "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters =
                new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        // Vuforia License Key
        parameters.vuforiaLicenseKey = "AcBslPH/////AAABmdIsKH48AUtFgqZowN2tKAE9QKaxlJhsIfCyTmm/zjeMHNMyQXD7kidLriCV3tr2hOE2WJGAFKv9TLFaCpd23sDG9ms9fqZbLDxdDjt3Vfa9jdPid72/Z1E6xOgTija5FPAqK0/DQe/Ngg7VXSM0zwUOIbjuhonntnbZjywQ7hRCZ4is7ybZwiRiA1vyNuT7WCK9pOwbeNEJc/n1+HfZeEGfM+6ZnjRrl96qudE4B6W2FhrBKy85WyybPaxNjX5XUFfyuouJYIKA/Lve1qgkkVYHExk7up/EuOdhfl97IpoYMo7Ru+4x8SNolAeIlzitsPsTrxb5eZgWJxF3T61/uUkMLSMu8XpYJc1myd66bAjo";
        vuforiaFTC = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforiaFTC.setFrameQueueCapacity(1); // change the frame queue capacity to 1
        setupVuforia();
        //CameraDevice.getInstance().setFlashTorchMode(true);

        Log.v("BOK", "Done initializing software");
        this.opMode = opMode;
        this.robot = robot;

        while (!opMode.gamepad1.x){
            opMode.telemetry.addData("Status", "Press \"X\" to start gyro init");
            opMode.telemetry.update();
        }

        opMode.telemetry.addData("Status", "Initializing gyro");
        opMode.telemetry.update();
        setupRobot();
        opMode.telemetry.addData("Status", "Done initializing gyro!");
        opMode.telemetry.update();
        return BoKAutoStatus.BOK_AUTO_SUCCESS;
    }

    String format(OpenGLMatrix transformationMatrix)
    {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    private void setupVuforia(){
        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsRoverRuckus = this.vuforiaFTC.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables= new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        final int CAMERA_FORWARD_DISPLACEMENT  = 102;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 0;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).
                    setPhoneInformation(phoneLocationOnRobot, BACK);
        }
        targetsRoverRuckus.activate();
    }

    @Override
    public abstract void runSoftware();

    public void setupRobot()
    {
        // now initialize the IMU
        robot.initializeImu();
    }

    protected static void writeFile(String fname, Mat img, boolean always)
    {
        if (always || DEBUG_OPEN_CV) {
            String filePath = "/sdcard/FIRST/" + fname;
            //Log.v("BOK", "Saving image" + filePath);
            Imgcodecs.imwrite(filePath, img);
        }
    }

    private Mat setupOpenCVImg(Image rgb, String fileName, boolean always)
    {
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(),
                                        rgb.getHeight(),
                                        Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        Mat img = new Mat(rgb.getHeight(), rgb.getWidth(), CvType.CV_8UC3);
        Utils.bitmapToMat(bm, img);

        // OpenCV only deals with BGR
        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2BGR);
        writeFile(fileName, img, always);

        // First convert from BGR to HSV; separate the color components from intensity.
        // Increase robustness to lighting changes.
        // Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV);
        return img;
    }

    // Algorithm to move forward using encoder sensor on the DC motors on the drive train
    protected void move(double leftPower,
                        double rightPower,
                        double inches,
                        boolean forward,
                        double waitForSec)
    {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            robot.resetDTEncoders();
            robot.startMove(leftPower, rightPower, inches, forward);

            runTime.reset();
            while (opMode.opModeIsActive() &&
                    /*(robot.getDTCurrentPosition() == false) &&*/
                    robot.areDTMotorsBusy()) {
                if (runTime.seconds() >= waitForSec) {
                    Log.v("BOK", "move timed out!" + String.format(" %.1f", waitForSec));
                    break;
                }
            }

            robot.stopMove();
        }
    }

    protected void moveRamp(double maxPower,
                            double inches,
                            boolean forward,
                            double waitForSec)
    {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            boolean steady = false;
            robot.resetDTEncoders();
            int targetEncCount = robot.startMove(DT_RAMP_SPEED_INIT,
                                                 DT_RAMP_SPEED_INIT,
                                                 inches,
                                                 forward);
            // speed up during the initial 1/4 enc counts
            // maintain max power during the next 1/2 enc counts
            // speed down during the last 1/4 enc counts
            int rampupEncCount = targetEncCount/4;
            int rampdnEncCount = targetEncCount - rampupEncCount;
            double ratePower = (maxPower - DT_RAMP_SPEED_INIT)/rampupEncCount;

            runTime.reset();
            while (opMode.opModeIsActive() &&
                    /*(robot.getDTCurrentPosition() == false) &&*/
                    robot.areDTMotorsBusy()) {
                if (runTime.seconds() >= waitForSec) {
                    Log.v("BOK", "moveRamp timed out!" + String.format(" %.1f", waitForSec));
                    break;
                }
                int lfEncCount = Math.abs(robot.getLFEncCount());
                if (lfEncCount < rampupEncCount) {
                    double power = DT_RAMP_SPEED_INIT + ratePower * lfEncCount;
                    //Log.v("BOK", lfEncCount + " power Up: " + String.format("%.2f", power));
                    robot.setPowerToDTMotors(power, forward);

                }
                else if (lfEncCount < rampdnEncCount) {
                    if (!steady) {
                        robot.setPowerToDTMotors(maxPower, forward);
                        steady = true;
                    }
                }
                else {
                    double power = DT_RAMP_SPEED_INIT -
                            ratePower * (lfEncCount - targetEncCount);
                    //Log.v("BOK", lfEncCount + " power dn: " + String.format("%.2f", power));
                    robot.setPowerToDTMotors(power, forward);
                }
            }
            // finally stop moving
            robot.stopMove();
        }
    }
/*
    protected void setPowerToDTMotorsStrafeForTime(double power, double time, boolean right)
    {   runTime.reset();
        while(opMode.opModeIsActive() && runTime.seconds() < time) {
                robot.setPowerToDTMotorsStrafe(power, right);
        }
        robot.setPowerToDTMotors(0);
    }*/
/*
    protected void strafe(double maxPower,
                              double rotations,
                              boolean right,
                              double waitForSec)
    {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            robot.resetDTEncoders();
            int targetEncCount = -1;
            try {
                targetEncCount = robot.startStrafeWEnc(maxPower, rotations, right);
            }
            catch (UnsupportedOperationException e) {
                return;
            }
            Log.v("BOK", "strafeRamp: " + targetEncCount);

            // speed up during the initial 1/4 enc counts
            // maintain max power during the next 1/2 enc counts
            // speed down during the last 1/4 enc counts
            int rampupEncCount = targetEncCount/4;
            int rampdnEncCount = targetEncCount - rampupEncCount;
            double ratePower = (maxPower - DT_RAMP_SPEED_INIT)/rampupEncCount;

            runTime.reset();
            while (opMode.opModeIsActive() &&
                    //(robot.getDTCurrentPosition() == false) &&
                    (robot.getAvgEncCount() < targetEncCount)) {
                if (runTime.seconds() >= waitForSec) {
                    Log.v("BOK", "strafeRamp timed out!" + String.format(" %.1f", waitForSec));
                    break;
                }

                //int lfEncCount = Math.abs(robot.getLFEncCount());
                //if (lfEncCount < rampupEncCount) {
                    //double power = DT_RAMP_SPEED_INIT + ratePower*lfEncCount;
                    //Log.v("BOK", lfEncCount + " power Up: " + String.format("%.2f", power));
                    //robot.setPowerToDTMotorsStrafe(power, right);
                //}
                //else if (lfEncCount < rampdnEncCount) {
                //robot.setPowerToDTMotorsStrafe(maxPower, right);
                //}
                //
                //else {
                //    double power = DT_RAMP_SPEED_INIT - ratePower*(lfEncCount - targetEncCount);
                //    //Log.v("BOK", lfEncCount + " power dn: " + String.format("%.2f", power));
                //    robot.setPowerToDTMotorsStrafe(power, right);
                //}
            }
            robot.stopMove();
        }
    }
    */
    /*
    protected void strafeRamp(double maxPower,
                              double rotations,
                              boolean right,
                              double waitForSec)
    {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            robot.resetDTEncoders();
            int targetEncCount = -1;
            try {
                targetEncCount = robot.startStrafe(DT_RAMP_SPEED_INIT, rotations, right);
            }
            catch (UnsupportedOperationException e) {
                return;
            }
            Log.v("BOK", "strafeRamp: " + targetEncCount);

            // speed up during the initial 1/4 enc counts
            // maintain max power during the next 1/2 enc counts
            // speed down during the last 1/4 enc counts
            int rampupEncCount = targetEncCount/4;
            int rampdnEncCount = targetEncCount - rampupEncCount;
            double ratePower = (maxPower - DT_RAMP_SPEED_INIT)/rampupEncCount;

            runTime.reset();
            while (opMode.opModeIsActive() &&
                    //(robot.getDTCurrentPosition() == false) &&
                    robot.areDTMotorsBusy()) {
                if (runTime.seconds() >= waitForSec) {
                    Log.v("BOK", "strafeRamp timed out!" + String.format(" %.1f", waitForSec));
                    break;
                }

                int lfEncCount = Math.abs(robot.getLFEncCount());
                if (lfEncCount < rampupEncCount) {
                    double power = DT_RAMP_SPEED_INIT + ratePower*lfEncCount;
                    //Log.v("BOK", lfEncCount + " power Up: " + String.format("%.2f", power));
                    robot.setPowerToDTMotorsStrafe(power, right);
                }
                else if (lfEncCount < rampdnEncCount) {
                    robot.setPowerToDTMotorsStrafe(maxPower, right);
                }

                else {
                    double power = DT_RAMP_SPEED_INIT - ratePower*(lfEncCount - targetEncCount);
                    //Log.v("BOK", lfEncCount + " power dn: " + String.format("%.2f", power));
                    robot.setPowerToDTMotorsStrafe(power, right);
                }
            }
            robot.stopMove();
        }
    }
    */

    private boolean isCubePresent(Mat imgHSV, Rect roi)
    {
        Mat hist = new Mat();
        MatOfInt histSize = new MatOfInt(180);
        MatOfFloat ranges = new MatOfFloat(0f, 180f);
        Mat mask = new Mat(imgHSV.rows(), imgHSV.cols(),
                CvType.CV_8UC1, new Scalar(0));
        float[] resFloat = new float[180];
        boolean foundYellow = false;

        Imgproc.rectangle(imgHSV, new Point(roi.x, roi.y),
                new Point(roi.x + roi.width,
                        roi.y + roi.height),
                new Scalar(0, 255, 0), 10);

        Mat subMask = mask.submat(roi);
        subMask.setTo(new Scalar(255));

        Imgproc.calcHist(Arrays.asList(imgHSV), new MatOfInt(0),
                mask, hist, histSize, ranges);
        //writeFile(HSV_IMG, img, DEBUG_OPEN_CV);
        //Core.normalize(hist, hist, 256, 0, Core.NORM_MINMAX);
        hist.get(0, 0, resFloat);

        int p, nYellowPixels = 0;
        int numPixels = roi.width * roi.height;
        // Red is 0 (in HSV),
        // but we need to check between 10 and 35
        for (p = 10; p < 35; p++) {
            nYellowPixels += (int) resFloat[p];
        }

        if (nYellowPixels >= ((numPixels * YELLOW_PERCENT)/100))
            foundYellow = true;

        Log.v("BOK", "num Yellow pixels: " + nYellowPixels + " out of " + numPixels);

        hist.release();
        histSize.release();
        ranges.release();
        mask.release();

        return foundYellow;
    }

    protected BoKAutoCubeLocation findCube() {
        BoKAutoCubeLocation ret = BoKAutoCubeLocation.BOK_CUBE_LEFT;
        VuforiaLocalizer.CloseableFrame frame;

        // takes the frame at the head of the queue
        try {
            frame = vuforiaFTC.getFrameQueue().take();
        } catch (InterruptedException e) {
            Log.v("BOK", "Exception while finding cube!!");
            return ret;
        }

        long numImages = frame.getNumImages();
        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                Image rgb = frame.getImage(i);
                // rgb is now the Image object that we’ve used in the video
                if (rgb != null) {
                    Mat src = setupOpenCVImg(rgb, VUFORIA_CUBE_IMG, true);
                    Mat srcHSV = new Mat();

                    Mat srcGray = new Mat(); // Convert image to gray scale
                    Imgproc.cvtColor(src, srcGray, Imgproc.COLOR_BGR2GRAY);
                    Imgproc.cvtColor(src, srcHSV, Imgproc.COLOR_BGR2HSV);

                    // Apply a blur to reduce noise and avoid false circle detection
                    //Imgproc.blur(srcGray, srcGray, new Size(3, 3));

                    Mat circles = new Mat();
                    Point[] centerPoints = new Point[3];

                    Imgproc.HoughCircles(
                            srcGray, // input image gray scale
                            circles, // a vector that stores 3 values: xc, yc, and r
                                     // for each detected circle
                            Imgproc.HOUGH_GRADIENT, // Detection method
                            1.0, // inverse ratio of resolution
                            (double) srcGray.rows() / 16, // min distance between centers
                            100.0, // Upper threshold for internal Canny edge detector
                            40.0, // Threshold for center detection
                            20, // Minimum radius to be detected
                            105); // Maximum radius to be detected

                    int numCircles = 0;
                    for (int x = 0; x < circles.cols(); x++) {
                        double[] c = circles.get(0, x);
                        Point pt = new Point(Math.round(c[0]), Math.round(c[1]));

                        if ((pt.y > SPHERE_LOC_Y_MIN) &&
                            (pt.y < SPHERE_LOC_Y_MAX)) {
                            // Now add histogram calculation for double check
                            Rect roi = new Rect((int)(pt.x - (ROI_WIDTH/2)),
                                    (int)(pt.y - (ROI_HEIGHT/2)),
                                    ROI_WIDTH,ROI_HEIGHT);

                            if (isCubePresent(srcHSV, roi)) {
                                Log.v("BOK", "Detected Yellow in a sphere!!");
                            }

                            centerPoints[numCircles] = pt;
                            // circle center
                            //Imgproc.circle(src, centerPoints[numCircles], 1, new Scalar(0,100,100), 3, 8, 0 );
                            // circle outline
                            int radius = (int) Math.round(c[2]);
                            //Log.v("BOK", "Center: " + center + ", Radius: " + radius);
                            Imgproc.circle(src, centerPoints[numCircles], radius, new Scalar(255,0,255), 3, 8, 0 );
                            numCircles++;
                        } // if detected circle is within the ROI band
                    } // for each circle detected

                    if (numCircles == 2) {
                        if ((centerPoints[0].x < 850) &&
                            (centerPoints[1].x < 850)) {
                            ret = BoKAutoCubeLocation.BOK_CUBE_RIGHT;
                        }
                        else if ((centerPoints[0].x > 350) &&
                                 (centerPoints[1].x) > 350) {
                            ret = BoKAutoCubeLocation.BOK_CUBE_LEFT;
                        }
                        else {
                            ret = BoKAutoCubeLocation.BOK_CUBE_CENTER;
                        }
                    }
                    else {
                        Log.v("BOK", "Detected " + numCircles + "!!");
                    }

                    writeFile(VUFORIA_ROI_IMG, src, true);
                    src.release();
                    srcHSV.release();
                    srcGray.release();
                    circles.release();
                }
                break;
            } // PIXEL_FORMAT.RGB565
        } // for (int i = 0; i < numImages; i++)
        frame.close();

        return ret;
    }

    protected void takePicture(String sFileName) {
        VuforiaLocalizer.CloseableFrame frame;
        // takes the frame at the head of the queue
        try {
            frame = vuforiaFTC.getFrameQueue().take();
        } catch (InterruptedException e) {
            Log.v("BOK", "Exception while taking picture!!");
            return;
        }
        long numImages = frame.getNumImages();
        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                Image rgb = frame.getImage(i);
                // rgb is now the Image object that we’ve used in the video
                if (rgb != null) {
                    Mat img = setupOpenCVImg(rgb, sFileName, true);
                    img.release();
                }
                break;
            } // PIXEL_FORMAT.RGB565
        } // for (int i = 0; i < numImages; i++)
        frame.close();
    }

    // Code copied from the sample PushbotAutoDriveByGyro_Linear
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public double gyroTurn(double speed,
                           double init_angle,
                           double angle,
                           int threshold,
                           boolean tank,
                           boolean leftTank,
                           double waitForSeconds)
    {
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runTime.reset();

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && 
               !onHeading(speed, init_angle, angle, threshold, tank, leftTank, P_TURN_COEFF)) {
            if (runTime.seconds() >= waitForSeconds) {
                Log.v("BOK", "gyroTurn timed out!" + String.format(" %.1f", waitForSeconds));
                break;
            }
            // Update telemetry & Allow time for other processes to run.
            opMode.telemetry.update();
            //opMode.sleep(BoKHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
        }

        robot.setPowerToDTMotors(0);
        Log.v("BOK", "turnF: " + angles.thirdAngle);
        return angles.thirdAngle;
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    protected boolean onHeading(double speed,
                                double init_angle,
                                double angle,
                                int threshold,
                                boolean tank,
                                boolean leftTank,
                                double PCoeff)
    {
        double   error ;
        double   steer ;
        boolean  onTarget = false;
        double   leftSpeed;
        double   rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);
        if (Math.abs(error) <= (Math.abs(angle-init_angle)*0.25)) {
            if (speed > DT_TURN_SPEED_HIGH) {
                speed = DT_TURN_SPEED_HIGH;
            }
            speed /= 2;
        }

        if (Math.abs(error) <= threshold) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);

            rightSpeed  = speed * steer;
            if (rightSpeed > 0)
                rightSpeed = Range.clip(rightSpeed,
                                        DT_TURN_SPEED_LOW,
                                        DT_TURN_SPEED_HIGH);
            else
                rightSpeed = Range.clip(rightSpeed,
                                        -DT_TURN_SPEED_HIGH,
                                        -DT_TURN_SPEED_LOW);

            if(!tank)
                leftSpeed   = rightSpeed;
            else if (leftTank) {
                leftSpeed = rightSpeed;
                rightSpeed = 0;
            }
            else
                leftSpeed = 0;
        }

        // Send desired speeds to motors.
        robot.setOnHeading(leftSpeed, rightSpeed);

        /*Log.v("BOK", "Err: " + String.format("%.2f", error) + ", Steer: " +
              String.format("%.2f", steer));
        Log.v("BOK", "Left Speed: " + String.format("%5.2f", leftSpeed) + ", Right Speed: " +
              String.format("%5.2f", rightSpeed));*/

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle
     *          Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of
     *          reference; +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    protected double getError(double targetAngle)
    {
        double robotError;

        // calculate error in -179 to +180 range  (
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                                                 AxesOrder.XYZ,
                                                 AngleUnit.DEGREES);
        robotError = targetAngle - angles.thirdAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    protected double getSteer(double error, double PCoeff)
    {
        return Range.clip(error * PCoeff, -1, 1);
    }

    protected void moveHangLift(double power, int targetPos){
        robot.hangMotor.setTargetPosition(targetPos);
        robot.hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hangMotor.setPower(power);
        while (robot.hangMotor.isBusy()){

        }
        robot.hangMotor.setPower(0);
    }

    protected void followHeadingPID(double heading,
                                    double power,
                                    double dist,
                                    boolean detectBump,
                                    double waitForSec) {
        double angle, error, sumError = 0, lastError = 0, diffError, turn, speedL, speedR,
                lastTime = 0;
        double targetEnc = robot.getTargetEncCount(dist);
        //String logString = "dTime,ang,err,sum,last,diff,turn,speedL,speedR\n";
        robot.resetDTEncoders();
        runTime.reset();
        while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)
                && (robot.getAvgEncCount() < targetEnc)) {
            double currTime = runTime.seconds();
            double deltaTime = currTime - lastTime;
            if (deltaTime >= SAMPLE_RATE_SEC) {
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES);
                angle = angles.thirdAngle;
                error = angle - heading;
                sumError = sumError * 0.66 + error;
                diffError = error - lastError;
                turn = Kp * error + Ki * sumError + Kd * diffError;
                speedL = power + turn;
                speedR = power - turn;
                speedR *= -1;

                speedL = Range.clip(speedL, DT_MOVE_LOW, Math.abs(2 * power));
                speedR = Range.clip(speedR, -Math.abs(2 * power), -DT_MOVE_LOW);

                speedL = Range.clip(speedL, -1, 1);
                speedR = Range.clip(speedR, -1, 1);
                robot.setPowerToDTMotors(speedL, speedR);

                Log.v("BOK", "angle " + String.format("%.2f", angle) + ",error " + String.format("%.2f",error)
                        + ",turn " + String.format("%.2f", turn) + ",L: " + String.format("%.2f", speedL)
                        + ",R: " + String.format("%.2f", speedR) + " enc " + String.format("%.2f", robot.getAvgEncCount()));
                /*logString = logString + deltaTime + "," +angle + "," + error + ","
                        + sumError + "," + lastError + "," + diffError + ","
                        + turn + "," + speedL + "," + speedR + "\n";*/
                lastError = error;
                lastTime = currTime;
                if (detectBump) {
                    Log.v("BOK", "theta x " + angles.firstAngle + " theta y " + angles.thirdAngle);
                    if (Math.abs(angles.secondAngle) > DETECT_BUMP_THRESHOLD) {
                        Log.v("BOK", "theta y in if " + angles.secondAngle);
                        break;
                    }
                }
            }
        }
        robot.setPowerToDTMotors(0);
        /*File file = AppUtil.getInstance().getSettingsFile("BoKGyroDataFwd.csv");
        ReadWriteFile.writeFile(file,
                logString);*/
    }

    protected void followHeadingPIDBack(double heading,
                                    double power,
                                    double dist,
                                    boolean detectBump,
                                    double waitForSec) {
        double angle, error, sumError = 0, lastError = 0, diffError, turn, speedL, speedR,
                lastTime = 0;
        double targetEnc = robot.getTargetEncCount(dist);
        //String logString = "dTime,ang,err,sum,last,diff,turn,speedL,speedR\n";
        robot.resetDTEncoders();
        runTime.reset();
        while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)
                && (robot.getAvgEncCount() < targetEnc)) {
            double currTime = runTime.seconds();
            double deltaTime = currTime - lastTime;
            if (deltaTime >= SAMPLE_RATE_SEC) {
                angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES).thirdAngle;
                error = angle - heading;
                sumError = sumError * 0.66 + error;
                diffError = error - lastError;
                turn = Kp * error + Ki * sumError + Kd * diffError;
                speedL = power + turn;
                speedR = power - turn;
                speedR *= -1;

                speedL = Range.clip(speedL, -Math.abs(2 * power), -DT_MOVE_LOW);
                speedR = Range.clip(speedR, DT_MOVE_LOW, Math.abs(2 * power));

                speedL = Range.clip(speedL, -1, 1);
                speedR = Range.clip(speedR, -1, 1);
                robot.setPowerToDTMotors(speedL, speedR);

                Log.v("BOK", "angle " + String.format("%.2f", angle) + ",error " + String.format("%.2f",error)
                        + ",turn " + String.format("%.2f", turn) + ",L: " + String.format("%.2f", speedL)
                        + ",R: " + String.format("%.2f", speedR) + " enc " + String.format("%.2f", robot.getAvgEncCount()));
                /*logString = logString + deltaTime + "," +angle + "," + error + ","
                        + sumError + "," + lastError + "," + diffError + ","
                        + turn + "," + speedL + "," + speedR + "\n";*/
                lastError = error;
                lastTime = currTime;
                if (detectBump) {
                    if (angles.firstAngle < DETECT_BUMP_THRESHOLD) {
                        Log.v("BOK", "theta x " + angles.firstAngle);
                        break;
                    }
                }
            }
        }
        robot.setPowerToDTMotors(0);
        /*File file = AppUtil.getInstance().getSettingsFile("BoKGyroDataBack.csv");
        ReadWriteFile.writeFile(file,
                logString);*/
    }

    protected void followHeadingPIDWithDistance(double heading,
                                                double power,
                                                double distTarget,
                                                boolean detectBump,
                                                double waitForSec)
    {
        double angle, error, sumError = 0, lastError = 0, diffError, turn, speedL, speedR,
                lastTime = 0;
        //String logString = "dTime,ang,err,sum,last,diff,turn,speedL,speedR\n";
        robot.resetDTEncoders();
        runTime.reset();
        double dist = robot.getDistanceCM(robot.distanceBack, distTarget+50, 0.25);
        Log.v("BOK", "Distance outside loop FWD" + dist);
        while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {

            if (dist > distTarget)
                break;

            //Log.v("BOK", "Distance in loop " + dist);
            double currTime = runTime.seconds();
            double deltaTime = currTime - lastTime;
            if (deltaTime >= SAMPLE_RATE_SEC) {
                angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES).thirdAngle;
                error = angle - heading;
                sumError = sumError * 0.66 + error;
                diffError = error - lastError;
                turn = Kp * error + Ki * sumError + Kd * diffError;
                speedL = power + turn;
                speedR = power - turn;
                speedR *= -1;

                speedL = Range.clip(speedL, DT_MOVE_LOW, Math.abs(2 * power));
                speedR = Range.clip(speedR, -Math.abs(2 * power), -DT_MOVE_LOW);

                speedL = Range.clip(speedL, -1, 1);
                speedR = Range.clip(speedR, -1, 1);
                robot.setPowerToDTMotors(speedL, speedR);

                Log.v("BOK", "angle " + String.format("%.2f", angle) + ",error " + String.format("%.2f",error)
                        + ",turn " + String.format("%.2f", turn) + ",L: " + String.format("%.2f", speedL)
                        + ",R: " + String.format("%.2f", speedR) + " dist " + String.format("%.2f", dist));
                lastError = error;
                lastTime = currTime;
                dist = robot.getDistanceCM(robot.distanceBack, distTarget+50, 0.25);
                if (detectBump) {
                    if (angles.firstAngle < DETECT_BUMP_THRESHOLD) {
                        Log.v("BOK", "theta y " + angles.firstAngle);
                        break;
                    }
                }
            }
        }
        robot.setPowerToDTMotors(0);
        /*File file = AppUtil.getInstance().getSettingsFile("BoKGyroDataFwdDist.csv");
        ReadWriteFile.writeFile(file,
                logString);*/
    }

    protected void followHeadingPIDWithDistanceBack (double heading,
                                                double power,
                                                double distTarget,
                                                boolean detectBump,
                                                double waitForSec)
    {
        double angle, error, sumError = 0, lastError = 0, diffError, turn, speedL, speedR,
                lastTime = 0;
        //String logString = "dTime,ang,err,sum,last,diff,turn,speedL,speedR\n";
        robot.resetDTEncoders();
        runTime.reset();
        double dist = robot.getDistanceCM(robot.distanceBack, distTarget+50, 0.25);
        Log.v("BOK", "Distance outside loop BACK" + dist);
        while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {

            if (dist < distTarget)
                break;

            //Log.v("BOK", "Distance in loop " + dist);
            double currTime = runTime.seconds();
            double deltaTime = currTime - lastTime;
            if (deltaTime >= SAMPLE_RATE_SEC) {
                angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES).thirdAngle;
                error = angle - heading;
                sumError = sumError * 0.66 + error;
                diffError = error - lastError;
                turn = Kp * 0.5 * error + Ki * sumError + Kd * diffError;
                speedL = power + turn;
                speedR = power - turn;
                speedR *= -1;

                speedL = Range.clip(speedL, -Math.abs(2 * power), -DT_MOVE_LOW);
                speedR = Range.clip(speedR, DT_MOVE_LOW, Math.abs(2 * power));

                speedL = Range.clip(speedL, -1, 1);
                speedR = Range.clip(speedR, -1, 1);
                robot.setPowerToDTMotors(speedL, speedR);

                Log.v("BOK", "angle " + String.format("%.2f", angle) + ",error " + String.format("%.2f",error)
                        + ",turn " + String.format("%.2f", turn) + ",L: " + String.format("%.2f", speedL)
                        + ",R: " + String.format("%.2f", speedR) + " dist " + String.format("%.2f", dist));
                lastError = error;
                lastTime = currTime;
                dist = robot.getDistanceCM(robot.distanceBack, distTarget+50, 0.25);
                if (detectBump) {
                    if (angles.firstAngle < DETECT_BUMP_THRESHOLD) {
                        Log.v("BOK", "theta x " + angles.firstAngle);
                        break;
                    }
                }
            }
        }
        robot.setPowerToDTMotors(0);
        /*File file = AppUtil.getInstance().getSettingsFile("BoKGyroDataFwdDist.csv");
        ReadWriteFile.writeFile(file,
                logString);*/
    }

    public boolean moveWithRangeSensor(double power,
                                       int targetDistanceCm,
                                       int capDistCm,
                                       double waitForSec)
    {
        boolean result = true;

        double cmCurrent, diffFromTarget = targetDistanceCm, pCoeff, wheelPower;
        robot.setModeForDTMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        AnalogInput rangeSensor = robot.distanceBack;
        opMode.sleep(40);
        cmCurrent = robot.getDistanceCM(rangeSensor, capDistCm, 0.5);
        Log.v("BOK", "Distance from wall (outside loop)" + cmCurrent);
        //if (!Double.isNaN(cmCurrent))
        diffFromTarget = targetDistanceCm - cmCurrent;
        runTime.reset();

        while (opMode.opModeIsActive() &&
                (Math.abs(diffFromTarget) >= RS_DIFF_THRESHOLD_CM)) {
            //Log.v("BOK", "Distance from wall "+cmCurrent);
            if (runTime.seconds() >= waitForSec) {
                Log.v("BOK", "moveWithRS timed out!" + String.format(" %.1f", waitForSec));
                result = false;
                break;
            }

            //if (robot.getLFEncCount() >= encTarget) {
            //    Log.v("BOK", "moveWithRS moved too far!" + robot.getLFEncCount());
            //    result = false;
            //    break;
            //}

            cmCurrent = robot.getDistanceCM(rangeSensor, capDistCm, 0.5);
            if (Double.isNaN(cmCurrent) || (cmCurrent >= 255)) // Invalid sensor reading
                continue;

            diffFromTarget = targetDistanceCm - cmCurrent;
            pCoeff = diffFromTarget / 15;

            wheelPower = Range.clip(power * pCoeff, -Math.abs(power), Math.abs(power));
            if (wheelPower > 0 && wheelPower < 0.4)
                wheelPower = 0.4; // min power to move
            if (wheelPower < 0 && wheelPower > -0.4)
                wheelPower = -0.4;

            //Log.v("BOK", "CM current " + cmCurrent + " diff "+diffFromTarget);
            // back range sensor
            if (diffFromTarget < 0) // we are still far away!
                robot.setPowerToDTMotors(wheelPower, false /* going back*/);
            else
                // if diffFromTarget > 0 then wheelPower is +ve
                robot.setPowerToDTMotors(wheelPower, true /* going forward */);
            //Log.v("BOK", "Back current RS: " + cmCurrent +
            //        " Difference: " + diffFromTarget +
            //        " Power: (move fwd) " + wheelPower);
        }

        Log.v("BOK", "Back current RS: " + cmCurrent);
        robot.setPowerToDTMotors(0);

        return result;
    }

    public boolean moveWithRangeSensorBack(double power,
                                           int targetDistanceCm,
                                           int capDistCm,
                                           double waitForSec)
    {
        boolean result = true;

        double cmCurrent, diffFromTarget = targetDistanceCm, pCoeff, wheelPower;
        robot.setModeForDTMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        AnalogInput rangeSensor = robot.distanceBack;
        opMode.sleep(40);
        cmCurrent = robot.getDistanceCM(rangeSensor, capDistCm, 0.5);
        Log.v("BOK", "Distance from wall (outside loop)" + cmCurrent);

        //if (!Double.isNaN(cmCurrent))
        diffFromTarget = targetDistanceCm - cmCurrent;
        runTime.reset();

        while (opMode.opModeIsActive() &&
                (Math.abs(diffFromTarget) >= RS_DIFF_THRESHOLD_CM)) {
            //Log.v("BOK", "Distance from wall "+cmCurrent);
            if (runTime.seconds() >= waitForSec) {
                Log.v("BOK", "moveWithRS timed out!" + String.format(" %.1f", waitForSec));
                result = false;
                break;
            }

            //if (robot.getLFEncCount() >= encTarget) {
            //    Log.v("BOK", "moveWithRS moved too far!" + robot.getLFEncCount());
            //    result = false;
            //    break;
            //}

            cmCurrent = robot.getDistanceCM(rangeSensor, capDistCm, 0.5);
            if (Double.isNaN(cmCurrent) || (cmCurrent >= 255)) // Invalid sensor reading
                continue;

            diffFromTarget = targetDistanceCm - cmCurrent;
            pCoeff = diffFromTarget / 15;
            wheelPower = Range.clip(power * pCoeff, -Math.abs(power), Math.abs(power));
            if (wheelPower > 0 && wheelPower < 0.4)
                wheelPower = 0.4; // min power to move
            if (wheelPower < 0 && wheelPower > -0.4)
                wheelPower = -0.4;

            //Log.v("BOK", "CM current " + cmCurrent + " diff "+diffFromTarget);
            // back range sensor
            if (diffFromTarget < 0) // we are still far away!
                robot.setPowerToDTMotors(wheelPower, false /* going back*/);
            else
                // if diffFromTarget > 0 then wheelPower is +ve
                robot.setPowerToDTMotors(wheelPower, true /* going forward */);
            //Log.v("BOK", "Back current RS: " + cmCurrent +
            //        " Difference: " + diffFromTarget +
            //        " Power: (move fwd) " + wheelPower);
        }

        Log.v("BOK", "Back current RS: " + cmCurrent);
        robot.setPowerToDTMotors(0);

        return result;
    }

    class moveLiftDown extends Thread
    {
        @Override
        public void run()
        {
            moveHangLift(-0.75, 0);
        }

    }

    protected void dumpMarker(){
        robot.markerServo.setPosition(robot.MARKER_SERVO_FINAL);
        opMode.sleep(500);
        robot.markerServo.setPosition(robot.MARKER_SERVO_INIT);
    }

    public void runAuto(int xTarget, int yTarget, boolean doMarker, boolean atCrater)
    {
        BoKAutoCubeLocation loc = BoKAutoCubeLocation.BOK_CUBE_UNKNOWN;
        Log.v("BOK", "Angle at start " + robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES).thirdAngle);

        // Step 1: find gold location
        loc = findCube();
        // Step 2: Start motor for bringing the robot down (hanging lift)
        robot.hangMotor.setTargetPosition(robot.HANG_LIFT_HIGH_POS);
        robot.hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hangMotor.setPower(HANGLIFT_POWER);

        runTime.reset();
        while (robot.hangMotor.isBusy() && (runTime.seconds() < 10)) {
            // Do nothing while the robot is moving down
            // Log.v("BOK", "hang enc: " + robot.hangMotor.getCurrentPosition());
        }
        robot.hangMotor.setPower(0);
        if (runTime.seconds() >= 10)
            Log.v("BOK", "hang lift timed out");

        // Step 3: Lower the hook and move forward
        robot.hangHookServo.setPosition(robot.HANG_HOOK_SERVO_FINAL);
        opMode.sleep(250);
        //moveRamp(0.3, 2, true, 2);

        // Step 4: Point the distance sensor so that it is perpendicular to the footprint picture:
        // RED Crater; or the galaxy picture: Red Depot. For Blue Crater, we are pointing at the
        // rover picture & for Blue Depot, we are pointing to the crater picture.
        robot.distanceRotateServo.setPosition(robot.DISTANCE_ROTATE_SERVO_FINAL);

        Log.v("BOK", "Angle at end " + robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES).thirdAngle);

        // Step 5: straighten the robot after coming down from the hanging position
        gyroTurn(0.1, 0, 0, DT_TURN_THRESHOLD_LOW, false, false, 5);

        Log.v("BOK", "Angle before strafe " + robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES).thirdAngle);
        // Step 6: Turn, go forward, turn
        gyroTurn(0.6, 0, 90, DT_TURN_THRESHOLD_LOW+1, false, false, 6);
        moveRamp(0.5, 10, true, 5);
        gyroTurn(0.6, 90, 0, DT_TURN_THRESHOLD_LOW, false, false, 6);
        // Step 6: Strafe towards sampling
        //strafe(STRAFE_POWER, STRAFE_ROTATIONS, false, 6);
        opMode.sleep(250);
        Log.v("BOK", "Angle after strafe " + robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES).thirdAngle);
        // Step 7: again straighten the robot after strafing
        gyroTurn(0.1, 0, 0, DT_TURN_THRESHOLD_LOW, false, false, 3);
        Log.v("BOK", "Angle after turn " + robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES).thirdAngle);
        //
        // Step 8: find the distance to the wall
        double distSampling = robot.getDistanceCM(robot.distanceBack, 200, 0.5);
        Log.v("BOK", "Distance to wall " + distSampling);

        // Step 9: complete the sampling
        if (loc == BoKAutoCubeLocation.BOK_CUBE_LEFT) {
            Log.v("BOK", "Cube on left");
            // 9a: Move back till the sampling arm is between center and left minerals
            moveWithRangeSensorBack(-MOVE_POWER, DISTANCE_TO_WALL_LEFT_CUBE_INIT, 150, 2);
            // 9b: Lower the sampler arm!
            robot.samplerServo.setPosition(robot.SAMPLER_SERVO_FINAL);
            opMode.sleep(250);
            // 9c: Move past the cube
            moveWithRangeSensorBack(-MOVE_POWER, DISTANCE_TO_WALL_LEFT_CUBE_FINAL, 100, 2);
            // 9d: Raise the sampler arm
            robot.samplerServo.setPosition(robot.SAMPLER_SERVO_INIT);
        }
        else if (loc == BoKAutoCubeLocation.BOK_CUBE_CENTER){
            Log.v("BOK", "Cube on center");
            // 9a: Move forward till the sampling arm is between center and right minerals
            moveWithRangeSensor(MOVE_POWER, DISTANCE_TO_WALL_CENTER_CUBE_INIT, 150, 2);
            // 9b: Lower the sampler arm!
            robot.samplerServo.setPosition(robot.SAMPLER_SERVO_FINAL);
            opMode.sleep(250);
            // 9c: Move past the cube
            moveWithRangeSensorBack(-MOVE_POWER/2, DISTANCE_TO_WALL_CENTER_CUBE_FINAL, 125, 2);
            // 9d: Raise the sampler arm
            robot.samplerServo.setPosition(robot.SAMPLER_SERVO_INIT);
            followHeadingPIDWithDistanceBack(0, -MOVE_POWER, 110, false, 5);
            gyroTurn(0.1, 0, -5, DT_TURN_THRESHOLD_LOW, false, false, 5);
        }
        else { // loc == BoKAutoCubeLocation.BOK_CUBE_RIGHT
            Log.v("BOK", "Cube on right");
            // 9a: Move forward till the sampling arm is between center and right minerals
            moveWithRangeSensor(MOVE_POWER, DISTANCE_TO_WALL_RIGHT_CUBE_INIT, 150, 2);
            // 9b: Lower the sampler arm!
            robot.samplerServo.setPosition(robot.SAMPLER_SERVO_FINAL);
            // 9c: Move forward past the cube
            moveWithRangeSensor(MOVE_POWER, DISTANCE_TO_WALL_RIGHT_CUBE_FINAL, 175, 2);
            // 9d: Raise the sampler arm
            robot.samplerServo.setPosition(robot.SAMPLER_SERVO_INIT);
            followHeadingPIDWithDistanceBack(0, -MOVE_POWER, 110, false, 5);
            gyroTurn(0.1, 0, -5, DT_TURN_THRESHOLD_LOW, false, false, 5);
        }

        // Finished Sampling
        if(atCrater) {
            // Step 10: Go backwards towards the wall
            followHeadingPIDWithDistanceBack(0, -0.3, DISTANCE_TO_WALL_BEFORE_TURN,
                                         false, 6);

            // Turn towards the farther wall
            robot.distanceRotateServo.setPosition(robot.DISTANCE_ROTATE_SERVO_INIT);
            gyroTurn(DT_TURN_SPEED_LOW, 0, 40, DT_TURN_THRESHOLD_LOW,
                    false, false, 4);

            if (doMarker) {
                Log.v("BOK", "Dist to front wall " +
                        robot.getDistanceCM(robot.distanceBack, 190, 0.5));
                double distToWall = robot.getDistanceCM(robot.distanceBack, 190, 0.5) / 2.54;
                // Move to target distance of 80cm
                followHeadingPIDWithDistanceBack(45, -0.5, 60, false, 7);
                // Once distance reached, dump the marker
                dumpMarker();
                gyroTurn(DT_TURN_SPEED_LOW, 45, 48, DT_TURN_THRESHOLD_LOW, false, false, 4);
                //move backwards to 200cm
                followHeadingPID(48, 0.5, distToWall + 10, true, 15);// distToWall + 3
                dropIntakeArmAndExtend();
            }// if(doMarker)

        } // if(atCrater)
        else{
            robot.distanceRotateServo.setPosition(robot.DISTANCE_ROTATE_SERVO_FINAL);
            followHeadingPIDWithDistanceBack(0, -0.3, DISTANCE_TO_WALL_BEFORE_TURN, false, 10);
            robot.distanceRotateServo.setPosition(robot.DISTANCE_ROTATE_SERVO_INIT);
            gyroTurn(0.6, 0, -130, DT_TURN_THRESHOLD_LOW, false, false, 5);
            opMode.sleep(250);
            Log.v("BOK", "Dist to front wall " +
                    robot.getDistanceCM(robot.distanceBack, 700, 0.5));
            double distToWall = robot.getDistanceCM(robot.distanceBack, 200, 0.5) / 2.54;
            followHeadingPIDWithDistanceBack(-130, -0.4, 60, false, 10);
            dumpMarker();
            followHeadingPID(-135, 0.5, distToWall + 3, true, 15);
            dropIntakeArmAndExtend();
        }
    }

    protected void dropIntakeArmAndExtend() {
        robot.moveIntakeArmPID(1000, 0.5, 0.5, 3);
        robot.intakeArmMotor.setTargetPosition(1100);
        robot.intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.intakeArmMotor.setPower(0.5);
        moveRamp(0.25, 13, false, 6);
    }
}

