package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvException;
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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

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

    // Sampling locations in the image; phone's image is 1280x720s
    private static final int HOUGH_CIRCLE_MIN_RAD = 30;
    private static final int HOUGH_CIRCLE_MAX_RAD = 105;
    private static final int SPHERE_LOC_Y_MIN = 275;
    private static final int SPHERE_LOC_Y_MAX = 600;
    private static final int CUBE_LOC_LEFT_X_MIN = 350;
    private static final int CUBE_LOC_RIGHT_X_MAX = 850;
    private static final int ROI_WIDTH = 50;
    private static final int ROI_HEIGHT = 50;
    private static final int WHITE_PERCENT = 50;
    private static final String VUFORIA_CUBE_IMG = "vuImage.png";
    private static final String VUFORIA_ROI_IMG = "vuImageROI.png";

    private static final double  SAMPLE_RATE_SEC = 0.05;
    private static final int RS_DIFF_THRESHOLD_CM = 2;
    private static final double DETECT_BUMP_THRESHOLD = 1.5;

    // Constants for runAuto
    private static final double HANGLIFT_POWER = 0.95;
    //private static final double STRAFE_POWER = 0.5;
    //private static final double STRAFE_ROTATIONS = 3;
    private static final double MOVE_POWER_LOW = 0.3;
    private static final double MOVE_POWER_HIGH = 0.5;
    private static final int DISTANCE_TO_WALL_BEFORE_TURN = 30; // cm
   // private static final int LEFT_INTAKE_SLIDE_EXTENSION_INIT   = -300;
    //private static final int CENTER_INTAKE_SLIDE_EXTENSION_INIT = -400;
    //private static final int RIGHT_INTAKE_SLIDE_EXTENSION_INIT  = -400;
    //private static final int INTAKE_SLIDE_EXTENSION_FINAL       = -50;
    //private static final int DISTANCE_FORWARD_BEFORE_DUMPER_RESET = 20;

    public enum BoKAutoCubeLocation {
        BOK_CUBE_UNKNOWN,
        BOK_CUBE_LEFT,
        BOK_CUBE_CENTER,
        BOK_CUBE_RIGHT
    }

    protected static final boolean DEBUG_OPEN_CV = false;

    private AppUtil appUtil = AppUtil.getInstance();
    private VuforiaLocalizer vuforiaFTC;

    protected ElapsedTime runTime  = new ElapsedTime();

    protected BoKAllianceColor allianceColor;
    protected BoKAutoOpMode opMode;  // save a copy of the current opMode and robot
    protected BoKHardwareBot robot;

    protected Orientation angles;

    // All BoKAuto*OpModes must be derived from BoKAutoCommon. They must override runSoftware
    // method in order to run specific methods from BoKAutoCommon based on the missions.
    @Override
    public abstract void runSoftware();

    // OpenCV Manager callback when we connect to the OpenCV manager
    private BaseLoaderCallback loaderCallback = new BaseLoaderCallback(appUtil.getActivity())
    {
        @Override
        public void onManagerConnected(int status)
        {
            super.onManagerConnected(status);
        }
    };

    /*
     * initSoftware
     * Initialize OpenCV, Vuforia. Setup the frame queue in Vuforia to 1 so that we can
     * grab a frame from Vuforia and run OpenCV algorithm on the frame.
     * Wait for the drive team to press X on Gamepad1 before initializing IMU.
     * Parameters:
     * 1. opMode: caller's opMode
     * 2. robot: hardware used by the opMode
     * Returns BOK_AUTO_SUCCESS
     */
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
        //setupVuforia();
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

    /*
     * setupRobot
     * Helper method called by initSoftware to complete any software setup tasks.
     */
    private void setupRobot()
    {
        // now initialize the IMU
        robot.initializeImu();
    }

    /*
     * writeFile
     * Helper method to save a copy of the image seen by the robot. This image can be opened in
     * Paint for further analysis as the image is saved in PNG format.
     */
    protected static void writeFile(String fname, Mat img, boolean always)
    {
        if (always || DEBUG_OPEN_CV) {
            String filePath = "/sdcard/FIRST/" + fname;
            //Log.v("BOK", "Saving image" + filePath);
            Imgcodecs.imwrite(filePath, img);
        }
    }

    /*
     * setupOpenCVImg
     * Helper  method to setup OpenCV mat object from a rgb image in Vuforia.
     */
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

    /*
     * move
     * Algorithm to move forward or back using encoder sensor on the DC motors on the drive train
     * Parameters
     * leftPower, rightPower, inches to move, forward or back, waitForSec before timing out
     */
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

    /*
     * moveRamp
     * Algorithm to move forward or back using encoder sensor on the DC motors on the drive train
     * using a ramp up and ramp down period to prevent the robot from getting jerks.
     * Parameters
     * maxPower, inches to move, forward or back, waitForSec before timing out
     */
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

    protected void setPowerToDTMotorsStrafeForTime(double power, double time, boolean right)
    {   runTime.reset();
        while(opMode.opModeIsActive() && runTime.seconds() < time) {
                robot.setPowerToDTMotorsStrafe(power, right);
        }
        robot.setPowerToDTMotors(0);
    }

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

    /*
     * isItWhite
     * Helper method that checks for white values in the Value plane of the ROI. The
     * input image is in HSV format to prevent the effect of ambient light conditions.
     */
    private boolean isItWhite(Mat imgHSV, Rect roi)
    {
        Mat hist2 = new Mat();
        MatOfInt histSize2 = new MatOfInt(256);
        MatOfFloat ranges2 = new MatOfFloat(0f, 256f);
        Mat mask = new Mat(imgHSV.rows(), imgHSV.cols(),
                CvType.CV_8UC1, new Scalar(0));
        float[] resFloat2 = new float[256];
        boolean isItWhite = true;
        List<Mat> hsvPlanes = new ArrayList<>();
        Core.split(imgHSV, hsvPlanes);

        Mat subMask;
        try {
            subMask = mask.submat(roi);
        } catch (CvException cvE) {
            Log.v("BOK", "Caught CvException " + cvE.toString());
            try {
                Rect newRoi = new Rect(roi.x, roi.y, roi.width/2, roi.height/2);
                roi = newRoi;
                subMask = mask.submat(roi);
            } catch (CvException e) {
                Log.v("BOK", "Caught another CvException!" + cvE.toString());
                return true;
            }
        }
        subMask.setTo(new Scalar(255));

        Imgproc.calcHist(Arrays.asList(hsvPlanes.get(2)), new MatOfInt(0),
                mask, hist2, histSize2, ranges2);
        //Core.normalize(hist, hist, 256, 0, Core.NORM_MINMAX);
        hist2.get(0, 0, resFloat2);

        int p, nWhitePixels = 0;
        int numPixels = roi.width * roi.height;
        // White area has a very good distribution of high value pixels
        for (p = 175; p < 256; p++) {
            nWhitePixels += (int) resFloat2[p];
        }

        if (nWhitePixels < ((numPixels * WHITE_PERCENT)/100))
            isItWhite = false;

        Log.v("BOK", "num White pixels: " + nWhitePixels + " out of " + numPixels);

        hist2.release();
        histSize2.release();
        ranges2.release();
        mask.release();

        return isItWhite;
    }

    /*
     * findCube
     * Finds the location of the cube (sampling mission).
     * It takes a frame from Vuforia frame queue. It converts the frame from rgb to gray. It uses
     * the HoughCircles algorithm to detect the spheres in the band of the picture where the
     * spheres are most likely to be found.
     */
    protected BoKAutoCubeLocation findCube()
    {
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
                    Mat srcHSV = new Mat();  // Convert image to HSV
                    Mat srcGray = new Mat(); // Convert image to gray scale
                    Mat srcBlur = new Mat(); // Blurred image

                    Imgproc.cvtColor(src, srcGray, Imgproc.COLOR_BGR2GRAY);
                    Imgproc.cvtColor(src, srcHSV, Imgproc.COLOR_BGR2HSV);

                    // Apply a blur to reduce noise and avoid false circle detection
                    Imgproc.blur(srcGray, srcBlur, new Size(3, 3));

                    Mat circles = new Mat();
                    Point[] centerPointsBlur = new Point[4];
                    Point[] centerPointsSrc = new Point[4];
                    Scalar green = new Scalar(0, 255, 0);
                    Scalar blue = new Scalar(255, 0, 0);

                    Mat srcForHough = srcBlur;
                    Scalar rectColor = green;
                    Scalar circColor = blue;

                    Point[] cPoints = centerPointsBlur;
                    int[] numCircles = {0, 0};

                    // k = 0 for blur; k = 1 for srcGray (if needed)
                    for (int k = 0; k < 2; k++) { //
                        Imgproc.HoughCircles(
                                srcForHough, // input image gray scale
                                circles, // a vector that stores 3 values: xc, yc, and r
                                // for each detected circle
                                Imgproc.HOUGH_GRADIENT, // Detection method
                                1.0, // inverse ratio of resolution
                                (double) srcGray.rows() / 16, // min distance between centers
                                100.0, // Upper threshold for internal Canny edge detector
                                40.0, // Threshold for center detection
                                HOUGH_CIRCLE_MIN_RAD,   // Minimum radius to be detected
                                HOUGH_CIRCLE_MAX_RAD); // Maximum radius to be detected

                        for (int x = 0; x < circles.cols(); x++) {
                            double[] c = circles.get(0, x);
                            Point pt = new Point(Math.round(c[0]), Math.round(c[1]));

                            if ((pt.y > SPHERE_LOC_Y_MIN) &&
                                    (pt.y < SPHERE_LOC_Y_MAX)) {
                                // Now add histogram calculation for double check
                                Rect roi = new Rect((int) (pt.x - 37),
                                        (int) (pt.y - (ROI_HEIGHT / 2)),
                                        ROI_WIDTH, ROI_HEIGHT);

                                if (!isItWhite(srcHSV, roi)) {
                                    // This is just a safety net, we have never seen this!
                                    Log.v("BOK", "Not white!!");
                                    continue;
                                }

                                cPoints[numCircles[k]] = pt;
                                Imgproc.rectangle(src, new Point(roi.x, roi.y),
                                        new Point(roi.x + roi.width,
                                                roi.y + roi.height),
                                        rectColor, 3);
                                int radius = (int) Math.round(c[2]);
                                //Log.v("BOK", "Center: " + pt + ", Radius: " + radius);
                                Imgproc.circle(src,
                                        cPoints[numCircles[k]],
                                        radius,
                                        circColor, 3, 8, 0);
                                numCircles[k]++;
                                if (numCircles[k] > 3) {
                                    break; // something went terribly wrong; bail out!
                                }
                            } // if detected circle is within the ROI band
                        } // for each circle detected

                        // The image is 1280x720
                        if (numCircles[k] != 2) {
                            Log.v("BOK", "Could not find 2 spheres! " + numCircles[k]);
                        } else {
                            if ((cPoints[0].x < CUBE_LOC_RIGHT_X_MAX) &&
                                    (cPoints[1].x < CUBE_LOC_RIGHT_X_MAX)) {
                                // both the circles on the left of the image
                                ret = BoKAutoCubeLocation.BOK_CUBE_RIGHT;
                            } else if ((cPoints[0].x > CUBE_LOC_LEFT_X_MIN) &&
                                    (cPoints[1].x) > CUBE_LOC_LEFT_X_MIN) {
                                // both the circles on the right of the image
                                ret = BoKAutoCubeLocation.BOK_CUBE_LEFT;
                            } else {
                                ret = BoKAutoCubeLocation.BOK_CUBE_CENTER;
                            }
                            writeFile(VUFORIA_ROI_IMG, src, true);
                            break;
                        } // if (numCircles[k] == 2)

                        srcForHough = srcGray;
                        rectColor = blue;
                        circColor = green;
                        cPoints = centerPointsSrc;
                    } // for (int k = 0; k < 2; k++)

                    src.release();
                    srcHSV.release();
                    srcGray.release();
                    circles.release();

                    if ((numCircles[0] != 2) && (numCircles[1] != 2)) {
                        Log.v("BOK", "Failed both blur & srcGray!");
                        if ((numCircles[0] == 1) || (numCircles[1] == 1)) {
                            // check which side is it on?
                            if (numCircles[0] == 1) {
                                if (centerPointsBlur[0].x < 250) {
                                    // Do something here!
                                    ret = BoKAutoCubeLocation.BOK_CUBE_CENTER;
                                }
                                else if (centerPointsBlur[0].x > 1000) {
                                    // Do something here!
                                    ret = BoKAutoCubeLocation.BOK_CUBE_CENTER;
                                }
                            }
                            else {
                                if (centerPointsSrc[0].x < 250) {
                                    // Do something here!
                                    ret = BoKAutoCubeLocation.BOK_CUBE_CENTER;
                                }
                                else if (centerPointsSrc[0].x > 1000) {
                                    // Do something here!
                                    ret = BoKAutoCubeLocation.BOK_CUBE_CENTER;
                                }
                            }
                        }
                    }
                    break;
                } // (if rgb != null)
            } // PIXEL_FORMAT.RGB565
        } // for (int i = 0; i < numImages; i++)
        frame.close();

        return ret;
    }

    /*
     * takePicture
     * Helper method to take picture from Vuforia and save it to a file.
     */
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
            // slow down as we are withing 1/4th of the target
            if (speed > DT_TURN_SPEED_LOW) {
                speed = DT_TURN_SPEED_LOW;
            }
            //speed /= 2;
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
            if (rightSpeed > 0) {
                rightSpeed = Range.clip(rightSpeed,
                        DT_TURN_SPEED_LOW,
                        DT_TURN_SPEED_HIGH);
            }
            else {
                rightSpeed = Range.clip(rightSpeed,
                        -DT_TURN_SPEED_HIGH,
                        -DT_TURN_SPEED_LOW);
            }

            if (!tank) {
                leftSpeed = rightSpeed;
            }
            else if (leftTank) {
                leftSpeed = rightSpeed;
                rightSpeed = 0;
            }
            else {
                leftSpeed = 0;
            }
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

    /*
     * followHeadingPID
     * Parameters:
     * heading: gyro heading (z axis)
     * power: input power (steady state)
     * dist: distance to move in inches
     * detectBump: true if the robot should stop after detecting bump
     * waitForSec: seconds before timing out
     */
    protected void followHeadingPID(double heading,
                                    double power,
                                    double distFast,
                                    double distMax,
                                    boolean detectBump,
                                    double waitForSec)
    {
        double angle, error, diffError, turn, speedL, speedR,
                sumError = 0, lastError = 0, lastTime = 0;
        double Kp = 0.1, Ki = 0, Kd = 0; // Ki = 0.165; Kd = 0.093;
        double targetEncMax = robot.getTargetEncCount(distMax); // convert inches to target enc count
        double targetEncFast = robot.getTargetEncCount(distFast);
        double avgEncCount = 0;
        double powerSlow = power/2;
        //String logString = "dTime,ang,err,sum,last,diff,turn,speedL,speedR\n";
        Log.v("BOK", "followHeadingPID: " + heading + ", distM: " + distMax + ", (" + distFast + ")");
        robot.resetDTEncoders();
        runTime.reset();

        while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec) &&
               (avgEncCount < targetEncMax)) {
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

                //Log.v("BOK", "angle " + String.format("%.2f", angle) +
                //             ",error " + String.format("%.2f",error) +
                //             ",turn " + String.format("%.2f", turn) +
                //             ",L: " + String.format("%.2f", speedL) +
                //             ",R: " + String.format("%.2f", speedR) +
                //             " enc " + String.format("%.2f", robot.getAvgEncCount()));
                /*logString = logString + deltaTime + "," +angle + "," + error + ","
                        + sumError + "," + lastError + "," + diffError + ","
                        + turn + "," + speedL + "," + speedR + "\n";*/
                lastError = error;
                lastTime = currTime;
                if (detectBump) {
                    //Log.v("BOK", "theta x " + angles.firstAngle +
                    //      " theta y " + angles.thirdAngle);
                    if (Math.abs(angles.thirdAngle - heading) > 5) {
                        Log.v("BOK", "theta z in if " + angles.thirdAngle);
                        break;
                    }
                }
            }
            avgEncCount = robot.getAvgEncCount();
            if (avgEncCount >= targetEncFast) {
                Log.v("BOK", "Power Slow set " + avgEncCount);
                power = powerSlow;
            }
        }
        robot.setPowerToDTMotors(0);
        if (runTime.seconds() >= waitForSec) {
            Log.v("BOK", "followHeadingPID timed out!");
        }
        /*
        File file = AppUtil.getInstance().getSettingsFile("BoKGyroDataFwd.csv");
        ReadWriteFile.writeFile(file, logString);
        */
    }

    /*
     * followHeadingPIDWithDistanceBack
     * Parameters:
     * heading: gyro heading (z axis)
     * power: input power (steady state)
     * distTarget: distance to the wall before stopping the robot
     * detectBump: true if the robot should stop after detecting bump
     * waitForSec: seconds before timing out
     */
    protected void followHeadingPIDWithDistanceBack(double heading,
                                                    double power,
                                                    double distTarget,
                                                    boolean detectBump,
                                                    boolean atCrater,
                                                    double waitForSec)
    {
        double angle, error, diffError, turn, speedL, speedR,
               sumError = 0, lastError = 0, lastTime = 0;
        //String logString = "dTime,ang,err,sum,last,diff,turn,speedL,speedR\n";
        double Kp = 0.1, Ki = 0, Kd = 0; // Ki = 0.165; Kd = 0.093;
        runTime.reset();
        double maxDist = atCrater ? distTarget + 50: distTarget + 100;
        double dist = robot.getDistanceCM(robot.distanceBack, maxDist, 0.25);
        Log.v("BOK", "followHeadingPIDWithDistanceBack: heading: " + heading + ", d: " + dist);

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

                //Log.v("BOK", "angle " + String.format("%.2f", angle) +
                //             ",error " + String.format("%.2f",error) +
                //             ",turn " + String.format("%.2f", turn) +
                //             ",L: " + String.format("%.2f", speedL) +
                //             ",R: " + String.format("%.2f", speedR) +
                //             " dist " + String.format("%.2f", dist));
                lastError = error;
                lastTime = currTime;
                dist = robot.getDistanceCM(robot.distanceBack, maxDist, 0.25);
                if (detectBump) {
                    if (angles.firstAngle < DETECT_BUMP_THRESHOLD) {
                        Log.v("BOK", "theta x " + angles.firstAngle);
                        break;
                    }
                }
            }
        }
        robot.setPowerToDTMotors(0);
        if (runTime.seconds() >= waitForSec) {
            Log.v("BOK", "followHeadingPIDWithDistanceBack timed out!");
        }
        Log.v("BOK", "Distance at end of followHeadingPIDWithDistanceBack " + dist);
        /*
        File file = AppUtil.getInstance().getSettingsFile("BoKGyroDataFwdDist.csv");
        ReadWriteFile.writeFile(file, logString);
        */
    }

    /*
     * followHeadingPIDWithDistanceBack
     * Parameters:
     * heading: gyro heading (z axis)
     * power: input power (steady state)
     * distTarget: distance to the wall before stopping the robot
     * detectBump: true if the robot should stop after detecting bump
     * waitForSec: seconds before timing out
     */
    protected void followHeadingPIDWithDistanceBackForDepot(double heading,
                                                    double power,
                                                    double distTarget,
                                                    double optimalDist,
                                                    int timeToWaitMsec,
                                                    boolean detectBump,
                                                    double waitForSec)
    {
        double angle, error, diffError, turn, speedL, speedR,
                sumError = 0, lastError = 0, lastTime = 0;
        //String logString = "dTime,ang,err,sum,last,diff,turn,speedL,speedR\n";
        double Kp = 0.1, Ki = 0, Kd = 0; // Ki = 0.165; Kd = 0.093;
        runTime.reset();
        double dist = robot.getDistanceCM(robot.distanceBack, distTarget+50, 0.25);
        double targetEnc = robot.getTargetEncCount(optimalDist); // convert inches to target enc count
        Log.v("BOK", "followHeadingPIDWithDistanceBackForDepot: heading: " + heading + ", d: " + dist);

        while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec)) {
            if (dist < distTarget) {
                if (robot.getAvgEncCount() < targetEnc) {
                    opMode.sleep(timeToWaitMsec);
                }
                else
                    break;
            }

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

                //Log.v("BOK", "angle " + String.format("%.2f", angle) +
                //             ",error " + String.format("%.2f",error) +
                //             ",turn " + String.format("%.2f", turn) +
                //             ",L: " + String.format("%.2f", speedL) +
                //             ",R: " + String.format("%.2f", speedR) +
                //             " dist " + String.format("%.2f", dist));
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
        if (runTime.seconds() >= waitForSec) {
            Log.v("BOK", "followHeadingPIDWithDistanceBackForDepot timed out!");
        }
        /*
        File file = AppUtil.getInstance().getSettingsFile("BoKGyroDataFwdDist.csv");
        ReadWriteFile.writeFile(file, logString);
        */
    }

    /*
     * moveWithRangeSensor
     * Parameters:
     * power: input power (steady state)
     * targetDistanceCm: distance to the wall before stopping the robot
     * capDistCm: max distance that can possibly be reported by the distance sensor
     * waitForSec: seconds before timing out
     */
    public void moveWithRangeSensor(double power,
                                    int targetDistanceCm,
                                    int capDistCm,
                                    double waitForSec)
    {
        double cmCurrent, diffFromTarget, pCoeff, wheelPower;
        robot.setModeForDTMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        AnalogInput rangeSensor = robot.distanceBack;
        opMode.sleep(40);
        cmCurrent = robot.getDistanceCM(rangeSensor, capDistCm, 0.25);
        Log.v("BOK", "moveWithRangeSensor: " + cmCurrent + ", target: " + targetDistanceCm);
        //if (!Double.isNaN(cmCurrent))
        diffFromTarget = targetDistanceCm - cmCurrent;
        runTime.reset();

        while (opMode.opModeIsActive() &&
                (Math.abs(diffFromTarget) >= RS_DIFF_THRESHOLD_CM)) {
            //Log.v("BOK", "Distance from wall "+cmCurrent);
            if (runTime.seconds() >= waitForSec) {
                Log.v("BOK", "moveWithRS timed out!" + String.format(" %.1f", waitForSec));
                break;
            }

            cmCurrent = robot.getDistanceCM(rangeSensor, capDistCm, 0.25);
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
            if (diffFromTarget < 0) {// we are still far away!
                robot.setPowerToDTMotors(wheelPower, false /* going back*/);
            }
            else {
                // if diffFromTarget > 0 then wheelPower is +ve
                robot.setPowerToDTMotors(wheelPower, true /* going forward */);
            }
            //Log.v("BOK", "Back current RS: " + cmCurrent +
            //        " Difference: " + diffFromTarget +
            //        " Power: (move fwd) " + wheelPower);
        }

        robot.setPowerToDTMotors(0);
        if (runTime.seconds() >= waitForSec) {
            Log.v("BOK", "moveWithRangeSensor timed out!");
        }
        Log.v("BOK", "moveWithRangeSensor: " + cmCurrent);
    }

    /*
     * moveWithRangeSensorBack
     * Parameters:
     * power: input power (steady state)
     * targetDistanceCm: distance to the wall before stopping the robot
     * capDistCm: max distance that can possibly be reported by the distance sensor
     * waitForSec: seconds before timing out
     */
    public void moveWithRangeSensorBack(double power,
                                        int targetDistanceCm,
                                        int capDistCm,
                                        double waitForSec)
    {
        double cmCurrent, diffFromTarget = targetDistanceCm, pCoeff, wheelPower;
        robot.setModeForDTMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        AnalogInput rangeSensor = robot.distanceBack;
        opMode.sleep(40);
        cmCurrent = robot.getDistanceCM(rangeSensor, capDistCm, 0.25);
        Log.v("BOK", "moveWithRangeSensorBack: " + cmCurrent + ", target: " + targetDistanceCm);

        //if (!Double.isNaN(cmCurrent))
        diffFromTarget = targetDistanceCm - cmCurrent;
        runTime.reset();

        while (opMode.opModeIsActive() &&
                (Math.abs(diffFromTarget) >= RS_DIFF_THRESHOLD_CM)) {
            //Log.v("BOK", "Distance from wall "+cmCurrent);
            if (runTime.seconds() >= waitForSec) {
                Log.v("BOK", "moveWithRSBack timed out!" + String.format(" %.1f", waitForSec));
                break;
            }

            cmCurrent = robot.getDistanceCM(rangeSensor, capDistCm, 0.25);
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
            if (diffFromTarget < 0) { // we are still far away!
                robot.setPowerToDTMotors(wheelPower, false /* going back*/);
            }
            else {
                // if diffFromTarget > 0 then wheelPower is +ve
                robot.setPowerToDTMotors(wheelPower, true /* going forward */);
            }
            //Log.v("BOK", "Back current RS: " + cmCurrent +
            //        " Difference: " + diffFromTarget +
            //        " Power: (move fwd) " + wheelPower);
        }

        robot.setPowerToDTMotors(0);
        if (runTime.seconds() >= waitForSec) {
            Log.v("BOK", "moveWithRangeSensorBack timed out!");
        }
        Log.v("BOK", "moveWithRangeSensorBack: " + cmCurrent);
    }


    /*
     * dumpMarker
     * Dumps the marker by moving the marker servo.
     */
    protected void dumpMarker()
    {
        robot.markerServo.setPosition(robot.MARKER_SERVO_FINAL);
        opMode.sleep(50);
        robot.markerServo.setPosition(robot.MARKER_SERVO_FINAL-0.1);
    }

    /*
     * collectMineral
     * Collects the gold mineral by moving the intake.
     */
    /*protected void collectMineral (int initExtension, double targetAngle)
    {
        robot.intakeLeftServo.setPosition(robot.INTAKE_LEFT_SERVO_MID);
        robot.intakeRightServo.setPosition(robot.INTAKE_RIGHT_SERVO_MID);
        robot.intakeMotor.setPower(1);

        robot.intakeSlideMotor.setTargetPosition(initExtension);
        robot.intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.intakeSlideMotor.setPower(0.95);
        while (robot.intakeSlideMotor.isBusy()){}
        robot.intakeSlideMotor.setPower(0);

        robot.intakeLeftServo.setPosition(robot.INTAKE_LEFT_SERVO_DOWN);
        robot.intakeRightServo.setPosition(robot.INTAKE_RIGHT_SERVO_DOWN);

        robot.intakeSlideMotor.setTargetPosition(initExtension + INTAKE_SLIDE_EXTENSION_FINAL);
        robot.intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.intakeSlideMotor.setPower(0.95);
        while (robot.intakeSlideMotor.isBusy()){}
        robot.intakeSlideMotor.setPower(0);

        gyroTurn(DT_TURN_SPEED_LOW, targetAngle, targetAngle+5,
                DT_TURN_THRESHOLD_LOW, false, false, 1);

        gyroTurn(DT_TURN_SPEED_LOW, targetAngle+5, targetAngle-5,
                DT_TURN_THRESHOLD_LOW, false, false, 1);

        robot.intakeLeftServo.setPosition(robot.INTAKE_LEFT_SERVO_UP);
        robot.intakeRightServo.setPosition(robot.INTAKE_RIGHT_SERVO_UP);

        boolean intakeOff = false;
        robot.intakeSlideMotor.setTargetPosition(-50);
        robot.intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.intakeSlideMotor.setPower(0.95);
        while (robot.intakeSlideMotor.isBusy()) {
            if (!intakeOff && (robot.intakeSlideMotor.getCurrentPosition() > 200)) {
                intakeOff = true;
                robot.intakeMotor.setPower(0);
            }
        }
        robot.intakeSlideMotor.setPower(0.5);

        gyroTurn(DT_TURN_SPEED_LOW, targetAngle, targetAngle,
                DT_TURN_THRESHOLD_LOW, false, false, 1);
    }*/

    protected void DumpMineral(BoKAutoCubeLocation loc, boolean atCrater)
    {
        robot.intakeLeftServo.setPosition(0.63);
        robot.intakeRightServo.setPosition(0.27);
        // Raise the dumper slide
        robot.dumperSlideMotor.setTargetPosition(1120);
        robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dumperSlideMotor.setPower(1);
        runTime.reset();
        while (robot.dumperSlideMotor.isBusy() && (runTime.seconds() < 1.5)){
            // Log.v("BOK", "Dumper pos up " + robot.dumperSlideMotor.getCurrentPosition());
            if ((runTime.seconds() > 0.5) && (robot.dumperSlideMotor.getCurrentPosition() < 100)) {
                robot.intakeSlideMotor.setTargetPosition(-185);
            }
        }

        // Open the dumper gate
        robot.dumperGateServo.setPosition(robot.DUMPER_GATE_SERVO_FINAL);

        int distMoveBwd = 2, distMoveFwd = 5;
        switch (loc) {
            case BOK_CUBE_CENTER:
                if (atCrater)
                    distMoveBwd = 5;
                else {
                    distMoveBwd = 2;
                    distMoveFwd = 2;
                }
                break;
            case BOK_CUBE_LEFT:
                distMoveBwd = 2;
                distMoveFwd = 2;
                break;
            case BOK_CUBE_RIGHT:
                distMoveFwd = 3;
                break;
        }
        if (distMoveBwd != 0)
            move(0.5, 0.5, distMoveBwd, false, 2);
        if (distMoveFwd != 0)
            move(0.5, 0.5, distMoveFwd, true, 2);

        // Close the intake gate servo
        robot.intakeGateServo.setPosition(robot.INTAKE_GATE_SERVO_CLOSED);
        robot.dumperGateServo.setPosition(robot.DUMPER_GATE_SERVO_INIT);
        opMode.sleep(250);
        //moveDumpLiftDown();
    }

    public void moveDumpLiftDown() {
        // Move the dumper slide down
        robot.dumperSlideMotor.setTargetPosition(20);
        robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dumperSlideMotor.setPower(0.5);
        boolean liftSlow = true;
        while(robot.dumperSlideMotor.isBusy()){
            // Log.v("BOK", "Dumper pos dn " + robot.dumperSlideMotor.getCurrentPosition());
            if (liftSlow &&
                    (robot.dumperSlideMotor.getCurrentPosition() < (robot.DUMPER_SLIDE_FINAL_POS/3))) {
                liftSlow = false;
                Log.v("BOK", "Dumper pos dn in if " + robot.dumperSlideMotor.getCurrentPosition());
                robot.dumperSlideMotor.setPower(0.8);
            }
        }
        robot.dumperSlideMotor.setPower(0);
    }

    /*
     * runAuto
     * Helper method called from BoKAutoRedCraterOpMode or BoKAutoRedDepotOpMode
     */
    protected void runAuto(boolean atCrater) {
        BoKAutoCubeLocation loc = BoKAutoCubeLocation.BOK_CUBE_UNKNOWN;

        Log.v("BOK", "Angle at runAuto start " +
                robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES).thirdAngle);

        // Step 1: find gold location
        loc = findCube();
        Log.v("BOK", "Location of cube " + loc);

        // Step 2: Start motor for bringing the robot down (hanging lift)
        robot.hangMotor.setPower(HANGLIFT_POWER);

        runTime.reset();
        while (opMode.opModeIsActive() &&
                (robot.hangMotor.getCurrentPosition() < robot.HANG_LIFT_HIGH_POS) &&
                (runTime.seconds() < 5)) {
            // Do nothing while the robot is moving down
            // Log.v("BOK", "hang enc: " + robot.hangMotor.getCurrentPosition());
        }
        robot.hangMotor.setPower(0);
        if (runTime.seconds() >= 5) {
            Log.v("BOK", "hang lift timed out");
        }
        Log.v("BOK", "Hang lift completed in " +
                String.format("%.2f", BoKAuto.runTimeOpMode.seconds()));

        // Step 3: reorient robot and move backward a bit
        gyroTurn(DT_TURN_SPEED_LOW, 0, 0, DT_TURN_THRESHOLD_LOW, false, false, 2/*second*/);
        Log.v("BOK", "Dist to wall after turn " + robot.getDistanceCM(robot.distanceBack, 200, 0.25));
        //opMode.sleep(250);
        moveRamp(MOVE_POWER_LOW, 2/*inches*/, false/*forward*/, 2/*seconds*/);

        // Strafe left about 4 inches
        strafe(0.35/*power*/, 0.35/*rotatiions*/, false, 2/*seconds*/);

        // Step 4: Point the distance sensor so that it is perpendicular to the footprint picture:
        // RED Crater; or the galaxy picture: Red Depot. For Blue Crater, we are pointing at the
        // rover picture & for Blue Depot, we are pointing to the crater picture.

        Log.v("BOK", "Angle after strafe " + robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES).thirdAngle);

        // Step 5: Prepare to intake the sampling cube. Turn 90 degrees left
        gyroTurn(DT_TURN_SPEED_HIGH, 0, 90, DT_TURN_THRESHOLD_LOW, false, false, 4);

        // 5a: Move forward
        moveRamp(0.4, 6, true, 3);
        Log.v("BOK", "Move forward completed in " +
                String.format("%.2f", BoKAuto.runTimeOpMode.seconds()));

        // 5b: Bring the intake arm to mid position
        if (loc != BoKAutoCubeLocation.BOK_CUBE_RIGHT) {
            robot.intakeLeftServo.setPosition(robot.INTAKE_LEFT_SERVO_MID);
            robot.intakeRightServo.setPosition(robot.INTAKE_RIGHT_SERVO_MID);
        }

        // 5c: Bring the intake slides out, this will lower the dumper slide down as
        // space is created
        robot.intakeSlideMotor.setTargetPosition(-175);
        robot.intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.intakeSlideMotor.setPower(0.95);
        while (robot.intakeSlideMotor.isBusy()) {
        }

        Log.v("BOK", "Intake setup completed in " +
                String.format("%.2f", BoKAuto.runTimeOpMode.seconds()));

        // Step 6: complete the sampling, pick up the mineral, score mineral in lander
        if (loc == BoKAutoCubeLocation.BOK_CUBE_LEFT) {
            // 6a: Turn slightly right of the gold
            gyroTurn(DT_TURN_SPEED_HIGH, 90, 105, DT_TURN_THRESHOLD_HIGH, false, false, 2);

            // 6b: Start the intake (a) intake motor, (b) lower the intake arm
            startIntake();

            // 6c: Move forward slightly, turn to sweep past the mineral,
            // move forward to ensure the gold is collected
            move(0.5, 0.5, 2, true, 2);
            gyroTurn(DT_TURN_SPEED_LOW, 105, 120, DT_TURN_THRESHOLD_LOW, false, false, 2);
            move(0.5, 0.5, 5, true, 2);

            // 6d: Raise the intake arm
            raiseIntakeArm();

            // 6e: Retract the intake arm
            retractIntakeArmAndOpenGate();

            // 6f: Turn towards the lander
            double headingForDump = atCrater ? 90 : 100;
            gyroTurn(DT_TURN_SPEED_LOW, 120, headingForDump, DT_TURN_THRESHOLD_LOW, false, false, 2);
            move(0.5, 0.5, 2, true, 2);

            // 6g: Dump the collected cube into the cargo hold of the lander
            DumpMineral(loc, atCrater);

            //moveRamp(0.5, 2, false, 2);
        } else if (loc == BoKAutoCubeLocation.BOK_CUBE_CENTER) {
            Log.v("BOK", "Cube on center");
            // 6a: Strafe to the right about 2 inches
            strafe(0.35/*power*/, 0.35/*rotations*/, true, 2);

            // 6b: Start the intake (a) intake motor, (b) lower the intake arm
            startIntake();

            // 6c: Move forward to pick up the sampling cube
            move(0.5, 0.5, 8, true, 2);
            //??opMode.sleep(250);

            // 6d: Raise the intake arm
            raiseIntakeArm();

            // 6e: Turn slightly to the right to face the lander
            double angleToTurn = atCrater?80:85;
            gyroTurn(DT_TURN_SPEED_LOW, 90, angleToTurn, DT_TURN_THRESHOLD_LOW, false, false, 2);
            //??move(0.5, 0.5, 2, false, 2);

            // 6f: Retract the intake arm
            retractIntakeArmAndOpenGate();

            if (!atCrater)
                move(0.5, 0.5, 2, true, 2);

            // 6g: Dump the collected cube into the cargo hold of the lander
            DumpMineral(loc, atCrater);

            if (!atCrater)
                move(0.5, 0.5, 2, false, 2);

            // 6h: Move the robot back before turning right, so that we don't hit the left sphere
            //moveRamp(0.5, 3, false, 3);
        } else { // loc == BoKAutoCubeLocation.BOK_CUBE_RIGHT
            Log.v("BOK", "Cube on right");
            // 6a: Turn slightly left of the gold
            gyroTurn(DT_TURN_SPEED_HIGH, 90, 57, DT_TURN_THRESHOLD_HIGH, false, false, 3);

            // 6b: extend the horizontal slides to reach the cube
            robot.intakeRightServo.setPosition(robot.INTAKE_RIGHT_SERVO_MID);
            robot.intakeLeftServo.setPosition(robot.INTAKE_LEFT_SERVO_MID);
            robot.intakeSlideMotor.setTargetPosition(-350);
            robot.intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.intakeSlideMotor.setPower(0.95);
            while (robot.intakeSlideMotor.isBusy()) { }

            // 6c: turn to sweep past the mineral, move forward to ensure the gold is collected
            startIntake();
            gyroTurn(DT_TURN_SPEED_LOW, 57, 42, DT_TURN_THRESHOLD_LOW, false, false, 2);
            move(0.5, 0.5, 5, true, 2);

            //??opMode.sleep(250);

            // 6e: Raise the intake arm
            raiseIntakeArm();

            // 6f: Retract the intake arm
            retractIntakeArmAndOpenGate();

            // 6g: Turn to face the lander (turn less on crater side)
            double angleBeforeDump = atCrater ? 75 : 85;
            gyroTurn(DT_TURN_SPEED_HIGH, 48, angleBeforeDump,
                    DT_TURN_THRESHOLD_LOW, false, false, 3);
            move(0.5, 0.5, 3, true, 2);

            // 6h: dumper mineral (and move forward on depot side to avoid the silver minerals)
            DumpMineral(loc, atCrater);

            if (atCrater)
                move(0.5, 0.5, 2, false, 2);
            //else
            //    move(0.5, 0.5, 3, false, 2);
        }

        Log.v("BOK", "Sampling completed in " +
                String.format("%.2f", BoKAuto.runTimeOpMode.seconds()));

        // Step 9: Turn and move to the side wall
        gyroTurn(DT_TURN_SPEED_HIGH, 85, 0, DT_TURN_THRESHOLD_HIGH, false, false, 6);
        moveDumpLiftDown();
        followHeadingPIDWithDistanceBack(0 /*heading*/,
                -0.6,
                DISTANCE_TO_WALL_BEFORE_TURN,
                false/*detectBump*/, atCrater, 3/*seconds*/);
        robot.distanceRotateServo.setPosition(robot.DISTANCE_ROTATE_SERVO_INIT);

        /* heading for wall pointing slightly away from the mineral of our partner */
        double headingForBackWall = (atCrater) ? 42 : -132;
        double headingForMarker = (atCrater) ? 45 : -135;

        // Step 10: Turn towards the back wall
        if (atCrater)
            gyroTurn(DT_TURN_SPEED_HIGH, 0, headingForBackWall ,
                     DT_TURN_THRESHOLD_LOW, false, false, 3/*seconds*/);
        else
            gyroTurn(DT_TURN_SPEED_HIGH, 0, headingForBackWall,
                     DT_TURN_THRESHOLD_LOW, false, false, 5/*seconds*/);

        double distToWall = robot.getDistanceCM(robot.distanceBack, 190, 0.5) / 2.54; // in inches
        Log.v("BOK", "Dist to back wall (inches) " + distToWall);
        Log.v("BOK", "Turning to back wall completed in " +
              String.format("%.2f", BoKAuto.runTimeOpMode.seconds()));

        // Step 11: Move to target distance of 60cm, move faster on depot side
        double speed = atCrater ? -MOVE_POWER_HIGH : -0.7;
        followHeadingPIDWithDistanceBack(headingForBackWall, speed, 60,
                false, atCrater, 7/*sec*/);

        // Before dumpdumping marker, straighten robot
        gyroTurn(DT_TURN_SPEED_LOW, 0, headingForMarker ,
                DT_TURN_THRESHOLD_LOW, false, false, 2/*seconds*/);


        // Step 12: Dump the marker
        dumpMarker();
        Log.v("BOK", "Dumping marker completed in " +
                String.format("%.2f", BoKAuto.runTimeOpMode.seconds()));

        // Step 13: Park robot at crater, while moving turn box to setup for teleop
        RaiseIntakeArmThread raiseIntakeArmThreadThread = new RaiseIntakeArmThread();
        raiseIntakeArmThreadThread.start();
        double distToMoveBack = Math.max(distToWall + 10, 60);
        if (atCrater) {
            // Step 13a: Turn away from our sampling sphere
            gyroTurn(DT_TURN_SPEED_LOW, 40, 46, DT_TURN_THRESHOLD_LOW, false, false, 3/*seconds*/);
            // Step 13b: Move forwards distToWall + 10 inches, detect bump with the crater wall
            followHeadingPID(48, MOVE_POWER_HIGH + 0.2, 0.8*distToMoveBack, distToMoveBack, true, 7 /*seconds*/);
        }
        else {
            // Step 13a: Turn away from our sampling sphere
            gyroTurn(DT_TURN_SPEED_LOW, -130, -136, DT_TURN_THRESHOLD_LOW, false, false, 3/*sec*/);
            // Step 13b: Move forwards distToWall + 10 inches, detect bump with the crater wall
            followHeadingPID(-135, MOVE_POWER_HIGH + 0.2, 0.8*distToMoveBack, distToMoveBack, true, 7 /*seconds*/);
        }
        Log.v("BOK", "Moving after arm in " +
                String.format("%.2f", BoKAuto.runTimeOpMode.seconds()));

        robot.dumperSlideMotor.setTargetPosition(0);
        robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dumperSlideMotor.setPower(0.4);
        while (opMode.opModeIsActive() && robot.dumperSlideMotor.isBusy()){}
        robot.dumperSlideMotor.setPower(0);
   }

   protected void startIntake()
   {
       // Start the intake
       // (a) intake motor
       robot.intakeMotor.setPower(1);
       // (b) lower the intake arm
       robot.intakeLeftServo.setPosition(robot.INTAKE_LEFT_SERVO_DOWN-0.05);
       robot.intakeRightServo.setPosition(robot.INTAKE_RIGHT_SERVO_DOWN+0.05);
   }

   private void raiseIntakeArm()
   {
       // raise the intake arm
       robot.intakeLeftServo.setPosition(robot.INTAKE_LEFT_SERVO_UP);
       robot.intakeRightServo.setPosition(robot.INTAKE_RIGHT_SERVO_UP);
       // (b) stop the intake
       // robot.intakeMotor.setPower(0);
   }

   private void retractIntakeArmAndOpenGate()
   {
       // Retract the intake arm
       robot.intakeSlideMotor.setTargetPosition(-60);
       robot.intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.intakeSlideMotor.setPower(0.5);
       while (robot.intakeSlideMotor.isBusy()) {
       }
       //robot.intakeSlideMotor.setPower(0);
       // Open the intake gate and wait
       robot.intakeGateServo.setPosition(robot.INTAKE_GATE_SERVO_OPEN);
       opMode.sleep(500);
       // Stop the intake motor
       robot.intakeMotor.setPower(0);
   }

   public class RaiseIntakeArmThread extends Thread{
        public void run() {
            raiseIntakeArm();
        }
   }
}

