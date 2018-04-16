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
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Scanner;
import java.util.concurrent.Semaphore;

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
    private static final double P_TURN_COEFF = 0.5;
    //private static final double HEADING_THRESHOLD = 1;
    private static final int RS_DIFF_THRESHOLD_CM = 1;
    private static final double DT_POWER_FOR_RS_MIN = 0.12;
    private static final double CRS_CRYPTO_TIMEOUT = 2.5;
    protected static double distToMoveFlick = 0;
    private static final int VUFORIA_LOCK_BALL_X_OFFSET = 240; // pixels offset from the center
    private static final int VUFORIA_LOCK_BALL_Y_OFFSET = 110; // of the Vuforia image
    private static final int VUFORIA_LOCK_BALL_RECT_WIDTH = 95;
    private static final int VUFORIA_LOCK_BALL_RECT_HEIGHT = 90;
    protected static final double FLIPPER_ANGLE_INIT_POS = 0.85; // FLIPPER_INIT_POS = 0.95
    protected static final boolean DEBUG_OPEN_CV = false;
    private static final String VUFORIA_LOCK_IMG = "vuImage.png";
    private static final String ROI_IMG = "roiImage.png";

    // Second & third glyphs during autonomous
    private static final double TIMEOUT_8S = 8.0; // Total time available
    private static final double TIMEOUT_ROLLERS_BURST = 0.5;
    private static final double ROLLER_POWER = 0.95;
    protected int numGlyphs = 0;
    protected int encCountsTillLine = 0;
    protected double turnAngle = 0;
    private int lastPosL, lastPosR;
    private int THRESHOLD_ROLLER_ENC_COUNT = 5;
    //int encCountSecGlyph = 0;
    //private static final String HSV_IMG = "hsvImage.png";

    private AppUtil appUtil = AppUtil.getInstance();
    private VuforiaLocalizer vuforiaFTC;
    private VuforiaTrackable relicTemplate;
    private VuforiaTrackables relicTrackables;

    protected ElapsedTime runTime  = new ElapsedTime();

    protected BoKAllianceColor allianceColor;
    protected boolean far = false;
    protected BoKAutoOpMode opMode;  // save a copy of the current opMode and robot
    protected BoKHardwareBot robot;
    protected boolean secGlyph = true;

    // NOTE: Even if we are unsuccessful with Vuforia, we will still go to the left column
    protected RelicRecoveryVuMark cryptoColumn = RelicRecoveryVuMark.LEFT;
    protected boolean foundRedOnLeft = false;
    protected double additional_turn_speed = 0;
    protected double fraction_turn_slow = 0.25;

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
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0,
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
        parameters.vuforiaLicenseKey = "ASodJvL/////AAAAGa9Isk5Oa0brtCRz7Z0fQngHmf2Selfx3RDk3MzjmK9DFWkQRsg1dH8Q8VvU/9L9nr9krXa+2nY5zoK6moC4UpRIg+gRsnG5M504q50Dd+z8DDATBaamc8t5qa8OeLjFKQ/+blHLbe8tjXSdVdl/xxGdowpeuQ18dnlf129q5NjM7Z9s/M8l693yEl28b+/LLJ4SiFLBTXwkEpVblemfJKZVHO5I8JmGmQ4jcwCWFIMCFxPRbCeDVqdCeqQzFa3BcCiuuGUgZDBCaidiW0/pzEFzdXcVCQfJPMgdZUWkPAk0QXVC8zYXaweeLuAONyTDkanRiyzqZbDVpJhVHaLBsUaC3OmZ/Xo+ThguyX3tNs3G";
        vuforiaFTC = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforiaFTC.setFrameQueueCapacity(1); // change the frame queue capacity to 1

        /*
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one
         * template, but differ in their instance id information.
         */
        relicTrackables = vuforiaFTC.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        Log.v("BOK", "Done initializing software");
        this.opMode = opMode;
        this.robot = robot;

        setupRobot();
        return BoKAutoStatus.BOK_AUTO_SUCCESS;
    }

    String format(OpenGLMatrix transformationMatrix)
    {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    @Override
    public abstract void runSoftware();

    public void setupRobot()
    {

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        String robotPosition = "", vuMarkInfo = "";

        // Show X, Z and Rotation on Y axis on the main activity
        Activity act = AppUtil.getInstance().getActivity();
        // Need to access xzPosView & layout in a separate thread, must declare these final
        final RelativeLayout layout = (RelativeLayout)act.findViewById(R.id.RelativeLayout);
        final TextView xzPosView = new TextView(act);
        // setup the text view and set its relative position in the relative layout
        xzPosView.setText("Setup");
        xzPosView.setRotation(90); // Rotate to landscape mode
        xzPosView.setTextColor(0xFFFFFF00); // Yellow (A: 0xFF, Red: 0xFF, Green: 0xFF, Blue: 0x00)
        // Make it bold & big (size 64)
        xzPosView.setTypeface(xzPosView.getTypeface(), Typeface.BOLD);
        xzPosView.setTextSize(
                act.getResources().getDimension(R.dimen.activity_horizontal_margin)*2);
        RelativeLayout.LayoutParams params =
                new RelativeLayout.LayoutParams(RelativeLayout.LayoutParams.MATCH_PARENT,
                        RelativeLayout.LayoutParams.WRAP_CONTENT);
        // above textGamepad1 in the relative layout
        params.addRule(RelativeLayout.ABOVE, R.id.textGamepad1);
        xzPosView.setLayoutParams(params);

        // we can only add the text view in the main UI thread
        act.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                xzPosView.setVisibility(View.VISIBLE);
                layout.addView(xzPosView);
            }
        });

        // Must pass String in the constructor of the Runnable object
        class UpdateRobotPosition implements Runnable
        {
            String pos;
            UpdateRobotPosition(String s)
            {
                pos = s;
            }
            public void run()
            {
                xzPosView.setText(pos);
            }
        }
        // activate
        relicTrackables.activate();
        CameraDevice.getInstance().setFlashTorchMode(true);

        while (true) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                //opMode.telemetry.addData("VuMark", "%s visible", vuMark);
                //Log.v("BOK", "VuMark " + vuMark + " visible");


                OpenGLMatrix rawPose =
                        ((VuforiaTrackableDefaultListener)
                                relicTemplate.getListener()).getRawUpdatedPose();
                if (rawPose != null) {
                    //opMode.telemetry.addData("Raw Pose", format(rawPose));
                    //Log.v("BOK", "Raw pose " + format(rawPose));

                    Matrix34F raw = new Matrix34F();
                    float[] rawData = Arrays.copyOfRange(rawPose.transposed().getData(), 0, 12);
                    raw.setData(rawData);

                    Vec2F pointCenter =
                            Tool.projectPoint(vuforiaFTC.getCameraCalibration(),
                                    raw, new Vec3F(0, 0, 0));

                    VectorF trans = rawPose.getTranslation();
                    Orientation rot = Orientation.getOrientation(rawPose,
                            AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target
                    // relative to the robot
                    double tX = trans.get(0);
                    //double tY = trans.get(1);
                    double tZ = trans.get(2);

                    //double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double sCM = robot.getDistanceCM(robot.mb1240SideR);
                    //double rZ = rot.thirdAngle;
                    robotPosition = String.format("Z: %.1f,  ROT Y: %.1f, S: %.1f",
                                                  tZ/10, rY, sCM);

                    // we can only update the text in the main UI thread
                    act.runOnUiThread(new UpdateRobotPosition(String.format("Z: %.1f\n" +
                            "S: %.1f", tZ/10, sCM)));

                    opMode.telemetry.addData("Trans", robotPosition);
                    vuMarkInfo = vuMark + " at :(" +
                                 (int)pointCenter.getData()[0] + ", " +
                                 (int)pointCenter.getData()[1] + ")";
                    opMode.telemetry.addData("VuMark", vuMarkInfo);
                    //opMode.telemetry.addData("Raw", "X: %.1f, Z: %.1f", tX, tZ);
                }
                opMode.telemetry.update();
            }

            flipFlipper(FLIP_FLIPPER_INIT); // initialize the flipper to stay parallel

            if (opMode.gamepad1.y) {
                relicTrackables.deactivate();
                CameraDevice.getInstance().setFlashTorchMode(false);
                robot.setPowerToDTMotors(0,0,0,0);
                if (!robotPosition.isEmpty()) {
                    Log.v("BOK", robotPosition);
                }
                break;
            }
            robot.waitForTick(BoKHardwareBot.WAIT_PERIOD);
        }

        // we can only remove the text view in the main UI thread
        act.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                xzPosView.setVisibility(View.INVISIBLE);
                layout.removeView(xzPosView);
                layout.refreshDrawableState();
            }
        });

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

        Mat img = new Mat(rgb.getHeight(), rgb.getWidth(),
                CvType.CV_8UC3);
        Utils.bitmapToMat(bm, img);

        // OpenCV only deals with BGR
        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2BGR);
        writeFile(fileName, img, always);

        // First convert from BGR to HSV; separate the color components from intensity.
        // Increase robustness to lighting changes.
        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV);
        return img;
    }

    private boolean calcNumRedPixels(Mat img, Rect roi)
    {
        Mat hist = new Mat();
        MatOfInt histSize = new MatOfInt(180);
        MatOfFloat ranges = new MatOfFloat(0f, 180f);
        Mat mask = new Mat(img.rows(), img.cols(),
                CvType.CV_8UC1, new Scalar(0));
        float[] resFloat = new float[180];
        boolean foundRed = false;

        Imgproc.rectangle(img, new Point(roi.x, roi.y),
                new Point(roi.x + VUFORIA_LOCK_BALL_RECT_WIDTH,
                          roi.y + VUFORIA_LOCK_BALL_RECT_HEIGHT),
                new Scalar(0, 255, 0), 10);
        writeFile(ROI_IMG, img, true);
        Mat subMask = mask.submat(roi);
        subMask.setTo(new Scalar(255));

        Imgproc.calcHist(Arrays.asList(img), new MatOfInt(0),
                mask, hist, histSize, ranges);
        //writeFile(HSV_IMG, img, DEBUG_OPEN_CV);
        //Core.normalize(hist, hist, 256, 0, Core.NORM_MINMAX);
        hist.get(0, 0, resFloat);

        int p, nRedPixels = 0;
        int numPixels = roi.width * roi.height;
        // Red is 0 (in HSV),
        // but we need to check between 160-179 and 0-9
        for (p = 0; p < 10; p++) {
            nRedPixels += (int) resFloat[p];
        }
        for (p = 160; p < 180; p++) {
            nRedPixels += (int) resFloat[p];
        }

        if (nRedPixels >= ((numPixels * 45)/100))
            foundRed = true;

        Log.v("BOK", "num Red pixels: " + nRedPixels + " out of " + numPixels);
        if (foundRed == true) {
            Log.v("BOK", "Left is RED");
        } else {
            Log.v("BOK", "Left is BLUE");
        }

        hist.release();
        histSize.release();
        ranges.release();
        mask.release();

        return foundRed;
    }

    public boolean getCryptoColumn(double waitForSec)
    {
        boolean vuMarkVisible = false;
        boolean vuforiaSuccess = false;
        // closest one in case we fail to detect Vuforia image
        RelicRecoveryVuMark vuMark = (allianceColor == BoKAllianceColor.BOK_ALLIANCE_BLUE) ?
                RelicRecoveryVuMark.LEFT : RelicRecoveryVuMark.RIGHT;
        // activate
        relicTrackables.activate();
        CameraDevice.getInstance().setFlashTorchMode(true);
        runTime.reset();

        while (opMode.opModeIsActive() && !vuMarkVisible) {
            if (runTime.seconds() >= waitForSec) {
                Log.v("BOK", "getCryptoColumn timed out!" + String.format(" %.1f", waitForSec));
                break;
            }
            /**
             * See if any of the instances of relicTemplate are currently visible.
             * RelicRecoveryVuMark is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by RelicRecoveryVuMark from VuforiaTrackable.
             */
            RelicRecoveryVuMark vuMarkLocal = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMarkLocal != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                //opMode.telemetry.addData("VuMark", "%s visible", vuMark);
                vuMarkVisible = true;
                vuMark = vuMarkLocal;
                Log.v("BOK", "VuMark " + vuMark + " visible");

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose
                 * information, but we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose =
                    ((VuforiaTrackableDefaultListener)
                        relicTemplate.getListener()).getRawUpdatedPose();
                // opMode.telemetry.addData("Pose", format(pose));

                if (pose != null) {
                    Log.v("BOK", "Img processing: " + String.format("%.2f", runTime.seconds()));

                    VuforiaLocalizer.CloseableFrame frame;
                    // takes the frame at the head of the queue
                    try {
                        frame = vuforiaFTC.getFrameQueue().take();
                    } catch (InterruptedException e) {
                        Log.v("BOK", "Exception!!");
                        break;
                    }

                    // Convert the data in rawPose back to the format that Vuforia expects -
                    // 3x4 row major matrix. where as the OpenGLMatrix is 4x4
                    // column major matrix.
                    Matrix34F raw = new Matrix34F();
                    float[] rawData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                    raw.setData(rawData);

                    Vec2F pointCenter =
                            Tool.projectPoint(vuforiaFTC.getCameraCalibration(),
                                    raw, new Vec3F(0, 0, 0));

                    Log.v("BOK", "Center: " + (int)pointCenter.getData()[0] +
                            ", " + (int)pointCenter.getData()[1]);

                    long numImages = frame.getNumImages();
                    for (int i = 0; i < numImages; i++) {
                        if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                            Image rgb = frame.getImage(i);
                            // rgb is now the Image object that we’ve used in the video
                            if (rgb != null) {
                                Mat img = setupOpenCVImg(rgb, VUFORIA_LOCK_IMG, true);

                                Rect roi = new Rect((int)pointCenter.getData()[0] +
                                                         VUFORIA_LOCK_BALL_X_OFFSET,
                                                    (int)pointCenter.getData()[1] +
                                                         VUFORIA_LOCK_BALL_Y_OFFSET,
                                                    VUFORIA_LOCK_BALL_RECT_WIDTH,
                                                    VUFORIA_LOCK_BALL_RECT_HEIGHT);

                                // Next check if Red ball is on the left
                                foundRedOnLeft = calcNumRedPixels(img, roi);
                                vuforiaSuccess = true;
                                img.release();
                            } // if (rgb != null)
                            break;
                        } // if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565)
                    } // for (int i = 0; i < numImages; i++)
                    frame.close();
                } // if (pose != null)
            }
        } // while (!vuMarkVisible)
        // deactivate
        //relicTrackables.deactivate();
        cryptoColumn = vuMark;
        //CameraDevice.getInstance().setFlashTorchMode(false);
        Log.v("BOK", "Detecting Crypto Column: " + runTime.seconds());
        return vuforiaSuccess;
    }

    public void detectVuforiaImgAndDrop(int waitForServoMs)
    {
        if (getCryptoColumn(VUFORIA_TIMEOUT)) {
            // Straighten the jewel flicker
            robot.jewelFlicker.setPosition(robot.JF_FINAL);
            opMode.sleep(waitForServoMs);
            // Lower the jewel arm
            robot.jewelArm.setPosition(robot.JA_FINAL);
            opMode.sleep(waitForServoMs);
        }
        else { // failed to detect Vuforia image
            // Position the jewel flicker to face the cryptobox
            robot.jewelArm.setPosition(robot.JA_INIT);
            robot.jewelFlicker.setPosition(robot.JF_FINAL);
        }
    }

    public void moveAndFlick()
    {
        if (distToMoveFlick != 0)
            move(DT_POWER_FOR_FLICK, DT_POWER_FOR_FLICK, Math.abs(distToMoveFlick),
                (distToMoveFlick > 0 ? true : false), DT_TIMEOUT_2S);

        if (allianceColor == BoKAllianceColor.BOK_ALLIANCE_RED) {
            if (foundRedOnLeft) // we are red
                robot.jewelFlicker.setPosition(robot.JF_RIGHT);
            else
                robot.jewelFlicker.setPosition(robot.JF_LEFT);
        }
        else {
            if (foundRedOnLeft) // we are blue
                robot.jewelFlicker.setPosition(robot.JF_LEFT);
            else
                robot.jewelFlicker.setPosition(robot.JF_RIGHT);
        }
        opMode.sleep(WAIT_FOR_JEWEL_FLICKER_MS);

        robot.jewelArm.setPosition(robot.JA_INIT);
        robot.jewelFlicker.setPosition(robot.JF_FINAL);
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
                    if (forward) {
                        robot.setPowerToDTMotors(power, power, -power, -power);
                    } else {
                        robot.setPowerToDTMotors(-power, -power, power, power);
                    }
                }
                else if (lfEncCount < rampdnEncCount) {
                    if (!steady) {
                        if (forward) {
                            robot.setPowerToDTMotors(maxPower, maxPower, -maxPower, -maxPower);
                        } else {
                            robot.setPowerToDTMotors(-maxPower, -maxPower, maxPower, maxPower);
                        }
                        steady = true;
                    }
                }
                else {
                    double power = DT_RAMP_SPEED_INIT -
                            ratePower * (lfEncCount - targetEncCount);
                    //Log.v("BOK", lfEncCount + " power dn: " + String.format("%.2f", power));
                    if (forward) {
                        robot.setPowerToDTMotors(power, power, -power, -power);
                    } else {
                        robot.setPowerToDTMotors(-power, -power, power, power);
                    }
                }
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
            int targetEncCount = robot.startStrafe(DT_RAMP_SPEED_INIT, rotations, right);
            Log.v("BOK", "strafeRamp: " + targetEncCount);

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
                    Log.v("BOK", "strafeRamp timed out!" + String.format(" %.1f", waitForSec));
                    break;
                }

                int lfEncCount = Math.abs(robot.getLFEncCount());
                if (lfEncCount < rampupEncCount) {
                    double power = DT_RAMP_SPEED_INIT + ratePower*lfEncCount;
                    //Log.v("BOK", lfEncCount + " power Up: " + String.format("%.2f", power));
                    if (right) {
                        robot.setPowerToDTMotors(power, -power, power, -power);
                    }
                    else {
                        robot.setPowerToDTMotors(-power, power, -power, power);
                    }
                }
                else if (lfEncCount < rampdnEncCount) {
                    if (right) {
                        robot.setPowerToDTMotors(maxPower, -maxPower, maxPower, -maxPower);
                    }
                    else {
                        robot.setPowerToDTMotors(-maxPower, maxPower, -maxPower, maxPower);
                    }
                }
                else {
                    double power = DT_RAMP_SPEED_INIT - ratePower*(lfEncCount - targetEncCount);
                    //Log.v("BOK", lfEncCount + " power dn: " + String.format("%.2f", power));
                    if (right) {
                        robot.setPowerToDTMotors(power, -power, power, -power);
                    }
                    else {
                        robot.setPowerToDTMotors(-power, power, -power, power);
                    }
                }
            }
            robot.stopMove();
        }
    }

    private void takePicture(String sFileName) {
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

    public void moveTowardsCrypto(double power,
                                  boolean forward,
                                  int waitForServoMs,
                                  double waitForSeconds)
    {
        double cmCurrent = 0;
        robot.resetDTEncoders(); // reset encoders
        runTime.reset();

        //cmCurrent = robot.rangeSensorJA.rawOptical();
        //Log.v("BOK", "Distance RS (start raw optical): " + cmCurrent);
        //Log.v("BOK", "Distance RS (start optical): " + robot.rangeSensorJA.cmOptical());
        cmCurrent = robot.rangeSensorJA.cmUltrasonic();
        Log.v("BOK", String.format("Distance RS (start): %.2f", cmCurrent));

        double targetEncCount = robot.getTargetEncCount(2); // fail safe
        boolean targetEncCountReached = false;

        if(forward){
            robot.setPowerToDTMotors(power, power, -power, -power);
        }
        else{
            robot.setPowerToDTMotors(-power, -power, power, power);
        }

        while (opMode.opModeIsActive() &&
                (runTime.seconds() < waitForSeconds) &&
                (cmCurrent > 20) &&
                (!targetEncCountReached))
        {
            cmCurrent = robot.rangeSensorJA.cmUltrasonic();
            double currentEncCount = (robot.getLFEncCount() + robot.getRFEncCount()) / 2;
            if (currentEncCount >= targetEncCount) {
                targetEncCountReached = true;
            }
        }
        robot.stopMove();
        if (!targetEncCountReached) {
            double cmCurrentNow = robot.rangeSensorJA.cmUltrasonic();
            if (cmCurrentNow <= 20) {
                cmCurrent = cmCurrentNow;
            }
        }

        // Raise the jewel arm
        robot.jewelArm.setPosition(robot.JA_INIT);
        opMode.sleep(waitForServoMs);

        // take a picture
        //takePicture("c_crypto.png");

        if (allianceColor == BoKAllianceColor.BOK_ALLIANCE_BLUE) {
            // Need to move past the crypto column for blue
            double distanceToMove = 5; // in inches
            if (!targetEncCountReached) {
                Log.v("BOK", "Blue: cmCurrent: " + cmCurrent);
                // we got a valid ultrasonic value
                distanceToMove = (cmCurrent/2.54)-4;// + 2.4;
            }
            Log.v("BOK", "Blue targetReached: " + targetEncCountReached +
                  ", dist: " + String.format("%.2f", distanceToMove));
            move(DT_POWER_FOR_CRYPTO,
                 DT_POWER_FOR_CRYPTO,
                 Math.abs(distanceToMove),
                 (distanceToMove < 0) ? true : false,
                 DT_TIMEOUT_5S);

            // take a picture
            //takePicture("cb_crypto.png");
        }
        else {
            double distanceToMove = 5; // in inches
            if (!targetEncCountReached) {
                Log.v("BOK", "Red: cmCurrent: " + cmCurrent);
                // we got a valid ultrasonic value
                if (cryptoColumn == RelicRecoveryVuMark.RIGHT) {
                    distanceToMove = (cmCurrent / 2.54) - 2;
                }
                else if (cryptoColumn == RelicRecoveryVuMark.CENTER) {
                    distanceToMove = (cmCurrent / 2.54) + 5.75;
                }
                else {
                     distanceToMove = (cmCurrent / 2.54) + 5.75;
                }
            }
            Log.v("BOK", "Red targetReached: " + targetEncCountReached +
                    ", dist: " + String.format("%.2f", distanceToMove));

            move(DT_POWER_FOR_CRYPTO,
                 DT_POWER_FOR_CRYPTO,
                 Math.abs(distanceToMove),
                    (distanceToMove > 0) ? true: false,
                 DT_TIMEOUT_4S);

            // take a picture
            //takePicture("cr_crypto.png");
        }
    }
    
    public boolean moveWithRangeSensor(double power,
                                    int targetDistanceCm,
                                    boolean sensorFront,
                                    double waitForSec)
    {
        boolean result = true;

        double cmCurrent, diffFromTarget = targetDistanceCm, pCoeff, wheelPower;
        robot.setModeForDTMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double encTarget;
        if(allianceColor == BoKAllianceColor.BOK_ALLIANCE_RED)
            encTarget = robot.getTargetEncCount(42);
        else
            encTarget = robot.getTargetEncCount(52);

        AnalogInput rangeSensor; // First choose which range sensor to use
        if(sensorFront) {
            rangeSensor = robot.mb1240Front;
        }
        else {
            rangeSensor = robot.mb1240Back;
        }

        cmCurrent = robot.getDistanceCM(rangeSensor);
        if (!Double.isNaN(cmCurrent))
            diffFromTarget = targetDistanceCm - cmCurrent;
        runTime.reset();

        while (opMode.opModeIsActive() &&
                (Math.abs(diffFromTarget) >= RS_DIFF_THRESHOLD_CM)) {
            if (runTime.seconds() >= waitForSec) {
                Log.v("BOK", "moveWithRS timed out!" + String.format(" %.1f", waitForSec));
                result = false;
                break;
            }

            if (robot.getLFEncCount() >= encTarget) {
                Log.v("BOK", "moveWithRS moved too far!" + robot.getLFEncCount());
                result = false;
                break;
            }

            cmCurrent = robot.getDistanceCM(rangeSensor);
            if (Double.isNaN(cmCurrent) || (cmCurrent >= 255)) // Invalid sensor reading
                continue;

            diffFromTarget = targetDistanceCm - cmCurrent;
            pCoeff = diffFromTarget/15;
            wheelPower = Range.clip(power*pCoeff, -power, power);
            if (wheelPower > 0 && wheelPower < DT_POWER_FOR_RS_MIN)
                wheelPower = DT_POWER_FOR_RS_MIN; // min power to move
            if (wheelPower < 0 && wheelPower > -DT_POWER_FOR_RS_MIN)
                wheelPower = -DT_POWER_FOR_RS_MIN;

            if (sensorFront) {
                // if diffFromTarget > 0 then wheelPower is +ve, but we need to move
                // backward (BLUE FAR).
                //Log.v("BOK", "Front current RS: " + cmCurrent +
                //        " Difference: " + diffFromTarget +
                //        " Power: " + wheelPower);
                robot.setPowerToDTMotors(-wheelPower, -wheelPower, wheelPower, wheelPower);
            }
            else { // back range sensor
                // if diffFromTarget > 0 then wheelPower is +ve
                //Log.v("BOK", "Back current RS: " + cmCurrent +
                //        " Difference: " + diffFromTarget +
                //        " Power: (move fwd) " + wheelPower);
                robot.setPowerToDTMotors(wheelPower, wheelPower, -wheelPower, -wheelPower);
            }
        }



        if (sensorFront)
            Log.v("BOK", "Front current RS: " + cmCurrent);
        else
            Log.v("BOK", "Back current RS: " + cmCurrent);
        robot.setPowerToDTMotors(0, 0, 0, 0);

        return result;
    }

    public boolean strafeWithRangeSensor(double power,
                                         int targetDistanceCm,
                                         boolean sensorRight,
                                         double waitForSec)
    {
        boolean result = true;

        double cmCurrent, diffFromTarget = targetDistanceCm, pCoeff, wheelPower;
        robot.resetDTEncoders();
        //robot.setModeForDTMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        AnalogInput rangeSensor; //, backSensor; // First choose which range sensor to use
        rangeSensor = (sensorRight) ? robot.mb1240SideR : robot.mb1240SideL;
        // backSensor = robot.mb1240Back;

        cmCurrent = robot.getDistanceCM(rangeSensor);
        if (!Double.isNaN(cmCurrent))
            diffFromTarget = targetDistanceCm - cmCurrent;
        runTime.reset();

        while (opMode.opModeIsActive() &&
                (Math.abs(diffFromTarget) >= RS_DIFF_THRESHOLD_CM)) {
            if (runTime.seconds() >= waitForSec) {
                Log.v("BOK", "moveWithRS timed out!" + String.format(" %.1f", waitForSec));
                result = false;
                break;
            }

            cmCurrent = robot.getDistanceCM(rangeSensor);
            if (Double.isNaN(cmCurrent) || (cmCurrent >= 255)) // Invalid sensor reading
                continue;

            diffFromTarget = targetDistanceCm - cmCurrent;
            pCoeff = diffFromTarget/15;
            wheelPower = Range.clip(power*pCoeff, -power, power);
            if (wheelPower > 0 && wheelPower < DT_POWER_FOR_RS_MIN)
                wheelPower = DT_POWER_FOR_RS_MIN; // min power to move
            if (wheelPower < 0 && wheelPower > -DT_POWER_FOR_RS_MIN)
                wheelPower = -DT_POWER_FOR_RS_MIN;

            //if (strafeLeft) {
                // if diffFromTarget > 0 then wheelPower is +ve, but we need to move
                // backward (BLUE FAR).
            //    Log.v("BOK", "Left current RS: " + cmCurrent +
            //            " Difference: " + diffFromTarget +
            //           " Power: " + wheelPower);
            if(allianceColor == BoKAllianceColor.BOK_ALLIANCE_RED)
                wheelPower *= -1;
            robot.setPowerToDTMotors(-wheelPower, wheelPower, -wheelPower, wheelPower);
            //}
            //else { // back range sensor
                // if diffFromTarget > 0 then wheelPower is +ve
                //Log.v("BOK", "Right current RS: " + cmCurrent +
                //        " Difference: " + diffFromTarget +
                //        " Power: (move fwd) " + wheelPower +
                //        " Back RS " + robot.getDistanceCM(backSensor));
            //    robot.setPowerToDTMotors(wheelPower, -wheelPower, wheelPower, -wheelPower);
            //}
        }

        Log.v("BOK", "Side current RS: " + cmCurrent);
        robot.setPowerToDTMotors(0, 0, 0, 0);

        return result;
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
        if (Math.abs(error) <= (Math.abs(angle-init_angle)*fraction_turn_slow)) {
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
                rightSpeed = Range.clip(rightSpeed, DT_TURN_SPEED_LOW,
                                        DT_TURN_SPEED_HIGH+additional_turn_speed);
            else
                rightSpeed = Range.clip(rightSpeed,
                                        -DT_TURN_SPEED_HIGH-additional_turn_speed,
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
        robot.setPowerToDTMotors(-leftSpeed, -leftSpeed, -rightSpeed, -rightSpeed);

        //Log.v("BOK", "Err: " + String.format("%.2f", error) + ", Steer: " +
        //      String.format("%.2f", steer));
        //Log.v("BOK", "Left Speed: " + String.format("%5.2f", leftSpeed) + ", Right Speed: " +
        //      String.format("%5.2f", rightSpeed));

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

    protected void moveToCrypto(double initGyroAngle, int waitForServoMs, boolean secGlyph)
    {
        if (opMode.opModeIsActive()) {
            // take a picture
            //takePicture("b_crypto.png");

            // Lower the jewel arm & the range sensor
            robot.jewelArm.setPosition(robot.JA_MID);
            opMode.sleep(waitForServoMs*2); // let the flicker settle down
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                                                     AxesOrder.XYZ,
                                                     AngleUnit.DEGREES);
            Log.v("BOK", String.format("IMU angle %.1f", angles.thirdAngle));

            // Move forward towards cryptobox using range sensor
            moveTowardsCrypto(DT_POWER_FOR_CRYPTO,
                    (allianceColor == BoKAllianceColor.BOK_ALLIANCE_RED) ? true : false,
                    waitForServoMs,
                    CRS_CRYPTO_TIMEOUT);

            // Now prepare to unload the glyph
            // move the jewel flicker to init and slowly move the wrist down
            robot.jewelFlicker.setPosition(robot.JF_INIT);
            opMode.sleep(waitForServoMs);

            double distBack = 8.0;
            if(allianceColor == BoKAllianceColor.BOK_ALLIANCE_BLUE){
                //move(DT_POWER_HIGH, DT_POWER_HIGH, 2, true, DT_TIMEOUT_2S);
                distBack -= 4;
            }
            else {
                distBack += 0.5;
            }

            turnAngle = gyroTurn(DT_TURN_SPEED_HIGH,
                                 initGyroAngle,
                                 TURN_LEFT_DEGREES, // final angle RED and BLUE Near
                                 DT_TURN_THRESHOLD_LOW, // threshold,
                                 false, // NOT a tank turn
                                 false,
                                 DT_TURN_TIMEOUT);

            if(allianceColor == BoKAllianceColor.BOK_ALLIANCE_BLUE)
                moveRamp(DT_POWER_HIGH, 5.5, false, DT_TIMEOUT_4S); // we are too far away!

            flipFlipper(FLIP_FLIPPER_DUMP); // dump the glyph

            moveRamp(DT_POWER_HIGH, distBack, false, DT_TIMEOUT_4S);

            // just park in the safe zone
            if (!secGlyph)
                moveRamp(DT_POWER_HIGH, distBack - 4.5, true, DT_TIMEOUT_4S);
        } // opMode.isActive
    }

    void moveFlipperGates (boolean up){
        if (up){
            robot.ridingGateLeft.setPosition(robot.RGL_LOCK);
            robot.ridingGateRight.setPosition(robot.RGR_LOCK);
        }
        else{
            robot.ridingGateLeft.setPosition(robot.RGL_UNLOCK);
            robot.ridingGateRight.setPosition(robot.RGR_UNLOCK);
        }
    }

    void moveLiftEnc(int encCount)
    {
        robot.flipperLift.setTargetPosition(encCount);
        robot.flipperLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.flipperLift.setPower(0.4);
        runTime.reset();
        while (robot. flipperLift.isBusy() && opMode.opModeIsActive()) {
            if (runTime.seconds() >= 2) {
                Log.v("BOK", "RAISE LIFT timed out");
            }
        }
        robot.flipperLift.setPower(0);
    }

    void flipFlipper (int state)
    {
        int RAISE_LIFT_ENC_COUNTS = 700;
        switch (state){
            case (FLIP_FLIPPER_INIT): // setupRobot
                robot.flipper.setPosition(FLIPPER_ANGLE_INIT_POS);
                break;
            case (FLIP_FLIPPER_DUMP): // dump
                moveFlipperGates(false);
                double pos = robot.flipper.getPosition();
                for (;pos > robot.FLIPPER_UP_POS; pos -= 0.05){
                    robot.flipper.setPosition(pos);
                    opMode.sleep(robot.OPMODE_SLEEP_INTERVAL_MS_SHORT*2);
                }
                break;
            case (FLIP_FLIPPER_LOWER):
                robot.flipper.setPosition(robot.FLIPPER_DOWN_POS);
                break;
            case (RAISE_LIFT):
                robot.flipper.setPosition(robot.FLIPPER_ANGLE_POS);
                moveLiftEnc(RAISE_LIFT_ENC_COUNTS);
                //flipFlipper(FLIP_FLIPPER_DUMP);
                break;
            case (LOWER_LIFT_AND_RESET_FLIPPER):
                moveLiftEnc(0);
                robot.flipper.setPosition(robot.FLIPPER_DOWN_POS);
                break;
        }
    }

    void moveRollers(int direction, double power, double waitForSeconds, ElapsedTime timeKeeper)
    {
        double startTime = timeKeeper.seconds();
        switch (direction) {
            case (INTAKE_ROLLERS):
                robot.leftRoller.setPower(power);
                robot.rightRoller.setPower(power);
                while (opMode.opModeIsActive() &&
                        ((timeKeeper.seconds() - startTime) < waitForSeconds)) {
                }
                break;
            case (REVERSE_ROLLERS):
                robot.leftRoller.setPower(-power);
                robot.rightRoller.setPower(-power);
                while (opMode.opModeIsActive() &&
                        ((timeKeeper.seconds() - startTime) < waitForSeconds)) {
                }
                break;
        }
        robot.leftRoller.setPower(0);
        robot.rightRoller.setPower(0);
    }

    boolean targetColorReached(ColorSensor cs, boolean red)
    {
        float [] currentColor = robot.getHue(cs);
        //Log.v("BOK", "Hue val " + currentColor[0]);
        if (red) {
            if ((currentColor[0] > 300) || (currentColor[0] < 5)) {
                Log.v("BOK", "Reached RED!");
                return true;
            }
        }
        else {
            if (currentColor[0] > 175) {
                Log.v("BOK", "Reached BLUE!");
                return true;
            }
        }
        return false;
    }

    // Algorithm to move forward using encoder sensor on the DC motors on the drive train
    protected void moveWColor(double maxPower,
                              double inches,
                              boolean forward,
                              double waitForSec)
    {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            ColorSensor cs = robot.colorBottom;
            boolean red = (allianceColor == BoKAllianceColor.BOK_ALLIANCE_RED);

            robot.resetDTEncoders();
            robot.startMove(maxPower, maxPower, inches, forward);

            runTime.reset();
            while (opMode.opModeIsActive() &&
                    robot.areDTMotorsBusy()) {
                if (runTime.seconds() >= waitForSec) {
                    Log.v("BOK", "moveWColor timed out!" + String.format(" %.1f", waitForSec));
                    break;
                }
                if (targetColorReached(cs, red)) {
                    Log.v("BOK", "moveWColor Line detected");
                    break;
                }
            }

            robot.stopMove();

            if (forward) {
                encCountsTillLine = (Math.abs(robot.getLFEncCount()) +
                                     Math.abs(robot.getRFEncCount())) / 2;
                Log.v("BOK", "encCountsTillLine " + encCountsTillLine);
            }
        }
    }

    class PulseRollersThread extends Thread {
        @Override
        public void run() {
            ElapsedTime rTime = new ElapsedTime();
            rTime.reset();
            // Move rollers in & out slowly
            moveRollers(INTAKE_ROLLERS,
                    robot.ROLLER_POWER_LOW,
                    TIMEOUT_ROLLERS_BURST,
                    rTime);
            moveRollers(REVERSE_ROLLERS,
                    robot.ROLLER_POWER_LOW,
                    TIMEOUT_ROLLERS_BURST,
                    rTime);
        }
    }

    private void moveRobotForIntake(double power, double distance, boolean forward)
    {
        boolean stuck = false, reverse = false;
        double timeStuckStart = 0, timeReverseStart = 0;
        double CRAWL_POWER = 0.01;
        float STUCK_TIME_BEFORE_REVERSE = 0.5f;
        float REVERSE_TIME = 0.25f;

        robot.flipper.setPosition(FLIP_FLIPPER_LOWER);
        robot.resetDTEncoders();
        robot.startMove(power, power, distance, forward);

        Log.v("BOK", "MoveRobotForIntake");
        while (opMode.opModeIsActive() &&
                robot.areDTMotorsBusy()) {

            if ((runTime2ndGlyph.seconds() >= TIMEOUT_8S)) {
                Log.v("BOK", "Ran out of time!");
                break;
            }

            double distFar = robot.distFar.getDistance(DistanceUnit.CM);
            double distNear = robot.distNear.getDistance(DistanceUnit.CM);
            boolean glyphFound = !Double.isNaN(distFar) && !Double.isNaN(distNear);
            if (glyphFound) {
                Log.v("BOK", "Near: " + distNear + ", Far: " + distFar);
                opMode.sleep(robot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
                distFar = robot.distFar.getDistance(DistanceUnit.CM);
                distNear = robot.distNear.getDistance(DistanceUnit.CM);
                if (!Double.isNaN(distFar) && !Double.isNaN(distNear)) {
                    if ((distNear < 20) && (distFar < 20)) {
                        numGlyphs = 2;
                        robot.stopMove();
                        break;
                    }
                }
            }

            int currentPosL = robot.leftRoller.getCurrentPosition();
            int currentPosR = robot.rightRoller.getCurrentPosition();

            int deltaL = Math.abs(currentPosL - lastPosL);
            int deltaR = Math.abs(currentPosR - lastPosR);

            //Log.v("BOK", "CL: " + currentPosL + ", DL: " + deltaL +
            //             ", CR: " + currentPosR + ", DR: " + deltaR);
            if (reverse) {
                if ((runTime2ndGlyph.seconds() - timeReverseStart ) > REVERSE_TIME) {
                    reverse = false;
                }
                lastPosL = currentPosL;
                lastPosR = currentPosR;
            }
            else {
                if ((deltaL <= THRESHOLD_ROLLER_ENC_COUNT) ||
                        (deltaR <= THRESHOLD_ROLLER_ENC_COUNT)) {
                    lastPosL = currentPosL;
                    lastPosR = currentPosR;
                    if (!stuck) {
                        stuck = true;
                        Log.v("BOK", "Stuck");
                        timeStuckStart = runTime2ndGlyph.seconds();
                    } else {
                        if ((runTime2ndGlyph.seconds() - timeStuckStart)
                                > STUCK_TIME_BEFORE_REVERSE) {
                            if (!reverse) {
                                Log.v("BOK", "Reversing");
                                timeReverseStart = runTime2ndGlyph.seconds();
                                reverse = true;
                                robot.setPowerToDTMotors(CRAWL_POWER, CRAWL_POWER,
                                                         CRAWL_POWER, CRAWL_POWER);
                            }
                            robot.leftRoller.setPower(-ROLLER_POWER);
                            robot.rightRoller.setPower(-ROLLER_POWER);
                        }
                    }
                } else {
                    lastPosL = currentPosL;
                    lastPosR = currentPosR;
                    stuck = false;
                    reverse = false;
                    //Log.v("BOK", "Forward");
                    robot.setPowerToDTMotors(0.1, 0.1, 0.1, 0.1);
                    robot.leftRoller.setPower(ROLLER_POWER);
                    robot.rightRoller.setPower(ROLLER_POWER);
                }
            }
        } // while

        Log.v("BOK", "Stop");
        robot.stopMove();
    }

    protected void getSecondGlyph(double POWER_FORWARD, double POWER_BACK)
    {
        int FORWARD_DISTANCE = 6;
        int BACKWARD_DISTANCE = 3;
        int SLEEP_MS = 250;
        int NUM_TRIES = 3;

        robot.leftRoller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRoller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRoller.setPower(ROLLER_POWER);
        robot.rightRoller.setPower(ROLLER_POWER);

        lastPosL = -(THRESHOLD_ROLLER_ENC_COUNT*2);
        lastPosR = -(THRESHOLD_ROLLER_ENC_COUNT*2);

        runTime2ndGlyph.reset();
        Log.v("BOK", "Start 2nd glyph intake");

        for (int i = 0; i < NUM_TRIES; i++) {
            if (numGlyphs == 2)
                break;
            moveRobotForIntake(POWER_FORWARD, FORWARD_DISTANCE, true);
            if (numGlyphs == 2)
                break;
            //opMode.sleep(SLEEP_MS);
            moveRobotForIntake(POWER_BACK, BACKWARD_DISTANCE, false);
        }

        robot.leftRoller.setPower(0);
        robot.rightRoller.setPower(0);

        if (numGlyphs == 0) {
            double distNear = robot.distNear.getDistance(DistanceUnit.CM);
            double distFar = robot.distFar.getDistance(DistanceUnit.CM);
            boolean glyphFound = (!Double.isNaN(distNear) && (distNear < 20)) ||
                    (!Double.isNaN(distFar) && (distFar < 20));
            if (glyphFound) {
                Log.v("BOK", "Check near: " + distNear + ", far: " + distFar);
                numGlyphs = 1;
                if(!Double.isNaN(distNear) && !Double.isNaN(distFar))
                    numGlyphs = 2;
            }
        }
        if (numGlyphs > 0) {
            PulseRollersThread pulseRollers = new PulseRollersThread();
            pulseRollers.start();
        }

        Log.v("BOK", "Num of glyphs: " + numGlyphs);
    }

    class InitRelicArmThread extends Thread {
        @Override
        public void run()
        {
            robot.initRelicArm();
        }
    }

    class FlipFlipperThread extends Thread {
        @Override
        public void run() {
            flipFlipper(FLIP_FLIPPER_DUMP);
        }
    }
}
