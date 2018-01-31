package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Krishna Saxena on 11/15/2017.
 */

public class BoKAutoOCVHelper
{
    private static int MIN_POLYGON_SIDES = 6;
    private static int MAX_POLYGON_SIDES = 8;
    private static int MIN_POLYGON_AREA = 200;
    private static int MAX_POLYGON_AREA = 300;
    private static int MIN_RECT_AREA = 3000;
    private static final String OCV_LOW_IMG = "ocvLow.png";
    private static final String OCV_HIGH_IMG = "ocvHigh.png";
    private static final String OCV_LOW_EDGES_IMG = "ocvLowEdges.png";
    private static final String OCV_HIGH_EDGES_IMG = "ocvHighEdges.png";

    // Returns true if the two OpenCV Rect r1 & r2 intersect, false otherwise
    private static boolean ocvRectsIntersect(Rect r1, Rect r2)
    {
        // If one rectangle is on left side of other
        if ((r1.x > (r2.x + r2.width)) ||
                (r2.x > (r1.x + r1.width)))
            return false;

        // If one rectangle is above other
        if ((r1.y > (r2.y + r2.height)) ||
                (r2.y > r1.y + r1.height))
            return false;

        return true;
    }

    private static Mat filterImg(Mat src, Scalar lower, Scalar upper, String fname)
    {
        Mat dst = new Mat();
        Core.inRange(src, lower, upper, dst);
        BoKAutoCommon.writeFile(fname, dst, BoKAutoCommon.DEBUG_OPEN_CV); // Debug
        return dst;
    }

    private static Mat findEdges(Mat src, boolean l2Gradient, String fname)
    {
        Mat edges = new Mat();
        Imgproc.Canny(src, edges, 30, 90, 3, l2Gradient);
        //Mat dstEdges = new Mat();
        //Core.add(dstEdges, Scalar.all(0), dstEdges); // empty mat
        //dst.copyTo(dstEdges, edges);
        //BoKAutoCommon.writeFile(fname, dstEdges, BoKAutoCommon.DEBUG_OPEN_CV); // Debug
        //dstEdges.release();
        return edges;
    }

    private static List<MatOfPoint> findContours(Mat edges)
    {
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges,
                contours,
                hierarchy,
                Imgproc.RETR_LIST,
                Imgproc.CHAIN_APPROX_SIMPLE);

        hierarchy.release();
        return contours;
    }

    private static RelicRecoveryVuMark findHexagons(List<MatOfPoint> contours,
                                                    Mat contourImg)
    {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        List<Rect> hexagons = new ArrayList<Rect>();
        MatOfPoint2f approxCurve = new MatOfPoint2f();
        int nRightHexagons = 0;
        Log.v("BOK", "Contours: " + contours.size());

        for (int i = 0; i < contours.size(); i++) {
            // Ignore small contours
            double area = Imgproc.contourArea(contours.get(i));
            if ((area >= MIN_POLYGON_AREA) && (area <= MAX_POLYGON_AREA)) {
                MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(i).toArray());
                double approxDistance = Imgproc.arcLength(contour2f, true)*0.04; // epsilon
                Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

                if ((approxCurve.size().height >= MIN_POLYGON_SIDES) &&
                    (approxCurve.size().height <= MAX_POLYGON_SIDES)) {
                    MatOfPoint points = new MatOfPoint( approxCurve.toArray() );

                    // Get bounding rect of contour
                    Rect rect = Imgproc.boundingRect(points);
                    if ((rect.x < 540) && (rect.y > 360)) {
                        // bottom left quadrant of the screen (1280x720)
                        boolean unique = true;
                        for (int j = 0; j < hexagons.size(); j++) {
                            Rect r = hexagons.get(j);
                            if (ocvRectsIntersect(r, rect)) {
                                unique = false;
                                break;
                            }
                        }
                        if (unique) {
                            if (BoKAutoCommon.DEBUG_OPEN_CV)
                                Imgproc.drawContours(contourImg,
                                                     contours, i,
                                                     new Scalar(0,0,0,255), -1);
                            hexagons.add(rect);
                            // Also count the number of black hexagons in the right column
                            if (rect.x > 310)
                                nRightHexagons++;
                            Log.v("BOK", "Hexagon at: " + rect.toString());
                        } // unique
                    } // if (rect.x < 540) && (rect.y > 360)
                    points.release();
                } // n sides in the contour
                contour2f.release();
            } // contour area > 100
        } // for (int i = 0; i < contours.size(); i++)

        approxCurve.release();
        Log.v("BOK", "Num hexagons: " + hexagons.size() +
                ", Right: " + nRightHexagons);
        if (hexagons.size() > 7) {
            vuMark =  RelicRecoveryVuMark.LEFT;
        }
        else if (hexagons.size() == 7) {
            if (nRightHexagons > 4)
                vuMark = RelicRecoveryVuMark.RIGHT;
            else
                vuMark = RelicRecoveryVuMark.CENTER;
        }
        return vuMark;
    }

    protected static RelicRecoveryVuMark findVuMark(Mat img, Mat contourImg)
    {
        // img is in HSV format and has been blurred
        // Filter out the low value pixels;
        // Note that the range of hue in OpenCV is [0 - 179]
        Mat dst = filterImg(img,
                            new Scalar(0, 0, 0),
                            new Scalar(179, 255, 80),
                            OCV_LOW_IMG);

        // Run Canny edge detection on the low intensity image
        Mat edges = findEdges(dst, false, OCV_LOW_EDGES_IMG);
        // Done with dst mat
        dst.release();
        // Find contours for the low intensity image
        List<MatOfPoint> contours = findContours(edges);
        // Done with edges
        edges.release();

        return findHexagons(contours, contourImg);
    }

    private static Rect findWhiteRectangle(List<MatOfPoint> contours,
                                           Mat contourImg)
    {
        List<Rect> rectangles = new ArrayList<Rect>();
        MatOfPoint2f approxCurve = new MatOfPoint2f();
        Log.v("BOK", "Contours: " + contours.size());

        for (int i = 0; i < contours.size(); i++) {
            // Ignore small contours
            double area = Imgproc.contourArea(contours.get(i));
            if (area > MIN_RECT_AREA) {
                MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(i).toArray());
                double approxDistance = Imgproc.arcLength(contour2f, true)*0.04; // epsilon
                Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

                if (approxCurve.size().height == 4) {
                    MatOfPoint points = new MatOfPoint( approxCurve.toArray() );

                    // Get bounding rect of contour
                    Rect rect = Imgproc.boundingRect(points);
                    if (rect.y > 500) {
                        // bottom left quadrant of the screen (1280x720)
                        boolean unique = true;
                        for (int j = 0; j < rectangles.size(); j++) {
                            Rect r = rectangles.get(j);
                            if (ocvRectsIntersect(r, rect)) {
                                unique = false;
                                break;
                            }
                        }
                        if (unique) {
                            if (BoKAutoCommon.DEBUG_OPEN_CV)
                                Imgproc.drawContours(contourImg,
                                        contours, i,
                                        new Scalar(0,0,0,255), -1);
                            rectangles.add(rect);
                            // Also count the number of black hexagons in the right column
                            Log.v("BOK", "Rectangle at: " + rect.toString());
                        } // unique
                    } // if rect.y > 500
                    points.release();
                } // n sides in the contour
                contour2f.release();
            } // contour area > 100
        } // for (int i = 0; i < contours.size(); i++)

        approxCurve.release();

        if (rectangles.size() == 1)
            return rectangles.get(0);
        Log.v("BOK", "Too many rectangles: " + rectangles.size());
        return null;
    }

    protected static Rect findWhiteRectangle(Mat img, Mat contourImg)
    {
        // img is in HSV format and has been blurred
        // Filter out the high value pixels
        // Note that the range of hue in OpenCV is [0 - 179]
        Mat dst = filterImg(img,
                            new Scalar(0, 0, 200),
                            new Scalar(179, 255, 255),
                            OCV_HIGH_IMG);

        // Run Canny edge detection on high intensity image
        Mat edges = findEdges(dst, true, OCV_HIGH_EDGES_IMG);
        // Done with dst mat
        dst.release();
        // Find contours for the low intensity image
        List<MatOfPoint> contours = findContours(edges);
        // Done with edges
        edges.release();

        return findWhiteRectangle(contours, contourImg);
    }
}
