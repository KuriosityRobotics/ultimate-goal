package org.firstinspires.ftc.teamcode.ultimategoal.vision;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.imgproc.Imgproc.*;

public class GoalFinder extends OpenCvPipeline {
    public GoalLocationData getLocationData() {
        return locationData;
    }

    private GoalLocationData locationData = null;
    public boolean isBlue = false;

    static final double[][] CAMERA_MATRIX = {
            {1468, 0, 0},
            {0, 1468, 0},
            {960, 540, 1}
    };

    static final Scalar ORANGE = new Scalar(0, 165, 255);
    static final Scalar BLACK = new Scalar(0, 0, 0);

    public static class GoalLocationData {
        double yaw;
        double pitch;
        double x;
        double y;

        public GoalLocationData(double yaw, double pitch, double x, double y) {
            this.yaw = yaw;
            this.pitch = pitch;
            this.x = x;
            this.y = y;
        }

        public double getYaw() {
            return yaw;
        }

        public double getPitch() {
            return pitch;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public String toString() {
            return String.format("Angle (radians, euler xy): {%f, %f}, Location (in pixels): {%f, %f}", getYaw(), getPitch(), getX(), getY());
        }
    }

    /**
     * Finds the largest contour by enclosed area in a list
     *
     * @param contours
     * @return the index of contours which the largest is located
     */
    static int largestContour(final List<MatOfPoint> contours) {
        if (contours.size() == 0)
            return -1;

        double maxVal = 0;
        int maxValIdx = 0;
        for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
            double contourArea = Imgproc.contourArea(contours.get(contourIdx));
            if (maxVal < contourArea) {
                maxVal = contourArea;
                maxValIdx = contourIdx;
            }
        }

        return maxValIdx;
    }


    static int closestToPoint(final List<MatOfPoint> contours, Point point) {
        if(contours.size() == 0)
            return -1;

        double closest = 0;
        int maxValIdx = 0;

        for (int i = 0; i < contours.size(); i++) {
            Moments moments = Imgproc.moments(contours.get(i));
            double distanceToCentre = Math.hypot(point.x - (moments.m10 / moments.m00), point.y - (moments.m01 / moments.m00));

            if(closest > distanceToCentre) {
                closest = distanceToCentre;
                maxValIdx = i;
            }
        }

        return maxValIdx;
    }

    /**
     * This method takes two points, point1 and point2, and then projects these points to form two rays. It then returns the angle between these two rays
     * in radians.
     *
     * @param point1 The first point. This usually will be the centre of the frane
     * @param point2 The second point. In our case, this will be the calculated location (in pixels) of the goal
     * @return An angle in radians representing the angle between them. This is calculated through all dimensions. To find the angle of just a single axis,
     * make one of the coordinate values the same. For example, to find just the yaw (horizontal angle), ensure that the y-value of point1 and point2 are the same.
     */
    public static double findAngle(Point point1, Point point2) {
        RealMatrix K = MatrixUtils.createRealMatrix(CAMERA_MATRIX);
        RealMatrix Ki = MatrixUtils.inverse(K);

        RealVector ray1 = Ki.preMultiply(new ArrayRealVector(new double[]{point1.x, point1.y, 0}));
        RealVector ray2 = Ki.preMultiply(new ArrayRealVector(new double[]{point2.x, point2.y, 0}));

        double cosAngle = ray1.dotProduct(ray2) / (ray1.getNorm() * ray2.getNorm());
        return Math.acos(cosAngle);
    }

    /**
     * processes the frame, returns goallocationdata which has a bunch of cool stuff about goal location data
     *
     * @param input
     * @return an int which represents the calculated number of rings
     */
    public Mat processFrame(final Mat input) {
        return processFrame(input, true, this.isBlue);
    }

    /**
     * processes the frame, returns goallocationdata which has a bunch of cool stuff about goal location data
     *
     * @param input              frame input
     * @param shouldWriteToImage If debug data should be written to the image or not (default/overloaded true)
     * @param isBlue             If true, look for blue goals instead of red
     * @return an int which represents the calculated number of rings
     * @see GoalLocationData
     */
    public Mat processFrame(Mat input, boolean shouldWriteToImage, boolean isBlue) {
        Imgproc.resize(input, input, new Size(480, 270));
        input = input.submat(new Rect(new Point(0, 0), new Point(input.width(), 2*(input.height()/3f))));

        Imgproc.cvtColor(input, input, COLOR_RGB2HSV);

        Point goalLocation;
        Point centre = new Point(input.width() / 2d, input.height() / 2d);
        GoalLocationData loc = null;

        Mat mask = new Mat();
        Mat hierarchy = new Mat(); // we don't need to use this, but opencv requires it

//        if (!isBlue) {
//            Mat mask1 = new Mat(), mask2 = new Mat();
//            Core.inRange(input, new Scalar(0, 70, 50), new Scalar(10, 255, 255), mask1);
//            Core.inRange(input, new Scalar(170, 70, 50), new Scalar(180, 255, 255), mask2);
//            Core.bitwise_or(mask1, mask2, mask);
//            mask1.release();
//            mask2.release();
//        } else {
//            Core.inRange(input, new Scalar(110, 50, 50), new Scalar(130, 255, 255), mask);
//        }

        Core.inRange(input, new Scalar(0, 0, 0), new Scalar(255, 255, 25), mask);

//        Imgproc.morphologyEx(mask, mask, MORPH_OPEN, Imgproc.getStructuringElement(MORPH_ELLIPSE, new Size(5, 5)));
        input.copyTo(input, mask);

        List<MatOfPoint> contours = new ArrayList<>(); // List for storing contours
        findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); // Find all the contours (edges) on the mask
//        int largestContourIndex = closestToPoint(contours, new Point(input.width() / 2f, input.height() / 2f)); // Find the index of contours of the largest contour
        int largestContourIndex = largestContour(contours); // Find the index of contours of the largest contour

        Imgproc.cvtColor(input, input, COLOR_HSV2RGB);

        if (largestContourIndex > -1 && largestContourIndex < contours.size() && contours.get(largestContourIndex) != null) { // Do we even have contours?
            Rect contourBoundingBox = Imgproc.boundingRect(contours.get(largestContourIndex)); // Draw a bounding box around the largest contour
            if (contourBoundingBox.area() > 0.0008 * input.size().area() || true) { // Min size of contour relative to area of image. Using area of BB because mat.size.area is slow
                Moments moments = Imgproc.moments(contours.get(largestContourIndex)); // Calculate the average "centre of mass" of the enclosed area
                double avgX = moments.m10 / moments.m00;
                double avgY = moments.m01 / moments.m00;
                goalLocation = new Point(avgX, avgY);

//                double yaw = findAngle(centre, new Point(goalLocation.x, centre.y)) * (180/Math.PI); // Now isolate the two angles
//                double pitch = findAngle(centre, new Point(centre.x, goalLocation.y)) * (180/Math.PI);
                double yaw = (Math.toRadians(70.42) / input.width()) * (goalLocation.x - centre.x); // TODO: this is pepega
                double pitch = (Math.toRadians(43.30) / input.width()) * (goalLocation.y - centre.y);

                loc = new GoalLocationData(yaw, pitch, goalLocation.x, goalLocation.y);

                if (shouldWriteToImage) {
                    Imgproc.circle(mask, goalLocation, 3, ORANGE);
                    Imgproc.putText(mask, String.valueOf((int) Math.toDegrees(loc.yaw)), centre, FONT_HERSHEY_TRIPLEX, 1, ORANGE);
                    Imgproc.line(mask, new Point(centre.x, centre.y * 2), centre, ORANGE, 2);
                    Imgproc.line(mask, new Point(centre.x, centre.y * 2), new Point(loc.x, centre.y), ORANGE, 2);

                    for (int i = 0; i < contours.size(); i++) {
                        Imgproc.drawContours(input, contours, i, ORANGE, 1, LINE_8, hierarchy, 0);
                    }
                }
            }
        }
        hierarchy.release();
        contours.forEach(Mat::release);
        input.copyTo(input, mask);
        input.release();


        this.locationData = loc;
        return mask;
    }
}

