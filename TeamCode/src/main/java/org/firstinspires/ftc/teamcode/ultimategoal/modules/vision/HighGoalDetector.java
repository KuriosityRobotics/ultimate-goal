package org.firstinspires.ftc.teamcode.ultimategoal.modules.vision;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

public class HighGoalDetector implements VisionModule {
    public boolean isOn;

    private CameraPosition cameraPosition;
    private Point robotPosition;

    // constants
    double fx = 965, fy = 964, cx = 627, cy = 356;

    // helpers
    private static final int[] VALUES = new int[]{80, 151, 50, 93, 255, 255}; // Tuned values to detect high goal contours
    private MatOfPoint3f threeDimensionalPoints; // Coordinate of labelled 3D points to solve for position
    private Mat cameraMatrix; // Camera matrix
    private MatOfDouble distortCoeffs; // Distortion coefficients

    class CameraPosition {
        public double[] rotation;
        public double[] position;

        public CameraPosition(double[] rotation, double[] position) {
            this.rotation = rotation;
            this.position = position;
        }
    }

    public HighGoalDetector(boolean isOn) {
        this.isOn = isOn;

        cameraMatrix = new Mat(3, 3, CvType.CV_64F);

        cameraMatrix.put(0, 0, fx, 0, cx, 0, fy, cy, 0, 0, 1);
        threeDimensionalPoints =
                new MatOfPoint3f(
                        new Point3(0, 14.375, 0),
                        new Point3(0, 0, 0),
                        new Point3(23.875, 0, 0),
                        new Point3(23.875, 14.375, 0));
        distortCoeffs = new MatOfDouble(0.3, -2.05, -0.00178, -0.00648, 6.3);
    }

    @Override
    public void update(Mat inputMat) {
        this.cameraPosition = calculateCameraPosition(inputMat);

        // todo translate to robit pos
    }

    private CameraPosition calculateCameraPosition(Mat inputMat) {
        Mat matInv = inputMat.clone();
        Core.bitwise_not(matInv, matInv);
        Imgproc.cvtColor(inputMat, inputMat, Imgproc.COLOR_BGR2HSV);
        Imgproc.cvtColor(matInv, matInv, Imgproc.COLOR_BGR2HSV);

        Mat mat1 = new Mat(), mat2 = new Mat();
        Core.inRange(inputMat, new Scalar(97, 108, 60), new Scalar(140, 255, 255), mat1); // blue
        Imgproc.morphologyEx(
                mat1,
                mat1,
                Imgproc.MORPH_OPEN,
                Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));

        Core.inRange(matInv, new Scalar(VALUES[0], VALUES[1], VALUES[2]), new Scalar(VALUES[3], VALUES[4], VALUES[5]), mat2); // red
        Imgproc.morphologyEx(
                mat2,
                mat2,
                Imgproc.MORPH_OPEN,
                Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
        Imgproc.morphologyEx(
                mat2,
                mat2,
                Imgproc.MORPH_CLOSE,
                Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));

        Mat step2 = new Mat();
        Core.bitwise_or(mat1, mat2, step2);

        Mat step3 = new Mat(), hierarchy = new Mat();
        ArrayList<MatOfPoint> contoursBlue = new ArrayList<>();
        Imgproc.findContours(
                mat1, contoursBlue, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        sortContours(contoursBlue);
        inputMat.copyTo(step3, step2);
        ArrayList<MatOfPoint2f> contours2fBlue =
                contoursBlue.stream()
                        .map(n -> new MatOfPoint2f(n.toArray()))
                        .collect(Collectors.toCollection(ArrayList::new));

        MatOfPoint2f largestBlueContour = contours2fBlue.get(contours2fBlue.size() - 1);

        RotatedRect largestRotatedRectBlue = Imgproc.minAreaRect(largestBlueContour);

        MatOfInt hull = new MatOfInt();

        Imgproc.convexHull(convertfMatToPointMat(largestBlueContour), hull);
        Point[] contourArray = largestBlueContour.toArray();
        Point[] hullPoints = new Point[hull.rows()];
        List<Integer> hullContourIdxList = hull.toList();
        for (int i = 0; i < hullContourIdxList.size(); i++) {
            hullPoints[i] = contourArray[hullContourIdxList.get(i)];
        }

        ArrayList<MatOfPoint> contoursRedOnly = new ArrayList<>();
        Imgproc.findContours(
                mat2, contoursRedOnly, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        ArrayList<MatOfPoint2f> countours2fRed =
                contoursRedOnly.stream()
                        .map(n -> new MatOfPoint2f(n.toArray()))
                        .collect(Collectors.toCollection(ArrayList::new));

        ArrayList<RotatedRect> rectList =
                countours2fRed.stream()
                        .map(Imgproc::minAreaRect)
                        .collect(Collectors.toCollection(ArrayList::new));
        ArrayList<RotatedRect> rectListFiltered =
                rectList.stream()
                        .filter(
                                n ->
                                        minDistanceOfRectangles(
                                                n.boundingRect(), largestRotatedRectBlue.boundingRect())
                                                < 50)
                        .collect(Collectors.toCollection(ArrayList::new));

        ArrayList<MatOfPoint> vertexList = new ArrayList<>();
        rectListFiltered.forEach(
                n -> {
                    Point[] points = new Point[4];
                    n.points(points);
                    vertexList.add(new MatOfPoint(points));
                });

        Point[] pointsBlue = new Point[4];
        largestRotatedRectBlue.points(pointsBlue);

//    Imgproc.fillConvexPoly(step3, new MatOfPoint(pointsBlue), new Scalar(200, 200, 200));
        Imgproc.cvtColor(step3, step3, Imgproc.COLOR_HSV2BGR);
        Imgproc.cvtColor(inputMat, inputMat, Imgproc.COLOR_HSV2BGR);

        MatOfPoint2f points2 = new MatOfPoint2f(pointsBlue);

        Mat imageWithShapeOverlay = new Mat();
        Mat inputWithShape = inputMat.clone();
        MatOfPoint2f approxDP = new MatOfPoint2f();

        double epsilon = 0;
        do {
            Imgproc.approxPolyDP(new MatOfPoint2f(hullPoints), approxDP, epsilon, true);
            epsilon = epsilon + 1;
        } while (approxDP.toArray().length > 4);

        Imgproc.fillConvexPoly(inputWithShape, convertfMatToPointMat(approxDP), new Scalar(200, 200, 200));
        vertexList.forEach(n -> Imgproc.fillConvexPoly(inputWithShape, n, new Scalar(200, 200, 200)));

        Core.addWeighted(inputMat, .3, inputWithShape, .7, .0, imageWithShapeOverlay);

        return solvePnP(threeDimensionalPoints, approxDP, cameraMatrix, distortCoeffs);
    }

    private CameraPosition solvePnP(MatOfPoint3f _objPoints, MatOfPoint2f _imgPoints, Mat cameraMatrix, MatOfDouble distortionCoefficients) {
        Mat rvecs = new Mat(3, 1, CvType.CV_64FC1);
        Mat tvecs = new Mat(3, 1, CvType.CV_64FC1);

        try {
            Calib3d.solvePnP(_objPoints, _imgPoints, cameraMatrix, distortionCoefficients, rvecs, tvecs);

            Mat rMat = new Mat();
            Calib3d.Rodrigues(rvecs, rMat);

            // Transpose the matrix
            //   rMat = rMat.t();
            Mat viewMatrix = new Mat(4, 4, CvType.CV_64FC1);
            //            Mat transferMatrix = new Mat(4, 4, CvType.CV_64FC1);
            Mat transferMatrix = new Mat(4, 4, CvType.CV_64FC1);

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    viewMatrix.put(i, j, rMat.get(i, j));
                }

                viewMatrix.put(i, 3, tvecs.get(i, 0));
                viewMatrix.put(3, i, 0.0f);
            }

            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    if (i == 1 || i == 2) {
                        transferMatrix.put(i, j, -1.0f);
                    } else {
                        transferMatrix.put(i, j, 1.0f);
                    }
                }
            }

            viewMatrix.put(3, 3, 1.0f);

            double[] x = new double[1];
            double[] y = new double[1];
            double[] z = new double[1];

            double[] xR = new double[1];
            double[] yR = new double[1];
            double[] zR = new double[1];

            tvecs.get(0, 0, x);
            tvecs.get(1, 0, y);
            tvecs.get(2, 0, z);

            rMat.get(0, 0, xR);
            rMat.get(1, 0, yR);
            rMat.get(2, 0, zR);

            double[] pos = {x[0], y[0], z[0]};
            double[] rot = {xR[0], yR[0], zR[0]};

            return new CameraPosition(rot, pos);
        } catch (Exception e) {
            e.printStackTrace();

            return null;
        }
    }

    static void sortContours(List<MatOfPoint> contours) {
        contours.sort(Comparator.comparingInt(n -> (int) Imgproc.contourArea(n)));
    }

    static double minDistanceOfRectangles(Rect rect1, Rect rect2) {
        double min_dist;

        Point C1 = new Point(), C2 = new Point();
        C1.x = rect1.x + (rect1.width / 2.);
        C1.y = rect1.y + (rect1.height / 2.);
        C2.x = rect2.x + (rect2.width / 2.);
        C2.y = rect2.y + (rect2.height / 2.);

        double Dx, Dy;
        Dx = Math.abs(C2.x - C1.x);
        Dy = Math.abs(C2.y - C1.y);

        if ((Dx < ((rect1.width + rect2.width) / 2.)) && (Dy >= ((rect1.height + rect2.height) / 2.))) {
            min_dist = Dy - ((rect1.height + rect2.height) / 2.);
        } else if ((Dx >= ((rect1.width + rect2.width) / 2.))
                && (Dy < ((rect1.height + rect2.height) / 2.))) {
            min_dist = Dx - ((rect1.width + rect2.width) / 2.);
        } else if ((Dx >= ((rect1.width + rect2.width) / 2.))
                && (Dy >= ((rect1.height + rect2.height) / 2.))) {
            double delta_x = Dx - ((rect1.width + rect2.width) / 2.);
            double delta_y = Dy - ((rect1.height + rect2.height) / 2.);
            min_dist = Math.sqrt(delta_x * delta_x + delta_y * delta_y);
        } else {
            min_dist = -1;
        }

        return min_dist;
    }

    static MatOfPoint convertfMatToPointMat(MatOfPoint2f mat) {
        return new MatOfPoint(mat.toArray());
    }

    public CameraPosition getCameraPosition() {
        return cameraPosition;
    }

    public Point getRobotPosition() {
        return robotPosition;
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public String getName() {
        return "HighGoalDetector";
    }
}
