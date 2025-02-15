package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.CvType;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class SampleDetector extends OpenCvPipeline {

    public double distance;
    ArrayList<RotatedRect> filteredRects;
    double minDistanceThreshold = 1000;


    public enum SampleColor {
        YELLOW(),
        BLUE(),
        RED();
    }

    public Mat frame;

    public static Telemetry telemetry;

    public static Scalar lowerYellow = new Scalar(19.0, 150.0, 130.1); // hsv
    public static Scalar upperYellow = new Scalar(30.0, 255.0, 255.0); // hsv
    public static Scalar lowerBlue = new Scalar(90.0, 150.0, 100.0); // hsv
    public static Scalar upperBlue = new Scalar(120.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedH = new Scalar(10.0, 0.0, 0.0); // hsv
    public static Scalar upperRedH = new Scalar(160.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedSV = new Scalar(0.0, 150.0, 120.0); // hsv
    public static Scalar upperRedSV = new Scalar(255.0, 255.0, 255.0); // hsv

    private double sampleAngle = 0;
    private double k_translation = 1d/640d;

    public static SampleColor colorType = SampleColor.BLUE;
//    @Override
//    public void init(int width, int height, CameraCalibration calibration) {
//
//    }

    public SampleDetector(Telemetry telemetry){
        this.telemetry = telemetry;
//        this.colorType = colorType;
    }

    @Override
    public Mat processFrame(Mat input) {
        frame = input.clone();
        Mat hsv = new Mat(); // convert to hsv
        Mat gray = new Mat(); // convert to hsv
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);


        // Getting representative brightness of image and correcting brightness
        MatOfDouble muMat = new MatOfDouble();
        MatOfDouble sigmaMat = new MatOfDouble();
        Core.meanStdDev(gray, muMat, sigmaMat);
        telemetry.addData("gray mu", muMat.get(0,0)[0]);
        telemetry.addData("gray sigma", sigmaMat.get(0,0)[0]);

        double mu = muMat.get(0,0)[0];
        double sigma = sigmaMat.get(0,0)[0];
        double k = 1;
        Scalar lowerBound = new Scalar(mu-k*sigma);
        Scalar upperBound = new Scalar(mu+k*sigma);

        Mat mask = new Mat();
        Core.inRange(gray, lowerBound, upperBound, mask);
        Scalar maskedMean = Core.mean(gray, mask);
        double averageInRange = maskedMean.val[0];
        double targetAverageInRange = 90;
        frame.convertTo(frame, -1, targetAverageInRange/averageInRange, 0);

        telemetry.addData("averageInRange", averageInRange);



        // Color threshold
        Mat inRange = new Mat();
//        Core.inRange(hsv, PixelColor.YELLOW.LOWER, PixelColor.YELLOW.UPPER, inRange);
        if (colorType.equals(SampleColor.BLUE)) {
            Core.inRange(hsv, lowerBlue, upperBlue, inRange);
        } else if (colorType.equals(SampleColor.RED)) {
            Mat inHRange = new Mat();
            Mat inSVRange = new Mat();
            Core.inRange(hsv, lowerRedH, upperRedH, inHRange);
            Core.bitwise_not(inHRange, inHRange);
            Core.inRange(hsv, lowerRedSV, upperRedSV, inSVRange);
            Core.bitwise_and(inHRange, inSVRange, inRange);
        } else {
            Core.inRange(hsv, lowerYellow, upperYellow, inRange);
        }

        // Morphology
        Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(25, 25));
        Mat kernel2 = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(10, 10));

//        Imgproc.erode(inRange, inRange, kernel);
//        Imgproc.dilate(inRange, inRange, kernel2);

        // Find all contours
        List<MatOfPoint> unfilteredContours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(inRange, unfilteredContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter contours by size and get rotated rects
        int minArea = 20000;
        ArrayList<RotatedRect> rotatedRects = new ArrayList<>();
        List<MatOfPoint> filteredContours = new ArrayList<>();
        for (MatOfPoint contour : unfilteredContours) {
            RotatedRect minAreaRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            double area = minAreaRect.size.area();
            if (area > minArea) {
                filteredContours.add(contour);
                rotatedRects.add(minAreaRect);
            }
        }
        Imgproc.drawContours(frame, filteredContours, -1, new Scalar(0, 255, 0), 1);

        // Get overlapping rotated rect groups
        double overlapThreshold = 0.3; // % of smaller box covered
        Set<Integer> toSkip = new HashSet<>();
        ArrayList<ArrayList<Double[]>> overlapGroups = new ArrayList<>();
        for (int i = 0; i < rotatedRects.size(); i++) {
            if (toSkip.contains(i)) continue;
            toSkip.add(i);
            ArrayList<Double[]> overlapGroup = new ArrayList<>();
            double iArea = rotatedRects.get(i).size.area();
            overlapGroup.add(new Double[]{(double)i, iArea});
            for (int j = i+1; j < rotatedRects.size(); j++) {
                if (toSkip.contains(j)) continue;
                double jArea = rotatedRects.get(j).size.area();
                double overlapArea = getIntersectionArea(rotatedRects.get(i), rotatedRects.get(j));
                double distance = Math.sqrt(
                        Math.pow(rotatedRects.get(i).center.x - rotatedRects.get(j).center.x, 2) +
                                Math.pow(rotatedRects.get(i).center.y - rotatedRects.get(j).center.y, 2)
                );
                // Only merge if overlap is significant AND rectangles are close together
                if (overlapArea / Math.min(iArea, jArea) >= overlapThreshold && distance <= minDistanceThreshold) {
                    overlapGroup.add(new Double[]{(double)j, jArea});
                    toSkip.add(j);
                }
            }
            overlapGroups.add(overlapGroup);
        }

        // telemetry
        ArrayList<ArrayList<Double>> overlapGroups2 = new ArrayList<>();
        for (ArrayList<Double[]> overlapGroup : overlapGroups) {
            overlapGroups2.add(new ArrayList<>());
            for (Double[] index : overlapGroup) {
                overlapGroups2.get(overlapGroups2.size()-1).add(index[0]);
            }
        }
        telemetry.addData("overlapGroups", overlapGroups2);

        // Filter out overlapping rotated rects
        filteredRects = new ArrayList<>();
        for (ArrayList<Double[]> overlapGroup : overlapGroups) {
            int maxIndex = overlapGroup.get(0)[0].intValue();
            double maxArea = overlapGroup.get(0)[1];
            for (Double[] rect : overlapGroup) {
                if (rect[1] > maxArea) {
                    maxArea = rect[1];
                    maxIndex = rect[0].intValue();
                }
            }
            filteredRects.add(rotatedRects.get(maxIndex));
        }

        telemetry.addData("filteredRects.size()", filteredRects.size());


//        // Draw unfiltered rects as blue
//        for (RotatedRect rotatedRect : rotatedRects) {
//            Point[] vertices = new Point[4];
//            rotatedRect.points(vertices);
//            for (int i = 0; i < 4; i++) {
//                Imgproc.line(frame, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 0, 255), 2);
//            }
//        }

        // Draw filtered rects as green
        ArrayList<Point> real = getOffsets(filteredRects);
        for (int i = 0; i < filteredRects.size(); i++) {
            RotatedRect rotatedRect = filteredRects.get(i);
            Point[] vertices = new Point[4];
            rotatedRect.points(vertices);
            for (int j = 0; j < 4; j++) {
                Imgproc.line(frame, vertices[j], vertices[(j + 1) % 4], new Scalar(0, 255, 0), 2);
            }
            Point center = rotatedRect.center;



            double procAngle = rotatedRect.angle;
            if (filteredRects.get(0).size.width > filteredRects.get(0).size.height)
                procAngle *= -1;
            else
                procAngle = 90-procAngle;

            double length = 200;
            double vecX = center.x+length*Math.cos(Math.toRadians(procAngle));
            double vecY = center.y-length*Math.sin(Math.toRadians(procAngle));
            telemetry.addData("vecX", vecX);
            telemetry.addData("vecY", vecY);
            if (vecX < 0) {
                vecY = center.y - (1+vecX/(length*Math.cos(Math.toRadians(procAngle))))*length*Math.sin(Math.toRadians(procAngle));
                vecX = 0;
            } if (vecY < 0) {
                vecX = center.x + (1+vecY/(length*Math.sin(Math.toRadians(procAngle))))*length*Math.cos(Math.toRadians(procAngle));
                vecY = 0;
            } if (vecX > 640) {
                vecY = center.y - (1-(vecX-640)/ (length*Math.cos(Math.toRadians(procAngle))))*length*Math.sin(Math.toRadians(procAngle));
                vecX = 640;
            } if (vecY > 480) {
                vecX = center.x + (1+(vecY-480)/(length*Math.sin(Math.toRadians(procAngle))))*length*Math.cos(Math.toRadians(procAngle));
                vecY = 480;
            }
            Imgproc.line(frame, center, new Point(vecX,vecY), new Scalar(0, 255, 255), 1);
            Imgproc.line(frame, center, new Point(center.x+length/2,center.y), new Scalar(0, 255, 255), 1);
            // Define arc parameters
            int radius = 20;
            double endAngle = -procAngle;
            Scalar color = new Scalar(0, 255, 255);
            int thickness = 1;

            // Generate points on the arc
            MatOfPoint points = new MatOfPoint();
            Imgproc.ellipse2Poly(center, new Size(radius, radius), 0, 0, (int) endAngle, 1, points);

            // Draw the arc
            List<MatOfPoint> listThing = new ArrayList<>();
            listThing.add(points);
            Imgproc.polylines(frame, listThing, false, color, thickness);
            Imgproc.putText(frame, (Math.round(10*procAngle)/10d)+" deg", new Point(center.x+30, center.y-10), 0, 0.5, new Scalar(0, 255, 255));





            double sampleHeight = (1d/rotatedRect.size.area()+2.28e-5)/(7.14e-6);
            sampleHeight = (1/rotatedRect.size.area()+2.57e-5)/(7.6e-6); // calculate height of camer based on area of sample
            telemetry.addData("sampleHeight", sampleHeight);

            distance = calculateDistance(sampleHeight);

            double real_x = realX(); // in inches
            double real_y = realY();
            telemetry.addData("real_x", real_x);
            telemetry.addData("real_y", real_y);
            Imgproc.line(frame, center, new Point(320, center.y), new Scalar(255, 255, 0), 1);
            Imgproc.putText(frame, (Math.round(10*real_x)/10d)+(Math.abs(real_x)>1?" in":""), new Point(320+(center.x-320)*0.5-20, center.y+15), 0, 0.5, new Scalar(255, 255, 0));
            Imgproc.line(frame, new Point(320, center.y), new Point(320,240), new Scalar(255, 255, 0), 1);
            Imgproc.putText(frame, (Math.round(10*real_y)/10d)+(Math.abs(real_y)>1?" in":""), new Point(315-10*Double.toString(Math.round(10*real_y)/10d).length()-(Math.abs(real_y)>1?22:0), 240+(center.y-240)*0.5+10), 0, 0.5, new Scalar(255, 255, 0));




            Imgproc.circle(frame, rotatedRect.center, 1, new Scalar(255, 255, 0), 3);
        }
        Imgproc.circle(frame, new Point(320, 240), 1, new Scalar(255, 255, 0), 3);


        // telemetry
        if (!filteredRects.isEmpty()) {
            telemetry.addData("width ", filteredRects.get(0).size.width);
            telemetry.addData("height ", filteredRects.get(0).size.height);
            telemetry.addData("area ", filteredRects.get(0).size.area());
            telemetry.addData("angle ", filteredRects.get(0).angle);
            telemetry.addData("center ", filteredRects.get(0).center);
//            telemetry.addData("center scaled", new Point((filteredRects.get(0).center.x - 320) / 640 * 3.0/8.0, -(filteredRects.get(0).center.y - 240) / 480));
//            output.add(new Point((i.center.x - 320) / 640 * canvasHorizontal, -(i.center.y - 240) / 240 * canvasVertical));
            double procAngle = filteredRects.get(0).angle;
            if (filteredRects.get(0).size.width > filteredRects.get(0).size.height)
                procAngle *= -1;
            else
                procAngle = 90-procAngle;
            telemetry.addData("procAngle ", procAngle);
            sampleAngle = procAngle;
        }
        telemetry.addData("sampleAngle", sampleAngle);
        telemetry.addData("Distance", distance);







        telemetry.update();


        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//        Paint p = new Paint();
//        p.setColor(Color.BLUE);
//        p.setStrokeWidth(4);
//        canvas.drawCircle((float) getPixelsCenter().x, (float) getPixelsCenter().y, 0, p);
//        canvas.drawCircle((float) getPixelsCenter().x, (float) getPixelsCenter().y, 6, p);
    }

    private double getIntersectionArea(RotatedRect rect1, RotatedRect rect2) {
        // Get vertices of the rectangles
        Point[] vertices1 = new Point[4];
        rect1.points(vertices1);

        Point[] vertices2 = new Point[4];
        rect2.points(vertices2);

        // Convert vertices arrays to MatOfPoint2f
        MatOfPoint2f poly1 = new MatOfPoint2f(vertices1);
        MatOfPoint2f poly2 = new MatOfPoint2f(vertices2);

        // Output MatOfPoint2f for the intersection polygon
        MatOfPoint2f intersection = new MatOfPoint2f();

        // Calculate intersection
        return Imgproc.intersectConvexConvex(poly1, poly2, intersection, true);
    }


    public MatOfPoint2f convertMatToMatOfPoint2f(Mat mat) {
        // Check if the Mat is in the correct format (CV_32FC2)
        if (mat.type() != CvType.CV_32FC2) {
            throw new IllegalArgumentException("Mat must be of type CV_32FC2");
        }

        // Create a MatOfPoint2f object
        MatOfPoint2f matOfPoint2f = new MatOfPoint2f();

        // Convert Mat rows to Point objects
        Point[] points = new Point[(int) mat.total()];
        for (int i = 0; i < mat.rows(); i++) {
            float[] data = new float[2];
            mat.get(i, 0, data);
            points[i] = new Point(data[0], data[1]);
        }

        // Set points to MatOfPoint2f
        matOfPoint2f.fromArray(points);

        return matOfPoint2f;
    }

    public ArrayList<Point> getOffsets(ArrayList<RotatedRect> input) {
        // Note: This method only works when the camera is directly above the samples, looking straight down

        ArrayList<Point> output = new ArrayList<Point>();
        double cameraAngle = 0;

        double height = 10.0; // in inches
        // TODO: Make height not hardcoded, instead base it off of robot position


        double canvasVertical = height*3.0/8.0; // inches
        double canvasHorizontal = height / 2.0;

        for (RotatedRect i : input) {
            // real center is (320, 480), positive direction is right and down
//            output.add(new Point(i.center.x - 320, -(i.center.y - 240)));
            output.add(new Point((i.center.x - 320) / 320 * canvasHorizontal, -(i.center.y - 240) / 240 * canvasVertical));


            // 4 in height = 1.5 width vertical (half width, not full)
            // 6 : 2.25
            // 2 : 0.75
            // 8 : 3
            // horizontal: 8 / 4
        }


        return output;
    }

    public double realX() {
        if (filteredRects.isEmpty()) return 0; // No rectangles detected
        ArrayList<Point> real = getOffsets(filteredRects);

        // Find the rectangle with the lowest realY value
        int minYIndex = 0;
        double minY = real.get(0).y;
        for (int i = 1; i < real.size(); i++) {
            if (real.get(i).y < minY) {
                minY = real.get(i).y;
                minYIndex = i;
            }
        }

        // Draw a bright red contour around the chosen rectangle
        RotatedRect chosenRect = filteredRects.get(minYIndex);
        Point[] vertices = new Point[4];
        chosenRect.points(vertices);
        for (int j = 0; j < 4; j++) {
            Imgproc.line(frame, vertices[j], vertices[(j + 1) % 4], new Scalar(255, 0, 0), 4); // Bright red contour
        }

        // Return the realX of the rectangle with the lowest realY
        return real.get(minYIndex).x;
    }

    public double realY() {
        if (filteredRects.isEmpty()) return 0; // No rectangles detected
        ArrayList<Point> real = getOffsets(filteredRects);

        // Find the rectangle with the lowest realY value
        int minYIndex = 0;
        double minY = real.get(0).y;
        for (int i = 1; i < real.size(); i++) {
            if (real.get(i).y < minY) {
                minY = real.get(i).y;
                minYIndex = i;
            }
        }

        // Return the realY of the rectangle with the lowest realY
        return real.get(minYIndex).y;
    }

    public double realAngle() {
        if (filteredRects.isEmpty()) return 0; // No rectangles detected
        ArrayList<Point> real = getOffsets(filteredRects);

        // Find the rectangle with the lowest realY value
        int minYIndex = 0;
        double minY = real.get(0).y;
        for (int i = 1; i < real.size(); i++) {
            if (real.get(i).y < minY) {
                minY = real.get(i).y;
                minYIndex = i;
            }
        }

        // Get the angle of the rectangle with the lowest realY
        RotatedRect targetRect = filteredRects.get(minYIndex);
        double angle = targetRect.angle;

        // Adjust the angle based on rectangle orientation
        if (targetRect.size.width > targetRect.size.height) {
            angle *= -1;
        } else {
            angle = 90 - angle;
        }

        return angle;
    }

    private double calculateDistance(double objectHeightInPixels) {
        double KNOWN_OBJECT_HEIGHT = 89;
        double FOCAL_LENGTH = 4;
        return (KNOWN_OBJECT_HEIGHT * FOCAL_LENGTH) / objectHeightInPixels;
    }

    public double getDistance(){  return distance; }
}