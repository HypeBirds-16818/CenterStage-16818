package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
public class RedElement extends OpenCvPipeline{
    public static int position;
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    public static int region1X = 260;
    public static int region1Y = 160;

    public static int region2X = 500;
    public static int region2Y = 220;
    static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(region1X,region1Y);
    static Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(region2X,region2Y);

    static final int REGION_WIDTH = 80;
    static final int REGION_HEIGHT = 80;

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    public static double hMin = 0;
    public static double sMin = 135;
    public static double vMin = 0;

    public static double hMax = 255;
    public static double sMax = 200;
    public static double vMax = 255;

    static Scalar redHsvMin = new Scalar(hMin, sMin, vMin, 0);
    static Scalar redHsvMax = new Scalar(hMax, sMax, vMax, 255);
    Mat region1_Cb, region2_Cb;
    Mat YCrCb = new Mat();
    Mat mask = new Mat();

    Mat maskedFrame = new Mat();

    int avg1, avg2;

    void mask(Mat frame) {
        Imgproc.cvtColor(frame, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(YCrCb, redHsvMin, redHsvMax, mask);
    }

    @Override
    public void init(Mat firstFrame)
    {
        mask(firstFrame);
        region1_Cb = mask.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = mask.submat(new Rect(region2_pointA, region2_pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        redHsvMin = new Scalar(hMin, sMin, vMin, 0);
        redHsvMax = new Scalar(hMax, sMax, vMax, 255);

        REGION1_TOPLEFT_ANCHOR_POINT = new Point(region1X,region1Y);
        REGION2_TOPLEFT_ANCHOR_POINT = new Point(region2X,region2Y);

        region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        region1_Cb = mask.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = mask.submat(new Rect(region2_pointA, region2_pointB));

        /*
         * Get the Cb channel of the input frame after conversion to YCrCb
         */
        mask(input);

        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
        avg1 = (int) Core.mean(region1_Cb).val[0];
        avg2 = (int) Core.mean(region2_Cb).val[0];

        maskedFrame.release();

        Core.bitwise_and(input, input, maskedFrame, mask);

        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                maskedFrame, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                maskedFrame, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */
        if(avg1 >= (255f / 2)) // Was it from region 1?
        {
            position = 1; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    maskedFrame, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(avg2  >= (255f / 2)) // Was it from region 2?
        {
            position = 2; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    maskedFrame, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else // Was it from region 3?
        {
            position = 3; // Record our analysis
        }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return maskedFrame;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */

    public int getAnalysis() {
        return position;
    }
}
