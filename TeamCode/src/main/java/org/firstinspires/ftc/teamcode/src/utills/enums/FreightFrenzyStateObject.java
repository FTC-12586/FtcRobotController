package org.firstinspires.ftc.teamcode.src.utills.enums;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import kotlin.NotImplementedError;

/**
 * A Enum for each object the bucket can pick up
 */
public enum FreightFrenzyStateObject {
    BALL,
    CUBESMOOTH,
    CUBEWAFFLE,
    DUCK,
    EMPTY;

    private static final double[] BallRGB = new double[]{60, 60, 54};
    private static final double[] CubeSmoothRGB = new double[]{24, 14, 8};
    private static final double[] CubeWaffleRGB = new double[]{16, 11, 6};
    private static final double[] DuckRGB = new double[]{10, 10, 6};
    private static final double[] EmptyRGB = new double[]{4, 6, 5};


    /**
     * A Array of every possible enum value
     */
    private static final FreightFrenzyStateObject[] gameObjectArray = FreightFrenzyStateObject.values();
    /**
     * This array holds the RGB values of each item in the same order that gameObjectArray does
     */
    private static final double[][] RGBValuesOfEachItem;

    static {
        RGBValuesOfEachItem = new double[FreightFrenzyStateObject.gameObjectArray.length][3];
        for (int x = 0; x < FreightFrenzyStateObject.gameObjectArray.length; x++) {
            RGBValuesOfEachItem[x] = FreightFrenzyStateObject.getRGBOfObject((FreightFrenzyStateObject.gameObjectArray[x]));
        }
    }

    /**
     * Returns the LED Color that the LED's should be based on the game object provided
     *
     * @param item The game object
     * @return The color the LED's should be based on the game object
     */
    public static RevBlinkinLedDriver.BlinkinPattern getLEDColorFromItem(@Nullable final FreightFrenzyStateObject item) {
        if (item == null) {
            return null;
        }
        switch (item) {
            case BALL:
                return RevBlinkinLedDriver.BlinkinPattern.WHITE;

            case DUCK:
                return RevBlinkinLedDriver.BlinkinPattern.GREEN;

            case CUBESMOOTH:
            case CUBEWAFFLE:
                return RevBlinkinLedDriver.BlinkinPattern.ORANGE;

            default:
                return null;
        }
    }

    /**
     * Returns the RGB of the given game object
     *
     * @param item The item whose RGB to look up
     * @return A reference to the BallRGB, CubeSmoothRGB, CubeWaffleRGB, DuckRGB, or EmptyRGB array that stores the color values in the form R,G,B
     */
    private static double[] getRGBOfObject(final FreightFrenzyStateObject item) {
        switch (item) {
            case BALL:
                return BallRGB;

            case DUCK:
                return DuckRGB;

            case CUBESMOOTH:
                return CubeSmoothRGB;

            case CUBEWAFFLE:
                return CubeWaffleRGB;

            case EMPTY:
                return EmptyRGB;

            default:
                return null;
        }
    }

    /**
     * Determines what game object best matches the color pattern provided
     *
     * @param RGB The color pattern in the form of RGB
     * @return Returns what game object the RGB best matches
     */
    public static FreightFrenzyStateObject identify(double[] RGB) {

        final int numberOfGameElements = FreightFrenzyStateObject.gameObjectArray.length;

        final double[] differences = new double[numberOfGameElements];

        for (int x = 0; x < numberOfGameElements; x++) {
            differences[x] = getDifferenceOfColor(RGB, RGBValuesOfEachItem[x]);
        }


        double smallestValue = Double.MAX_VALUE;
        int index = -1;

        for (int i = 0; i < numberOfGameElements; i++) {
            if (differences[i] < smallestValue) {
                smallestValue = differences[i];
                index = i;
            }
        }

        return FreightFrenzyStateObject.gameObjectArray[index];
    }

    /**
     * It treats color as 3D space and returns the distance between the two points
     *
     * @param sight  The first set of RGB values
     * @param object The second set of RGB values
     * @return The distance between the two, smaller is closer
     */
    private static double getDifferenceOfColor(final double[] sight, final double[] object) {
        final double rDifference = Math.abs(sight[0] - object[0]);
        final double gDifference = Math.abs(sight[1] - object[1]);
        final double bDiffference = Math.abs(sight[2] - object[2]);
        // this calculates the 3D distance between colors
        return Math.sqrt(Math.pow(Math.sqrt(Math.pow(rDifference, 2) + Math.pow(gDifference, 2)), 2) + Math.pow(bDiffference, 2));
    }

    public FreightFrenzyGameObject toFFGO() {
        switch (this) {
            case BALL:
                return FreightFrenzyGameObject.BALL;
            case DUCK:
                return FreightFrenzyGameObject.DUCK;
            case EMPTY:
                return FreightFrenzyGameObject.EMPTY;
            case CUBESMOOTH:
                return FreightFrenzyGameObject.CUBESMOOTH;

            case CUBEWAFFLE:
                return FreightFrenzyGameObject.CUBEWAFFLE;
        }

        throw new NotImplementedError("This method must convert all members of " + FreightFrenzyGameObject.class);
    }


}