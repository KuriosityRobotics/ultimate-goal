package org.firstinspires.ftc.teamcode.ultimategoal.util;

public class LinearInterpolation {
    private double[][] data;

    /**
     * Create a LinearInterpolation from a 2D array of data. This first element of each row should
     * be the input of the function, while the rest of each row can be any desired outputs.
     *
     * @param data
     */
    public LinearInterpolation(double[][] data) {
        this.data = data;
    }

    public double[] interpolate(double input) {
        // find the last datapoint w/ distance smaller than the current distance
        int indexBelowInput = data.length - 2; // lower bound
        for (int i = 0; i < data.length - 1; i++) {
            if (data[i + 1][0] > input) {
                indexBelowInput = i;
                break;
            }
        }

        double deltaD = input - data[indexBelowInput][0];
        double deltaInput = data[indexBelowInput + 1][0] - data[indexBelowInput][0];

        double[] output = new double[data[indexBelowInput].length - 1];

        // for each output that we need to linearly interpolate
        for (int i = 1; i < data[indexBelowInput].length; i++) {
            double deltaOutput = data[indexBelowInput + 1][i] - data[indexBelowInput][i];

            double slope = deltaOutput / deltaInput;

            output[i-1] = (slope * deltaD) + data[indexBelowInput][i];
        }

        return output;
    }
}
