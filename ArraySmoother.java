package org.firstinspires.ftc.teamcode;

public class ArraySmoother {
    private double[] values;
    private double lastReturnedValue;
    public ArraySmoother(int length) {
        this.values = new double[length];
    }

    public double smooth(double value){

        if (values.length == 1) {
            values[0] = value;
            return value;
        }

        double tempAverage = value;

        // Shift values and start summing for average calculation, except for the last one.
        for (int i = 0; i < values.length - 1; i++){
            // Shift values over one position,
            values[i] = values[i+1];
            tempAverage += values[i];
        }

        // Add newValue into array.
        values[values.length - 1] = value;

        lastReturnedValue = (tempAverage / values.length);
        return lastReturnedValue;
    }

    public double getSmoothedValue() {
        return lastReturnedValue;
    }
}
