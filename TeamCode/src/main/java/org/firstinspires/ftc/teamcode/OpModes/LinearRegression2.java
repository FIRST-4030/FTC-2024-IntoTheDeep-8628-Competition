package org.firstinspires.ftc.teamcode.OpModes;

public class LinearRegression2 {
    private double[] xValues;
    private double[] yValues;
    private int n;
    private double slope;
    private double intercept;

    public LinearRegression2(double[] xValues, double[] yValues){
        if (xValues.length != yValues.length) {
            throw new IllegalArgumentException("Array lengths must be equal");
        }
        this.xValues = xValues;
        this.yValues = yValues;
        this.n = xValues.length;
        calculateCoefficients();
    }
    private void calculateCoefficients(){
        double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumX2=0.0;
        for (int i = 0; i < n; i++){
            sumX += xValues[i];
            sumY += yValues[i];
            sumXY += xValues[i] * yValues[i];
            sumX2 += xValues[i] * xValues[i];
        }
        slope = (n * sumXY-sumX*sumY) / (n*sumX2-sumX*sumX);
        intercept = (sumY-slope * sumX) / n;
    }
    public double predict(double x){
        return slope * x + intercept;
    }

    public double getSlope() {
        return slope;
    }

    public double getIntercept() {
        return intercept;
    }
}
