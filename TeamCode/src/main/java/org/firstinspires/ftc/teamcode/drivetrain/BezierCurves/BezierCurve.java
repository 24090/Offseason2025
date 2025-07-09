package org.firstinspires.ftc.teamcode.drivetrain.BezierCurves;

import android.util.Pair;

import java.util.ArrayList;

public class BezierCurve {
    ArrayList<Pair<Double,Double>> interpolateList = new ArrayList<Pair<Double,Double>>();
    public Pair<Double,Double> interpolate(double t, Pair<Double,Double> point1, Pair<Double,Double> point2){
        double parameter;
        parameter = Math.max(Math.min(1,t),0);
        double xCoord =point1.first*(1-parameter)+point2.first*t;
        double yCoord =point1.second*(1-parameter)+point2.second*t;
        return new Pair<Double,Double>(xCoord,yCoord);
    }
    public Pair<Double,Double> getPointOnBezierFromT(double t,ArrayList<Pair<Double,Double>> pointList){
        ArrayList<Pair<Double,Double>> runningInterpolationList = new ArrayList<Pair<Double,Double>>();
        interpolateList = pointList;
        for (int i = 0; i < pointList.size()-1 ; i++){
            runningInterpolationList.clear();
            for (int j =0; j < pointList.size() - i - 1; i++){
                runningInterpolationList.add(
                        interpolate(t,interpolateList.get(j), interpolateList.get(j+1))
                );
            }
            interpolateList.clear();
            interpolateList = runningInterpolationList;
        }
        return interpolateList.get(0);
    }
}
