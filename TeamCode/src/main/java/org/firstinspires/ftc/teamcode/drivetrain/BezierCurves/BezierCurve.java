package org.firstinspires.ftc.teamcode.drivetrain.BezierCurves;


import org.firstinspires.ftc.teamcode.drivetrain.Pose;
import java.util.ArrayList;

public class BezierCurve {
    ArrayList<Pose> interpolateList = new ArrayList<>();
    public Pose interpolate(Double t, Pose poseA, Pose poseB){
        double parameter;
        parameter = Math.max(Math.min(1,t),0);
        return poseA.times(1-parameter).plus(poseB.times(parameter));
    }

    public Pose getPointOnBezierFromT(double t, ArrayList<Pose> pointList){
        ArrayList<Pose> runningInterpolationList = new ArrayList<Pose>();
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
