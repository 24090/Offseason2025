package org.firstinspires.ftc.teamcode.controlsystems;

public class SquID {
    double kp;
    double kd;
    double ki;
    double currentPos;
    double targetPos;
    double error;
    public double calculatePower(double targetPos, double currentPos){
        this.currentPos=currentPos;
        this.targetPos=targetPos;
        this.error=this.currentPos - this.targetPos;
        return kp*Math.sqrt(Math.abs(error))*Math.signum(error);
    }

}
