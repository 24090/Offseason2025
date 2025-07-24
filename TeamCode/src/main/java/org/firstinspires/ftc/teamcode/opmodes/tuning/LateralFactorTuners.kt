package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drivetrain.Drive
import org.firstinspires.ftc.teamcode.drivetrain.Pose
import org.firstinspires.ftc.teamcode.drivetrain.Vector
import org.firstinspires.ftc.teamcode.util.BulkReads
import kotlin.math.max

@Disabled
abstract class MaxSpeedTuner: LinearOpMode () {
    abstract val drive: Double;
    abstract val strafe: Double;
    abstract val turn: Double;
    private var maxSpeed: Double = 0.0;
    override fun runOpMode() {
        val drivetrain = Drive(hardwareMap);
        val bulkReads = BulkReads(hardwareMap);
        val speeds = ArrayList<Double>()
        drivetrain.drive = drive;
        drivetrain.strafe = strafe;
        drivetrain.turn = turn;

        waitForStart();
        drivetrain.localizer.pose = Pose(0.0,0.0,0.0);
        drivetrain.setMotorPowers();

        while (opModeIsActive()){
            bulkReads.update()
            speeds.add(Vector.fromPose(drivetrain.localizer.poseVel).length)
        };

        telemetry.addData("Avg Speed", speeds.average())
        telemetry.update()
    }
}

@TeleOp(group = "Drive")
class StrafeSpeedTuner: MaxSpeedTuner() {
    override val drive: Double = 0.0;
    override val strafe: Double = 1.0;
    override val turn: Double = 0.0;
}

@TeleOp(group = "Drive")
class DriveSpeedTuner: MaxSpeedTuner() {
    override val drive: Double = 0.0;
    override val strafe: Double = 1.0;
    override val turn: Double = 0.0;
}