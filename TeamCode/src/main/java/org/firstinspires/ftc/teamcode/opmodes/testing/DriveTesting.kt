package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drivetrain.Drive
import org.firstinspires.ftc.teamcode.drivetrain.Pose
import org.firstinspires.ftc.teamcode.util.BulkReads
import kotlin.math.absoluteValue

@Autonomous(group = "drive")
class SquareTest: LinearOpMode() {
    override fun runOpMode() {
        val drive = Drive(hardwareMap)
        val bulkReads = BulkReads(hardwareMap)

        waitForStart()
        var state = 0;
        drive.localizer.pose = Pose(0.0,0.0,0.0)
        while (opModeIsActive()){
            bulkReads.update()
            drive.update()

            when {
                state == 0 -> {
                    drive.targetPose = Pose(12.0, 12.0, 0.0)
                    if (drive.localizer.pose.inSquare(drive.targetPose, 1.5,1.5,0.04)) {
                        state = 1
                    }
                }
            }

            telemetry.addData("heading", drive.localizer.heading)
            telemetry.addData("drive", drive.drive)
            telemetry.addData("strafe", drive.strafe)
            telemetry.addData("turn", drive.turn)
            telemetry.update()
        }
    }

}