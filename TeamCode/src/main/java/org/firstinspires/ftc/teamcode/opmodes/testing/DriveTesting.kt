package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drivetrain.Drive
import org.firstinspires.ftc.teamcode.drivetrain.Pose
import org.firstinspires.ftc.teamcode.util.BulkReads
import kotlin.math.PI
import kotlin.math.absoluteValue

@TeleOp(group = "drive")
class MoveTest: LinearOpMode() {
    override fun runOpMode() {
        val drive = Drive(hardwareMap)
        val bulkReads = BulkReads(hardwareMap)

        waitForStart()
        drive.localizer.pose = Pose(0.0,0.0,0.0)
        while (opModeIsActive()){
            bulkReads.update()
            drive.update()
            if (gamepad1.yWasReleased()) {
                drive.targetPose.x += 6
            }
            if (gamepad1.aWasReleased()) {
                drive.targetPose.x -= 12
            }
            if (gamepad1.xWasReleased()) {
                drive.targetPose.y += 12
            }
            if (gamepad1.bWasReleased()) {
                drive.targetPose.y -= 12
            }
            if (gamepad1.leftBumperWasReleased()) {
                drive.targetPose.heading += PI/4
            }
            if (gamepad1.rightBumperWasReleased()) {
                drive.targetPose.heading -= PI/4
            }
            telemetry.addData("error", drive.error)
            telemetry.addData("dError", drive.dError)
            telemetry.addData("drive", drive.drive)
            telemetry.addData("strafe", drive.strafe)
            telemetry.addData("turn", drive.turn)
            telemetry.update()
        }
    }

}