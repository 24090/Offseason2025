package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drivetrain.Drive
import org.firstinspires.ftc.teamcode.drivetrain.Pose

@TeleOp(group = "Drive")
class DriveDirections: LinearOpMode() {
    fun Boolean.toDouble() = if (this) 1.0 else 0.0

    override fun runOpMode() {
        val drive = Drive(hardwareMap)
        waitForStart()
        drive.localizer.pose = Pose(0.0,0.0,0.0);
        while (opModeIsActive()){
            drive.localizer.update()
            telemetry.addLine("Switch directions of backwards motors in Drive.kt")
            telemetry.addLine("X -> FL")
            telemetry.addLine("Y -> FR")
            telemetry.addLine("A -> BL")
            telemetry.addLine("B -> BR")
            telemetry.addLine("Switch directions of backwards encoders in Localizer.kt")
            telemetry.addData("Strafe", drive.localizer.y)
            telemetry.addData("Drive", drive.localizer.x)
            telemetry.addData("Heading", drive.localizer.heading)
            telemetry.update()
            drive.setFlPower(gamepad1.x.toDouble())
            drive.setFrPower(gamepad1.y.toDouble())
            drive.setBlPower(gamepad1.a.toDouble())
            drive.setBrPower(gamepad1.b.toDouble())
        }
    }
}
