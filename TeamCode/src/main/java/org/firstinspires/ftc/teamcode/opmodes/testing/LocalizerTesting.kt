package org.firstinspires.ftc.teamcode.opmodes.testing

import com.bylazar.ftcontrol.panels.Panels
import com.bylazar.ftcontrol.panels.integration.TelemetryManager
import com.bylazar.ftcontrol.panels.json.Canvas
import com.bylazar.ftcontrol.panels.json.CanvasPreset
import com.bylazar.ftcontrol.panels.json.CanvasPresets
import com.bylazar.ftcontrol.panels.json.Circle
import com.bylazar.ftcontrol.panels.json.Line
import com.bylazar.ftcontrol.panels.json.Look
import com.bylazar.ftcontrol.panels.json.Point
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drivetrain.Drive
import org.firstinspires.ftc.teamcode.drivetrain.Pose
import org.firstinspires.ftc.teamcode.drivetrain.Vector
import org.firstinspires.ftc.teamcode.util.BulkReads


@TeleOp(group = "Drive")
class LocalizerTesting: LinearOpMode() {
    override fun runOpMode() {
        val drive = Drive(hardwareMap)
        val panelTelemetry = Panels.getTelemetry()
        val bulkReads = BulkReads(hardwareMap)
        waitForStart()
        drive.localizer.pose = Pose(0.0,0.0,0.0)
        while (opModeIsActive()){
            bulkReads.update()
            drive.localizer.update()
            showPose(drive.localizer.pose, panelTelemetry)
            panelTelemetry.debug(
                "x: ${drive.localizer.x}",
                "y: ${drive.localizer.y}",
                "heading: ${drive.localizer.heading}"
            )
            panelTelemetry.update(telemetry)
        }
    }
}

fun showPose(
    pose: Pose,
    panelTelemetry: TelemetryManager,
    look: Look = Look("", "blue", 1.0, 1.0)
){
    val headingVector = Vector.fromPolar(pose.heading, 5.0)
    panelTelemetry.debug(
        Circle(Point(pose.x, pose.y), 10.0).withLook(look),
        Line(
            Point(pose.x, pose.y),
            Point(pose.x + headingVector.x, pose.y + headingVector.y)
        ).withLook(look)
    )
}