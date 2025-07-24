package org.firstinspires.ftc.teamcode.drivetrain

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit

class Localizer(hwMap: HardwareMap) {
    private var pinpoint: GoBildaPinpointDriver = hwMap.get(GoBildaPinpointDriver::class.java, "pinpoint")

    init {
        TODO("Encoder Distances, Directions should be set")
    }

    var pose
        get() = Pose(pinpoint.position)
        set(v){ pinpoint.position = Pose2D(DistanceUnit.INCH, v.x, v.y, AngleUnit.RADIANS, v.heading) }

    val x
        get() = pose.x
    val y
        get() =  pose.y
    val heading
        get() = pose.heading

    val poseVel
        get() = Pose(xVel, yVel, headingVel)
    val xVel
        get() = pinpoint.getVelX(DistanceUnit.INCH)
    val yVel
        get() =  pinpoint.getVelY(DistanceUnit.INCH);
    val headingVel
        get() = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

    fun fieldPoseToRelative(fieldPose: Pose): Pose {
        val translation = (Vector.fromCartesian(fieldPose.x, fieldPose.y) - Vector.fromCartesian(x, y)).rotated(-heading)
        return Pose(translation.x, translation.y, fieldPose.heading - heading)
    }

    fun relativePoseToField(relativePose: Pose): Pose {
        val translation = Vector.fromCartesian(relativePose.x, relativePose.y).rotated(heading) + Vector.fromCartesian(x, y)
        return Pose(translation.x, translation.y, heading + relativePose.heading)
    }

}