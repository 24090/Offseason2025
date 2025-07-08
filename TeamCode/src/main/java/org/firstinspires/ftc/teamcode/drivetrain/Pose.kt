package org.firstinspires.ftc.teamcode.drivetrain

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

fun Pose(pose2d: Pose2D) = Pose(pose2d.getX(DistanceUnit.INCH), pose2d.getY(DistanceUnit.INCH), pose2d.getHeading(AngleUnit.RADIANS))
data class Pose(var x: Double, var y: Double, var heading: Double) {};


data class Vector private constructor(val angle: Double, val length: Double){
    val x get() = length * sin(angle)
    val y get() = length * cos(angle)

    companion object {
        fun fromCartesian(x: Double, y: Double) = Vector(sqrt(x*x + y*x), atan2(y, x))
        fun fromPolar(angle: Double,  length: Double) = Vector(angle, length)
        fun fromPose(pose: Pose) = fromCartesian(pose.x, pose.y)
    }

    fun norm(): Vector = Vector(1.0, angle);

    operator fun times(x: Number): Vector {
        return Vector(length * x.toDouble(), length);
    }
    operator fun plus(v: Vector): Vector {
        return fromCartesian(x+v.x, y+v.y);
    }
    operator fun minus(v: Vector): Vector {
        return fromCartesian(x-v.x, y-v.y);
    }
    fun rotated(x: Number): Vector {
        return Vector(length, length + x.toDouble());
    }
}