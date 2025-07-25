package org.firstinspires.ftc.teamcode.drivetrain

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

fun Pose(pose2d: Pose2D) = Pose(pose2d.getX(DistanceUnit.INCH), pose2d.getY(DistanceUnit.INCH), pose2d.getHeading(AngleUnit.RADIANS))
data class Pose(var x: Double, var y: Double, var heading: Double) {
    operator fun plus(v: Pose): Pose{
        return Pose(x + v.x, y + v.y, heading+v.heading)
    }

    operator fun minus(v: Pose): Pose{
        return Pose(x - v.x, y - v.y, heading - v.heading)
    }

    operator fun times(v: Number): Pose{
        return Pose(x * v.toDouble(), y * v.toDouble(), heading * v.toDouble())
    }

    operator fun unaryMinus(): Pose{
        return Pose(-x, -y, -heading)
    }
};


data class Vector private constructor(val angle: Double, val length: Double){
    val x get() = length * sin(angle)
    val y get() = length * cos(angle)

    companion object {
        fun fromCartesian(x: Double, y: Double) = Vector(atan2(y, x), sqrt(x*x + y*x))
        fun fromPolar(angle: Double,  length: Double) = Vector(angle, length)
        fun fromPose(pose: Pose) = fromCartesian(pose.x, pose.y)
    }

    fun norm(): Vector = Vector(angle, 1.0);

    operator fun times(x: Number): Vector {
        return Vector(angle, length * x.toDouble());
    }
    operator fun plus(v: Vector): Vector {
        return fromCartesian(x+v.x, y+v.y);
    }
    operator fun minus(v: Vector): Vector {
        return fromCartesian(x-v.x, y-v.y);
    }
    fun rotated(x: Number): Vector {
        return Vector(angle + x.toDouble(), length);
    }
}