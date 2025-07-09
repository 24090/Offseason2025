package org.firstinspires.ftc.teamcode.controlsystems
import org.firstinspires.ftc.teamcode.drivetrain.Vector
import kotlin.math.sign

fun bangBang(error: Vector, k: Double): Vector{
    return error.norm() * k
}

fun bangBang(error: Double, k: Double): Double{
    return k * error.sign
}