package org.firstinspires.ftc.teamcode.controlsystems

import org.firstinspires.ftc.teamcode.drivetrain.Vector
import kotlin.math.absoluteValue
import kotlin.math.sign
import kotlin.math.sqrt

fun PD(error: Double, dError: Double, kP: Double, kD: Double): Double{
    return error * kP + dError * kD
}

fun PD(error: Vector, dError: Vector, kP: Double, kD: Double): Vector{
    return error * kP + dError * kD
}

fun PDL(error: Double, dError: Double, kP: Double, kD: Double, kL: Double): Double{
    return PD(error, dError, kP, kD) + BangBang(error, kL)
}

fun PDL(error: Vector, dError: Vector, kP: Double, kD: Double, kL: Double): Vector{
    return PD(error, dError, kP, kD) + BangBang(error, kL)
}

fun SquID(error: Vector, kSq: Double): Vector {
    return error.norm() * sqrt(error.length * kSq)
}

fun SquID(error: Double, kSq: Double): Double{
    return error.sign * sqrt(error.absoluteValue * kSq)
}

fun BangBang(error: Vector, k: Double): Vector{
    return error.norm() * k
}

fun BangBang(error: Double, k: Double): Double{
    return k * error.sign
}