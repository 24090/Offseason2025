package org.firstinspires.ftc.teamcode.controlsystems

import org.firstinspires.ftc.teamcode.drivetrain.Vector
import kotlin.math.sign

class BangBang1D(
    val getError: () -> Double, val setOutput: (v: Double) -> Unit, val getK: () -> Double
): ControlSystem()
{
    override fun update() {
        setOutput(getK() * getError().sign)
    }
}

class BangBang2D(
    val getError: () -> Vector, val setOutput: (Vector) -> Unit, val getK: () -> Double
): ControlSystem()
{
    override fun update() {
        setOutput(getError().norm() * getK())
    }
}