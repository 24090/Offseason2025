package org.firstinspires.ftc.teamcode.paths

import BezierCurve
import CubicHermiteSpline
import org.firstinspires.ftc.teamcode.drivetrain.Pose
import kotlin.math.floor

interface Path {
    fun getClosestT(p: Pose): Double
    fun getPathAccel(t: Double): Pose
    fun getPathVel(t: Double): Pose
    fun getPose(t: Double): Pose
}

/**
 * Creates a path based on hermite splines. Note that "velocity" here only means the shape of the path.
 * @param controlPoints a list of control points with the following pattern: position0, velocity0, position1, velocity1 ... positionN
 */
class HermitePath(vararg controlPoints: Pose): Path {
    lateinit var controlPoints: Array<Pose>;
    init {
        this.controlPoints = when {
            (controlPoints.size == 1) -> arrayOf(controlPoints[0],  Pose(0.0,0.0,0.0), controlPoints[0],  Pose(0.0,0.0,0.0))
            (controlPoints.size == 2) -> arrayOf(controlPoints[0],  controlPoints[1], controlPoints[0],  Pose(0.0,0.0,0.0))
            (controlPoints.size%2 == 1) -> arrayOf(controlPoints, Pose(0.0,0.0,0.0))
            else -> controlPoints
        } as Array<Pose>
    }

    private fun <R> getSplineProperty(t: Double, f: BezierCurve.(Double) -> (R)): R {
        assert(0 <= t && t <= 1)
        var tAdjusted = t * (controlPoints.size/2 - 1/2)
        val n = floor(tAdjusted).toInt()
        return CubicHermiteSpline(
            controlPoints[n],
            controlPoints[n + 1],
            controlPoints[n + 2],
            controlPoints[n + 3]
        ).f(t)
    }

    override fun getClosestT(p: Pose): Double {
        TODO("Not yet implemented")
    }

    override fun getPose(t: Double) = getSplineProperty(t, BezierCurve::getPose)
    override fun getPathVel(t: Double) = getSplineProperty(t, BezierCurve::getVelocity)
    override fun getPathAccel(t: Double) = getSplineProperty(t, BezierCurve::getAcceleration)
}