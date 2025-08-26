import org.firstinspires.ftc.teamcode.choose
import org.firstinspires.ftc.teamcode.drivetrain.Pose
import kotlin.math.pow

// math utility functions

class BezierCurve(vararg val controlPoints: Pose) {
    val n = controlPoints.size

    /**
     * Gets the rotational and translational acceleration at `t` as a `Pose`
     */
    fun getAcceleration(t: Double): Pose = 
        controlPoints.withIndex().map { (i, controlPoint) ->
            controlPoint * ((n choose i) * (
                i * (
                    1    *(1 - t).pow(i-1)   * (n-i)   * t.pow(n-i-1) +
                    (i-1)*(1 - t).pow(i-2)   * 1       * t.pow(n-i)
                ) + 
                (n-i) * (
                    i    *(1 - t).pow(i-1)   * 1       * t.pow(n-i-1) +
                    1    *(1 - t).pow(i)     * (n-i-1) * t.pow(n-i-2)
                )
            ))
        }.reduce(Pose::plus)

    /**
     * Gets the rotational and translational velocity at `t` as a `Pose`
     */
    fun getVelocity(t: Double): Pose =
        controlPoints.withIndex().map { (i, controlPoint) ->
            controlPoint * ((n choose i) * (
                i*(1 - t).pow(i - 1) *       t.pow(n-i)   +
                  (1 - t).pow(i)     * (n-i)*t.pow(n-i-1)
            ))
        }.reduce(Pose::plus)

    /**
     * Gets the heading and position at `t` as a `Pose`
     */
    fun getPose(t: Double): Pose =
        controlPoints.withIndex().map { (i, controlPoint) ->
            controlPoint * (
                (n choose i) *
                (1 - t).pow(i) * t.pow(n-i)
            )
        }.reduce(Pose::plus)

    // NOTE: only accounts for translation, not rotation
    fun getClosestPoint(p: Pose): Pose {
        TODO()
    }
}

fun CubicHermiteSpline(initialPose: Pose, initialVelocity:Pose, finalPose: Pose, finalVelocity: Pose) = BezierCurve(
    initialPose,
    initialPose + initialVelocity/3,
    finalPose - finalVelocity/3,
    finalPose
)