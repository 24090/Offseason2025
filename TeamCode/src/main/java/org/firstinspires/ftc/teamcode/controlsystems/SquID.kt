import org.firstinspires.ftc.teamcode.drivetrain.Vector
import kotlin.math.absoluteValue
import kotlin.math.sign
import kotlin.math.sqrt

fun SquID(error: Vector, kSq: Double): Vector {
    return error.norm() * sqrt(error.length * kSq)
}

fun SquID(error: Double, kSq: Double): Double{
    return error.sign * sqrt(error.absoluteValue * kSq)
}