package org.firstinspires.ftc.teamcode.drivetrain

import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.controlsystems.BangBang
import org.firstinspires.ftc.teamcode.controlsystems.PD
import org.firstinspires.ftc.teamcode.controlsystems.PDL
import org.firstinspires.ftc.teamcode.controlsystems.SquID

@Configurable
class Drive(hwMap: HardwareMap) {
    val localizer = Localizer(hwMap)

    companion object DriveConstants{
        @JvmField var lateralFactor = 1.0

        @JvmField var kSqH = 0.0;
        @JvmField var kPT = 0.0;
        @JvmField var kDT = 0.0;
        @JvmField var kLT = 0.0;
        @JvmField var threshT = 0.25;
    }

    var targetPose = Pose(0.0, 0.0, 0.0)
    private val flMotor: CachingDcMotor = CachingDcMotor(hwMap.get(DcMotor::class.java, "flMotor"));
        fun setFlPower(power: Double){flMotor.power = power}
    private val frMotor: CachingDcMotor = CachingDcMotor(hwMap.get(DcMotor::class.java, "frMotor"));
        fun setFrPower(power: Double){frMotor.power = power}
    private val blMotor: CachingDcMotor = CachingDcMotor(hwMap.get(DcMotor::class.java,"blMotor"));
        fun setBlPower(power: Double){blMotor.power = power}
    private val brMotor: CachingDcMotor = CachingDcMotor(hwMap.get(DcMotor::class.java,"brMotor"));
        fun setBrPower(power: Double){brMotor.power = power}

    var drive = 0.0;
    var strafe = 0.0;
    var turn = 0.0;

    init {
        flMotor.direction = Direction.REVERSE
        frMotor.direction = Direction.FORWARD
        blMotor.direction = Direction.REVERSE
        brMotor.direction = Direction.FORWARD
    }
    fun setMotorPowers(){
        flMotor.power = drive - strafe - turn
        frMotor.power = drive + strafe + turn
        blMotor.power = drive + strafe - turn
        brMotor.power = drive - strafe + turn
    }

    fun update(){
        localizer.update()
        val error = localizer.fieldPoseToRelative(targetPose)
        val dError = localizer.poseVel

        turn = SquID(AngleUnit.normalizeRadians(error.heading), kSqH)
        val translational = PDL(Vector.fromPose(error), Vector.fromPose(dError), kPT, kDT, kLT)
        drive = translational.x * lateralFactor
        strafe = translational.y
        setMotorPowers()
    }

}