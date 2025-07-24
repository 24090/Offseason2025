package org.firstinspires.ftc.teamcode.drivetrain

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotor
import org.firstinspires.ftc.teamcode.controlsystems.BangBang
import org.firstinspires.ftc.teamcode.controlsystems.PDL
import org.firstinspires.ftc.teamcode.controlsystems.SquID

class Drive(hwMap: HardwareMap) {
    val localizer = Localizer(hwMap)
    companion object DriveConstants{
        var lateralFactor = 1.0

        var kSqH = 0.0;
        var kPT = 0.0;
        var kDT = 0.0;
        var kLT = 0.0;
        var threshT = 0.25;
    }

    var targetPose = Pose(0.0, 0.0, 0.0)

    private val flMotor: CachingDcMotor = hwMap.dcMotor.get("flMotor") as CachingDcMotor;
    private val frMotor: CachingDcMotor = hwMap.dcMotor.get("frMotor") as CachingDcMotor;
    private val blMotor: CachingDcMotor = hwMap.dcMotor.get("blMotor") as CachingDcMotor;
    private val brMotor: CachingDcMotor = hwMap.dcMotor.get("bRMotor") as CachingDcMotor;

    var drive = 0.0;
    var strafe = 0.0;
    var turn = 0.0;

    init {
        flMotor.direction = Direction.FORWARD
        frMotor.direction = Direction.FORWARD
        blMotor.direction = Direction.FORWARD
        brMotor.direction = Direction.FORWARD
    }
    fun setMotorPowers(){
        flMotor.power = drive + strafe + turn
        frMotor.power = drive - strafe + turn
        blMotor.power = drive - strafe - turn
        brMotor.power = drive + strafe - turn
    }

    fun update(){
        val error = localizer.fieldPoseToRelative(targetPose)
        val dError = localizer.poseVel

        turn = SquID(error.heading, kSqH)
        val translational = PDL(Vector.fromPose(error), Vector.fromPose(dError), kPT, kDT, kLT)
        drive = translational.x * lateralFactor
        strafe = translational.y
        setMotorPowers()
    }

}