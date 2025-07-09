package org.firstinspires.ftc.teamcode.drivetrain

import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotor
import org.firstinspires.ftc.teamcode.controlsystems.BangBang
import org.firstinspires.ftc.teamcode.controlsystems.PDL
import org.firstinspires.ftc.teamcode.controlsystems.SquID

class Drive(hwMap: HardwareMap) {
    private val localizer = Localizer(hwMap)

    var targetPose = Pose(0.0, 0.0, 0.0)

    val kSqH = 0.0;
    val kPT = 0.0;
    val kDT = 0.0;
    val kLT = 0.0;
    val threshT = 0.25;

    private val flMotor: CachingDcMotor = hwMap.dcMotor.get("flMotor") as CachingDcMotor;
    private val frMotor: CachingDcMotor = hwMap.dcMotor.get("frMotor") as CachingDcMotor;
    private val blMotor: CachingDcMotor = hwMap.dcMotor.get("blMotor") as CachingDcMotor;
    private val brMotor: CachingDcMotor = hwMap.dcMotor.get("bRMotor") as CachingDcMotor;

    var drive = 0.0;
    var strafe = 0.0;
    var turn = 0.0;


    private fun update(){
        val error = localizer.fieldPoseToRelative(targetPose)
        val dError = localizer.poseVel

        turn = SquID(error.heading, kSqH)
        val translational = PDL(Vector.fromPose(error), Vector.fromPose(dError), kPT, kDT, kLT)
        drive = translational.x
        strafe = translational.y

        flMotor.power = drive + strafe + turn
        frMotor.power = drive - strafe + turn
        blMotor.power = drive - strafe - turn
        brMotor.power = drive + strafe - turn
    }

}