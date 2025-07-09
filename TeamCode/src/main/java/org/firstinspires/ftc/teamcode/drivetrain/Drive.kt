package org.firstinspires.ftc.teamcode.drivetrain

import SquID
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotor
import org.firstinspires.ftc.teamcode.controlsystems.bangBang

class Drive(hwMap: HardwareMap) {
    private val localizer = Localizer(hwMap)

    var targetPose = Pose(0.0, 0.0, 0.0)

    val kHeading = 0.0;
    val kTranslational = 0.0;

    private val flMotor: CachingDcMotor = hwMap.dcMotor.get("flMotor") as CachingDcMotor;
    private val frMotor: CachingDcMotor = hwMap.dcMotor.get("frMotor") as CachingDcMotor;
    private val blMotor: CachingDcMotor = hwMap.dcMotor.get("blMotor") as CachingDcMotor;
    private val brMotor: CachingDcMotor = hwMap.dcMotor.get("bRMotor") as CachingDcMotor;

    var drive = 0.0;
    var strafe = 0.0;
    var turn = 0.0;


    private fun update(){
        val error = localizer.fieldPoseToRelative(targetPose)
        turn = SquID(error.heading, kHeading)
        val translational = bangBang(Vector.fromPose(error), kTranslational)
        drive = translational.x
        strafe = translational.y

        flMotor.power = drive + strafe + turn
        frMotor.power = drive - strafe + turn
        blMotor.power = drive - strafe - turn
        brMotor.power = drive + strafe - turn
    }

}