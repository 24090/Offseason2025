package org.firstinspires.ftc.teamcode.drivetrain

import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotor
import org.firstinspires.ftc.teamcode.controlsystems.BangBang1D
import org.firstinspires.ftc.teamcode.controlsystems.BangBang2D

class Drive(hwMap: HardwareMap) {
    private val localizer = Localizer(hwMap)

    var targetPose = Pose(0.0, 0.0, 0.0)

    private val flMotor: CachingDcMotor = hwMap.dcMotor.get("flMotor") as CachingDcMotor;
    private val frMotor: CachingDcMotor = hwMap.dcMotor.get("frMotor") as CachingDcMotor;
    private val blMotor: CachingDcMotor = hwMap.dcMotor.get("blMotor") as CachingDcMotor;
    private val brMotor: CachingDcMotor = hwMap.dcMotor.get("bRMotor") as CachingDcMotor;

    private val translationalController: BangBang2D = BangBang2D(
        getError = {Vector.fromPose(localizer.fieldPoseToRelative(targetPose))},
        setOutput = { v ->
            drive = v.x
            strafe = v.y
        },
        getK = { 0.2 },
    );
    private val rotationalController: BangBang1D = BangBang1D(
        getError = {localizer.fieldPoseToRelative(targetPose).heading},
        setOutput = { v ->
            turn = v
        },
        getK = { 0.2 },
    );

    var drive = 0.0;
    var strafe = 0.0;
    var turn = 0.0;


    private fun update(){
        translationalController.update()
        rotationalController.update()
        flMotor.power = drive + strafe + turn
        frMotor.power = drive - strafe + turn
        blMotor.power = drive - strafe - turn
        brMotor.power = drive + strafe - turn
    }

}