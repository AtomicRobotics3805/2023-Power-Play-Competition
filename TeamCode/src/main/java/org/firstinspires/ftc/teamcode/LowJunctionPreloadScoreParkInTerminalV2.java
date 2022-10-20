package org.firstinspires.ftc.teamcode;

import com.atomicrobotics.cflib.Constants;
import com.atomicrobotics.cflib.driving.MecanumDriveConstants;
import com.atomicrobotics.cflib.driving.drivers.MecanumDrive;
import com.atomicrobotics.cflib.driving.localizers.Localizer;
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryConstants;
import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryLocalizer;
import com.atomicrobotics.cflib.opmodes.AutonomousOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name ="Score Preload in Low Junction & Park in Terminal")
public class LowJunctionPreloadScoreParkInTerminalV2 extends AutonomousOpMode {
    public LowJunctionPreloadScoreParkInTerminalV2() {
        super(
                Constants.Color.BLUE,
                CompetitionTrajectoryFactory.INSTANCE,
                Routines.INSTANCE::getLeftMainRoutine,
                Routines.INSTANCE::getInitializationRoutine,
                new MecanumDrive(
                        (MecanumDriveConstants)CompetitionMecanumDriveConstants.INSTANCE,
                        (Localizer)(new TwoWheelOdometryLocalizer(
                                (TwoWheelOdometryConstants)(new CompetitionOdometryConstants())
                        )),
                        () -> CompetitionTrajectoryFactory.leftStartPose)
        );
    }
}