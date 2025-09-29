package frc.robot.commands.auto.autoAlign;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.Arrays;
import java.util.List;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;

public class ComboAutoAlign extends Command {

    List<Integer> validTags = Arrays.asList(
        17, 18, 19, 20, 21, 22, //blue
        6, 7, 8, 9, 10, 11 //red
    );

    final Command globalAutoAlign;
    final Command localAutoAlign;

    public ComboAutoAlign(Command globalAutoAlign, Command localAutoAlign) {
        this.globalAutoAlign = globalAutoAlign;
        this.localAutoAlign = localAutoAlign;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        boolean targetVisible = LimelightHelpers.getTV(Constants.LIMELIGHT_4_NAME);
        double tagId = LimelightHelpers.getFiducialID(Constants.LIMELIGHT_4_NAME);

        if(targetVisible && validTags.contains((int)tagId)) { //can see valid april tag
            globalAutoAlign.cancel();
            localAutoAlign.schedule();
        } else {
            localAutoAlign.cancel();
            globalAutoAlign.schedule();
        }
    }

    @Override
    public boolean isFinished() {return true;}

    @Override
    public void end(boolean interrupted) {}
}
