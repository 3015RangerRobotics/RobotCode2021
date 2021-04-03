package lib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LoopingSequentialGroup extends SequentialCommandGroup {
    public LoopingSequentialGroup(int repetitions, Command... commands){
        for(int i = 0; i < repetitions; i++){
            addCommands(commands);
        }
    }
}
