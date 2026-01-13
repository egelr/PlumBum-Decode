package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

/**
 * Simplified PatternShooter:
 *  - Takes a desired pattern (e.g. "PPG")
 *  - Uses the loaded pattern string you pass in
 *  - Computes slot order to shoot (0/1/2)
 *  - Shooter on -> shoot 3 balls
 */
public class PatternShooter {

    private final Shooter shooter;
    private final Transfer transfer;
    private final Sorter sorter;

    public PatternShooter(Shooter shooter, Transfer transfer, Sorter sorter) {
        this.shooter = shooter;
        this.transfer = transfer;
        this.sorter = sorter;
    }

    public Action shootPatternMid(String targetPattern, String loaded) {
        int[] order = computeOrder(targetPattern, loaded);

        return new SequentialAction(
                //shooter.ShooterOn(),
                shootOne(order[0]),
                shootOne(order[1]),
                shootOne(order[2])
                // shooter.ShooterOff()
        );
    }

    private Action shootOne(int slot) {
        return new SequentialAction(
                new ParallelAction(
                        shooter.ShooterOn(),
                        sorter.moveOuttakeSlotAndWait(slot)),// OUTTAKE voltage targets
                transfer.launch(),                   // kicker UP voltage target
                transfer.preset()                    // kicker DOWN voltage target
        );
    }
    public Action shootPatternFar(String targetPattern, String loaded) {
        int[] order = computeOrder(targetPattern, loaded);

        return new SequentialAction(
                //shooter.ShooterOn(),
                shootOneFar(order[0]),
                shootOneFar(order[1]),
                shootOneFar(order[2])
                // shooter.ShooterOff()
        );
    }

    private Action shootOneFar(int slot) {
        return new SequentialAction(
                new ParallelAction(
                        shooter.ShooterOnFar(),
                        sorter.moveOuttakeSlotAndWait(slot)),// OUTTAKE voltage targets
                transfer.launch(),                   // kicker UP voltage target
                transfer.preset()                    // kicker DOWN voltage target
        );
    }

    private int[] computeOrder(String targetPattern, String loadedPattern) {

        if (targetPattern == null) targetPattern = "PPG";
        if (loadedPattern == null) loadedPattern = "PPG";

        targetPattern = targetPattern.toUpperCase();
        loadedPattern = loadedPattern.toUpperCase();

        int[] order = new int[3];
        boolean[] used = new boolean[3];

        for (int i = 0; i < 3; i++) {

            char desired = targetPattern.charAt(i);
            int chosen = -1;

            for (int slot = 0; slot < 3; slot++) {
                if (!used[slot] && loadedPattern.charAt(slot) == desired) {
                    chosen = slot;
                    break;
                }
            }

            if (chosen == -1) {
                for (int slot = 0; slot < 3; slot++) {
                    if (!used[slot]) {
                        chosen = slot;
                        break;
                    }
                }
            }

            used[chosen] = true;
            order[i] = chosen;
        }

        return order;
    }
}
