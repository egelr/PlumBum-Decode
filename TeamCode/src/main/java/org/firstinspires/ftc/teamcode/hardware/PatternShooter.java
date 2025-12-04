package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

/**
 * Simplified PatternShooter:
 *  - Takes a desired pattern (e.g. "PPG")
 *  - Reads actual loaded pattern from Sorter
 *  - Computes slot order to shoot (0/1/2)
 *  - Spins shooter up, shoots 3 balls, spins shooter down
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

    // ------------------------------------------------------------
    // MAIN USER FUNCTION
    // ------------------------------------------------------------

    /**
     * Shoot balls so the output colour order matches targetPattern.
     *
     * Example:
     *   targetPattern = "PGP"
     *   loadedPattern = "PPG"
     *
     * Will compute a slot order such as [0,2,1].
     */
    public Action shootPatternMid(String targetPattern) {

        String loaded = sorter.getPatternString(); // e.g. "PGP"
        int[] order = computeOrder(targetPattern, loaded);

        return new SequentialAction(
                shooter.ShooterOn(),  // spin shooter up

                shootOne(order[0]),
                shootOne(order[1]),
                shootOne(order[2]),

                shooter.ShooterOff()  // stop shooter
        );
    }

    // ------------------------------------------------------------
    // ONE BALL FIRING SEQUENCE
    // ------------------------------------------------------------
    private Action shootOne(int slot) {
        return new SequentialAction(
                new MoveSorter(slot),
                new SleepAction(1),

                transfer.launch(),
                new SleepAction(1),

                transfer.preset(),
                new SleepAction(1)
        );
    }

    // Moves sorter to the correct slot (0/1/2)
    private class MoveSorter implements Action {
        private final int slot;
        private boolean done = false;

        MoveSorter(int slot) { this.slot = slot; }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!done) {
                sorter.moveSorterToSlot(slot);
                done = true;
            }
            return false; // one-shot action
        }
    }

    // ------------------------------------------------------------
    // PATTERN TRANSFORM LOGIC
    // ------------------------------------------------------------

    /**
     * Computes slot firing order to transform LOADED order → TARGET order.
     *
     * @return int[3] with slot indexes (0,1,2)
     */
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

            // 1) Prefer slot with matching colour
            for (int slot = 0; slot < 3; slot++) {
                if (!used[slot] && loadedPattern.charAt(slot) == desired) {
                    chosen = slot;
                    break;
                }
            }

            // 2) If no matching colour found → choose first unused slot
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
