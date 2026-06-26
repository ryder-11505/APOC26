package org.firstinspires.ftc.teamcode.config.staticData;

import com.pedropathing.geometry.Pose;

/**
 * Simple static field serving as a storage medium for the bot's pose.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 */
public class PoseStorage {
    public static Pose currentPose = new Pose(0, 0, 0);

    public static boolean isRedAlliance = false;
    public static boolean isInit = true;

    public static boolean splitControls = true;

    public static boolean shouldHallucinate = false;
}
