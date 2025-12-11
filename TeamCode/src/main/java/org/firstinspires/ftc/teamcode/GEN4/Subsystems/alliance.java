package org.firstinspires.ftc.teamcode.GEN4.Subsystems;

public class alliance {

    public enum Color {
        RED,
        BLUE
    }



    // Global alliance value for the entire robot code
    private static Color currentAlliance = Color.BLUE;  // default


    // Set alliance
    public static void set(Color alliance) {
        currentAlliance = alliance;
    }

    // Get alliance
    public static Color get() {
        return currentAlliance;
    }

    // Convenience checks
    public static boolean isRed() {
        return currentAlliance == Color.RED;
    }

    public static boolean isBlue() {
        return currentAlliance == Color.BLUE;
    }

}
