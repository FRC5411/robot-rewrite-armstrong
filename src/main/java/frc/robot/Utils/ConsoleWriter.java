
package frc.robot.Utils;

public class ConsoleWriter {

    private static final boolean isVS = false;

    /**
     * Basic class to print messages with colour. Useful for
     * printing hand written errors or making telemetry data
     * more noticeable in console. Be sure to modify the 
     * isVS boolean if you are using VSCode or not.
     */
    public ConsoleWriter() {}

    /**
     * Prints an error message in red text to console
     * 
     * @param x Error to print to console
     * @param space Class that this is being called from
     */
    public static void printError(String x, String space) {
        if (isVS) {System.out.println("\033[31mERROR:\033[0m" + spacePrefix(space) + "\033[31m " + x + "\033[0m"); }
        else { System.out.println("ERROR: " + spacePrefix(space) + " " + x); }
    }

    /**
     * Prints a warning message in red text to console
     * 
     * @param x Warning to print to console
     * @param space Class that this is being called from
     */
    public static void printWarning(String x, String space) {
        if (isVS) {System.out.println("\033[31mWARNING:\033[0m" + spacePrefix(space) + "\033[31m " + x + "\033[0m"); }
        else { System.out.println("WARNING: " + spacePrefix(space) + " " + x); }
    }

    private static String spacePrefix(String space) {
        return space + "/ ";
    }
}
