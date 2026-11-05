package org.firstinspires.ftc.teamcode;

import android.content.Context;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Writes a CSV log to the Control Hub so a run can be checked after the fact instead of
 * only on the Driver Station during the run.
 *
 * WHERE IT LANDS. The file goes in the robot app's own external files folder:
 *
 *     /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/
 *
 * Pull it to a laptop with adb (the Hub must be connected):
 *
 *     adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/ .
 *
 * The Driver Station also shows the exact file name at the end of the run.
 *
 * WHY NOT JUST TELEMETRY. Telemetry is a live screen: it scrolls, it is gone when the
 * OpMode ends, and nobody can read it while the robot is driving. A file keeps every row
 * so the power and rate can be plotted or compared between runs.
 *
 * NEVER BREAK THE ROBOT. Every method swallows I/O errors and does nothing once writing
 * fails. A full disk or a missing folder must not stop an Autonomous.
 */
public final class TeamRunLog {

    private static final String PACKAGE = "com.qualcomm.ftcrobotcontroller";

    private final BufferedWriter writer;
    private final String path;

    private TeamRunLog(BufferedWriter writer, String path) {
        this.writer = writer;
        this.path = path;
    }

    /**
     * Open a new timestamped CSV, truncating nothing (each run gets its own name).
     * Returns a logger that silently does nothing when the folder cannot be opened.
     */
    public static TeamRunLog open(Context context, String prefix) {
        if (context == null) return new TeamRunLog(null, null);

        File dir = context.getExternalFilesDir(null);
        if (dir == null) dir = context.getFilesDir();

        String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
        File file = new File(dir, prefix + "-" + stamp + ".csv");
        try {
            return new TeamRunLog(new BufferedWriter(new FileWriter(file, false)),
                    file.getAbsolutePath());
        } catch (IOException e) {
            return new TeamRunLog(null, null);
        }
    }

    /** True when rows are actually being written. */
    public boolean isOpen() { return writer != null; }

    /** The full path on the Hub, or null when logging is off. */
    public String path() { return path; }

    /** Append one row. Values are converted to text and quoted when they contain a comma. */
    public void row(Object... cells) {
        if (writer == null) return;

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < cells.length; i++) {
            if (i > 0) sb.append(',');
            sb.append(csv(cells[i]));
        }
        sb.append('\n');

        try {
            writer.write(sb.toString());
            writer.flush();   // flush every row so a stop or crash still leaves the data
        } catch (IOException e) {
            // Stop trying: a broken log must not break the robot. Keep the writer so the
            // caller's close() is still harmless.
            try { writer.close(); } catch (IOException ignored) { }
        }
    }

    /** Close the file. Safe to call more than once. */
    public void close() {
        if (writer == null) return;
        try {
            writer.close();
        } catch (IOException ignored) {
            // Nothing useful to do at the end of a run.
        }
    }

    /** The directory adb should pull from, for telemetry. */
    public static String folderOnHub() {
        return "/sdcard/Android/data/" + PACKAGE + "/files/";
    }

    private static String csv(Object value) {
        if (value == null) return "";
        String s = String.valueOf(value);
        if (s.indexOf(',') < 0 && s.indexOf('"') < 0
                && s.indexOf('\n') < 0 && s.indexOf('\r') < 0) {
            return s;
        }
        return '"' + s.replace("\"", "\"\"") + '"';
    }
}
