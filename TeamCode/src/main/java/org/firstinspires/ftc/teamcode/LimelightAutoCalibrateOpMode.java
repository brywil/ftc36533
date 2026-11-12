package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "4b. Limelight Auto Calibrate (AprilTags)", group = "Bringup")
public class LimelightAutoCalibrateOpMode extends LinearOpMode {

    private static final String LIMELIGHT_NAME = "limelight";
    private static final String LIMELIGHT_HOST = "172.29.0.1";
    private static final int LIMELIGHT_PORT = 5807;
    private static final int FIDUCIAL_PIPELINE = 1;

    private static final double[] EXPOSURES = { 100, 200, 400, 600, 800, 1000, 1200 };
    private static final double[] GAINS = { 20, 30, 40, 50, 60, 70, 80 };

    private static final long SETTLE_MS = 800;
    private static final long SAMPLE_MS = 1000;
    private static final long STALE_MS = 200;
    private static final int MIN_FRAMES = 5;
    private static final int MIN_DETECTED_FRAMES = 3;
    private static final double MIN_DETECT_FRACTION = 0.5;

    private static final String BASE = "http://" + LIMELIGHT_HOST + ":" + LIMELIGHT_PORT;

    private static final class Result {
        final double exposure;
        final double gain;
        final int frames;
        final int detectedFrames;
        final int tags;

        Result(double exposure, double gain, int frames, int detectedFrames, int tags) {
            this.exposure = exposure;
            this.gain = gain;
            this.frames = frames;
            this.detectedFrames = detectedFrames;
            this.tags = tags;
        }

        double detectionFraction() {
            return frames == 0 ? 0.0 : (double) detectedFrames / frames;
        }

        boolean passed() {
            return frames >= MIN_FRAMES
                    && detectedFrames >= MIN_DETECTED_FRAMES
                    && detectionFraction() >= MIN_DETECT_FRACTION;
        }
    }

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.tryGet(Limelight3A.class, LIMELIGHT_NAME);

        telemetry.addLine("Point the Limelight at the HIVE AprilTags and hold still.");
        telemetry.addLine("START sweeps exposure and gain, then applies the median passing setting.");
        telemetry.addData("limelight", limelight == null ? "MISSING" : "OK");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        if (limelight == null) {
            telemetry.addLine("Add the Limelight as an Ethernet Device named \"" + LIMELIGHT_NAME + "\".");
            telemetry.update();
            return;
        }

        limelight.pipelineSwitch(FIDUCIAL_PIPELINE);
        limelight.start();
        // Let the first /results poll land, so the link line below is truthful.
        sleep(300);

        String original = null;
        boolean winnerApplied = false;

        try {
            LLStatus status = limelight.getStatus();
            String pipelineType = status == null ? "" : status.getPipelineType();
            boolean connected = limelight.isConnected();
            telemetry.addData("link", connected
                    ? String.format(Locale.US, "connected (%d ms since frame)",
                            limelight.getTimeSinceLastUpdate())
                    : "NOT CONNECTED -- check USB/power");
            telemetry.addData("pipeline", status == null ? "no status"
                    : status.getPipelineIndex() + " " + pipelineType);
            telemetry.update();

            // Tell the three failures apart. An unreachable camera also reports an
            // empty pipeline type, so without the connectivity check first this would
            // wrongly blame the pipeline setting.
            if (!connected) {
                telemetry.addLine("The camera is in the Robot Configuration but is not answering.");
                telemetry.addLine("Check its USB-C cable and its power/status LED.");
                reportAndHold();
                return;
            }
            if (pipelineType == null || pipelineType.isEmpty()) {
                telemetry.addLine("The camera answered but reported no pipeline type.");
                telemetry.addLine("Open its web page and check the pipeline slots.");
                reportAndHold();
                return;
            }
            if (!pipelineType.toLowerCase(Locale.US).contains("fiducial")) {
                telemetry.addLine("Pipeline " + FIDUCIAL_PIPELINE + " is not a fiducial pipeline.");
                telemetry.addLine("Set it to Fiducial, 36h11, 3D on.");
                reportAndHold();
                return;
            }

            String pipelinePath = "/pipeline-atindex?index=" + FIDUCIAL_PIPELINE;
            original = http("GET", pipelinePath, null);
            JSONObject pipeline = new JSONObject(original);

            if (!pipeline.has("exposure") || !pipeline.has("lcgain")) {
                telemetry.addLine("The pipeline JSON has no exposure or lcgain field.");
                telemetry.update();
                return;
            }

            double originalExposure = pipeline.optDouble("exposure");
            double originalGain = pipeline.optDouble("lcgain");
            telemetry.addLine(String.format(Locale.US,
                    "original: exposure %.0f, gain %.0f", originalExposure, originalGain));
            telemetry.update();

            List<Result> results = new ArrayList<>();
            int total = EXPOSURES.length * GAINS.length;
            int done = 0;

            for (double exposure : EXPOSURES) {
                for (double gain : GAINS) {
                    if (!opModeIsActive()) return;

                    pipeline.put("exposure", exposure);
                    pipeline.put("lcgain", gain);
                    http("POST", "/upload-pipeline?index=" + FIDUCIAL_PIPELINE, pipeline.toString());
                    sleep(SETTLE_MS);

                    int[] measured = measure(limelight);
                    Result result = new Result(exposure, gain, measured[0], measured[1], measured[2]);
                    results.add(result);
                    done++;

                    telemetry.addLine(String.format(Locale.US,
                            "%d/%d  exposure %.0f  gain %.0f  frames %d  detected %d  tags %d  %s",
                            done, total, exposure, gain, result.frames, result.detectedFrames,
                            result.tags, result.passed() ? "PASS" : "fail"));
                    telemetry.update();
                }
            }

            List<Result> passing = new ArrayList<>();
            for (Result result : results) {
                if (result.passed()) passing.add(result);
            }

            showGrid(results);
            telemetry.addData("passing settings", passing.size());
            telemetry.update();

            if (passing.isEmpty()) {
                telemetry.addLine("No exposure/gain setting detected AprilTags reliably.");
                telemetry.addLine("Check the tags, lighting, and fiducial pipeline.");
                telemetry.update();
                return;
            }

            List<Double> passingExposures = new ArrayList<>();
            List<Double> passingGains = new ArrayList<>();
            for (Result result : passing) {
                passingExposures.add(result.exposure);
                passingGains.add(result.gain);
            }

            double medianExposure = median(passingExposures);
            double medianGain = median(passingGains);
            Result winner = nearestPassing(passing, medianExposure, medianGain);

            pipeline.put("exposure", winner.exposure);
            pipeline.put("lcgain", winner.gain);
            http("POST", "/upload-pipeline?index=" + FIDUCIAL_PIPELINE, pipeline.toString());
            winnerApplied = true;

            telemetry.addLine();
            telemetry.addLine(String.format(Locale.US,
                    "median passing: exposure %.0f, gain %.0f", medianExposure, medianGain));
            telemetry.addLine(String.format(Locale.US,
                    "APPLIED: exposure %.0f, gain %.0f", winner.exposure, winner.gain));
            telemetry.addLine(String.format(Locale.US,
                    "detection %.0f%% (%d/%d frames), tags seen %d",
                    100.0 * winner.detectionFraction(), winner.detectedFrames,
                    winner.frames, winner.tags));
            telemetry.update();
        } catch (Exception e) {
            telemetry.addLine("Sweep failed: " + e.getMessage());
            reportAndHold();
        } finally {
            if (!winnerApplied && original != null) {
                try {
                    http("POST", "/upload-pipeline?index=" + FIDUCIAL_PIPELINE, original);
                } catch (Exception ignored) {
                }
            }
            limelight.stop();
        }
    }

    /**
     * Keep a failure on the Driver Station until the driver presses STOP. Without
     * this the OpMode returns and the DS immediately clears the telemetry, which
     * looks exactly like "it disabled itself and said nothing".
     */
    private void reportAndHold() {
        telemetry.update();
        while (opModeIsActive()) idle();
    }

    private int[] measure(Limelight3A limelight) {
        long end = System.currentTimeMillis() + SAMPLE_MS;
        int frames = 0;
        int detectedFrames = 0;
        int tags = 0;

        while (System.currentTimeMillis() < end && opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid() && result.getStaleness() <= STALE_MS) {
                frames++;
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                int count = fiducials == null ? 0 : fiducials.size();
                if (count > 0) detectedFrames++;
                tags += count;
            }
            sleep(20);
        }

        return new int[] { frames, detectedFrames, tags };
    }

    private void showGrid(List<Result> results) {
        telemetry.addLine();
        telemetry.addLine("P = detects, . = no AprilTags");

        StringBuilder header = new StringBuilder("exp\\gain");
        for (double gain : GAINS) {
            header.append(String.format(Locale.US, "%4.0f", gain));
        }
        telemetry.addLine(header.toString());

        for (double exposure : EXPOSURES) {
            StringBuilder row = new StringBuilder(String.format(Locale.US, "%7.0f", exposure));
            for (double gain : GAINS) {
                Result result = find(results, exposure, gain);
                row.append(String.format(Locale.US, "%4s",
                        result == null ? "?" : result.passed() ? "P" : "."));
            }
            telemetry.addLine(row.toString());
        }
    }

    private static Result find(List<Result> results, double exposure, double gain) {
        for (Result result : results) {
            if (result.exposure == exposure && result.gain == gain) return result;
        }
        return null;
    }

    private static Result nearestPassing(List<Result> passing, double exposure, double gain) {
        double exposureSpan = EXPOSURES[EXPOSURES.length - 1] - EXPOSURES[0];
        double gainSpan = GAINS[GAINS.length - 1] - GAINS[0];
        Result best = passing.get(0);
        double bestScore = Double.POSITIVE_INFINITY;

        for (Result result : passing) {
            double de = (result.exposure - exposure) / exposureSpan;
            double dg = (result.gain - gain) / gainSpan;
            double score = de * de + dg * dg;
            if (score < bestScore) {
                bestScore = score;
                best = result;
            }
        }

        return best;
    }

    private static double median(List<Double> values) {
        List<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        int middle = sorted.size() / 2;
        if (sorted.size() % 2 == 1) return sorted.get(middle);
        return 0.5 * (sorted.get(middle - 1) + sorted.get(middle));
    }

    private static String http(String method, String path, String body) throws IOException {
        HttpURLConnection connection = (HttpURLConnection) new URL(BASE + path).openConnection();
        connection.setRequestMethod(method);
        connection.setConnectTimeout(3000);
        connection.setReadTimeout(5000);
        connection.setRequestProperty("Accept", "application/json");

        if (body != null) {
            byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
            connection.setDoOutput(true);
            connection.setRequestProperty("Content-Type", "application/json");
            connection.setFixedLengthStreamingMode(bytes.length);
            try (OutputStream output = connection.getOutputStream()) {
                output.write(bytes);
            }
        }

        int code = connection.getResponseCode();
        InputStream stream = code >= 200 && code < 300
                ? connection.getInputStream() : connection.getErrorStream();
        String response = read(stream);
        connection.disconnect();

        if (code < 200 || code >= 300) {
            throw new IOException("HTTP " + code + ": " + response);
        }
        return response;
    }

    private static String read(InputStream input) throws IOException {
        if (input == null) return "";
        StringBuilder text = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(
                new InputStreamReader(input, StandardCharsets.UTF_8))) {
            String line;
            while ((line = reader.readLine()) != null) {
                text.append(line).append('\n');
            }
        }
        return text.toString();
    }
}
