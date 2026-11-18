package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * נהיגה משלט + מצלמה שמזהה AprilTag (תגים 30 עד 37).
 *
 *   R2 (לחיצה)                 - נוסע קדימה
 *   L2 (לחיצה)                 - נוסע אחורה
 *   ג'ויסטיק שמאל, ימינה/שמאלה - פנייה ימינה / שמאלה
 *   X (ריבוע ✕ ב-PlayStation, החזקה) - הרובוט מסתובב לבד לכיוון התג הקרוב ביותר
 *
 * המצלמה: goBILDA Global Shutter USB Camera (70° FOV), מחוברת ל-USB 3.0 ב-Control Hub.
 *
 * חומרה (Configure Robot):
 *   מנועים בשמות "left" ו-"right"
 *   מצלמה בשם "Webcam 1"
 */
@TeleOp(name = "DyRobotCam", group = "Competition")
public class DyRobotCam extends LinearOpMode {

    // ===== נהיגה =====
    // false = המנוע מסתובב קדימה כשנותנים לו כוח חיובי.
    static final boolean LEFT_REVERSED = false;
    static final boolean RIGHT_REVERSED = true;   // מנועים מותקנים משני הצדדים, אחד הפוך
    static final double DEADZONE = 0.05;

    // ===== מצלמה =====
    static final String CAMERA_NAME = "Webcam 1";
    static final int CAMERA_WIDTH = 640;
    static final int CAMERA_HEIGHT = 480;

    // כיול העדשה של המצלמה ברזולוציה 640x480, מתוך מדריך המשתמש של goBILDA.
    // אם משנים רזולוציה, צריך להחליף גם את הערכים האלה (ראה הסבר בהודעה).
    static final double LENS_FX = 545.584;
    static final double LENS_FY = 545.584;
    static final double LENS_CX = 350.588;
    static final double LENS_CY = 223.15;

    // מספרי התגים של הקבוצה
    static final int[] TARGET_IDS = {30, 31, 32, 33, 34, 35, 36, 37};

    // גודל הריבוע השחור של התג באינצ'ים. הערך הזה משפיע על חישוב המרחק.
    // חובה לבדוק בחוקי המשחק מה הגודל האמיתי של התגים בעונה, ולעדכן כאן.
    static final double TAG_SIZE_INCHES = 6.5;

    // ===== כיוון אוטומטי (מחזיקים X) =====
    static final double AIM_KP = 0.02;           // כמה חזק להסתובב ביחס לשגיאה (במעלות)
    static final double AIM_MAX_TURN = 0.35;     // מהירות סיבוב מקסימלית בכיוון אוטומטי
    static final double AIM_TOLERANCE_DEG = 2.0; // מתחת לשגיאה הזו נחשב "מכוון"
    // bearing חיובי = התג משמאל, ולכן צריך לפנות שמאלה (ערך שלילי בפנייה).
    // אם הרובוט מסתובב לכיוון הלא נכון בבדיקה, משנים ל-1.0
    static final double AIM_DIRECTION = -1.0;

    private DcMotor leftDrive, rightDrive;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {
        initDrive();
        initVision();

        // בזמן INIT מציגים מה המצלמה רואה, כדי לוודא שהיא עובדת לפני Start
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("INIT: כוון את המצלמה לתג ובדוק שהוא מופיע");
            telemetry.addData("מצב מצלמה", visionPortal.getCameraState());
            AprilTagDetection initTag = findClosestTarget();
            addTagTelemetry(initTag);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            // שלב 1: נסיעה. R2 פחות L2: קדימה חיובי, אחורה שלילי.
            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            power = Range.clip(power, -1.0, 1.0);

            // שלב 2: פנייה מהג'ויסטיק השמאלי (ימינה חיובי)
            double turn = applyDeadzone(gamepad1.left_stick_x);

            // שלב 3: מצלמה. מחפשים את התג הקרוב ביותר מבין התגים שלנו.
            AprilTagDetection target = findClosestTarget();
            boolean aiming = false;

            // שלב 4: כיוון אוטומטי - מחזיקים X והתג נראה
            if (gamepad1.x && target != null) {
                aiming = true;
                double bearing = target.ftcPose.bearing;
                if (Math.abs(bearing) < AIM_TOLERANCE_DEG) {
                    turn = 0;
                } else {
                    turn = Range.clip(bearing * AIM_KP, -AIM_MAX_TURN, AIM_MAX_TURN) * AIM_DIRECTION;
                }
            }

            // שלב 5: חישוב כוח לכל צד. פנייה ימינה: שמאל מקדימה וימין לאחור.
            double left = power + turn;
            double right = power - turn;

            // אם הסכום עולה על 1, מקטינים את שניהם באותו יחס כדי לשמור על הכיוון
            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            leftDrive.setPower(left);
            rightDrive.setPower(right);

            // שלב 6: מידע בתחנת הנהג
            telemetry.addData("R2 / L2", "%.2f / %.2f", gamepad1.right_trigger, gamepad1.left_trigger);
            telemetry.addData("ג'ויסטיק שמאל (X)", "%.2f", gamepad1.left_stick_x);
            telemetry.addData("Left / Right power", "%.2f / %.2f", left, right);
            telemetry.addData("כיוון אוטומטי (X)", aiming ? "פעיל" : "כבוי");
            telemetry.addData("FPS מצלמה", "%.0f", visionPortal.getFps());
            addTagTelemetry(target);
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        visionPortal.close();
    }

    /** הכנת המנועים */
    private void initDrive() {
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");

        leftDrive.setDirection(LEFT_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(RIGHT_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** הכנת המצלמה ומעבד ה-AprilTag */
    private void initVision() {
        // ספריית תגים משלנו: התגים 30 עד 37, בגודל שהגדרנו למעלה.
        // כך הרובוט מחשב מרחק וזווית לתגים שלנו גם אם הם לא בספריית ברירת המחדל של ה-SDK.
        AprilTagLibrary.Builder library = new AprilTagLibrary.Builder();
        for (int id : TARGET_IDS) {
            library.addTag(id, "Tag " + id, TAG_SIZE_INCHES, DistanceUnit.INCH);
        }

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(library.build())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(LENS_FX, LENS_FY, LENS_CX, LENS_CY)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();

        // decimation נמוך = זיהוי מדויק יותר מרחוק, אבל פחות FPS
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)  // מומלץ ע"י goBILDA למצלמה הזו
                .addProcessor(aprilTag)
                .build();
    }

    /** מחזיר את התג הקרוב ביותר מבין התגים שלנו, או null אם אף אחד לא נראה */
    private AprilTagDetection findClosestTarget() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection closest = null;
        for (AprilTagDetection d : detections) {
            if (d.ftcPose == null || !isTargetId(d.id)) continue;
            if (closest == null || d.ftcPose.range < closest.ftcPose.range) {
                closest = d;
            }
        }
        return closest;
    }

    private boolean isTargetId(int id) {
        for (int target : TARGET_IDS) {
            if (target == id) return true;
        }
        return false;
    }

    private void addTagTelemetry(AprilTagDetection tag) {
        if (tag == null) {
            telemetry.addData("תג", "לא נראה");
            return;
        }
        telemetry.addData("תג ID", tag.id);
        telemetry.addData("מרחק (אינץ')", "%.1f", tag.ftcPose.range);
        telemetry.addData("זווית bearing (מעלות)", "%.1f", tag.ftcPose.bearing);
        telemetry.addData("yaw (מעלות)", "%.1f", tag.ftcPose.yaw);
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0 : value;
    }
}
