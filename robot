package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * TeleOp טנק + איסוף + ירייה עם עזרה של מצלמה (AprilTag).
 *
 * חומרה (Configuration):
 *   leftDrive, rightDrive, intake, shooter, feeder  - כמו בגרסה הקודמת
 *   Webcam 1                                        - המצלמה (שם ניתן לשינוי למטה)
 *
 * בקרה (גיימפד 1):
 *   ג'ויסטיקים - הנעת טנק         LB - מצב איטי
 *   RT / LT    - איסוף פנימה / החוצה
 *   A          - הפעלה/כיבוי פלייוויל
 *   D-pad ↑/↓  - כוונון ידני של מהירות הירייה
 *   X (החזקה)  - AUTO-AIM: הרובוט מסתובב למטרה והמהירות נקבעת לפי מרחק
 *   RB         - הזנת כדור (רק כשהפלייוויל מוכן, ובמצב X גם רק כשהרובוט מכוון)
 *
 * בזמן ה-INIT (לפני Start): לחץ X לבחירת ברית כחולה, או B לאדומה.
 */
@TeleOp(name = "Tank TeleOp Vision", group = "Competition")
public class TankTeleOpVision extends LinearOpMode {

    // ===== מצלמה =====
    static final String CAMERA_NAME = "Webcam 1";
    static final int CAMERA_WIDTH = 640;
    static final int CAMERA_HEIGHT = 480;

    // מספרי AprilTag של המטרות (עונת DECODE). אם העונה/המטרה שונה - לשנות כאן.
    static final int BLUE_GOAL_TAG_ID = 20;
    static final int RED_GOAL_TAG_ID = 24;

    // ===== כיוון אוטומטי =====
    static final double AIM_KP = 0.02;           // כמה חזק להסתובב ביחס לשגיאה (מעלות)
    static final double AIM_MAX_TURN = 0.35;     // מהירות סיבוב מקסימלית
    static final double AIM_TOLERANCE_DEG = 2.0; // כמה מעלות שגיאה נחשב "מכוון"
    static final double AIM_DIRECTION = 1.0;     // אם הרובוט מסתובב לכיוון הלא נכון - שנה ל -1.0

    // ===== טבלת מרחק -> מהירות פלייוויל (דורשת כיול בשטח!) =====
    // מרחק באינצ'ים מהמצלמה אל התג, ומהירות ב-ticks לשנייה.
    static final double[] DIST_TABLE_IN = {30, 50, 70, 90, 110};
    static final double[] VELOCITY_TABLE = {1000, 1150, 1300, 1450, 1600};

    // ===== קבועים כלליים =====
    static final double SLOW_MODE_FACTOR = 0.4;
    static final double INTAKE_POWER = 1.0;
    static final double FEEDER_POWER = 1.0;
    static final double SHOOTER_START_VELOCITY = 1200;
    static final double SHOOTER_MAX_VELOCITY = 2400;
    static final double SHOOTER_STEP = 50;
    static final double SHOOTER_TOLERANCE = 50;
    static final double DEADZONE = 0.05;

    // ===== חומרה =====
    private DcMotor leftDrive, rightDrive, intake;
    private DcMotorEx shooter;
    private CRServo feeder;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // ===== מצב =====
    private boolean shooterOn = false;
    private double shooterTargetVelocity = SHOOTER_START_VELOCITY;
    private boolean lastA = false, lastDpadUp = false, lastDpadDown = false;
    private int targetTagId = BLUE_GOAL_TAG_ID;

    // תוצאות הראייה של הלולאה הנוכחית
    private boolean tagVisible = false;
    private double tagRange = 0;    // אינצ'ים
    private double tagBearing = 0;  // מעלות
    private boolean aimActive = false;

    @Override
    public void runOpMode() {
        initHardware();
        initVision();

        // בחירת ברית בזמן INIT
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) targetTagId = BLUE_GOAL_TAG_ID;
            if (gamepad1.b) targetTagId = RED_GOAL_TAG_ID;
            telemetry.addData("ברית", targetTagId == BLUE_GOAL_TAG_ID ? "כחולה (X)" : "אדומה (B)");
            telemetry.addData("מצלמה", visionPortal.getCameraState());
            telemetry.addLine("לחץ X=כחול / B=אדום, ואז Start");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            updateVision();
            driveTank();
            runIntake();
            runShooter();
            updateTelemetry();
        }

        stopAll();
        visionPortal.close();
    }

    /** הכנת מנועים */
    private void initHardware() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        feeder = hardwareMap.get(CRServo.class, "feeder");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** הכנת המצלמה ומעבד ה-AprilTag */
    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();

        // decimation נמוך = זיהוי מדויק יותר מרחוק, אבל פחות FPS
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .setCameraResolution(new android.util.Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .addProcessor(aprilTag)
                .build();
    }

    /** קריאת הזיהוי האחרון של תג המטרה */
    private void updateVision() {
        tagVisible = false;
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.id == targetTagId && d.ftcPose != null) {
                tagVisible = true;
                tagRange = d.ftcPose.range;
                tagBearing = d.ftcPose.bearing;
                break;
            }
        }
    }

    /** הנעת טנק. כשלוחצים X והתג נראה - הרובוט מסתובב למטרה בעצמו */
    private void driveTank() {
        aimActive = gamepad1.x && tagVisible;

        double left;
        double right;

        if (aimActive) {
            // בקר P פשוט: ככל שהשגיאה גדולה יותר - מסתובבים מהר יותר
            double turn = Range.clip(tagBearing * AIM_KP, -AIM_MAX_TURN, AIM_MAX_TURN) * AIM_DIRECTION;
            if (Math.abs(tagBearing) < AIM_TOLERANCE_DEG) turn = 0;
            left = -turn;
            right = turn;
        } else {
            left = applyDeadzone(-gamepad1.left_stick_y);
            right = applyDeadzone(-gamepad1.right_stick_y);
            left = Math.copySign(left * left, left);
            right = Math.copySign(right * right, right);
            if (gamepad1.left_bumper) {
                left *= SLOW_MODE_FACTOR;
                right *= SLOW_MODE_FACTOR;
            }
        }

        leftDrive.setPower(Range.clip(left, -1.0, 1.0));
        rightDrive.setPower(Range.clip(right, -1.0, 1.0));
    }

    private void runIntake() {
        if (gamepad1.right_trigger > 0.1) {
            intake.setPower(INTAKE_POWER);
        } else if (gamepad1.left_trigger > 0.1) {
            intake.setPower(-INTAKE_POWER);
        } else {
            intake.setPower(0);
        }
    }

    private void runShooter() {
        if (gamepad1.a && !lastA) shooterOn = !shooterOn;
        lastA = gamepad1.a;

        if (gamepad1.dpad_up && !lastDpadUp) shooterTargetVelocity += SHOOTER_STEP;
        if (gamepad1.dpad_down && !lastDpadDown) shooterTargetVelocity -= SHOOTER_STEP;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        // במצב X ועם תג בשדה הראייה - המהירות נקבעת אוטומטית לפי המרחק
        if (aimActive) {
            shooterTargetVelocity = velocityForDistance(tagRange);
        }
        shooterTargetVelocity = Range.clip(shooterTargetVelocity, 0, SHOOTER_MAX_VELOCITY);

        shooter.setVelocity(shooterOn ? shooterTargetVelocity : 0);

        // הזנה: הפלייוויל חייב להיות מוכן, ובמצב auto-aim גם הרובוט חייב להיות מכוון
        boolean canFeed = isShooterReady() && (!aimActive || isAimed());
        feeder.setPower(gamepad1.right_bumper && canFeed ? FEEDER_POWER : 0);
    }

    private boolean isShooterReady() {
        return shooterOn
                && Math.abs(shooter.getVelocity() - shooterTargetVelocity) <= SHOOTER_TOLERANCE;
    }

    private boolean isAimed() {
        return tagVisible && Math.abs(tagBearing) <= AIM_TOLERANCE_DEG;
    }

    /** אינטרפולציה לינארית בין נקודות הכיול בטבלה */
    private double velocityForDistance(double distanceIn) {
        if (distanceIn <= DIST_TABLE_IN[0]) return VELOCITY_TABLE[0];
        int last = DIST_TABLE_IN.length - 1;
        if (distanceIn >= DIST_TABLE_IN[last]) return VELOCITY_TABLE[last];
        for (int i = 0; i < last; i++) {
            if (distanceIn <= DIST_TABLE_IN[i + 1]) {
                double t = (distanceIn - DIST_TABLE_IN[i]) / (DIST_TABLE_IN[i + 1] - DIST_TABLE_IN[i]);
                return VELOCITY_TABLE[i] + t * (VELOCITY_TABLE[i + 1] - VELOCITY_TABLE[i]);
            }
        }
        return VELOCITY_TABLE[last];
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0 : value;
    }

    private void updateTelemetry() {
        telemetry.addData("ברית", targetTagId == BLUE_GOAL_TAG_ID ? "כחולה" : "אדומה");
        telemetry.addData("Tag visible", tagVisible);
        if (tagVisible) {
            telemetry.addData("Range (in)", "%.1f", tagRange);
            telemetry.addData("Bearing (deg)", "%.1f", tagBearing);
        }
        telemetry.addData("Auto-aim", aimActive);
        telemetry.addData("Aimed", isAimed());
        telemetry.addData("Shooter ON", shooterOn);
        telemetry.addData("Shooter target", "%.0f", shooterTargetVelocity);
        telemetry.addData("Shooter actual", "%.0f", shooter.getVelocity());
        telemetry.addData("Shooter READY", isShooterReady());
        telemetry.update();
    }

    private void stopAll() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        intake.setPower(0);
        shooter.setVelocity(0);
        feeder.setPower(0);
    }
}
