package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp לרובוט טנק עם איסוף כדורים ומערכת ירייה (פלייוויל).
 *
 * שמות החומרה שצריך להגדיר ב-Configuration של ה-Control Hub:
 *   leftDrive  - מנוע גלגלים שמאל
 *   rightDrive - מנוע גלגלים ימין
 *   intake     - מנוע איסוף כדורים
 *   shooter    - מנוע הפלייוויל של הירייה (עדיף עם אנקודר)
 *   feeder     - סרוו רציף (CRServo) שמזין כדור לתוך הפלייוויל
 *
 * בקרה (גיימפד 1):
 *   ג'ויסטיק שמאל (Y)  - גלגלים שמאל
 *   ג'ויסטיק ימין (Y)  - גלגלים ימין
 *   LB                 - מצב איטי (דיוק)
 *   RT                 - איסוף כדורים פנימה
 *   LT                 - פליטת כדורים החוצה (אם נתקע)
 *   A                  - הפעלה/כיבוי של הפלייוויל
 *   RB                 - הזנת כדור לירייה (עובד רק כשהפלייוויל מוכן)
 *   D-pad למעלה/למטה   - הגדלה/הקטנה של מהירות הירייה
 */
@TeleOp(name = "Tank TeleOp", group = "Competition")
public class TankTeleOp extends LinearOpMode {

    // ===== קבועים שאפשר לכוון =====
    static final double SLOW_MODE_FACTOR = 0.4;       // כמה להאט במצב איטי
    static final double INTAKE_POWER = 1.0;           // עוצמת איסוף
    static final double FEEDER_POWER = 1.0;           // עוצמת ההזנה
    static final double SHOOTER_START_VELOCITY = 1200; // ticks לשנייה
    static final double SHOOTER_MAX_VELOCITY = 2400;  // תקרה בטוחה
    static final double SHOOTER_STEP = 50;            // צעד שינוי מהירות
    static final double SHOOTER_TOLERANCE = 50;       // טווח שבו נחשב "מוכן"
    static final double DEADZONE = 0.05;              // התעלמות מרעידות ג'ויסטיק

    // ===== חומרה =====
    private DcMotor leftDrive, rightDrive, intake;
    private DcMotorEx shooter;
    private CRServo feeder;

    // ===== משתני מצב =====
    private boolean shooterOn = false;
    private double shooterTargetVelocity = SHOOTER_START_VELOCITY;
    private boolean lastA = false, lastDpadUp = false, lastDpadDown = false;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("מוכן! לחץ על Start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            driveTank();
            runIntake();
            runShooter();
            updateTelemetry();
        }

        stopAll();
    }

    /** שלב 1: הגדרת החומרה וכיווני המנועים */
    private void initHardware() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        feeder = hardwareMap.get(CRServo.class, "feeder");

        // המנועים בצד אחד מותקנים הפוך, לכן הופכים צד אחד.
        // אם הרובוט נוסע הפוך או מסתובב במקום - החלף את FORWARD/REVERSE.
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // BRAKE עוצר מהר ולא מחליק
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // הפלייוויל עובד בבקרת מהירות באמצעות האנקודר, כדי שהירייה תהיה עקבית
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** שלב 2: הנעת טנק - כל ג'ויסטיק שולט בצד אחד */
    private void driveTank() {
        double left = applyDeadzone(-gamepad1.left_stick_y);
        double right = applyDeadzone(-gamepad1.right_stick_y);

        // העלאה בריבוע (עם שמירת סימן) נותנת שליטה עדינה יותר במהירויות נמוכות
        left = Math.copySign(left * left, left);
        right = Math.copySign(right * right, right);

        if (gamepad1.left_bumper) {
            left *= SLOW_MODE_FACTOR;
            right *= SLOW_MODE_FACTOR;
        }

        leftDrive.setPower(Range.clip(left, -1.0, 1.0));
        rightDrive.setPower(Range.clip(right, -1.0, 1.0));
    }

    /** שלב 3: איסוף כדורים */
    private void runIntake() {
        if (gamepad1.right_trigger > 0.1) {
            intake.setPower(INTAKE_POWER);      // פנימה
        } else if (gamepad1.left_trigger > 0.1) {
            intake.setPower(-INTAKE_POWER);     // החוצה (שחרור תקיעה)
        } else {
            intake.setPower(0);
        }
    }

    /** שלב 4: מערכת ירייה - פלייוויל + הזנה */
    private void runShooter() {
        // לחיצה אחת על A מחליפה מצב (זיהוי לחיצה חדשה, לא החזקה)
        if (gamepad1.a && !lastA) {
            shooterOn = !shooterOn;
        }
        lastA = gamepad1.a;

        // כוונון מהירות בזמן אמת
        if (gamepad1.dpad_up && !lastDpadUp) {
            shooterTargetVelocity += SHOOTER_STEP;
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            shooterTargetVelocity -= SHOOTER_STEP;
        }
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        shooterTargetVelocity = Range.clip(shooterTargetVelocity, 0, SHOOTER_MAX_VELOCITY);

        if (shooterOn) {
            shooter.setVelocity(shooterTargetVelocity);
        } else {
            shooter.setVelocity(0);
        }

        // מזינים כדור רק אם הפלייוויל הגיע למהירות - אחרת הכדור יצא חלש ויחטיא
        if (gamepad1.right_bumper && isShooterReady()) {
            feeder.setPower(FEEDER_POWER);
        } else {
            feeder.setPower(0);
        }
    }

    /** האם הפלייוויל קרוב מספיק למהירות היעד */
    private boolean isShooterReady() {
        return shooterOn
                && Math.abs(shooter.getVelocity() - shooterTargetVelocity) <= SHOOTER_TOLERANCE;
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0 : value;
    }

    /** שלב 5: מידע בזמן אמת בתחנת הנהג - עוזר מאוד בכוונון */
    private void updateTelemetry() {
        telemetry.addData("מצב איטי", gamepad1.left_bumper);
        telemetry.addData("Left / Right power", "%.2f / %.2f",
                leftDrive.getPower(), rightDrive.getPower());
        telemetry.addData("Intake power", intake.getPower());
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
