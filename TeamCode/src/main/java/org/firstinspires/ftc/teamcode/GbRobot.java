package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * נהיגה משלט (גיימפד 1) עם מצב דריפט:
 *
 *   ג'ויסטיק ימין, למעלה/למטה - קדימה / אחורה (עד המהירות המקסימלית שהוגדרה)
 *   ג'ויסטיק שמאל, ימינה/שמאלה - פנייה ימינה / שמאלה
 *   L1 (החזקה)                - מצב דריפט: מהירות מלאה, פנייה חדה יותר,
 *                               והמנועים נעים חופשי (בלי בלימה) כדי שהרובוט יחליק
 *
 * חומרה (Configure Robot): מנועים בשמות "left" ו-"right" (בדיוק כך, אותיות קטנות).
 */
@TeleOp(name = "GbRobot", group = "Competition")
public class GbRobot extends LinearOpMode {

    // ===== מהירות מקסימלית בנהיגה רגילה (בין 0 ל-1) =====
    static final double MAX_FORWARD_SPEED = 0.7;    // מקסימום בנסיעה קדימה
    static final double MAX_BACKWARD_SPEED = 0.7;   // מקסימום בנסיעה אחורה

    // ===== הגדרות דריפט =====
    static final double DRIFT_SPEED = 1.0;          // מהירות מקסימלית בזמן דריפט
    static final double DRIFT_TURN_GAIN = 1.6;      // כמה לחדד את הפנייה בדריפט (1 = רגיל)

    // false = המנוע מסתובב קדימה כשנותנים לו כוח חיובי.
    // אם גלגל נוסע אחורה כשדוחפים קדימה, משנים אותו ל-true.
    static final boolean LEFT_REVERSED = false;
    static final boolean RIGHT_REVERSED = true;     // הרובוט הסתובב במקום, לכן הימני הפוך

    static final double DEADZONE = 0.05;            // התעלמות מרעידות קטנות בג'ויסטיק

    private DcMotor leftDrive, rightDrive;
    private boolean lastDrift = false;

    @Override
    public void runOpMode() {
        // שלב 1: הכנת המנועים
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");

        leftDrive.setDirection(LEFT_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(RIGHT_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        setBrake(true);  // נהיגה רגילה: בלימה מיד כשמשחררים
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("מוכן. לחץ Start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // שלב 2: האם מצב דריפט פעיל (L1 לחוץ)
            boolean drift = gamepad1.left_bumper;

            // החלפה בין בלימה לתנועה חופשית רק ברגע שהמצב משתנה
            if (drift != lastDrift) {
                setBrake(!drift);
                lastDrift = drift;
            }

            // שלב 3: קדימה/אחורה מהג'ויסטיק הימני. דחיפה למעלה נותנת ערך שלילי, לכן הופכים סימן.
            double power = applyDeadzone(-gamepad1.right_stick_y);
            power = Range.clip(power, -1.0, 1.0);

            // מגבלת המהירות: בדריפט המגבלה היא DRIFT_SPEED, ובנהיגה רגילה המגבלות לפי כיוון
            double maxForward = drift ? DRIFT_SPEED : MAX_FORWARD_SPEED;
            double maxBackward = drift ? DRIFT_SPEED : MAX_BACKWARD_SPEED;
            power *= (power > 0) ? maxForward : maxBackward;

            // שלב 4: פנייה מהג'ויסטיק השמאלי (ימינה חיובי). בדריפט הפנייה חדה יותר.
            double turn = applyDeadzone(gamepad1.left_stick_x);
            if (drift) {
                turn *= DRIFT_TURN_GAIN;
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
            telemetry.addData("דריפט (L1)", drift ? "פעיל" : "כבוי");
            telemetry.addData("ג'ויסטיק ימין (Y)", "%.2f", gamepad1.right_stick_y);
            telemetry.addData("ג'ויסטיק שמאל (X)", "%.2f", gamepad1.left_stick_x);
            telemetry.addData("Left / Right power", "%.2f / %.2f", left, right);
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    /** brake=true: עוצר מיד. brake=false: המנוע נע חופשי, והרובוט מחליק */
    private void setBrake(boolean brake) {
        DcMotor.ZeroPowerBehavior mode = brake
                ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        leftDrive.setZeroPowerBehavior(mode);
        rightDrive.setZeroPowerBehavior(mode);
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0 : value;
    }
}
