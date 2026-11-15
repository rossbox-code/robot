package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * נסיעה קדימה ואחורה בלבד:
 *
 *   ג'ויסטיק ימין, למעלה - נוסע קדימה (ככל שדוחפים יותר, מהר יותר)
 *   ג'ויסטיק ימין, למטה  - נוסע אחורה
 *   ג'ויסטיק שמאל, ימינה - פונה ימינה
 *   ג'ויסטיק שמאל, שמאלה - פונה שמאלה
 *
 * חומרה (Configure Robot): מנועים בשמות "left" ו-"right" (בדיוק כך, אותיות קטנות).
 */
@TeleOp(name = "GbRobot", group = "Competition")
public class GbRobot extends LinearOpMode {

    // ===== מהירות מקסימלית (בין 0 ל-1). 1.0 = מהירות מלאה, 0.5 = חצי מהירות =====
    static final double MAX_FORWARD_SPEED = 0.7;    // מקסימום בנסיעה קדימה
    static final double MAX_BACKWARD_SPEED = 0.7;   // מקסימום בנסיעה אחורה

    // false = המנוע מסתובב קדימה כשנותנים לו כוח חיובי.
    // אם גלגל נוסע אחורה כשלוחצים R2, משנים אותו ל-true.
    static final boolean LEFT_REVERSED = false;
    static final boolean RIGHT_REVERSED = true;   // הרובוט הסתובב במקום, לכן הימני הפוך

    private DcMotor leftDrive, rightDrive;

    @Override
    public void runOpMode() {
        // שלב 1: הכנת המנועים
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");

        // שני המנועים באותו כיוון: R2 מפעיל את שניהם קדימה, L2 את שניהם אחורה.
        // אם גלגל מסוים מסתובב הפוך, משנים את המתג שלו (LEFT_REVERSED / RIGHT_REVERSED) בראש הקוד.
        leftDrive.setDirection(LEFT_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(RIGHT_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("מוכן. לחץ Start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // שלב 2: הג'ויסטיק הימני. ב-FTC דחיפה למעלה נותנת ערך שלילי, לכן הופכים סימן.
            // למעלה = קדימה, למטה = אחורה.
            double power = -gamepad1.right_stick_y;
            if (Math.abs(power) < 0.05) power = 0;   // התעלמות מרעידות קטנות
            power = Range.clip(power, -1.0, 1.0);

            // הגבלת מהירות מקסימלית: דחיפה מלאה של הג'ויסטיק נותנת בדיוק את המקסימום שהגדרת.
            if (power > 0) {
                power *= MAX_FORWARD_SPEED;      // קדימה
            } else {
                power *= MAX_BACKWARD_SPEED;     // אחורה
            }

            // שלב 3: הג'ויסטיק השמאלי (ציר X) לפנייה. ימינה = ערך חיובי, שמאלה = שלילי.
            double turn = gamepad1.left_stick_x;
            if (Math.abs(turn) < 0.05) turn = 0;     // התעלמות מרעידות קטנות

            // שלב 4: חישוב כוח לכל צד.
            // נסיעה קדימה מפעילה את שני הצדדים באותו כיוון.
            // פנייה ימינה: שמאל מקדימה וימין לאחור. פנייה שמאלה: להפך.
            double left = power + turn;
            double right = power - turn;

            // שלב 5: אם הסכום עולה על 1, מקטינים את שניהם באותו יחס כדי לשמור על הכיוון
            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            leftDrive.setPower(left);
            rightDrive.setPower(right);

            // שלב 4: מידע בתחנת הנהג (עוזר לבדוק שהשלט עובד)
            telemetry.addData("ג'ויסטיק ימין (Y)", "%.2f", gamepad1.right_stick_y);
            telemetry.addData("ג'ויסטיק שמאל (X)", "%.2f", gamepad1.left_stick_x);
            telemetry.addData("Left / Right power", "%.2f / %.2f", left, right);
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
