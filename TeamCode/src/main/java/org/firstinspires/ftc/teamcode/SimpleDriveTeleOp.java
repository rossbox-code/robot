package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * שליטה רגילה משלט (גיימפד 1) - נסיעה בלבד:
 *
 *   ג'ויסטיק שמאל - למעלה: קדימה | למטה: אחורה
 *   ג'ויסטיק ימין - ימינה: פנייה ימינה | שמאלה: פנייה שמאלה
 *   RB (החזקה)    - מצב איטי, לדיוק
 *
 * חומרה (Configure Robot): leftDrive, rightDrive
 */
@TeleOp(name = "Simple Drive", group = "Competition")
public class SimpleDriveTeleOp extends LinearOpMode {

    static final double SLOW_FACTOR = 0.4;  // כמה להאט כשמחזיקים RB
    static final double DEADZONE = 0.05;    // התעלמות מרעידות קטנות בג'ויסטיק

    private DcMotor leftDrive, rightDrive;

    @Override
    public void runOpMode() {
        // שלב 1: הכנת המנועים
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        // מנוע אחד הפוך כי הם פונים לכיוונים הפוכים.
        // אם הרובוט נוסע אחורה כשדוחפים קדימה - מחליפים FORWARD/REVERSE בשניהם.
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("מוכן. לחץ Start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // שלב 2: קריאת הג'ויסטיקים. ב-FTC דחיפה למעלה נותנת ערך שלילי, לכן הופכים סימן.
            double forward = applyDeadzone(-gamepad1.left_stick_y);
            double turn = applyDeadzone(gamepad1.right_stick_x);

            // שלב 3: חישוב כוח לכל צד.
            // קדימה מפעיל את שני הצדדים באותו כיוון, פנייה מפעילה אותם בכיוונים הפוכים.
            double left = forward + turn;
            double right = forward - turn;

            // שלב 4: אם סכום הפקודות עולה על 1, מקטינים את שניהם באותו יחס כדי לא לאבד את הכיוון
            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            // שלב 5: מצב איטי
            if (gamepad1.right_bumper) {
                left *= SLOW_FACTOR;
                right *= SLOW_FACTOR;
            }

            leftDrive.setPower(Range.clip(left, -1.0, 1.0));
            rightDrive.setPower(Range.clip(right, -1.0, 1.0));

            telemetry.addData("קדימה/אחורה", "%.2f", forward);
            telemetry.addData("פנייה", "%.2f", turn);
            telemetry.addData("Left / Right power", "%.2f / %.2f", left, right);
            telemetry.addData("מצב איטי", gamepad1.right_bumper);
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0 : value;
    }
}
