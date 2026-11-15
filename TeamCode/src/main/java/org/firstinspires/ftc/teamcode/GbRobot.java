package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * נסיעה קדימה ואחורה בלבד:
 *
 *   R2 (לחיצה) - נוסע קדימה. ככל שלוחצים חזק יותר, מהר יותר
 *   L2 (לחיצה) - נוסע אחורה. ככל שלוחצים חזק יותר, מהר יותר
 *
 * חומרה (Configure Robot): מנועים בשמות "left" ו-"right" (בדיוק כך, אותיות קטנות).
 */
@TeleOp(name = "GbRobot", group = "Competition")
public class GbRobot extends LinearOpMode {

    private DcMotor leftDrive, rightDrive;

    @Override
    public void runOpMode() {
        // שלב 1: הכנת המנועים
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");

        // מנוע אחד הפוך כי הם פונים לכיוונים הפוכים.
        // אם הרובוט נוסע אחורה כשלוחצים R2 - מחליפים FORWARD/REVERSE בשני המנועים.
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
            // שלב 2: R2 הוא right_trigger ו-L2 הוא left_trigger, שניהם ערך בין 0 ל-1.
            // קדימה פחות אחורה: אם לוחצים על שניהם הם מבטלים אחד את השני.
            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            power = Range.clip(power, -1.0, 1.0);

            // שלב 3: שני המנועים מקבלים אותו כוח, ולכן הרובוט נוסע ישר
            leftDrive.setPower(power);
            rightDrive.setPower(power);

            // שלב 4: מידע בתחנת הנהג (עוזר לבדוק שהשלט עובד)
            telemetry.addData("R2 (קדימה)", "%.2f", gamepad1.right_trigger);
            telemetry.addData("L2 (אחורה)", "%.2f", gamepad1.left_trigger);
            telemetry.addData("כוח למנועים", "%.2f", power);
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
