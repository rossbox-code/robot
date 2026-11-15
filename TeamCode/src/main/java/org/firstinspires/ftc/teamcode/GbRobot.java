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
 *
 * חומרה (Configure Robot): מנועים בשמות "left" ו-"right" (בדיוק כך, אותיות קטנות).
 */
@TeleOp(name = "GbRobot", group = "Competition")
public class GbRobot extends LinearOpMode {

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

            // שלב 3: שני המנועים מקבלים אותו כוח, ולכן הרובוט נוסע ישר
            leftDrive.setPower(power);
            rightDrive.setPower(power);

            // שלב 4: מידע בתחנת הנהג (עוזר לבדוק שהשלט עובד)
            telemetry.addData("ג'ויסטיק ימין (Y)", "%.2f", gamepad1.right_stick_y);
            telemetry.addData("כוח למנועים", "%.2f", power);
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
