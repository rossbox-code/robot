package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * אוטונומי פשוט (מבוסס זמן):
 *   1. נסיעה קדימה 4 שניות במהירות בינונית
 *   2. עצירה 2 שניות
 *   3. נסיעה אחורה 3 שניות
 *
 * חומרה (Configure Robot): leftDrive, rightDrive - אותם שמות כמו ב-TeleOp.
 *
 * הערה: נסיעה לפי זמן תלויה במצב הסוללה ובמשטח, ולכן המרחק לא יהיה מדויק.
 * לדיוק אמיתי צריך אנקודרים (אפשר להוסיף בהמשך).
 */
@Autonomous(name = "Auto Forward-Stop-Back", group = "Competition")
public class AutoDriveForwardBack extends LinearOpMode {

    // ===== קבועים לכוונון =====
    static final double DRIVE_POWER = 0.5;     // מהירות בינונית (0 עד 1)
    static final long FORWARD_MS = 4000;       // 4 שניות קדימה
    static final long STOP_MS = 2000;          // 2 שניות עצירה
    static final long BACKWARD_MS = 3000;      // 3 שניות אחורה

    private DcMotor leftDrive, rightDrive;

    @Override
    public void runOpMode() {
        // שלב 1: הכנת החומרה (אותם כיווני מנועים כמו ב-TeleOp)
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("מוכן. לחץ Start");
        telemetry.update();

        // שלב 2: ממתינים ללחיצה על Start
        waitForStart();
        if (isStopRequested()) return;

        // שלב 3: קדימה 4 שניות
        setDrivePower(DRIVE_POWER);
        waitMs(FORWARD_MS, "קדימה");

        // שלב 4: עצירה 2 שניות
        setDrivePower(0);
        waitMs(STOP_MS, "עצירה");

        // שלב 5: אחורה 3 שניות
        setDrivePower(-DRIVE_POWER);
        waitMs(BACKWARD_MS, "אחורה");

        // שלב 6: עוצרים סופית
        setDrivePower(0);
        telemetry.addLine("סיום");
        telemetry.update();
    }

    private void setDrivePower(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }

    /** ממתין את הזמן המבוקש, ויוצא מיד אם לחצו Stop */
    private void waitMs(long ms, String stage) {
        long end = System.currentTimeMillis() + ms;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            telemetry.addData("שלב", stage);
            telemetry.addData("נשאר (ms)", end - System.currentTimeMillis());
            telemetry.update();
            idle();
        }
    }
}
