# robot – קוד TeleOp לרובוט FTC

רובוט עם הנעת טנק, איסוף כדורים ומערכת ירייה (פלייוויל + מזין), עם אפשרות לכיוון אוטומטי באמצעות מצלמה ו-AprilTag.

## מה יש במאגר

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── TankTeleOp.java         – נהיגה, איסוף וירייה (בלי מצלמה)
└── TankTeleOpVision.java   – אותו דבר + כיוון אוטומטי ומהירות ירייה לפי מרחק
```

## איך מכניסים את זה לרובוט

1. מורידים את הפרויקט הרשמי: https://github.com/FIRST-Tech-Challenge/FtcRobotController
   (Code ← Download ZIP) ופותחים אותו ב-Android Studio (File ← Open).
2. ממתינים לסיום ה-Gradle Sync.
3. מעתיקים את שני קבצי ה-`.java` מהמאגר הזה אל:
   `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`
4. מחברים את ה-Control Hub (USB, או Wi-Fi עם `adb connect 192.168.43.1:5555`) ולוחצים **Run**.

## הגדרת חומרה ב-Driver Station (Configure Robot)

השמות חייבים להיות זהים לקוד:

| שם | סוג |
|---|---|
| `leftDrive` | מנוע – גלגלים שמאל |
| `rightDrive` | מנוע – גלגלים ימין |
| `intake` | מנוע – איסוף כדורים |
| `shooter` | מנוע עם אנקודר – פלייוויל |
| `feeder` | סרוו רציף (CRServo) – מזין כדור לפלייוויל |
| `Webcam 1` | מצלמה (רק לגרסת Vision) |

## בקרה (גיימפד 1)

| כפתור | פעולה |
|---|---|
| ג'ויסטיק שמאל / ימין (Y) | גלגלים שמאל / ימין |
| LB | מצב איטי |
| RT / LT | איסוף פנימה / החוצה |
| A | הפעלה / כיבוי פלייוויל |
| D-pad ↑ / ↓ | שינוי ידני של מהירות הירייה |
| RB | הזנת כדור (רק כשהפלייוויל במהירות) |
| X (החזקה) – Vision בלבד | כיוון אוטומטי למטרה + מהירות לפי מרחק |

בגרסת Vision, בזמן INIT לוחצים **X** לברית כחולה או **B** לאדומה.

## כיוונון בבדיקה הראשונה

- רובוט נוסע הפוך או מסתובב במקום: להחליף `FORWARD` / `REVERSE` של מנועי הגלגלים.
- הכיוון האוטומטי מסתובב לצד הלא נכון: `AIM_DIRECTION = -1.0`.
- מהירות ירייה: לכייל את `DIST_TABLE_IN` ואת `VELOCITY_TABLE` בשטח (הערכים הנוכחיים הם התחלה בלבד).
- מספרי תגי המטרה (`BLUE_GOAL_TAG_ID`, `RED_GOAL_TAG_ID`) מתאימים לעונת DECODE.
