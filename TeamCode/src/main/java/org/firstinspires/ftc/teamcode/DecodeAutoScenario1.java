package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "DECODE: Auto near", group = "Competition")
public class DecodeAutoScenario1 extends OpMode {

    // --- HARDWARE ---
    private Follower follower;
    private GoBildaPinpointDriver odo;

    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx intake, turret360, shooter1, shooter2;
    private Servo servoGate, servoDegree;

    // --- LOGIC VARIABLES ---
    private Timer opModeTimer = new Timer();
    private Timer stateTimer = new Timer();

    // Переменные для Турели
    private final double TICKS_PER_DEGREE_TURRET = 10.5; // ПОДБЕРИ ЭТО ЗНАЧЕНИЕ (Encoder Ticks / 360)
    private final double TURRET_HOME_POS = 0;

    // Переменные для Шутера
    private final double SHOOTER_VELOCITY = 2000; // Ticks/sec
    private final double DEGREE_SHOOT_POS = 0.45; // Позиция серво угла для стрельбы
    private final double DEGREE_INTAKE_POS = 0.1; // Позиция серво угла для прохода под фермами (если надо)

    // Переменные для Гейта (Field Interaction)
    private final double GATE_ACTIVATE = 0.6; // Нажать на рычаг
    private final double GATE_RESET = 0.2;    // Убрать рычаг

    // --- POSES (КООРДИНАТЫ - НУЖНО НАСТРОИТЬ ПОД ПОЛЕ DECODE) ---
    // Start Pose
    private final Pose startPose = new Pose(9, 111, Math.toRadians(0));

    // Shooting Pose (Откуда стреляем в корзину)
    private final Pose shootPose = new Pose(50, 111, Math.toRadians(0));

    // Gate Trigger Pose (Где робот нажимает рычаг, чтобы сбросить мячи из корзины)
    private final Pose gateInteractPose = new Pose(15, 125, Math.toRadians(-45));

    // Sample Locations (Мячи)
    private final Pose sample1 = new Pose(24, 120, 0);
    private final Pose sample2 = new Pose(24, 108, 0);
    private final Pose sample3 = new Pose(24, 96, 0);

    // Park Pose
    private final Pose parkPose = new Pose(10, 10, Math.toRadians(0));

    // --- PATH CHAINS ---
    private PathChain toShootPreload;
    private PathChain collectBatch1, toGateFromSamples, toShootFromGate;
    private PathChain collectBatch2, toShootFromSamples2;
    private PathChain collectBatch3, toShootFromSamples3;
    private PathChain toPark;

    // --- STATE MACHINE ---
    private enum State {
        INIT,
        SHOOT_PRELOAD,
        COLLECT_1,
        TRIGGER_GATE,   // Открываем ворота корзины
        SHOOT_1,        // Стреляем первую партию (после сбора и гейта)
        COLLECT_2,
        SHOOT_2,
        COLLECT_3,
        SHOOT_3,
        PARK,
        IDLE
    }
    private State currentState = State.INIT;
    private int shotsFired = 0; // Счетчик выстрелов внутри цикла

    @Override
    public void init() {
        // 1. Init Pedro Follower
        follower = Constants.createFollower(hardwareMap);

        // 2. Init Pinpoint (Критически важно для точности)
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-84.0, -168.0); // ВВЕДИ СВОИ СМЕЩЕНИЯ В ММ (от центра робота)
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        // 3. Init Motors & Servos
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret360 = hardwareMap.get(DcMotorEx.class, "turret360");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        servoGate = hardwareMap.get(Servo.class, "servoGate");
        servoDegree = hardwareMap.get(Servo.class, "servoDegree");

        // Directions
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD); // Как ты и сказал

        // Turret Init
        turret360.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret360.setTargetPosition(0);
        turret360.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret360.setPower(1.0); // Максимальная скорость для PID внутри мотора

        // Init Positions
        servoGate.setPosition(GATE_RESET);
        servoDegree.setPosition(DEGREE_INTAKE_POS);

        buildPaths();

        telemetry.addData("Status", "DECODE Auto Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        stateTimer.resetTimer();
        follower.setStartingPose(startPose);

        // Start Sequence
        currentState = State.SHOOT_PRELOAD;
        follower.followPath(toShootPreload, true);
    }

    @Override
    public void loop() {
        // Обновляем одометрию Pinpoint и передаем в Pedro
        odo.update();
        Pose2D pinPose = odo.getPosition();
        // ВАЖНО: Pedro может использовать свои энкодеры, но если ты настроил его на Pinpoint в Constants,
        // то эта строка ниже может быть не нужна. Оставляю для надежности, если Pedro не настроен.
        // follower.setPose(new Pose(pinPose.getX(DistanceUnit.INCH), pinPose.getY(DistanceUnit.INCH), pinPose.getHeading(AngleUnit.RADIANS)));

        follower.update();

        // Логика состояний
        stateMachineUpdate();

        // Логика турели (всегда смотрит на корзину, когда мы в режиме стрельбы)
        updateTurretLogic();

        telemetry.addData("State", currentState);
        telemetry.addData("Time", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("Battery", hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.update();
    }

    private void stateMachineUpdate() {
        switch (currentState) {
            case SHOOT_PRELOAD:
                prepareShooter();
                if (!follower.isBusy()) {
                    shootBalls(3); // Функция стрельбы (блокирующая или по таймеру)
                    // После стрельбы едем собирать
                    if (shotsFired >= 3) {
                        stopShooter();
                        intake.setPower(1.0); // Включаем интейк заранее
                        follower.followPath(collectBatch1, true);
                        currentState = State.COLLECT_1;
                        stateTimer.resetTimer();
                    }
                }
                break;

            case COLLECT_1:
                // Едем по мячам
                servoDegree.setPosition(DEGREE_INTAKE_POS); // Опускаем шутер, чтобы не задеть конструкции
                if (!follower.isBusy()) {
                    // Собрали 3 мяча -> Едем к Гейту
                    intake.setPower(0.2); // Hold mode
                    follower.followPath(toGateFromSamples, true);
                    currentState = State.TRIGGER_GATE;
                    stateTimer.resetTimer();
                }
                break;

            case TRIGGER_GATE:
                // Мы у корзины. Открываем заслонку, чтобы выкинуть старые мячи
                if (!follower.isBusy()) {
                    servoGate.setPosition(GATE_ACTIVATE);
                    if (stateTimer.getElapsedTimeSeconds() > 0.8) { // Ждем 0.8 сек пока выпадут
                        servoGate.setPosition(GATE_RESET);
                        // Теперь едем стрелять наши новые мячи
                        follower.followPath(toShootFromGate, true);
                        currentState = State.SHOOT_1;
                        shotsFired = 0;
                    }
                }
                break;

            case SHOOT_1:
                prepareShooter();
                if (!follower.isBusy()) {
                    shootBalls(3);
                    if (shotsFired >= 3) {
                        stopShooter();
                        intake.setPower(1.0);
                        follower.followPath(collectBatch2, true);
                        currentState = State.COLLECT_2;
                    }
                }
                break;

            case COLLECT_2:
                servoDegree.setPosition(DEGREE_INTAKE_POS);
                if (!follower.isBusy()) {
                    intake.setPower(0.2);
                    follower.followPath(toShootFromSamples2, true); // Во 2 цикле гейт не нужен? (По твоему описанию)
                    // Если нужен гейт каждый раз - добавь состояние TRIGGER_GATE сюда тоже
                    currentState = State.SHOOT_2;
                    shotsFired = 0;
                }
                break;

            case SHOOT_2:
                prepareShooter();
                if (!follower.isBusy()) {
                    shootBalls(3);
                    if (shotsFired >= 3) {
                        stopShooter();
                        intake.setPower(1.0);
                        follower.followPath(collectBatch3, true);
                        currentState = State.COLLECT_3;
                    }
                }
                break;

            case COLLECT_3:
                servoDegree.setPosition(DEGREE_INTAKE_POS);
                if (!follower.isBusy()) {
                    intake.setPower(0.2);
                    follower.followPath(toShootFromSamples3, true);
                    currentState = State.SHOOT_3;
                    shotsFired = 0;
                }
                break;

            case SHOOT_3:
                prepareShooter();
                if (!follower.isBusy()) {
                    shootBalls(3);
                    if (shotsFired >= 3) {
                        stopShooter();
                        follower.followPath(toPark, true);
                        currentState = State.PARK;
                    }
                }
                break;

            case PARK:
                if (!follower.isBusy()) {
                    currentState = State.IDLE;
                }
                break;

            case IDLE:
                stopShooter();
                intake.setPower(0);
                break;
        }
    }

    // --- HELPER METHODS ---

    private void prepareShooter() {
        shooter1.setVelocity(SHOOTER_VELOCITY);
        shooter2.setVelocity(SHOOTER_VELOCITY);
        servoDegree.setPosition(DEGREE_SHOOT_POS);
        // Турель наводится в updateTurretLogic()
    }

    private void stopShooter() {
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
        shotsFired = 0;
    }

    // Простая логика стрельбы (имитация)
    // В реальности здесь нужно управление толкателем (Feeder)
    private void shootBalls(int count) {
        // Проверяем, разогнался ли мотор
        if (Math.abs(shooter1.getVelocity() - SHOOTER_VELOCITY) < 100) {
            // Эмуляция выстрела: включаем интейк на полную на короткое время
            // В реальности лучше использовать таймер для каждого мяча
            if (stateTimer.getElapsedTimeSeconds() > 0.5) { // Стреляем каждые 0.5 сек
                intake.setPower(1.0);
                shotsFired++; // Увеличиваем счетчик (нужна более сложная логика с датчиком цвета)
                stateTimer.resetTimer();
            } else {
                intake.setPower(0); // Пауза между выстрелами
            }
        }
    }

    private void updateTurretLogic() {
        if (currentState.toString().contains("SHOOT")) {
            // Рассчитываем угол на корзину
            Pose robotPose = follower.getPose();
            double targetAngle = Math.atan2(shootPose.getY() - robotPose.getY(), shootPose.getX() - robotPose.getX());

            // Преобразуем в позицию мотора
            int targetTicks = (int) (Math.toDegrees(targetAngle) * TICKS_PER_DEGREE_TURRET);
            turret360.setTargetPosition(targetTicks);
        } else {
            turret360.setTargetPosition(0); // Смотрит прямо при езде
        }
    }

    private void buildPaths() {
        // 1. Start -> Shoot Preload
        toShootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // 2. Shoot -> Collect Batch 1 (3 samples)
        collectBatch1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, sample1))
                .setConstantHeadingInterpolation(Math.toRadians(0)) // Едем боком/прямо
                .addPath(new BezierLine(sample1, sample2))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(sample2, sample3))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // 3. Samples -> GATE Trigger
        toGateFromSamples = follower.pathBuilder()
                .addPath(new BezierLine(sample3, gateInteractPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), gateInteractPose.getHeading())
                .build();

        // 4. Gate -> Shoot Batch 1
        toShootFromGate = follower.pathBuilder()
                .addPath(new BezierLine(gateInteractPose, shootPose))
                .setLinearHeadingInterpolation(gateInteractPose.getHeading(), shootPose.getHeading())
                .build();

        // 5. Shoot -> Collect Batch 2
        // Здесь можно изменить координаты, если вторая кучка в другом месте
        collectBatch2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, sample1))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toShootFromSamples2 = follower.pathBuilder()
                .addPath(new BezierLine(sample1, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), shootPose.getHeading())
                .build();

        // ... (Аналогично для Batch 3) ...
        collectBatch3 = collectBatch2; // Упрощение для примера
        toShootFromSamples3 = toShootFromSamples2;

        // Park
        toPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }
}