# Резервная одометрия трамвая по модели (ROS 2 Humble)

Хакатон «Московский транспорт», кейс «Резервная одометрия по модели».
Оценка **скорости и положения** беспилотного трамвая **без GNSS и IMU** — только по положению
ручки контроллера и скоростям передней/задней тележек.

## Как это устроено

```
 /vehicle/front_bogie_velocity ─┐
 /vehicle/rear_bogie_velocity  ─┼─► tram_odometry_node (C++, 1 поток, событийный)
 /vehicle/driver_position_cmd  ─┤     ├ предобработка: км/ч→м/с, выбросы, пропуски, запаздывание
 /sensing/gnss/*/fix (≤ 5 с) ───┘     ├ модель привода: тяга/тормоз(u, v) + RLS-адаптация κ
                                      ├ продольная динамика: m·v̇ = F − R(v) − уклон(d) ; сцепление μ̂
                                      ├ EKF x=[v, s, d, b_f, b_r, μ, c] + IMM-режимы тележек
                                      │   grip / slip / fault → вес измерений
                                      ├ карта: pathgraph + разворотные петли → (x, y, z)(s)
                                      └ ориентиры: места остановок → коррекция s и износа колёс
        /result/velocity  (VelocitySensor)  ◄──┤
        /result/position  (nav_msgs/Odometry, ковариации из P) ◄──┤
        /result/slip_status (JSON), /diagnostics ◄──┘
```

Подробно: [docs/model.md](docs/model.md).

## Ключевые результаты

Проверочный bag гарнесса жюри (`30618_88aea4d9`, эталон `/localization/kinematic_state`,
в построении карт не участвовал):

| Метрика | Значение |
|---|---|
| Скорость RMSE / MAE / bias | **0.026 / 0.016 / +0.004 м/с** |
| Положение RMSE 3D (как у судьи) | **6.0 м** (до конечной с другим путём: ~1–2 м) |
| Частота публикации | 40 Гц |
| Время обработки события (ядро) | p99 ≈ 4–7 мкс |

Прогон в самом гарнессе жюри (Docker, ROS 2 Humble, судья `hackathon_solution_checker`):
скорость **0.0285 м/с**, положение **6.15 м**; CPU ноды ≈ 1.1 % ядра, RSS ≤ 36 МБ.

Весь датасет и кросс-валидация, устойчивость к отказам, методика — в
[docs/validation.md](docs/validation.md).

## Сборка и запуск

```bash
cd ros2_ws && source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash
ros2 launch tram_odometry tram_odometry.launch.py bag:=/path/to/bag      # или без bag — живые топики
ros2 run tram_odometry run_demo.sh /path/to/bag /tmp/demo                  # демо + метрики + графики
```

Проверка в гарнессе жюри `check-code-with-bag`: [docs/run_instructions.md](docs/run_instructions.md).

## Структура

| Путь | Что |
|---|---|
| `ros2_ws/src/tram_odometry/` | пакет решения: `src/core/` — ядро без ROS (EKF, модели, карта, UTM), `src/tram_odometry_node.cpp` — ROS-нода, `src/replay_main.cpp` — офлайн-реплеер ядра, `config/params.yaml`, `launch/`, `maps/` (pathgraph, замкнутый маршрут, ориентиры), `scripts/` (рекордер, метрики, демо), `test/` |
| `ros2_ws/src/tram_vehicle_msgs/` | сообщения из датасета (полная версия) |
| `docs/` | модель, допущения и параметры, валидация, инструкция, план развития, графики |
| `tools/` | офлайн-анализ данных, калибровка, построение карт, валидация (Python) |

## Документы для сдачи

1. [docs/model.md](docs/model.md) — математическая модель (привод, динамика, скольжение, EKF+IMM, положение)
2. [docs/assumptions.md](docs/assumptions.md) — допущения, ограничения, все параметры
3. [docs/validation.md](docs/validation.md) — точность и быстродействие
4. [docs/run_instructions.md](docs/run_instructions.md) — инструкция для жюри
5. [docs/future_work.md](docs/future_work.md) — ограничения и план развития
