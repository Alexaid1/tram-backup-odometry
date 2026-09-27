# Инструкция для жюри: сборка, запуск, проверка

Решение — ROS 2 Humble workspace `ros2_ws/` с пакетами:
- `tram_odometry` — оценщик (C++ нода `tram_odometry_node` + офлайн-реплеер ядра,
  Python-утилиты записи и метрик вне контура оценки);
- `tram_vehicle_msgs` — пакет сообщений из датасета (полная версия, с
  `DriverControllerCommand`; `VelocitySensor` идентичен версии гарнесса).

Зависимости — только стандартные пакеты `ros:humble-ros-base` (rclcpp, nav_msgs,
sensor_msgs, std_msgs, diagnostic_msgs, ament_index_cpp, rosbag2, launch_ros).
Никаких загрузок при сборке; Eigen/JSON-библиотеки не требуются (свои реализации).

## 1. Проверка в гарнессе `check-code-with-bag` (как у жюри)

Каталог гарнесса: `check-code-with-bag/check-code/` (Dockerfile на `ros:humble-ros-base`,
`src/checker_ros` — судья, `src/tram_vehicle_msgs` — сообщения, `bags/30618_88aea4d9`).

```bash
cd check-code-with-bag/check-code
# 1) положить пакет решения рядом с судьёй
cp -r <repo>/ros2_ws/src/tram_odometry src/
# (рекомендуется) заменить пакет сообщений гарнесса на полную версию: VelocitySensor
# идентичен, добавляется DriverControllerCommand -> bag play начнёт публиковать ручку
#   rm -rf src/tram_vehicle_msgs && cp -r <repo>/ros2_ws/src/tram_vehicle_msgs src/
# Без замены решение тоже собирается и работает (без входа ручки контроллера).

./scripts/build.sh && ./scripts/run.sh      # образ и контейнер (Windows: build.ps1 / run.ps1)
./scripts/enter.sh                          # shell внутри контейнера, /workspace
```

Внутри контейнера:

```bash
cd /workspace
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# терминал 1 — судья
ros2 run hackathon_solution_checker metrics
# терминал 2 — решение
ros2 launch tram_odometry tram_odometry.launch.py
# терминал 3 — данные
ros2 bag play bags/30618_88aea4d9
```

То же одной командой (судья + решение + bag на скорости 1.0, замер CPU/RAM ноды, итог в
`harness_out/`): скопировать `check-code-with-bag/check-code/harness_run.sh` из репозитория
в каталог гарнесса и внутри контейнера выполнить `bash /workspace/harness_run.sh`. Ожидаемый итог на `30618_88aea4d9`:
скорость RMSE ≈ 0.028 м/с, положение 3D RMSE ≈ 6.1 м (см. `validation.md` §5.1).

Судья каждые 5 с печатает `Velocity metrics [m/s]` и `Position metrics [m]` (RMSE и max по
x, y, z, 3D). Остановите судью Ctrl+C после окончания bag — будет итоговый отчёт.

> Сообщение `Ignoring a topic '/vehicle/driver_position_cmd'` от `ros2 bag play` означает,
> что в окружении нет типа `DriverControllerCommand` (пакет сообщений гарнесса). Нода
> пишет в лог `running without controller input` и продолжает работу.

## 2. Сборка отдельно (свой workspace)

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select tram_odometry && colcon test-result --verbose   # unit-тесты ядра
```

Если в workspace уже есть другой `tram_vehicle_msgs` — оставьте только один (наш —
надмножество версии гарнесса).

## 3. Запуск

```bash
# на живых топиках
ros2 launch tram_odometry tram_odometry.launch.py
# с проигрыванием bag (путь к каталогу bag)
ros2 launch tram_odometry tram_odometry.launch.py bag:=/data/30618_0e41eac3
# + запись результатов/задержки/CPU/RAM
ros2 launch tram_odometry tram_odometry.launch.py bag:=/data/30618_0e41eac3 record:=true out_dir:=/tmp/run1
# только исходный pathgraph (без восстановленных петель и ориентиров)
ros2 launch tram_odometry tram_odometry.launch.py use_closed_route:=false
# свой конфиг
ros2 launch tram_odometry tram_odometry.launch.py params_file:=/path/params.yaml
```

Нода работает по времени данных (штампам сообщений), `use_sim_time` не нужен.

## 4. Демо одной командой (пример работы для питча)

```bash
ros2 run tram_odometry run_demo.sh /data/30618_0e41eac3 /tmp/demo
```

Запускает ноду и рекордер, проигрывает bag, затем печатает метрики и сохраняет в
`/tmp/demo`: `metrics.json`, `run_summary.png` (скорость: оценка / тележки / эталон +
гистограмма задержки), `trajectory.png`, CSV-файлы, `node.log`.
Эталон: `/localization/kinematic_state`, если он есть в bag, иначе GNSS master с учётом
рычага антенны.

## 5. Выходные топики

| Топик | Тип | Содержимое |
|---|---|---|
| `/result/velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | `velocity` — продольная скорость, м/с; `header.stamp` = время входа из bag; `frame_id = base_link` |
| `/result/position` | `nav_msgs/msg/Odometry` | `pose.pose.position` — x, y, z в кадре `map` (UTM37N − 300000/6100000, как у эталона); ориентация = курс пути; `pose.covariance` из P фильтра; `twist.twist.linear.x` — скорость, `twist.covariance[0]` = P_vv; `child_frame_id = base_link` |
| `/result/slip_status` | `std_msgs/msg/String` (JSON, 5 Гц) | режим (`normal_grip` / `slip_detected` / `sensor_fault` / `dead_reckoning`), μ̂, по каждой тележке: режим, P(grip/slip/fault), λ, устаревание; задержка обработки |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | уровень OK/WARN/ERROR по режиму + ~30 значений: μ̂, вероятности режимов, скольжения, κ, d, масштабы колёс, сила тяги, момент ТЭД, число привязок к ориентирам, отброшенные сообщения, задержка |

Частота `/result/*`: 40 Гц (по событиям входа + таймер 25 мс; ≤ 50 Гц). Позиция публикуется
после начальной выставки (первый фикс GNSS, обычно < 1 с). Штамп = время данных; параметр
`publish.time_offset` (0.08 с) согласует выход с временной шкалой эталона `kinematic_state`,
для сравнения с GNSS без запаздывания задайте 0.

## 6. Логи и метрики

```bash
ros2 topic hz /result/velocity                 # частота
ros2 topic echo /result/slip_status            # режим, сцепление, вероятности
ros2 topic echo /diagnostics                   # полная диагностика
ros2 run tram_odometry evaluate_run.py /tmp/run1   # метрики записанного прогона
```

`evaluate_run.py` печатает: RMSE/MAE/bias скорости (всего и по режимам разгон / торможение /
выбег / стоянка), ошибки позиции x/y/z/3D (как у судьи), along-track и дрейф в %,
задержку вход → `/result/velocity` (p50/p95/max по реальному времени приёма сообщений),
частоту публикации, CPU % и RSS ноды.

## 7. Офлайн-валидация на всём датасете (без ROS)

Ядро оценщика не зависит от ROS и собирается как `tram_odometry_replay`
(`colcon build` собирает его вместе с нодой). Инструменты в `tools/` (Python 3 + numpy,
scipy, matplotlib, rosbags, pyproj):

```bash
python tools/cache_bags.py              # dataset/data/* и проверочный bag -> tools/_cache/*.npz
python tools/evaluate.py --all --json tools/_eval.json   # все уникальные прогоны
python tools/evaluate.py 30618_88aea4d9 --plots          # один прогон + графики в docs/figures
python tools/summarize.py tools/_eval.json
python tools/cross_validate.py          # 2-fold CV карты ориентиров
```

В Windows-среде разработки реплеер собирается `tools/build_replay.ps1` (zig c++).
