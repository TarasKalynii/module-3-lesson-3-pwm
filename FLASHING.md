# Збірка, прошивання та логи

Потрібен ESP-IDF 6.1, ціль esp32s3. Файл sdkconfig.defaults задає Flash 16 МБ
та дублювання консолі на USB Serial/JTAG.

Після активації ESP-IDF:

```bash
idf.py set-target esp32s3
idf.py build
```

Для цієї плати на macOS:

```bash
bash tools/idf.sh flash-monitor
```

Скрипт знаходить плату за USB-серійним номером E8:3D:C1:F2:9D:A4,
закриває монітор цього проєкту, збирає й записує прошивку, автоматично
запускає програму та відкриває монітор без додаткового скидання.
Для іншої плати адаптуй BOARD_SERIAL у tools/flash_guard.py.

Альтернатива у VS Code: Cmd+Shift+B. Вихід із монітора: Ctrl+].
Логи: SERVO_CTRL: ANGLE, MODE, LIMIT, LONG PRESS та HEARTBEAT кожні 5 секунд.
Журнали збірки та прошивання: build/flash-logs/.
