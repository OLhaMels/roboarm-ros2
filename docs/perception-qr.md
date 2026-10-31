# Камера і QR: upd-rasp-stream-tcp у ROS 2

[Головна](../README.md) · [Архітектура](architecture.md) · [Налаштування](configuration.md)

Цей документ описує, як програму `upd-rasp-stream-tcp` (picamera2 + zbar + iFurn + TCP-стрім в одному процесі) розкладено на вузли Perception Layer з блок-схеми.

## Що куди переїхало

| Було в `qr_detector.py` / `streamer.py` | Стало | Файл |
|---|---|---|
| `Picamera2()`, конфігурація, `start()/stop()` | `camera_node`, бекенди `libcamera` / `picamera2` / `v4l2` / `images` | `roboarm_nodes/camera.py` |
| ctypes-біндинг libzbar, `Scanner`, `Code` | без змін | `roboarm_nodes/qr.py` |
| `QRDetector` (потік сканування + mailbox) | `vision_node.on_image()`: кадри приходять з топіка, потік і mailbox не потрібні | `roboarm_nodes/vision.py` |
| `_is_new()` / `repeat_after` | без змін, параметр `repeat_after` | `vision.py` |
| `handle_code()` — заглушка | `/vision/target_pose` (для `task_manager_node`) + `/vision/qr` (JSON) | `vision.py` |
| iFurn → `parts.csv`, `PartRecorder` | без змін у парсингу; лог через ROS, кожен рядок ще й у `/vision/part` | `roboarm_nodes/parts.py` |
| `Osd` (рамка + плашка в кадрі) | малюється в копію кадру → `/vision/debug_image` | `roboarm_nodes/overlay.py` |
| `Streamer`, `TcpTsOutput`, slice-threading libx264 | `stream_node`: те саме, але з топіка | `roboarm_nodes/stream.py` |
| `--self-test`, `--fetch-part` | `pytest` у `src/roboarm_nodes/test/test_qr_vision.py`, бекенд `images` | |

Головна зміна: **камерою володіє лише `camera_node`**. Двом процесам одночасно камеру не відкрити, тож `vision_node` і будь-хто інший беруть кадри з `/camera/image_raw`.

## Граф

```text
camera_node ──/camera/image_raw (rgb8, SensorDataQoS)──► vision_node ──/vision/target_pose──► task_manager_node
            ──/camera/camera_info────────────────────►     │
                                                           ├──/vision/qr     (std_msgs/String, JSON на кожне нове зчитування)
                                                           ├──/vision/part   (std_msgs/String, JSON рядка parts.csv)
                                                           ├── parts.csv     (iFurn або текст етикетки, у фонових потоках)
                                                           └──/vision/debug_image ──► stream_node ──TCP :8554──► ffplay
                                                                                 └──► rqt_image_view
```

## vision_node: від QR до цілі для руки

1. Кадр `rgb8`/`bgr8`/`mono8` → сірий (Y800) з урахуванням `step` (паддінгу рядків) → zbar.
2. Кожне **нове** зчитування (як і раніше: раз, потім знову лише після `repeat_after` с відсутності) → лог, `/vision/qr`, і, якщо це посилання, — `PartRecorder` (iFurn → CSV).
3. **Ціль** для руки — лише код, payload якого відповідає `target_pattern` (за замовчуванням `2qi.us` та `*.ifurn.pro`). Так закрито ризик зі старого опису: «камера радо зчитала QR з Вікіпедії». Якщо в кадрі кілька таких кодів — береться найбільший (найближчий).
4. QR — квадрат відомого розміру `qr_size`, тож його 4 кути з zbar працюють як ArUco-маркер: `solvePnP(IPPE_SQUARE)` → центр коду в `camera_optical_frame` → TF → `base_link` → `/vision/target_pose`. Порядок кутів zbar нормалізується (`clockwise()`), бо для `solvePnP` важлива відповідність точок.

Для пози потрібні: OpenCV, **калібрування** камери (`camera_matrix`, `distortion` у `perception.yaml`, при тій самій роздільності) і **виміряний** TF `base_link → camera_optical_frame`. Без них коди все одно читаються й записуються в CSV, просто пози немає (у лозі буде попередження).

`qr_size` — сторона чорного квадрата коду **без білого поля**, у метрах.

## Етикетки без QR: читання тексту (OCR)

Не на всіх етикетках є QR, з якого можна взяти характеристики. Тоді `vision_node` читає надрукований текст: розміри, номер деталі, кількість, замовника, матеріал, дату, кромку. Рядок іде в той самий `parts.csv` і в `/vision/part` з `source: label`.

**Коли вмикається.** OCR не працює, поки в кадрі є QR, який уже дав запис з iFurn, і чекає, поки такий запит ще виконується. Він читає етикетку, якщо:
- QR немає;
- zbar не зміг прочитати QR;
- посилання веде не на картку деталі. Так поводяться короткі посилання `q.ifurn.pro/a/...`, наприклад на етикетках «Реставрація дверей».

Читання запускається через `ocr_quiet` с без корисного QR, не частіше ніж раз на `ocr_interval` с, у фоновому потоці (`labels.LabelReader`), тож обробка кадрів не гальмує. Одна й та сама етикетка записується один раз за запуск.

**Як читає** (`roboarm_nodes/labels.py`):
1. Вирівнює освітлення і лишає тільки чорний друк. Фактура ДСП, тіні, QR, штрих-коди й лінії прибираються.
2. Знаходить найщільніше скупчення друку, тобто саму етикетку, і вирівнює нахил за рядками тексту.
3. Збільшує кадр так, щоб літера мала приблизно 30 px.
4. Tesseract (`ukr+rus+eng`) у режимах 6 і 11. Орієнтацію (0/90/180/270°) спершу підказує OSD, а остаточно вибирається та, у якій текст розбирається на поля. Ліміт — `ocr_budget` с на кадр.
5. Розбір рядків (`parse_label`) під формати, які траплялися:

| Етикетка | Що розпізнається |
|---|---|
| `Коваленко Петро 30.09 / ДСП Сатин 18мм В/Ф / 1877 x 497 x 18 / №:13 K:3` | розміри, №, K, ім'я, матеріал, дата, кромка `(B) 1` |
| `П:Іван Бондар / М:ДСП... / Р:788.4 x 430.4 x 18 / №:21 (1:21) K:1` | те саме + десяткові розміри; `Н:№118` (шафа) не плутається з № деталі |
| `505365 Реставрац ЛВ-10073387 / #142-1 г/р 686 x 422x18` | розміри, деталь `142` / шт `1`, замовлення й код `ЛВ-…` |
| `Реставрація дверей / 2800-2070-18-swiss-krono… / 350_200_18 / 2026.10.07` | розміри, матеріал, дата |

Якщо розміри не знайдено, рядок **не записується**. Краще жодного запису, ніж хибний розмір для руки. Записи з OCR мають `status: ok, from label text (OCR) - check before use` і сирий текст у колонці `label_text` для перевірки.

**Точність на 11 фото з телефона** (`tests/label_ocr_eval.py`, Docker на x86):

| | |
|---|---|
| Розміри прочитано правильно | **7 з 11** |
| Хибні розміри | **0** (де не вийшло, поля порожні) |
| Час на етикетку | 5–22 с вдалі, до `ocr_budget` невдалі |

Не прочиталися: дві етикетки внахлест під кутом, етикетка під плівкою з відблисками, розмита під сильною перспективою та одна, де OCR склеїв `422x18` у `42318`. На Pi 5 OCR буде у 2–3 рази повільніший.

**Що найбільше вплине на точність на роботі:**
- Камера над деталлю, етикетка займає значну частину кадру. Для OCR 640×480 замало, краще `width: 1640`, `height: 1232` (режим IMX219 з біннінгом).
- Рівне світло без відблисків. Плівку з етикетки краще знімати.
- На цих же фото zbar читав QR лише після збільшення кадру вдвічі, тож вища роздільність допоможе і QR.

Нові колонки в `parts.csv`: `source` (`qr`/`label`), `edge_count`, `label_text`. Якщо `parts.csv` уже існує зі старим заголовком, ці колонки в нього не допишуться: файл зберігає свій заголовок. Щоб вони з'явились, перейменуй або видали старий файл.

Перевірка:

```bash
python3 -m pytest src/roboarm_nodes/test/test_labels.py -q      # розбір на реальному тексті OCR
python3 tests/label_ocr_eval.py --verbose                       # точність на фото з tests/labels/
ROS_DOMAIN_ID=85 python3 tests/label_smoke.py                   # фото -> camera_node -> vision_node -> /vision/part + CSV
```

Фото етикеток лежать у `tests/labels/` локально і в git не потрапляють (`.gitignore`), бо на них імена клієнтів.

## camera_node: бекенди

| `backend` | Де | Як |
|---|---|---|
| `libcamera` (за замовч.) | Yocto-образ vva-robot, Pi 5 + IMX219 | `libcamerasrc ! videoconvert ! appsink` (GStreamer через PyGObject), `rgb8` |
| `picamera2` | Raspberry Pi OS / старий Docker-образ | `capture_array('main')`, `bgr8` |
| `v4l2` | USB-камера | OpenCV `VideoCapture(device)`, `bgr8` |
| `images` | Тести без камери | PNG/JPEG з `images_path` по колу з частотою `fps` |

Час кадру — час отримання в ROS, не експозиції (як у vva-camera). Помилка камери завершує вузол з ненульовим кодом, щоб systemd його перезапустив.

На Yocto-образі вже є `vva_camera camera_node`, який публікує той самий `/camera/image_raw` (rgb8, 640×480, 15 fps). Можна запускати **або** його, **або** наш `camera_node` з `backend: libcamera` — не обидва одночасно. Наш додає `/camera/camera_info` з калібруванням, без якого немає пози.

## stream_node: старий TCP-стрім

```bash
ros2 launch roboarm_bringup bringup.launch.py demo_perception:=false video_stream:=true \
  perception_config:=$HOME/.config/roboarm/perception.yaml
```

На ноуті, як і раніше:

```bash
ffplay -fflags nobuffer -flags low_delay -framedrop tcp://<IP_Pi>:8554
```

Видно камеру з рамкою навколо QR і плашкою з payload. Поки ніхто не підключений, нічого не кодується. Потрібен PyAV (`sudo apt install python3-av`; для Yocto див. нижче). Усередині ROS те саме дивляться без стріму: `ros2 run rqt_image_view rqt_image_view /vision/debug_image`.

Відмінність від старої схеми: раніше 720p main-потік ішов у відео, а 640×480 lores — у zbar, обидва з ISP. Тепер у стрім іде той самий кадр, який сканувався (640×480 за замовч.). Якщо потрібна більша роздільність для зчитування дрібних етикеток, збільш `width`/`height` у `camera_node` (і перекалібруй камеру).

## Перевірка без робота

```bash
# 1. Офлайн-тести (zbar, конвертація кадрів, порядок кутів, поза з синтетичного кадру, парсер iFurn)
python3 -m pytest src/roboarm_nodes/test -q

# 2. Увесь шлях perception на фото етикеток замість камери
ros2 run roboarm_nodes camera_node --ros-args -p backend:=images -p images_path:=$HOME/labels
ros2 run roboarm_nodes vision_node
ros2 topic echo /vision/qr
ros2 topic echo /vision/part
```

Для пози в цьому тесті додай фіктивне калібрування в `camera_node` (`-p camera_matrix:="[600.,0.,320.,0.,600.,240.,0.,0.,1.]"`) і статичний TF:

```bash
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id camera_optical_frame
ros2 topic echo /vision/target_pose
```

## Yocto (meta-vva-robot)

`yocto/meta-vva-robot/recipes-ros/roboarm-nodes/` — рецепт `roboarm-nodes_0.1.0.bb` для шару meta-vva-robot (Scarthgap + meta-ros2-jazzy) і два systemd-юніти, вимкнені за замовчуванням, як vva-camera/vva-motor:

```bitbake
IMAGE_INSTALL:append = " roboarm-nodes"
```

- Для OCR етикеток: `tesseract` і мовні пакети `tesseract-lang-ukr/rus/eng/osd` (meta-oe).
- Залежності perception: `rclpy`, `tf2-ros-py`, `python3-numpy`, `python3-pillow`, `zbar` (meta-oe), `python3-opencv` (opencv з `PACKAGECONFIG` `python3`), `python3-pygobject` і GStreamer + `libcamera-gst` для бекенда `libcamera`.
- PyAV для `stream_node` у meta-python для Scarthgap немає; він у `RRECOMMENDS`. Без нього решта вузлів працює.
- Параметри — у `/etc/vva-robot/robot.yaml` під ключами `camera_node:` / `vision_node:` / `stream_node:`, як і для вузлів vva.
- Юніти: `vva-vision.service` (після `vva-camera.service`) і `vva-stream.service`. Увімкнути після перевірки на стенді: `systemctl enable --now vva-vision.service`.
- Сервіси працюють від root, тому `parts.csv` за замовчуванням лягає в `/root/.local/share/roboarm/parts.csv`; інше місце — `vision_node.parts_csv`.
- Для доступу до iFurn образу потрібен інтернет і правильний час (NTP), інакше HTTPS не пройде перевірку сертифіката.

Рецепт **не збирався** bitbake'ом (шару й Poky тут немає): імена пакетів (`zbar`, `libcamera-gst`, `python3-opencv`) треба звірити з ревізіями шарів у Влада першою збіркою `bitbake roboarm-nodes`.

## Що не перевірено

- Реальна камера: бекенди `libcamera` і `picamera2` не запускались на Pi (доступу до заліза немає). `images`, `vision_node`, `stream_node`, TF і поза перевірені в Docker на `ros:jazzy`.
- Калібрування камери і TF кріплення камери треба виміряти.
- Bitbake-збірка рецепта.
- OCR етикеток на Pi 5: швидкість і точність з камерою IMX219 на робочій відстані.
