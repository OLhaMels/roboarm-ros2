# Perception у Docker на Raspberry Pi 5

Контейнер `roboarm-perception`: ROS 2 Jazzy + `camera_node` + `vision_node` + `stream_node`.
Усередині зібрано libcamera з форку Raspberry Pi (з підтримкою Pi 5 / PiSP і `libcamerasrc`), бо в Ubuntu його немає.

## 1. Скопіювати проєкт на Pi

З Windows (PowerShell), замінивши користувача й адресу:

```powershell
scp -r .\roboarm-ros2 <user>@<IP_Pi>:~/
```

(або через git, якщо проєкт буде в репозиторії).

## 2. На Pi: зупинити старий контейнер

Камеру може тримати лише один процес:

```bash
docker ps                       # знайти контейнер upd-rasp-stream-tcp
docker stop <його_назва>
```

## 3. Зібрати й запустити

```bash
cd ~/roboarm-ros2
docker compose -f docker/docker-compose.yml up -d --build
docker logs -f roboarm-perception
```

Перша збірка на Pi 5 займає приблизно 15–30 хв (компілюється libcamera); наступні — секунди.
У логах має бути `Publishing libcamera frames on /camera/image_raw`, а при QR у кадрі — `QR-Code q=1 ... -> https://...`.

## 4. Дивитися

- Відео з рамкою QR, на ноуті: `ffplay -fflags nobuffer -flags low_delay -framedrop tcp://<IP_Pi>:8554`
- Топіки, на Pi:
  ```bash
  docker exec -it roboarm-perception /entrypoint.sh bash
  ros2 topic list
  ros2 topic echo /vision/qr
  ros2 topic hz /camera/image_raw --qos-reliability best_effort
  ```
- Записані деталі: `~/roboarm-ros2/docker/data/parts.csv` на Pi.

## Налаштування

`docker/perception.yaml` монтується в контейнер; після змін: `docker compose -f docker/docker-compose.yml restart`.
Зміни в коді (`src/`) потребують `up -d --build`.

Тест без камери: у `perception.yaml` поставити `backend: images`, `images_path: /data/labels` і покласти фото етикеток у `docker/data/labels/`.

## Якщо камера не стартує

`Cannot start camera pipeline` означає, що libcamera не знайшла камеру. Перевірити на самому Pi (поза Docker):

```bash
ls /dev/media* /dev/video* /dev/dma_heap
dmesg | grep -i imx219
```

Усередині контейнера: `docker exec roboarm-perception gst-launch-1.0 libcamerasrc ! fakesink` — у виводі буде причина.
Хост має бути 64-бітним (arm64) з драйверами камери в ядрі (Raspberry Pi OS / Ubuntu для Pi / образ vva-robot).
