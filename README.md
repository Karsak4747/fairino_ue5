# fairino_ue5
Для запуска необходимо установить docker-engine и docker-compose.
Compose файл запускает ros2 контейнер с роботом, виртуальный контейнер с роботом и сеть между ними с именем fairino-net.
Перед запуском необходимо установить [образ](https://fairino-doc-en.readthedocs.io/latest/VMMachine/controller_docker_machine.html) контейнера с контролером робота.

Запускается командой
```shell
docker-compose up -d
```