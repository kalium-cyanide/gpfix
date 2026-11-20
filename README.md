# Запуск

Для запуска нужен platfromIO:

Проверка зависимостей:

```bash
python3 --version
```

(установка)
```bash
#.deb
sudo apt install python3 python3-pip python3-venv curl

#.rpm
sudo dnf install python3 python3-pip curl
```

Затем запустите скрипт:
```bash
python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py)"
```

Для сборки:

```bash
pio run -e bike 
```
env можно выбрать из {bike, car}

Добавляем в path

```bash
echo 'export PATH="$PATH:$HOME/.platformio/penv/bin"' >> ~/.bashrc
source ~/.bashrc
```

Для прошивки:

```bash
pio run -e bike -t upload
```