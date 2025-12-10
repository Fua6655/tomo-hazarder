README — Raspberry Pi setup za control_tomo (PS4 + failsafe + systemd)

Ovaj README opisuje kako postaviti i pokretati tomo_power_node (GPIO relej za paljenje Tomi) kao systemd servis na Raspberry Pi (Ubuntu 24.04, ROS2 Jazzy). Servis koristi SSH/ROS instalaciju i pokreće node kao root kako bi imao pristup /dev/gpiomem.

File systemd servisa: /etc/systemd/system/tomo_power.service
Node izvršna komanda (u servisu):
ros2 run control_tomo tomo_power_node
Ako je tvoj workspace instaliran drugačije, promijeni ExecStart kako je opisano dolje.

# 1. Pretpostavke

Raspberry Pi 4B, Ubuntu Server 24.04 

ROS2 Jazzy instaliran u /opt/ros/jazzy 

Tvoj ROS paket control_tomo buildan i instaliran u workspace (npr. /home/ubuntu/ros2_ws/install) — ili kad nema, servis može izostaviti source workspace liniju

Privilegije sudo na Pi

# 2. Systemd servis 
`
Path to save: /etc/systemd/system/tomo_power.service

# 3. Instalacija servisa

U terminalu na Pi:

```bash
sudo tee /etc/systemd/system/tomo_power.service > /dev/null << 'EOF'
[Unit]
Description=Tomo Power Node (control_tomo)
After=network.target

[Service]
Type=simple
User=root
Group=root
Environment="ROS_DISTRO=jazzy"
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && source /home/tomo/ros2_ws/install/setup.bash 2>/dev/null || true && exec ros2 run control_tomo tomo_power_node"
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF
```

# 4 Reload systemd i enable/start
```bash
sudo systemctl daemon-reload
sudo systemctl enable --now tomo_power.service
```

# 5 Provjera statusa i logova
```bash
sudo systemctl status tomo_power.service
sudo journalctl -u tomo_power.service -f
```


# 6 Kontrole servisa
```bash
sudo systemctl start tomo_power.service
sudo systemctl stop tomo_power.service
sudo systemctl restart tomo_power.service
sudo systemctl status tomo_power.service
```

# 7 Monitor log
```bash
sudo journalctl -u tomo_power.service -f
```

# 8 Onemogući/omogući auto-start pri boot
```bash
sudo systemctl disable tomo_power.service
sudo systemctl enable tomo_power.service
```
# 9 Testiranje (quick checks)

Provjeri da je /dev/gpiomem prisutan:
```bash
ls -l /dev/gpiomem
```

Ako servis radi, gledaj logove:
```bash
sudo journalctl -u tomo_power.service -n 200 --no-pager
```

# 10 Dodavanje aliasa

Otvori svoj .bashrc
```bash
nano ~/.bashrc
```

Na kraj fajla dodaj sljedeće linije:

 Aliasi za Tomo Power Node
```bash
alias tomo='sudo systemctl'
alias tomo_on='sudo systemctl start tomo_power.service'
alias tomo_off='sudo systemctl stop tomo_power.service'
alias tomo_restart='sudo systemctl restart tomo_power.service'
alias tomo_status='sudo systemctl status tomo_power.service'
alias tomo_logs='sudo journalctl -u tomo_power.service -f'
```
# 11 Primjeni promjene
```bash
source ~/.bashrc
```

Testiraj PS4 joystick (USB):
```bash
ls /dev/input/js*
jstest /dev/input/js0
```

