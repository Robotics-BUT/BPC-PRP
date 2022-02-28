#/bin/bash

# https://learn.sparkfun.com/tutorials/how-to-run-a-raspberry-pi-program-on-startup#method-3-systemd

strip robot
sudo cp -f robot /opt/robot
sudo cp -f robot.service /lib/systemd/system/robot.service
sudo systemctl daemon-reload
sudo systemctl enable robot.service
sudo systemctl start robot.service

# journalctl -u robot.log
# systemctl status robot.service
# sudo systemctl stop robot.service
# sudo systemctl disable robot.service