echo "========================================="
echo "   配置下位机串口 -> /dev/gm_robot_1"
echo "========================================="
sudo bash -c 'echo "KERNEL==\"ttyACM*\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"55d4\", ATTRS{serial}==\"596F003650\", MODE:=\"0777\", SYMLINK+=\"gm_robot_1\"" > /etc/udev/rules.d/gm_robot_1.rules'

echo ""
echo "========================================="
echo "   配置激光雷达 -> /dev/gm_laser"  
echo "========================================="
sudo bash -c 'echo "KERNEL==\"ttyACM*\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"55d4\", ATTRS{serial}==\"5A6D014210\", MODE:=\"0777\", SYMLINK+=\"gm_laser\"" > /etc/udev/rules.d/gm_laser.rules'
service udev reload
sleep 2
service udev restart
