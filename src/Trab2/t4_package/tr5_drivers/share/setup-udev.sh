sudo cp udev/99-tr5.rules /etc/udev/rules.d/ 
echo "TR5 udev rules file has been copied to /etc/udev/rules.d/"
sudo udevadm control --reload-rules
