killall wpa_supplicant
sleep 5
wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf
sleep 5
ifup wlan0
sleep 5
ifconfig
hostname -I
