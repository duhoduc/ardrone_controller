import subprocess
import time
import os

# enable wifi
print("Enable Wifi...")
subprocess.check_output(["nmcli", "nm", "wifi", "on"])

while True:
    result = subprocess.check_output(["nmcli", "-t", "-f", "WIFI", "nm", "wifi"])
    if result.strip() == "enabled":
        break
    time.sleep(1)
print("Done")

# check for network of interest
myssid = None
while myssid == None:
    result = subprocess.check_output(["nmcli", "-t", "-f", "SSID", "dev", "wifi" ,"list"])
    result = result.replace("'", "")
    for essid in result.splitlines():
        if essid.startswith("ardrone2"):
            myssid = essid
            break
    time.sleep(1)

print("Found: " + myssid)

# try to connect to it
print("Connecting...")
result = subprocess.check_output(["nmcli", "-t", "-f", "SSID", "dev", "wifi" ,"connect", myssid])

while True:
    result = subprocess.check_output(["nmcli", "-t", "-f", "NAME", "con", "status"])
    if myssid in result:
        break
    time.sleep(1)
print("Connected")

result = os.system("echo \"{ ifconfig ath0 0.0.0.0; iwconfig ath0 essid 'ACT Cage' && wpa_supplicant -B -Dwext -iath0 -c/etc/wpa_supplicant.conf ; wait 5; /sbin/udhcpc -i ath0; } &\" | telnet 192.168.1.1")
print(result)

# disable wifi
print("Disable Wifi")
print(subprocess.check_output(["nmcli", "nm", "wifi", "off"]))
