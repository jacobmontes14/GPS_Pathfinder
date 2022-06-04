import gps
import bearing
import berryIMU
import distance
from signal import signal, SIGTERM, SIGHUP, pause
from rpi_lcd import LCD

lcd = LCD()

def safe_exit(signum, frame):
    exit(1)

signal(SIGTERM, safe_exit)
signal(SIGHUP, safe_exit)

try:
    lcd.text("Hello, world!", 1)
    lcd.text("This is Test!", 2)

except KeyboardInterrupt:
    pass

finally:
    lcd.clear()