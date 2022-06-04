from berryIMU import Heading
import time


x = Heading()

while(True):
    print(x.get_heading())
    time.sleep(0.03)




