import SANDriver
import time
from random import seed
from random import randint

# This driver Generates simulated pulses of 5 seconds with a frequency 
# based no a random interval between 10 and 30s
# Each instance of this class will have diferent period

class PulseDriver(SANDriver.SANDriver):

    def setup(self):

        self.setMeta('sensorManufacturer', self.fromConfig('sensorManufacturer'))
        self.setMeta('measurementType', self.fromConfig('measurementType'))
        self.setMeta('sensorType', self.fromConfig('sensorType'))


	#Example print to tty
        self.start_time = time.time()

        # Get random number to define pulse frequency
        #Dont seed(1)  Default seed uses clock in ms
        self.pulse_period = randint(10, 30)
        print('STARTED period={0} {1} and {2}'.format(self.pulse_period,self.fromConfig('sensorManufacturer'), 'self.sensorType'))

    def get_reading(self):

        t = time.time() - self.start_time
        print("Reading period={} : t={}".format(self.pulse_period,time.ctime(),t))
        if (t > self.pulse_period)  :
            if (t >= self.pulse_period+5)  : self.start_time = time.time()  #start timer again to create a pulse
            print("*********************TRUE*********************")
            return True
        else:
            return False
