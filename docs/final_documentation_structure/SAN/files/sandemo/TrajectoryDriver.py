import SANDriver
import time
from random import seed
from random import randint
from math import sin

# This driver Generates simulated circular x (y) trajectory with amplitude 
# based no a random interval between 1 and 5
# Each instance of this class will have diferent amplitude

class TrajectoryDriver(SANDriver.SANDriver):

    def setup(self):

        self.setMeta('sensorManufacturer', self.fromConfig('sensorManufacturer'))
        self.setMeta('measurementType', self.fromConfig('measurementType'))
        self.setMeta('sensorType', self.fromConfig('sensorType'))

	#Example print to tty
        self.start_time = time.time()

        # Get random number to define pulse frequency
        #Dont seed(1)  Default seed uses clock in ms
        self.amplitude = randint(10, 50)/10
        print('STARTED amplitude={0} {1} and {2}'.format(self.amplitude,self.fromConfig('sensorManufacturer'), 'self.sensorType'))

    def get_reading(self):

        t = time.time() - self.start_time
        print("Reading amplitude={0} : t={1} {2}".format(self.amplitude,time.ctime(),t))
        return sin(t)*self.amplitude

