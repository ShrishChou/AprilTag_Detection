from xarm.wrapper import XArmAPI
from xarm.version import __version__


print('xArmPythonSDK Version: {}'.format(__version__))

arm = XArmAPI('192.168.1.206')

print('xArm Version: {}'.format(arm.version))

arm.disconnect()
