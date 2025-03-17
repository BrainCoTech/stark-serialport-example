from utils import inspect_class
import bc_device_sdk

libstark = bc_device_sdk.stark

inspect_class(libstark.DeviceInfo)
inspect_class(libstark.ForceLevel)

print(libstark.ForceLevel(1))
print(libstark.ForceLevel.Small.int_value)

inspect_class(libstark.DfuState)
