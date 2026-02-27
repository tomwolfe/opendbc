import os
import capnp
import sys
from opendbc.car.common.basedir import BASEDIR

# Always load car schema from opendbc package
# This makes opendbc standalone without requiring cereal

# Check if cereal has already loaded the car schema
# If so, reuse it to avoid duplicate ID errors
if "cereal" in sys.modules:
  car = sys.modules["cereal"].car
else:
  capnp.remove_import_hook()
  car = capnp.load(os.path.join(BASEDIR, "car.capnp"))

CarState = car.CarState
RadarData = car.RadarData
CarControl = car.CarControl
CarParams = car.CarParams

CarStateT = capnp.lib.capnp._StructModule
RadarDataT = capnp.lib.capnp._StructModule
CarControlT = capnp.lib.capnp._StructModule
CarParamsT = capnp.lib.capnp._StructModule
