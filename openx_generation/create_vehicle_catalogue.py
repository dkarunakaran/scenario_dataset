""" 
    Example of how to create a vehicle catalog
    The properties are added in order to work with esmini 1.5-1.7
    
    Some features used:
    - Vehicle
    - BoundingBox
    - Axle
"""
import os
from scenariogeneration import xosc, prettyprint


# vehicle.tesla.model3
bb = xosc.BoundingBox(2.1,4.5,1.8,1.5,0,0.9)
fa = xosc.Axle(0.5,0.6,1.8,3.1,0.3)
ba = xosc.Axle(0.0,0.6,1.8,0.0,0.3)
ego_veh = xosc.Vehicle('ego',xosc.VehicleCategory.car,bb,fa,ba,100,10,10)
ego_veh.add_property('model_id','0')
ego_veh.add_property('type','ego_vehicle')
# dump it and create a new catalog file
ego_veh.dump_to_catalog('vehicleCatalog.xosc','VehicleCatalog','carla vehicle vehicle catalog','Dhanoop')

bb = xosc.BoundingBox(2.0,5.0,1.8,1.4,0.0,0.9)
fa = xosc.Axle(0.5,0.8,1.68,2.98,0.4)
ba = xosc.Axle(0.0,0.8,1.68,0.0,0.4)
adversary_veh = xosc.Vehicle('car',xosc.VehicleCategory.car,bb,fa,ba,100,10,10)
adversary_veh.add_property('model_id','1')
adversary_veh.append_to_catalog('vehicleCatalog.xosc')

bb = xosc.BoundingBox(1.8,4.5,1.5,1.3,0.0,0.8)
fa = xosc.Axle(0.5,0.8,1.68,2.98,0.4)
ba = xosc.Axle(0.0,0.8,1.68,0.0,0.4)
adversary_veh = xosc.Vehicle('van',xosc.VehicleCategory.van,bb,fa,ba,100,10,10)
adversary_veh.add_property('model_id','2')
adversary_veh.append_to_catalog('vehicleCatalog.xosc')

bb = xosc.BoundingBox(0.9,2.2,1.3,0.40,0.0,0.65)
fa = xosc.Axle(1.5,0.7,0.1,1.5,0.35)
ba = xosc.Axle(0.0,0.7,0.1,0.0,0.35)
adversary_veh = xosc.Vehicle('motorbike',xosc.VehicleCategory.motorbike,bb,fa,ba,100,10,10)
adversary_veh.add_property('model_id','3')
adversary_veh.append_to_catalog('vehicleCatalog.xosc')


