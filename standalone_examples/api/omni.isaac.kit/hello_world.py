# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

# The most basic usage for creating a simulation app
kit = SimulationApp()

import carb
# server_check = carb.settings.get_settings().get_as_string("/persistent/isaac/asset_root/default")
# print(server_check)

for i in range(100):
    kit.update()

kit.close()  # Cleanup application



# kit = SimulationApp()

# import numpy as np
# from omni.isaac.core.objects import DynamicCuboid
# from omni.isaac.core.objects.ground_plane import GroundPlane
# from omni.isaac.core.physics_context import PhysicsContext

# PhysicsContext()
# GroundPlane(prim_path="/World/groundPlane", size=10, color=np.array([0.5, 0.5, 0.5]))
# DynamicCuboid(prim_path="/World/cube",
#     position=np.array([-.5, -.2, 1.0]),
#     scale=np.array([.5, .5, .5]),
#     color=np.array([.2,.3,0.]))

# import carb
# server_check = carb.settings.get_settings().get_as_string("/persistent/isaac/asset_root/default")
# print(server_check)


# for i in range(100):
#     kit.update()


# kit.close() 