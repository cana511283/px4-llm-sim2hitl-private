# plant_sim: the aircraft plant (core + C Truth + C sensors) as an in-process PX4 simulator module.
# The plant is the validated libskysurfer_plant shared library of the active model release (or of a build
# cache), passed as -DPLANT_LIB=<path>; it is linked, not recompiled with PX4 flags.
if(NOT PLANT_LIB)
	set(PLANT_LIB $ENV{PLANT_LIB})
endif()
if(NOT EXISTS "${PLANT_LIB}")
	message(FATAL_ERROR "plant_sim: set PLANT_LIB to the libskysurfer_plant shared library")
endif()
get_filename_component(SIM2HITL_PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)

px4_add_module(
	MODULE modules__plant_sim
	MAIN plant_sim
	INCLUDES
		${SIM2HITL_PROJECT_ROOT}/plant/include
	SRCS
		PlantSim.cpp
		PlantSim.hpp
	)
# External modules are configured before src/lib, so PX4 libraries are linked by name here (resolved at
# generation time) instead of px4_add_module DEPENDS.
target_link_libraries(modules__plant_sim PRIVATE drivers_accelerometer drivers_gyroscope dataman_client ${PLANT_LIB})
