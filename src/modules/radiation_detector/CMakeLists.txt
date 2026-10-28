px4_add_module(
	MODULE modules__radiation_detector
	MAIN radiation_detector
	STACK_MAIN 2000
	SRCS
		radiation_detector.cpp
	DEPENDS
	)
