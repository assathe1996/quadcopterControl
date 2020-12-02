px4_add_module(
    MODULE examples__observer
    MAIN observer
    STACK_MAX 3500
	COMPILE_FLAGS
    SRCS
        observer.cpp
    DEPENDS
   		px4_work_queue	
    )

