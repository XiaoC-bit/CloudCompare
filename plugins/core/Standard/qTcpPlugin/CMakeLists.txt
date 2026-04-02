option( PLUGIN_STANDARD_QTCP "Install qTcpPlugin" OFF )

if( PLUGIN_STANDARD_QTCP )
	project( qTcpPlugin )
	
    find_package( Qt5 COMPONENTS Network REQUIRED )  # 加这一行
	AddPlugin( NAME ${PROJECT_NAME} )
	
	target_sources( ${PROJECT_NAME} PRIVATE
		qTcpPlugin.h
		qTcpPlugin.cpp
		CcTcpServer.h
		CcTcpServer.cpp
		CommandDispatcher.h
		CommandDispatcher.cpp
		CommLogger.h
		CommLogger.cpp
		ccRegistrationTools.h
		ccRegistrationTools.cpp
		EigenUtils.h
		EigenUtils.cpp
		LJS8_ACQ.h
		LJS8_ACQ.cpp
	)
	
	target_link_libraries( ${PROJECT_NAME} 
		CCCoreLib
		
    ${CMAKE_CURRENT_SOURCE_DIR}/LJS8_IF.lib
	Qt5::OpenGL
	Qt5::OpenGLExtensions
    Qt5::Network    
	)

	target_include_directories( ${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
	)

	target_compile_options( ${PROJECT_NAME} PRIVATE
    /utf-8
)
	
	
	set_property(GLOBAL APPEND PROPERTY CC_PLUGINS ${PROJECT_NAME})
	
	message(STATUS "\n==================")
	message(STATUS "qTcpPlugin plugin enabled")
	message(STATUS "==================\n")
	
	message(STATUS "\n==================")
	message(STATUS "Note: You need to manually add 'add_subdirectory(qTcpPlugin)' to")
	message(STATUS "plugins/core/Standard/CMakeLists.txt to compile this plugin.")
	message(STATUS "==================\n")
endif()