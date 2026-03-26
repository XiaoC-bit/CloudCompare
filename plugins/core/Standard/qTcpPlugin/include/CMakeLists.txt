
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/CcTcpServer.h
		${CMAKE_CURRENT_LIST_DIR}/CommandDispatcher.h
		${CMAKE_CURRENT_LIST_DIR}/qTcpPlugin.h
		${CMAKE_CURRENT_LIST_DIR}/CommLogger.h
		${CMAKE_CURRENT_LIST_DIR}/ccRegistrationTools.h
		${CMAKE_CURRENT_LIST_DIR}/EigenUtils.h
		)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
