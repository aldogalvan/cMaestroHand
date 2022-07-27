# build all targets
foreach (experiment experiment-joint-haptics)

    file (GLOB source ${experiment}/*.cpp )
    add_executable (${experiment} ${source})
    target_link_libraries (${experiment} maestro_chai3d)

endforeach ()