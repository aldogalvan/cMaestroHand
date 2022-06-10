
# build all targets
foreach (example example-01)

    file (GLOB source ${example}/*.cpp )
    add_executable (${example} ${source})
    target_link_libraries (${example} maestro_chai3d)

endforeach ()