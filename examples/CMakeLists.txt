
# build all targets
foreach (example example-01)

    file (GLOB source ${example}/*.cpp ${example}/*.h)
    add_executable (${example} ${source})
    target_link_libraries (${example} maestro-haptics)

endforeach ()