
# build all targets
foreach (example example-01 example-gripping example-deformable example-friction example-texture)

    file (GLOB source ${example}/*.cpp )
    add_executable (${example} ${source})
    target_link_libraries (${example} maestro_chai3d)

endforeach ()