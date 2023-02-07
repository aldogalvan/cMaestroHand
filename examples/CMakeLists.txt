
# build all targets
foreach (example example-teapot example-gripping example-friction example-multidigit example-deformable example-dynamic)

    file (GLOB source ${example}/*.cpp )
    add_executable (${example} ${source})
    target_link_libraries (${example} PUBLIC maestro_chai3d )
endforeach ()