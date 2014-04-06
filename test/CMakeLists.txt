add_executable(field-sensitive-test field-sensitive-test.cpp)
add_executable(dump-points-to dump-points-to.cpp)
add_executable(points-to-test points-to-test.cpp PTGTester.cpp)
add_executable(points-to-perf points-to-perf.cpp)

llvm_map_components_to_libraries(FST_LLVM_LIBS core engine asmparser bitreader bitwriter)

target_link_libraries(field-sensitive-test LLVMSlicer ${FST_LLVM_LIBS})
target_link_libraries(dump-points-to LLVMSlicer ${FST_LLVM_LIBS})
target_link_libraries(points-to-test LLVMSlicer ${FST_LLVM_LIBS})
target_link_libraries(points-to-perf LLVMSlicer ${FST_LLVM_LIBS})

add_test(Field-sensitive-test field-sensitive-test)
add_test(Points-to-test points-to-test)
