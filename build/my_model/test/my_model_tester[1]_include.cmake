if(EXISTS "/home/rishie/Documents/Semester-3/ENPM 700/Final Project/nav_guide_project/build/my_model/test/my_model_tester[1]_tests.cmake")
  include("/home/rishie/Documents/Semester-3/ENPM 700/Final Project/nav_guide_project/build/my_model/test/my_model_tester[1]_tests.cmake")
else()
  add_test(my_model_tester_NOT_BUILT my_model_tester_NOT_BUILT)
endif()