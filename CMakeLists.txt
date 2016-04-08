cmake_minimum_required(VERSION 2.8.10)

# To specify a chaste directory uncomment below
#find_package(Chaste COMPONENTS heart crypt PATHS /home/robinsonm/tmp/chaste-install NO_DEFAULT_PATH)

# Otherwise let CMake find chaste
find_package(Chaste COMPONENTS heart crypt)

chaste_do_project(Plos2013)
