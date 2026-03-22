# src/

Firmware source projects. Each subdirectory is a self-contained ESP-IDF project that can be built and flashed independently.

Add a new subdirectory here for each firmware project or experiment. Projects may share hardware knowledge from `../knowledge/` but have their own `CMakeLists.txt`, `sdkconfig.defaults`, and `main/idf_component.yml`.

Refer to `../skills/embedded-project-setup.md` for build conventions, dependency declarations, and flash procedures.
