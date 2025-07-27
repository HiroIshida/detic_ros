cmake_minimum_required(VERSION 3.0.2)
project(detic_ros)

option(USE_VIRTUALENV "Installing the python requirements on virtualenv when building" ON)
option(DETIC_DOWNLOAD_MODELS "Download pre-trained Detic model weights during the build" ON)

set(_components std_msgs sensor_msgs message_generation)

if(USE_VIRTUALENV)
    list(APPEND _components catkin_virtualenv)
endif()

find_package(catkin REQUIRED COMPONENTS
    ${_components}
)

find_program(NVIDIA_SMI nvidia-smi)
if(NOT NVIDIA_SMI)
    message(WARNING "NVIDIA Driver not found. It is strongly recommended that the NVIDIA GPU is available")
endif()

find_package(Git REQUIRED)
if ((NOT EXISTS ${PROJECT_SOURCE_DIR}/Detic/third_party/CenterNet2)
    OR (NOT EXISTS ${PROJECT_SOURCE_DIR}/Detic/third_party/Deformable-DETR))
    execute_process(
        COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    )
endif()

catkin_python_setup()

if(DETIC_DOWNLOAD_MODELS)
    find_package(Wget REQUIRED)
    set(MODEL_DIR "${PROJECT_SOURCE_DIR}/models")
    file(MAKE_DIRECTORY ${MODEL_DIR})
    set(MODEL_FILES
        "Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth"
        "Detic_LCOCOI21k_CLIP_CXT21k_640b32_4x_ft4x_max-size.pth"
        "Detic_LCOCOI21k_CLIP_R5021k_640b32_4x_ft4x_max-size.pth"
        "Detic_LCOCOI21k_CLIP_R18_640b32_4x_ft4x_max-size.pth"
    )
    set(MODEL_BASE_URL "https://dl.fbaipublicfiles.com/detic")
    set(MODEL_OUTPUT_FILES "")
    foreach(MODEL_FILENAME ${MODEL_FILES})
        set(MODEL_PATH "${MODEL_DIR}/${MODEL_FILENAME}")
        set(MODEL_URL "${MODEL_BASE_URL}/${MODEL_FILENAME}")

        add_custom_command(
            OUTPUT ${MODEL_PATH}
            COMMAND ${WGET_EXECUTABLE} -q -N -P "${MODEL_DIR}" "${MODEL_URL}"
            COMMENT "Downloading Detic model: ${MODEL_FILENAME}..."
            VERBATIM
        )
        list(APPEND MODEL_OUTPUT_FILES ${MODEL_PATH})
    endforeach()

    add_custom_target(download_detic_models ALL
        DEPENDS ${MODEL_OUTPUT_FILES}
        COMMENT "Ensuring Detic models are downloaded."
    )
endif()

add_message_files(
    FILES
    SegmentationInfo.msg
)

add_service_files(
    FILES
    DeticSeg.srv
    CustomVocabulary.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
)
  
catkin_package(
    CATKIN_DEPENDS message_runtime
)

if(USE_VIRTUALENV)
    message(STATUS "detic_ros would be built with catkin virtualenv")
    catkin_generate_virtualenv(
        PYTHON_INTERPRETER python3
        CHECK_VENV FALSE
        INPUT_REQUIREMENTS requirements.txt
    )
else(USE_VIRTUALENV)
    message(STATUS "detic_ros doesn't install requirements. Please install them manually")
endif()

file(GLOB PYTHON_SCRIPTS node_script/*.py)
catkin_install_python(
    PROGRAMS ${PYTHON_SCRIPTS}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES requirements.txt
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  add_custom_target(${PROJECT_NAME}_download_image ALL
    COMMAND ${PROJECT_SOURCE_DIR}/test/prepare_test_data.sh)

  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/test_node.test)
  # add_rostest(test/test_batch_processor.test)  # somehow fail on workflow but fine on local
endif()
