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
    add_custom_target(create_models_dir
        COMMAND ${CMAKE_COMMAND} -E make_directory "${PROJECT_SOURCE_DIR}/models"
    )
    add_custom_target(download_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x ALL
        DEPENDS create_models_dir
        COMMAND ${WGET_EXECUTABLE} -q "https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth" -O "${PROJECT_SOURCE_DIR}/models/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth"
    )
    add_custom_target(download_LCOCOI21k_CLIP_CXT21k_640b32_4x_ft4x ALL
        DEPENDS create_models_dir
        COMMAND ${WGET_EXECUTABLE} -q "https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_CXT21k_640b32_4x_ft4x_max-size.pth" -O "${PROJECT_SOURCE_DIR}/models/Detic_LCOCOI21k_CLIP_CXT21k_640b32_4x_ft4x_max-size.pth"
    )
    add_custom_target(download_LCOCOI21k_CLIP_R5021k_640b32_4x_ft4x ALL
        DEPENDS create_models_dir
        COMMAND ${WGET_EXECUTABLE} -q "https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_R5021k_640b32_4x_ft4x_max-size.pth" -O "${PROJECT_SOURCE_DIR}/models/Detic_LCOCOI21k_CLIP_R5021k_640b32_4x_ft4x_max-size.pth"
    )
    add_custom_target(download_LCOCOI21k_CLIP_R18_640b32_4x_ft4x ALL
        DEPENDS create_models_dir
        COMMAND ${WGET_EXECUTABLE} -q "https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_R18_640b32_4x_ft4x_max-size.pth" -O "${PROJECT_SOURCE_DIR}/models/Detic_LCOCOI21k_CLIP_R18_640b32_4x_ft4x_max-size.pth"
    )
    add_custom_target(download_ViT-B-32.pt ALL
        DEPENDS create_models_dir
        COMMAND ${WGET_EXECUTABLE} -q "https://openaipublic.azureedge.net/clip/models/40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af/ViT-B-32.pt" -O "${PROJECT_SOURCE_DIR}/models/ViT-B-32.pt"
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
