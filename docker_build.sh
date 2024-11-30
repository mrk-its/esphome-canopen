set -x
FRAMEWORK_OPTS="-s type esp-idf -s version 5.2.1 -s platform_version 6.7.0"
BOARD="-s board esp32-c3-devkitm-1"
# FRAMEWORK_OPTS="-s type esp-idf -s version 4.4.8 -s platform_version 5.4.0'  # recommended
# FRAMEWORK_OPTS="-s type arduino -s version recommended -s platform_version 5.4.0"
docker run --mount type=bind,source=$(pwd),target=/config -it esphome/esphome $FRAMEWORK_OPTS $BOARD compile test/config.yaml.template

