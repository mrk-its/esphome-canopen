set -x
FRAMEWORK="{type: esp-idf}"
#FRAMEWORK="{type: arduino}"
BOARD=esp32-c3-devkitm-1
framework="$FRAMEWORK" board="$BOARD" envsubst < test/config.yaml.template > test/config.yaml
docker run --rm --mount type=bind,source=$(pwd),target=/config -it esphome/esphome compile test/config.yaml
