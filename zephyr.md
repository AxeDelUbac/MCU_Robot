python3 -m venv ~/Documents/Dev/ZephyrOS/.venv

source ~/Documents/Dev/ZephyrOS/.venv/bin/activate


west init ~/Documents/Dev/ZephyrOS
cd ~/Documents/Dev/ZephyrOS
west update

west zephyr-export

west packages pip --install

cd ~/Documents/Dev/ZephyrOS/zephyr
west sdk install

# construction projet avec carte

# build
west build -b nucleo_g0b1re samples/hello_world/

# flash
west flash